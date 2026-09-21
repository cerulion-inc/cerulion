// SPDX-License-Identifier: AGPL-3.0-only
//! Pure key→Twist state machine for the Go2 keyboard teleop node.
//!
//! Everything here is a pure function of its arguments — NO I/O, no wall
//! clock, no `crossterm` types — so the whole teleop policy is deterministic
//! and oracle-testable. The hardware `crossterm` pump ([`crate::pump`])
//! translates raw terminal key events into the [`KeyOp`]s this module folds;
//! the node body ([`crate::KeyboardTeleop`]) folds them and calls
//! [`KeyState::resolve`] with the node clock to produce the emitted Twist
//! (including the timed auto-zero).
//!
//! Determinism scope: this purity claim is about the
//! FUNCTION — same `(ops, now)` in, same command out. Platform replay
//! verifies the NODE by byte-comparing its recorded published frames
//! (Principle #7); it does not re-drive key input. See the `lib.rs`
//! Determinism section for the precise node-level claim.
//!
//! # Model: latched velocity + keepalive + auto-zero
//!
//! A terminal cannot reliably report key RELEASE, so the command is LATCHED: a
//! direction key sets a per-axis direction that persists and republishes at
//! 10 Hz (the pump's keepalive ring cadence) until changed. `space`/`Esc`
//! zeroes immediately. Crucially, **any [`AUTO_ZERO_NS`] without a keypress
//! auto-zeroes** — a terminal teleop must never latch a runaway command. The
//! auto-zero is computed HERE, in [`KeyState::resolve`], from the node clock,
//! so the emitted value is a function of the node's own inputs, independent
//! of the pump's (wall-clock) ring timing.
//!
//! # Mapping (WASD-style bindings)
//!
//! WASD-style bindings — INSPIRED BY, but deliberately NOT identical to,
//! ROS's `teleop_twist_keyboard` (whose actual layout is `i`/`j`/`k`/`l`/
//! `u`/`o` etc.). The `w`/`a`/`s`/`d` + `q`/`e` + `1`..`5` set below is the
//! layout this node chose; it does not follow the `teleop_twist_keyboard`
//! conventions.
//!
//! | Key(s) | Effect |
//! |---|---|
//! | `w` / `s` | forward / back (`linear.x`, ±) |
//! | `a` / `d` | strafe left / right (`linear.y`, ±; left = +y) |
//! | `q` / `e` | yaw left / right (`angular.z`, ±; left = +z) |
//! | `space`, `Esc` | immediate zero |
//! | `1`..`5` | speed-scale presets (0.2 .. 1.0 of max) |
//!
//! Speed at full scale (preset `5`) is the saturation max; presets scale BOTH
//! the latched axes (rescaled live) and future presses. Saturation is
//! therefore structural — a direction is `{-1, 0, +1}` and the scale is
//! `≤ 1.0`, so `|output| ≤ max` always.

// The pure module is unsafe-free by construction. (The forbid lives HERE,
// not at the crate root, because the `#[cerulion_node]` macro in lib.rs
// expands cdylib FFI entry points containing `unsafe`.)
#![forbid(unsafe_code)]

/// Max forward/back speed (m/s) at full scale. DUPLICATED from the joystick
/// node's canonical constant; `joystick_teleop`'s
/// `tests/const_consistency_test.rs` pins the two in lockstep.
pub const MAX_VX: f64 = 0.6;
/// Max strafe speed (m/s) at full scale.
pub const MAX_VY: f64 = 0.4;
/// Max yaw rate (rad/s) at full scale.
pub const MAX_VYAW: f64 = 1.0;
/// Auto-zero window: after this long with NO keypress the latched command is
/// zeroed (the runaway guard). INCLUSIVE at the boundary — an age of exactly
/// `AUTO_ZERO_NS` auto-zeroes.
pub const AUTO_ZERO_NS: u64 = 500_000_000; // 500 ms

/// The speed scale for preset `n` (1..=5): `n/5`, i.e. 0.2, 0.4, 0.6, 0.8, 1.0.
/// Out-of-range `n` is clamped into `1..=5`.
#[inline]
pub fn preset_scale(n: u8) -> f64 {
    (n.clamp(1, 5) as f64) / 5.0
}

/// A 3-DOF holonomic velocity command (maps onto `geometry_msgs/Twist` as
/// `linear.x = vx`, `linear.y = vy`, `angular.z = vyaw`, rest `0.0`).
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

    /// True iff all three DOFs are exactly `0.0` (deliberate exact float
    /// equality; `-0.0 == 0.0`).
    #[allow(clippy::float_cmp)]
    #[inline]
    pub fn is_zero(&self) -> bool {
        self.vx == 0.0 && self.vy == 0.0 && self.vyaw == 0.0
    }
}

/// A pure, hardware-agnostic key input — the crossterm-free boundary. The pump
/// maps `crossterm::event::KeyCode::Char(c)` → `Char(c)` and `KeyCode::Esc` →
/// `Esc`; everything else is dropped before it reaches [`key_to_op`].
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum KeyInput {
    /// A character key (case-folded by [`key_to_op`]).
    Char(char),
    /// The Escape key.
    Esc,
}

/// A recognised teleop operation. [`key_to_op`] maps raw keys to these; the
/// node folds each into a [`KeyState`].
#[derive(Debug, Clone, Copy, PartialEq, Eq, cerulion_core::state::CerulionState)]
pub enum KeyOp {
    /// `w` — forward (`+linear.x`).
    Forward,
    /// `s` — back (`-linear.x`).
    Backward,
    /// `a` — strafe left (`+linear.y`).
    StrafeLeft,
    /// `d` — strafe right (`-linear.y`).
    StrafeRight,
    /// `q` — yaw left (`+angular.z`).
    YawLeft,
    /// `e` — yaw right (`-angular.z`).
    YawRight,
    /// `space` / `Esc` — immediate zero.
    Stop,
    /// `1`..`5` — set the speed-scale preset.
    Preset(u8),
}

/// Map a raw key to a [`KeyOp`], or `None` for a key the node ignores.
/// Alphabetic keys are case-folded (Shift/CapsLock make no difference).
pub fn key_to_op(key: KeyInput) -> Option<KeyOp> {
    match key {
        KeyInput::Esc => Some(KeyOp::Stop),
        KeyInput::Char(c) => match c.to_ascii_lowercase() {
            'w' => Some(KeyOp::Forward),
            's' => Some(KeyOp::Backward),
            'a' => Some(KeyOp::StrafeLeft),
            'd' => Some(KeyOp::StrafeRight),
            'q' => Some(KeyOp::YawLeft),
            'e' => Some(KeyOp::YawRight),
            ' ' => Some(KeyOp::Stop),
            '1'..='5' => Some(KeyOp::Preset((c as u8) - b'0')),
            _ => None,
        },
    }
}

/// The latched keyboard state — the "drained state" the node body reads.
/// Directions are `{-1, 0, +1}` per axis; the actual velocity is computed at
/// [`resolve`](KeyState::resolve) time as `dir * max * scale`.
#[derive(Debug, Clone, Copy, PartialEq, cerulion_core::state::CerulionState)]
pub struct KeyState {
    /// Forward/back latch: -1, 0, +1.
    pub dir_x: i8,
    /// Strafe latch: -1 (right), 0, +1 (left).
    pub dir_y: i8,
    /// Yaw latch: -1 (right), 0, +1 (left).
    pub dir_yaw: i8,
    /// Current speed-scale preset (0.2 .. 1.0); default full (1.0).
    pub scale: f64,
    /// Node-clock ns of the last keypress, or `None` before any activity
    /// (drives the auto-zero window + the pre-first-key startup zero).
    pub last_activity_ns: Option<u64>,
}

impl Default for KeyState {
    fn default() -> Self {
        Self {
            dir_x: 0,
            dir_y: 0,
            dir_yaw: 0,
            scale: 1.0,
            last_activity_ns: None,
        }
    }
}

impl KeyState {
    /// Fold one [`KeyOp`], stamping `now_ns` as the last activity (resets the
    /// auto-zero window). `Stop` clears all latches; `Preset` changes the
    /// scale (which rescales the latched axes live at the next resolve).
    pub fn apply(&mut self, op: KeyOp, now_ns: u64) {
        self.last_activity_ns = Some(now_ns);
        match op {
            KeyOp::Forward => self.dir_x = 1,
            KeyOp::Backward => self.dir_x = -1,
            KeyOp::StrafeLeft => self.dir_y = 1,
            KeyOp::StrafeRight => self.dir_y = -1,
            KeyOp::YawLeft => self.dir_yaw = 1,
            KeyOp::YawRight => self.dir_yaw = -1,
            KeyOp::Stop => {
                self.dir_x = 0;
                self.dir_y = 0;
                self.dir_yaw = 0;
            }
            KeyOp::Preset(n) => self.scale = preset_scale(n),
        }
    }

    /// Resolve the current command for node-clock time `now_ns`.
    ///
    /// - Before ANY keypress (`last_activity_ns == None`) ⇒ [`Cmd::ZERO`]
    ///   (the startup zero — no fabricated latched value).
    /// - Age `now_ns - last_activity_ns >= AUTO_ZERO_NS` ⇒ CLEAR the latches
    ///   (runaway guard) and return [`Cmd::ZERO`].
    /// - Otherwise `dir * max * scale` per axis.
    ///
    /// Mutates `self` on the auto-zero path (clears the latches) so a
    /// subsequent keepalive resolve stays zero until a fresh keypress.
    pub fn resolve(&mut self, now_ns: u64) -> Cmd {
        let Some(last) = self.last_activity_ns else {
            return Cmd::ZERO;
        };
        if now_ns.saturating_sub(last) >= AUTO_ZERO_NS {
            self.dir_x = 0;
            self.dir_y = 0;
            self.dir_yaw = 0;
            return Cmd::ZERO;
        }
        Cmd {
            vx: self.dir_x as f64 * MAX_VX * self.scale,
            vy: self.dir_y as f64 * MAX_VY * self.scale,
            vyaw: self.dir_yaw as f64 * MAX_VYAW * self.scale,
        }
    }
}

/// Render the minimal one-line teleop status (the sanctioned plain-text UI,
/// printed by the pump to stderr — see [`crate::pump`]). Pure + oracle-tested
/// so the format is a fixed contract. `moving` reflects whether the command is
/// non-zero (the "link" liveness cue for the operator).
pub fn status_line(cmd: Cmd) -> String {
    let state = if cmd.is_zero() { "idle " } else { "MOVING" };
    format!(
        "[teleop {state}] vx={:+.3} vy={:+.3} vyaw={:+.3} (m/s, rad/s)  [w/a/s/d/q/e move · space stop · 1-5 speed]",
        cmd.vx, cmd.vy, cmd.vyaw
    )
}

#[cfg(test)]
mod tests {
    // These tests deliberately assert EXACT float values that are exact by
    // construction (single multiplies by 0.0 / ±1.0 / a shared const, or
    // exact ratios like n/5), so a direct `==` is correct here.
    #![allow(clippy::float_cmp)]

    use super::*;

    const EPS: f64 = 1e-12;
    const T0: u64 = 1_000_000_000;

    fn fresh_after(op: KeyOp) -> KeyState {
        let mut s = KeyState::default();
        s.apply(op, T0);
        s
    }

    // 1 — key_to_op mapping (incl. case-fold, digits, Esc, unknown).
    #[test]
    fn key_to_op_maps_each_key() {
        use KeyInput::{Char, Esc};
        assert_eq!(key_to_op(Char('w')), Some(KeyOp::Forward));
        assert_eq!(key_to_op(Char('W')), Some(KeyOp::Forward)); // case-fold
        assert_eq!(key_to_op(Char('s')), Some(KeyOp::Backward));
        assert_eq!(key_to_op(Char('a')), Some(KeyOp::StrafeLeft));
        assert_eq!(key_to_op(Char('d')), Some(KeyOp::StrafeRight));
        assert_eq!(key_to_op(Char('q')), Some(KeyOp::YawLeft));
        assert_eq!(key_to_op(Char('e')), Some(KeyOp::YawRight));
        assert_eq!(key_to_op(Char(' ')), Some(KeyOp::Stop));
        assert_eq!(key_to_op(Esc), Some(KeyOp::Stop));
        assert_eq!(key_to_op(Char('1')), Some(KeyOp::Preset(1)));
        assert_eq!(key_to_op(Char('5')), Some(KeyOp::Preset(5)));
        assert_eq!(key_to_op(Char('6')), None); // out of preset range
        assert_eq!(key_to_op(Char('z')), None); // unmapped
    }

    // 2 — preset_scale bounds + clamp.
    #[test]
    fn preset_scale_values() {
        assert_eq!(preset_scale(1), 0.2);
        assert_eq!(preset_scale(3), 0.6);
        assert_eq!(preset_scale(5), 1.0);
        assert_eq!(preset_scale(0), 0.2); // clamps up
        assert_eq!(preset_scale(9), 1.0); // clamps down
    }

    // 3 — each direction at full scale (default) is exactly ±max, correct sign.
    #[test]
    fn directions_full_scale_signs() {
        assert_eq!(fresh_after(KeyOp::Forward).resolve(T0).vx, MAX_VX);
        assert_eq!(fresh_after(KeyOp::Backward).resolve(T0).vx, -MAX_VX);
        assert_eq!(fresh_after(KeyOp::StrafeLeft).resolve(T0).vy, MAX_VY);
        assert_eq!(fresh_after(KeyOp::StrafeRight).resolve(T0).vy, -MAX_VY);
        assert_eq!(fresh_after(KeyOp::YawLeft).resolve(T0).vyaw, MAX_VYAW);
        assert_eq!(fresh_after(KeyOp::YawRight).resolve(T0).vyaw, -MAX_VYAW);
    }

    // 4 — Stop zeroes all latches.
    #[test]
    fn stop_zeroes() {
        let mut s = KeyState::default();
        s.apply(KeyOp::Forward, T0);
        s.apply(KeyOp::StrafeLeft, T0);
        s.apply(KeyOp::Stop, T0);
        assert_eq!(s.resolve(T0), Cmd::ZERO);
    }

    // 5 — startup: fresh state resolves to zero at any time (no fabricated latch).
    #[test]
    fn startup_is_zero() {
        let mut s = KeyState::default();
        assert_eq!(s.resolve(T0), Cmd::ZERO);
        assert_eq!(s.resolve(T0 + 10 * AUTO_ZERO_NS), Cmd::ZERO);
    }

    // 6 — preset scaling (hand oracle): preset 2 (scale 0.4) then forward ⇒
    // 1 * 0.6 * 0.4. Recomputed from the spec expression, not the fn internals.
    #[test]
    fn preset_scales_command() {
        let mut s = KeyState::default();
        s.apply(KeyOp::Preset(2), T0);
        s.apply(KeyOp::Forward, T0);
        let want = 1.0_f64 * MAX_VX * 0.4;
        assert!((s.resolve(T0).vx - want).abs() < EPS);
    }

    // 7 — preset RESCALES latched axes live: forward (full) then preset 3 ⇒
    // the same latch now scaled by 0.6.
    #[test]
    fn preset_rescales_latched_axis() {
        let mut s = KeyState::default();
        s.apply(KeyOp::Forward, T0);
        assert_eq!(s.resolve(T0).vx, MAX_VX);
        s.apply(KeyOp::Preset(3), T0);
        let want = 1.0_f64 * MAX_VX * 0.6;
        assert!((s.resolve(T0).vx - want).abs() < EPS);
    }

    // 8 — multi-axis full deflection with the documented signs.
    #[test]
    fn multi_axis_combined() {
        let mut s = KeyState::default();
        s.apply(KeyOp::Forward, T0);
        s.apply(KeyOp::StrafeLeft, T0);
        s.apply(KeyOp::YawLeft, T0);
        assert_eq!(
            s.resolve(T0),
            Cmd {
                vx: MAX_VX,
                vy: MAX_VY,
                vyaw: MAX_VYAW
            }
        );
    }

    // 9 — auto-zero boundary is INCLUSIVE at exactly AUTO_ZERO_NS; just under
    // still moves. After auto-zero the latches are CLEARED (a keepalive
    // resolve stays zero) until a fresh keypress re-arms.
    #[test]
    fn auto_zero_boundary_inclusive_and_clears_latch() {
        let mut s = KeyState::default();
        s.apply(KeyOp::Forward, T0);
        // Just under the window → still moving.
        assert_eq!(s.resolve(T0 + AUTO_ZERO_NS - 1).vx, MAX_VX);
        // Exactly at the window → auto-zero.
        assert_eq!(s.resolve(T0 + AUTO_ZERO_NS), Cmd::ZERO);
        // Latch cleared: a later resolve (even a fresh window relative to a
        // hypothetical new activity time) stays zero because dir is now 0.
        assert_eq!(s.resolve(T0 + AUTO_ZERO_NS), Cmd::ZERO);
        // A fresh keypress re-arms.
        s.apply(KeyOp::Forward, T0 + AUTO_ZERO_NS);
        assert_eq!(s.resolve(T0 + AUTO_ZERO_NS).vx, MAX_VX);
    }

    // 10 — saturation is structural: full-scale is exactly max, never exceeds
    // (dir ∈ {-1,0,1}, scale ≤ 1).
    #[test]
    fn saturation_never_exceeds_max() {
        let mut s = KeyState::default();
        s.apply(KeyOp::Preset(5), T0); // scale 1.0
        s.apply(KeyOp::Forward, T0);
        assert_eq!(s.resolve(T0).vx, MAX_VX);
        assert!(s.resolve(T0).vx <= MAX_VX);
    }

    // 11 — DETERMINISM: fold + resolve an identical (op, now) script twice ⇒
    // bit-identical command sequence, and the final equals a hand value.
    #[test]
    fn folding_is_deterministic() {
        let script: &[(KeyOp, u64)] = &[
            (KeyOp::Preset(3), T0),
            (KeyOp::Forward, T0 + 10_000_000),
            (KeyOp::StrafeLeft, T0 + 20_000_000),
            (KeyOp::YawRight, T0 + 30_000_000),
        ];
        let run = || {
            let mut s = KeyState::default();
            let mut out = Vec::new();
            for &(op, now) in script {
                s.apply(op, now);
                out.push(s.resolve(now));
            }
            out
        };
        let a = run();
        let b = run();
        assert_eq!(a, b, "identical scripts must fold to identical commands");
        let last = *a.last().expect("non-empty");
        // scale 0.6; forward (dir +1), strafe-left (dir +1), yaw-RIGHT
        // (dir -1 ⇒ the hand value is the NEGATED scaled max).
        let want_yaw = -(MAX_VYAW * 0.6);
        assert!((last.vx - MAX_VX * 0.6).abs() < EPS);
        assert!((last.vy - MAX_VY * 0.6).abs() < EPS);
        assert!((last.vyaw - want_yaw).abs() < EPS);
    }

    // 12 — status_line format is a fixed contract (hand oracle strings).
    #[test]
    fn status_line_format() {
        assert_eq!(
            status_line(Cmd::ZERO),
            "[teleop idle ] vx=+0.000 vy=+0.000 vyaw=+0.000 (m/s, rad/s)  [w/a/s/d/q/e move · space stop · 1-5 speed]"
        );
        assert_eq!(
            status_line(Cmd { vx: 0.6, vy: -0.4, vyaw: 1.0 }),
            "[teleop MOVING] vx=+0.600 vy=-0.400 vyaw=+1.000 (m/s, rad/s)  [w/a/s/d/q/e move · space stop · 1-5 speed]"
        );
    }
}
