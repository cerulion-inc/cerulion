// SPDX-License-Identifier: AGPL-3.0-only
//! Pure staleness-arbitration core for the Go2 teleop safety mux.
//!
//! [`arbitrate`] is a pure function: given the current time, the last held
//! joystick + keyboard commands (each with the wire-header publish timestamp of
//! the frame that delivered it), and the zero-keepalive bookkeeping, it decides
//! which source drives the robot, what command to emit, and whether to publish
//! this tick. It performs NO I/O and holds the WHOLE safety policy, so it is
//! deterministic and oracle-testable in isolation (Principle #7).
//!
//! # Safety contract
//!
//! **Never let a stale source keep the robot moving.** A source is *fresh* only
//! while its command's age (`now_ns - wire_timestamp_ns`) is strictly below its
//! freshness window. Joystick (local, low-latency) wins when fresh; else the
//! keyboard (allowed more headroom for network latency) wins when fresh; else
//! the output degrades to an all-zero command with [`ActiveSource::None`]. So a
//! source that goes silent is dropped within its own freshness window, and a
//! total input loss lands the robot at zero within the larger keyboard window.
//!
//! **The mux never forwards non-finite values.** A command with ANY NaN or
//! ±Inf component is INVALID: its source is treated as ABSENT for that
//! arbitration — control falls to the next priority (a garbage joystick
//! yields to a fresh keyboard; both garbage ⇒ safety zero + `None`) — and it
//! wins again on its next finite frame. Fail-safe reasoning: garbage in →
//! the next healthy source or a stop, NEVER garbage through to the robot (a
//! NaN would otherwise be classified "nonzero" and forwarded verbatim for
//! its whole freshness window).

// The pure module is unsafe-free by construction. (The forbid lives HERE,
// not at the crate root, because the `#[cerulion_node]` macro in lib.rs
// expands cdylib FFI entry points containing `unsafe`.)
#![forbid(unsafe_code)]

/// Joystick freshness window: a joystick command younger than this wins.
/// EXCLUSIVE at the boundary — see [`arbitrate`] (age == 250 ms is STALE).
pub const JOY_FRESH_NS: u64 = 250_000_000; // 250 ms

/// Keyboard freshness window: a keyboard command younger than this wins when no
/// fresh joystick is present. Larger than [`JOY_FRESH_NS`] to tolerate network
/// latency on a remote keyboard link. EXCLUSIVE at the boundary.
pub const KEY_FRESH_NS: u64 = 750_000_000; // 750 ms

/// Steady-state zero republish period: while the mux is emitting a *steady*
/// zero (not a fresh transition), it republishes the zero only once per this
/// period — a 5 Hz keepalive so a downstream watchdog keeps seeing liveness.
/// INCLUSIVE at the boundary — see [`arbitrate`].
pub const KEEPALIVE_PERIOD_NS: u64 = 200_000_000; // 200 ms → 5 Hz

/// A geometry_msgs/Twist-shaped velocity command (local POD). On the wire the
/// node maps this to/from `native_ros2_messages::geometry_msgs::Twist` (see
/// `lib.rs`); here it is just six `f64`s so the arbiter stays pure std.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct TwistCmd {
    /// Linear velocity `[x, y, z]` (m/s).
    pub linear: [f64; 3],
    /// Angular velocity `[x, y, z]` (rad/s).
    pub angular: [f64; 3],
}

impl TwistCmd {
    /// The all-zero command (robot stop). Every component exactly `0.0`.
    pub const ZERO: Self = Self {
        linear: [0.0, 0.0, 0.0],
        angular: [0.0, 0.0, 0.0],
    };

    /// True iff ALL SIX components are exactly `0.0`. A *centered* joystick
    /// (every axis at rest) is a real, deliberate zero command — distinct from
    /// "no fresh source" — and this predicate is what makes it publish on the
    /// keepalive cadence rather than every tick.
    ///
    /// The `== 0.0` is intentional and exact: a component is "zero" iff it is
    /// bit-`0.0` or `-0.0` (both compare equal to `0.0`); any other value —
    /// however small — is a genuine command. This is the ONE deliberate float
    /// equality in the arbiter, hence the local clippy allow.
    #[allow(clippy::float_cmp)]
    #[inline]
    pub fn is_zero(&self) -> bool {
        self.linear.iter().all(|&c| c == 0.0) && self.angular.iter().all(|&c| c == 0.0)
    }

    /// True iff ALL SIX components are finite (no NaN, no ±Inf). A command
    /// failing this is INVALID and its source is treated as absent for the
    /// arbitration — see the module docs' non-finite contract. (NaN also
    /// fails `is_zero` — NaN ≠ 0.0 — so without this gate a NaN frame would
    /// read as a "nonzero command" and be forwarded verbatim.)
    #[inline]
    pub fn is_finite(&self) -> bool {
        self.linear.iter().all(|c| c.is_finite()) && self.angular.iter().all(|c| c.is_finite())
    }
}

/// Which input source is currently driving the robot.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ActiveSource {
    /// Neither source is usable — the mux is emitting a safety zero.
    ///
    /// Deliberately the FIRST variant so the natural discriminant is 0:
    /// the wire contract (examples/go2/schemas/mux_state.yaml) requires
    /// 0 = none as the FAIL-SAFE value (a zero-initialized frame must
    /// read "nothing in control"), and matching the natural order makes
    /// a future `as u8` serialization structurally correct instead of
    /// comment-guarded.
    None,
    /// A usable (fresh AND finite) joystick command is in control.
    Joystick,
    /// No usable joystick; a usable keyboard command is in control.
    Keyboard,
}

/// The observable arbitration state (for telemetry / diagnostics).
///
/// `joy_stale` / `key_stale` mean "this source is NOT a fresh usable source
/// right now" — `true` when the source went too old, was NEVER delivered, OR
/// is emitting non-finite garbage (there is no separate flag per cause; for
/// control purposes all three are indistinguishable: the source cannot be
/// trusted to drive). So both-stale ⇔ [`ActiveSource::None`].
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct MuxState {
    /// The source currently in control.
    pub active: ActiveSource,
    /// The joystick is not a fresh usable source (too old, never delivered,
    /// or non-finite).
    pub joy_stale: bool,
    /// The keyboard is not a fresh usable source (too old, never delivered,
    /// or non-finite).
    pub key_stale: bool,
}

/// The full arbitration result for one tick.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct MuxDecision {
    /// The command to emit this tick (the winning source's command, or
    /// [`TwistCmd::ZERO`] when no source is fresh).
    pub cmd: TwistCmd,
    /// The observable arbitration state.
    pub state: MuxState,
    /// Whether to actually PUBLISH `cmd` this tick — see [`arbitrate`]'s
    /// publish policy (nonzero every tick; transition-to-zero immediately;
    /// steady zero on the keepalive cadence).
    pub publish: bool,
}

/// The command from `src` iff it is USABLE: delivered (`Some`), FRESH (age
/// `now_ns - ts`, SATURATING, strictly LESS than `fresh_ns`), AND FINITE
/// (every component — see [`TwistCmd::is_finite`] and the module docs'
/// non-finite contract). `None` for a never-delivered, too-old, or
/// non-finite source — all three degrade identically: the source is absent
/// for this arbitration.
///
/// The age uses `saturating_sub`, so a FUTURE-stamped frame (`ts > now_ns` —
/// clock skew, or a frame delivered just ahead of the local clock read) yields
/// age `0` → fresh. That is fail-SAFE: a future stamp is a genuinely
/// just-arrived frame, not a stale one, so age `0` is correct. Saturating age
/// can ONLY ever make a FUTURE frame look fresh — it can never make a genuinely
/// OLD frame (`ts < now_ns` by more than the window) look fresh, so it never
/// lets a dead input keep the robot moving (the danger the whole mux guards
/// against). The window comparison is strictly `<`, so the boundary is
/// EXCLUSIVE: `age == fresh_ns` is STALE.
#[inline]
fn usable_cmd(now_ns: u64, src: Option<(u64, TwistCmd)>, fresh_ns: u64) -> Option<TwistCmd> {
    match src {
        Some((ts, cmd)) if now_ns.saturating_sub(ts) < fresh_ns && cmd.is_finite() => Some(cmd),
        _ => None,
    }
}

/// Arbitrate the teleop sources for one tick. Pure — see the module docs.
///
/// # Arguments
///
/// - `now_ns`: the node's current clock (ns).
/// - `joy` / `key`: the last-held command for each source as
///   `(wire_timestamp_ns, cmd)`, or `None` if that source has NEVER delivered a
///   frame. `wire_timestamp_ns` is the publish stamp of the frame that
///   delivered `cmd` — for a HELD latest-value input this is the ORIGINAL
///   stamp, so `now_ns - wire_timestamp_ns` is its true age.
/// - `last_published_zero_ns`: the `now_ns` at which the mux last PUBLISHED a
///   steady-zero keepalive, or `None` if it never has. Drives the 5 Hz cadence.
/// - `prev_was_nonzero`: whether the command emitted on the PREVIOUS tick was
///   nonzero. A `nonzero → zero` transition publishes IMMEDIATELY (stop now),
///   independent of the keepalive window.
///
/// # Selection (freshness + validity)
///
/// Joystick wins when USABLE (fresh — age `< JOY_FRESH_NS` — and finite);
/// else keyboard when usable (age `< KEY_FRESH_NS`, finite); else
/// [`TwistCmd::ZERO`] + [`ActiveSource::None`]. A *usable* joystick MUTES
/// the keyboard even when the joystick command is itself zero (a centered
/// stick is a real command). A NON-FINITE command (any NaN/±Inf component)
/// makes its source absent for this arbitration — never forwarded (module
/// docs). Freshness boundary is EXCLUSIVE: `age == window` is STALE. See
/// the private `usable_cmd` helper for future-stamp handling.
///
/// # Publish policy
///
/// - Any NONZERO `cmd` → `publish = true` every tick.
/// - A `nonzero → zero` transition (`prev_was_nonzero` and `cmd` is zero) →
///   `publish = true` immediately, regardless of the keepalive window.
/// - A STEADY zero (`cmd` zero, `prev_was_nonzero == false`) → `publish = true`
///   only when `now_ns - last_published_zero_ns >= KEEPALIVE_PERIOD_NS` (the
///   keepalive boundary is INCLUSIVE), or when no zero has been published yet
///   (`last_published_zero_ns == None` → publish the first one).
pub fn arbitrate(
    now_ns: u64,
    joy: Option<(u64, TwistCmd)>,
    key: Option<(u64, TwistCmd)>,
    last_published_zero_ns: Option<u64>,
    prev_was_nonzero: bool,
) -> MuxDecision {
    let joy_usable_cmd = usable_cmd(now_ns, joy, JOY_FRESH_NS);
    let key_usable_cmd = usable_cmd(now_ns, key, KEY_FRESH_NS);

    // Selection: usable (fresh + finite) joystick first (mutes the keyboard
    // even when centered), then usable keyboard, else a safety zero.
    let (active, cmd) = if let Some(c) = joy_usable_cmd {
        (ActiveSource::Joystick, c)
    } else if let Some(c) = key_usable_cmd {
        (ActiveSource::Keyboard, c)
    } else {
        (ActiveSource::None, TwistCmd::ZERO)
    };

    let state = MuxState {
        active,
        joy_stale: joy_usable_cmd.is_none(),
        key_stale: key_usable_cmd.is_none(),
    };

    // Publish policy (see the doc). The two "publish now" cases share one
    // branch: a NONZERO command (publish every tick) OR a `nonzero → zero`
    // transition (`cmd` zero but `prev_was_nonzero` — stop RIGHT NOW). Only a
    // STEADY zero (`cmd` zero AND the previous tick was also zero) falls through
    // to the keepalive cadence.
    let publish = if !cmd.is_zero() || prev_was_nonzero {
        true
    } else {
        // Steady zero: republish only on the keepalive cadence (inclusive at
        // the boundary), and always emit the very first zero.
        match last_published_zero_ns {
            None => true,
            Some(last) => now_ns.saturating_sub(last) >= KEEPALIVE_PERIOD_NS,
        }
    };

    MuxDecision {
        cmd,
        state,
        publish,
    }
}

/// The node-state threading policy for one tick: given this tick's
/// [`MuxDecision`], the tick clock, and the PRIOR `last_published_zero_ns`,
/// returns the NEW `(last_published_zero_ns, prev_was_nonzero)` pair the
/// node must carry into the next tick.
///
/// - `last_published_zero_ns` advances to `Some(now_ns)` exactly when the
///   contract PUBLISHES a zero (`decision.publish && decision.cmd.is_zero()`
///   — the first zero, a transition zero, or a keepalive-cadence zero);
///   otherwise the prior value is carried unchanged (suppressed zeros and
///   nonzero ticks never advance the keepalive anchor).
/// - `prev_was_nonzero` is simply whether THIS tick's emitted command was
///   nonzero (feeding the next tick's immediate nonzero→zero transition).
///
/// This is the DORMANT contract seam: the node currently writes the
/// command to the wire every tick (no conditional-publish surface on the
/// macro path — see `lib.rs` "Platform gaps"), but threads this state
/// faithfully per the PURE policy, so honoring `decision.publish` later
/// needs no state migration. Extracted from the tick so the threading is
/// oracle-testable without a transport.
pub fn state_after(
    decision: MuxDecision,
    now_ns: u64,
    last_published_zero_ns: Option<u64>,
) -> (Option<u64>, bool) {
    let last_zero = if decision.publish && decision.cmd.is_zero() {
        Some(now_ns)
    } else {
        last_published_zero_ns
    };
    (last_zero, !decision.cmd.is_zero())
}

#[cfg(test)]
mod tests {
    use super::*;

    // Distinct nonzero commands so a "wrong source won" bug shows up as the
    // wrong VALUE, not just the wrong tag.
    const NZ_JOY: TwistCmd = TwistCmd {
        linear: [2.0, 0.0, 0.0],
        angular: [0.0, 0.0, 0.5],
    };
    const NZ_KEY: TwistCmd = TwistCmd {
        linear: [0.5, 0.0, 0.0],
        angular: [0.0, 0.0, 0.0],
    };

    // 1 — both fresh → joystick wins; the keyboard value is ignored.
    #[test]
    fn both_fresh_joystick_wins_keyboard_ignored() {
        let now = 1_000_000_000;
        let got = arbitrate(now, Some((now, NZ_JOY)), Some((now, NZ_KEY)), None, false);
        assert_eq!(
            got,
            MuxDecision {
                cmd: NZ_JOY,
                state: MuxState {
                    active: ActiveSource::Joystick,
                    joy_stale: false,
                    key_stale: false,
                },
                publish: true,
            }
        );
    }

    // 2 — joystick stale, keyboard fresh → keyboard wins.
    #[test]
    fn joy_stale_key_fresh_keyboard_wins() {
        let now = 1_000_000_000;
        let got = arbitrate(
            now,
            Some((now - 300_000_000, NZ_JOY)), // age 300 ms ≥ 250 → stale
            Some((now - 100_000_000, NZ_KEY)), // age 100 ms < 750 → fresh
            None,
            true,
        );
        assert_eq!(
            got,
            MuxDecision {
                cmd: NZ_KEY,
                state: MuxState {
                    active: ActiveSource::Keyboard,
                    joy_stale: true,
                    key_stale: false,
                },
                publish: true,
            }
        );
    }

    // 3 — both stale (after moving) → zero + None; the nonzero→zero transition
    // publishes immediately even though a zero was just published.
    #[test]
    fn both_stale_after_moving_zero_none_publishes_on_transition() {
        let now = 2_000_000_000;
        let got = arbitrate(
            now,
            Some((now - 300_000_000, NZ_JOY)), // stale
            Some((now - 800_000_000, NZ_KEY)), // age 800 ms ≥ 750 → stale
            Some(now - 1),                     // a zero published 1 ns ago (inside window)
            true,                              // came from moving
        );
        assert_eq!(
            got,
            MuxDecision {
                cmd: TwistCmd::ZERO,
                state: MuxState {
                    active: ActiveSource::None,
                    joy_stale: true,
                    key_stale: true,
                },
                publish: true, // transition overrides the keepalive window
            }
        );
    }

    // 4 — both never delivered → zero + None; the first zero publishes.
    #[test]
    fn both_none_zero_none_first_zero_publishes() {
        let now = 500_000_000;
        let got = arbitrate(now, None, None, None, false);
        assert_eq!(
            got,
            MuxDecision {
                cmd: TwistCmd::ZERO,
                state: MuxState {
                    active: ActiveSource::None,
                    joy_stale: true, // never-delivered reads as stale
                    key_stale: true,
                },
                publish: true, // last_published_zero == None → emit the first zero
            }
        );
    }

    // 5 — joystick freshness boundary is EXCLUSIVE at exactly 250 ms.
    #[test]
    fn joy_freshness_boundary_exclusive_at_250ms() {
        let now = 300_000_000;
        // age exactly 250 ms → STALE (no fresh source, key absent → None).
        let stale = arbitrate(now, Some((now - JOY_FRESH_NS, NZ_JOY)), None, None, true);
        assert_eq!(stale.state.active, ActiveSource::None);
        assert_eq!(stale.cmd, TwistCmd::ZERO);
        assert!(stale.state.joy_stale);
        // age 249_999_999 ns → FRESH → joystick.
        let fresh = arbitrate(
            now,
            Some((now - (JOY_FRESH_NS - 1), NZ_JOY)),
            None,
            None,
            true,
        );
        assert_eq!(fresh.state.active, ActiveSource::Joystick);
        assert_eq!(fresh.cmd, NZ_JOY);
        assert!(!fresh.state.joy_stale);
    }

    // 6 — keyboard freshness boundary is EXCLUSIVE at exactly 750 ms.
    #[test]
    fn key_freshness_boundary_exclusive_at_750ms() {
        let now = 1_000_000_000;
        // age exactly 750 ms → STALE → None (joystick absent).
        let stale = arbitrate(now, None, Some((now - KEY_FRESH_NS, NZ_KEY)), None, true);
        assert_eq!(stale.state.active, ActiveSource::None);
        assert_eq!(stale.cmd, TwistCmd::ZERO);
        assert!(stale.state.key_stale);
        // age 749_999_999 ns → FRESH → keyboard.
        let fresh = arbitrate(
            now,
            None,
            Some((now - (KEY_FRESH_NS - 1), NZ_KEY)),
            None,
            true,
        );
        assert_eq!(fresh.state.active, ActiveSource::Keyboard);
        assert_eq!(fresh.cmd, NZ_KEY);
        assert!(!fresh.state.key_stale);
    }

    // 7 — a nonzero→zero transition publishes immediately, even 1 ns after the
    // last zero; the same state with prev_was_nonzero == false suppresses (the
    // control proving the transition flag is what flips it).
    #[test]
    fn nonzero_to_zero_transition_publishes_immediately() {
        let now = 5_000_000_000;
        // Both dead → zero. A zero was published 1 ns ago (keepalive would say NO).
        let transition = arbitrate(now, None, None, Some(now - 1), true);
        assert!(
            transition.publish,
            "a nonzero→zero transition must publish immediately regardless of keepalive"
        );
        // Control: identical inputs but NOT a transition → keepalive suppresses.
        let steady = arbitrate(now, None, None, Some(now - 1), false);
        assert!(
            !steady.publish,
            "a steady zero 1 ns after the last publish must be suppressed (keepalive)"
        );
    }

    // 8 — a steady zero inside the keepalive window is suppressed.
    #[test]
    fn steady_zero_inside_keepalive_window_suppresses() {
        let now = 10_000_000_000;
        let got = arbitrate(now, None, None, Some(now - 100_000_000), false); // 100 ms ago < 200 ms
        assert_eq!(got.cmd, TwistCmd::ZERO);
        assert!(
            !got.publish,
            "a steady zero 100 ms after the last publish is inside the 200 ms window → suppress"
        );
    }

    // 9 — the keepalive boundary is INCLUSIVE at exactly 200 ms.
    #[test]
    fn steady_zero_keepalive_boundary_inclusive_at_200ms() {
        let now = 10_000_000_000;
        // exactly 200 ms since the last publish → PUBLISH (inclusive `>=`).
        let at = arbitrate(now, None, None, Some(now - KEEPALIVE_PERIOD_NS), false);
        assert!(
            at.publish,
            "a steady zero exactly 200 ms after the last publish must republish (inclusive)"
        );
        // 199_999_999 ns → just under → suppress.
        let under = arbitrate(
            now,
            None,
            None,
            Some(now - (KEEPALIVE_PERIOD_NS - 1)),
            false,
        );
        assert!(
            !under.publish,
            "a steady zero 199_999_999 ns after the last publish is still inside the window"
        );
    }

    // 10 — a future-stamped joystick frame (ts > now) counts as fresh (age 0).
    #[test]
    fn future_stamped_joystick_counts_as_fresh() {
        let now = 1_000_000_000;
        let got = arbitrate(now, Some((now + 50_000_000, NZ_JOY)), None, None, false);
        assert_eq!(
            got,
            MuxDecision {
                cmd: NZ_JOY,
                state: MuxState {
                    active: ActiveSource::Joystick,
                    joy_stale: false,
                    key_stale: true,
                },
                publish: true,
            },
            "a future-stamped frame is a just-arrived frame → age 0 → fresh"
        );
    }

    // 11 — a FRESH but CENTERED (zero) joystick MUTES a fresh nonzero keyboard,
    // emitting the joystick's zero. The sharpest arbitration case.
    #[test]
    fn centered_joystick_mutes_keyboard_with_zero_output() {
        let now = 1_000_000_000;
        let got = arbitrate(
            now,
            Some((now, TwistCmd::ZERO)), // centered stick, FRESH
            Some((now, NZ_KEY)),         // fresh nonzero keyboard
            None,
            true, // came from moving → transition publishes the stop
        );
        assert_eq!(
            got,
            MuxDecision {
                cmd: TwistCmd::ZERO, // the joystick's zero, NOT the keyboard's nonzero
                state: MuxState {
                    active: ActiveSource::Joystick, // keyboard is MUTED
                    joy_stale: false,
                    key_stale: false,
                },
                publish: true,
            },
            "a fresh centered joystick is a real command that mutes the keyboard"
        );
    }

    // 12 — "zero" requires ALL SIX components to be exactly 0.0: a single
    // nonzero component (angular.z) is a real command → publishes every tick.
    #[test]
    fn is_zero_requires_all_six_components_zero() {
        let one_axis = TwistCmd {
            linear: [0.0, 0.0, 0.0],
            angular: [0.0, 0.0, 0.1], // only yaw
        };
        assert!(
            !one_axis.is_zero(),
            "a command with any nonzero component must NOT be treated as zero"
        );
        assert!(TwistCmd::ZERO.is_zero(), "the all-zero command is zero");

        // A fresh joystick emitting only-yaw is nonzero → publishes every tick,
        // even though a zero was published 1 ns ago (keepalive is irrelevant to
        // a nonzero command).
        let now = 1_000_000_000;
        let got = arbitrate(now, Some((now, one_axis)), None, Some(now - 1), false);
        assert_eq!(got.cmd, one_axis);
        assert!(
            got.publish,
            "a nonzero command publishes every tick regardless of the keepalive window"
        );
    }

    // 13 — ADVERSARIAL input-death sweep: both sources delivered once at t=0
    // then go silent. As `now` advances, control degrades joystick → keyboard
    // → zero at the exact freshness boundaries (the acceptance sequence: joy
    // dies at 250 ms, keyboard carries to 750 ms, then the robot is zeroed).
    #[test]
    fn input_death_sweep_joystick_then_keyboard_then_zero() {
        let joy = Some((0, NZ_JOY));
        let key = Some((0, NZ_KEY));
        // HAND ORACLE: (now_ns, expected active, expected cmd).
        let oracle: &[(u64, ActiveSource, TwistCmd)] = &[
            (100_000_000, ActiveSource::Joystick, NZ_JOY), // both fresh → joy
            (249_999_999, ActiveSource::Joystick, NZ_JOY), // joy still fresh (just under 250 ms)
            (250_000_000, ActiveSource::Keyboard, NZ_KEY), // joy just died → keyboard
            (500_000_000, ActiveSource::Keyboard, NZ_KEY), // keyboard carries
            (749_999_999, ActiveSource::Keyboard, NZ_KEY), // keyboard still fresh (just under 750 ms)
            (750_000_000, ActiveSource::None, TwistCmd::ZERO), // keyboard just died → zero
            (1_000_000_000, ActiveSource::None, TwistCmd::ZERO), // stays zero
        ];
        for &(now, want_active, want_cmd) in oracle {
            let got = arbitrate(now, joy, key, None, true);
            assert_eq!(
                got.state.active, want_active,
                "at now={now} the active source must be {want_active:?} (got {:?})",
                got.state.active
            );
            assert_eq!(
                got.cmd, want_cmd,
                "at now={now} the emitted command must be {want_cmd:?} (got {:?})",
                got.cmd
            );
        }
    }

    // ---- Non-finite invalidation: the mux never
    // forwards NaN/±Inf — a garbage source is ABSENT for the arbitration.

    /// A joystick command with a NaN component (fresh by stamp).
    const NAN_JOY: TwistCmd = TwistCmd {
        linear: [f64::NAN, 0.0, 0.0],
        angular: [0.0, 0.0, 0.5],
    };

    // 14 — a FRESH NaN joystick is skipped: the fresh keyboard wins (garbage
    // in → next source, never garbage through).
    #[test]
    fn nan_joystick_fresh_is_skipped_keyboard_wins() {
        let now = 1_000_000_000;
        let got = arbitrate(now, Some((now, NAN_JOY)), Some((now, NZ_KEY)), None, false);
        assert_eq!(
            got,
            MuxDecision {
                cmd: NZ_KEY, // the keyboard's finite command, NOT the NaN
                state: MuxState {
                    active: ActiveSource::Keyboard,
                    joy_stale: true, // non-finite reads as not-usable
                    key_stale: false,
                },
                publish: true,
            },
            "a fresh-by-stamp NaN joystick must be treated as absent — the \
             keyboard drives and no NaN reaches the wire"
        );
    }

    // 15 — a SINGLE ±Inf component invalidates the whole command (here
    // angular.z only; every other component finite and zero).
    #[test]
    fn single_infinite_component_invalidates_command() {
        let inf_yaw = TwistCmd {
            linear: [0.0, 0.0, 0.0],
            angular: [0.0, 0.0, f64::INFINITY],
        };
        let neg_inf_yaw = TwistCmd {
            linear: [0.0, 0.0, 0.0],
            angular: [0.0, 0.0, f64::NEG_INFINITY],
        };
        assert!(!inf_yaw.is_finite(), "+Inf in one component is non-finite");
        assert!(
            !neg_inf_yaw.is_finite(),
            "-Inf in one component is non-finite"
        );

        // Keyboard absent → the skipped Inf joystick degrades to zero + None
        // (an Inf yaw forwarded verbatim would spin the robot).
        let now = 1_000_000_000;
        for bad in [inf_yaw, neg_inf_yaw] {
            let got = arbitrate(now, Some((now, bad)), None, None, true);
            assert_eq!(
                got.cmd,
                TwistCmd::ZERO,
                "an Inf-component command must never be forwarded (got {:?})",
                got.cmd
            );
            assert_eq!(got.state.active, ActiveSource::None);
            assert!(got.state.joy_stale, "non-finite joystick reads stale");
        }
    }

    // 16 — BOTH sources non-finite → safety zero + None (both stale).
    #[test]
    fn both_non_finite_degrade_to_zero_none() {
        let now = 1_000_000_000;
        let nan_key = TwistCmd {
            linear: [0.0, f64::NAN, 0.0],
            angular: [0.0, 0.0, 0.0],
        };
        let got = arbitrate(now, Some((now, NAN_JOY)), Some((now, nan_key)), None, true);
        assert_eq!(
            got,
            MuxDecision {
                cmd: TwistCmd::ZERO,
                state: MuxState {
                    active: ActiveSource::None,
                    joy_stale: true,
                    key_stale: true,
                },
                publish: true, // came from moving → the transition zero publishes
            },
            "both sources emitting garbage must land the robot at the safety zero"
        );
    }

    // 17 — -0.0 in every component stays ZERO-class: is_zero() true (pinned
    // explicitly), is_finite() true, and a fresh all-neg-zero joystick still
    // MUTES the keyboard with a zero output (it is a real centered stick).
    #[test]
    fn negative_zero_components_stay_zero_class() {
        let neg_zero = TwistCmd {
            linear: [-0.0, -0.0, -0.0],
            angular: [-0.0, -0.0, -0.0],
        };
        assert!(
            neg_zero.is_zero(),
            "-0.0 compares equal to 0.0 — an all-neg-zero command IS the zero command"
        );
        assert!(neg_zero.is_finite(), "-0.0 is finite");

        let now = 1_000_000_000;
        let got = arbitrate(now, Some((now, neg_zero)), Some((now, NZ_KEY)), None, true);
        assert_eq!(got.state.active, ActiveSource::Joystick);
        assert!(
            got.cmd.is_zero(),
            "the muted output is the joystick's (negative) zero, not the keyboard's 0.5"
        );
    }

    // 18 — RECOVERY: a NaN frame loses the arbitration, the source's next
    // finite frame wins it back (the pure fn is stateless — invalidation
    // never latches).
    #[test]
    fn source_recovers_on_next_finite_frame() {
        let now = 1_000_000_000;
        // Garbage frame → keyboard drives.
        let during = arbitrate(now, Some((now, NAN_JOY)), Some((now, NZ_KEY)), None, false);
        assert_eq!(during.state.active, ActiveSource::Keyboard);
        // The joystick's NEXT frame is finite (fresh stamp) → joystick wins
        // again immediately.
        let after = arbitrate(
            now + 20_000_000,
            Some((now + 20_000_000, NZ_JOY)),
            Some((now, NZ_KEY)),
            None,
            false,
        );
        assert_eq!(
            after,
            MuxDecision {
                cmd: NZ_JOY,
                state: MuxState {
                    active: ActiveSource::Joystick,
                    joy_stale: false,
                    key_stale: false,
                },
                publish: true,
            },
            "a recovered (finite) joystick frame must win the arbitration back"
        );
    }

    // ---- state_after: the node-state threading policy,
    // oracle-tested without a transport (the dormant contract).

    // 19 — the three per-tick threading arms, each a hand-computed pair.
    #[test]
    fn state_after_threads_each_decision_arm() {
        let now = 1_000_000_000;

        // (a) a PUBLISHED zero advances the keepalive anchor to now.
        let published_zero = arbitrate(now, None, None, None, false);
        assert!(published_zero.publish && published_zero.cmd.is_zero());
        assert_eq!(
            state_after(published_zero, now, None),
            (Some(now), false),
            "a published zero must stamp the anchor and clear prev_was_nonzero"
        );

        // (b) a SUPPRESSED zero (inside the keepalive window) carries the
        // prior anchor unchanged.
        let prior = Some(now - 1);
        let suppressed = arbitrate(now, None, None, prior, false);
        assert!(!suppressed.publish && suppressed.cmd.is_zero());
        assert_eq!(
            state_after(suppressed, now, prior),
            (prior, false),
            "a suppressed zero must NOT advance the keepalive anchor"
        );

        // (c) a NONZERO command leaves the anchor alone and sets
        // prev_was_nonzero (feeding the next tick's transition publish).
        let nonzero = arbitrate(now, Some((now, NZ_JOY)), None, prior, false);
        assert!(nonzero.publish && !nonzero.cmd.is_zero());
        assert_eq!(
            state_after(nonzero, now, prior),
            (prior, true),
            "a nonzero tick must keep the anchor and set prev_was_nonzero"
        );
    }

    // 20 — closed-loop scripted sequence: driving arbitrate + state_after
    // exactly as the node's tick does (both sources absent, 20 ms ticks)
    // reproduces the 5 Hz keepalive cadence against a HAND oracle of
    // (publish, anchor) pairs. This is the coverage for the tick's state
    // threading, transport-free.
    #[test]
    fn state_threading_closed_loop_reproduces_keepalive_cadence() {
        const STEP_NS: u64 = 20_000_000; // the node's 20 ms period
        let mut last_zero: Option<u64> = None;
        let mut prev_nonzero = false;

        // HAND ORACLE per tick k (now = k*20 ms, k = 1..=23):
        //  k=1  (now 20 ms):  first zero, no anchor → publish, anchor 20 ms.
        //  k=2..=10 (now 40..=200 ms): now - 20 ms < 200 ms → suppressed.
        //  k=11 (now 220 ms): 220-20 = 200 ≥ 200 (inclusive) → publish,
        //       anchor 220 ms.
        //  k=12..=20 (now 240..=400 ms): suppressed.
        //  k=21 (now 420 ms): 420-220 = 200 → publish, anchor 420 ms.
        //  k=22..=23: suppressed. Publishes land at k = 1, 11, 21 — every
        //       10 ticks = 200 ms = exactly 5 Hz.
        let mut got: Vec<(bool, Option<u64>)> = Vec::new();
        for k in 1..=23u64 {
            let now = k * STEP_NS;
            let d = arbitrate(now, None, None, last_zero, prev_nonzero);
            let (lz, pn) = state_after(d, now, last_zero);
            last_zero = lz;
            prev_nonzero = pn;
            got.push((d.publish, last_zero));
        }

        let mut want: Vec<(bool, Option<u64>)> = Vec::new();
        for k in 1..=23u64 {
            let (publish, anchor) = match k {
                1 => (true, 1),
                2..=10 => (false, 1),
                11 => (true, 11),
                12..=20 => (false, 11),
                21 => (true, 21),
                _ => (false, 21),
            };
            want.push((publish, Some(anchor * STEP_NS)));
        }
        assert_eq!(
            got, want,
            "the closed arbitrate+state_after loop must reproduce the 5 Hz \
             keepalive cadence (publish every 10th 20 ms tick)"
        );
    }
}
