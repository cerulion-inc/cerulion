// SPDX-License-Identifier: AGPL-3.0-only
//! The PURE sport-request policy: a velocity command becomes exactly one
//! `unitree_api/Request`, or none. No transport, no DDS, no clock reads;
//! every decision is a function of (node clock, command, prior state), which
//! is what makes the driver replay bit-identical (Principle 7) and lets the
//! oracle-vector tests below pin the contract without a robot.
//!
//! # The contract
//!
//! | command | previous command | request sent |
//! |---|---|---|
//! | nonzero (any component) | any | `Move` (api 1008) with the clamped `{x, y, z}` JSON, EVERY tick |
//! | all zero | nonzero | `StopMove` (api 1003) immediately (the transition) |
//! | all zero | zero, last stop >= 200 ms ago | `StopMove` again (the 5 Hz keepalive) |
//! | all zero | zero, last stop < 200 ms ago | nothing |
//! | any non-finite component | any | treated as all zero and COUNTED (the mux never forwards NaN, this is defense in depth) |
//!
//! `Move` is re-sent on every tick because the sport API treats a velocity
//! request as an intent for the near future, not a latched setpoint; the
//! shipped teleop stack feeds this node at 50 Hz from the safety mux. The
//! StopMove keepalive mirrors the mux's own 5 Hz zero keepalive: an idle
//! robot keeps hearing "stopped" from a live driver, and a driver that dies
//! goes silent, which the mux's staleness gate above it already handles.
//!
//! Velocities are CLAMPED to the shipped teleop saturation limits (the
//! joystick crate's canonical constants, pinned in lockstep by
//! `tests/const_consistency_test.rs`). They are well under the sport
//! API's own maxima on purpose: this example's first actuation happens on a
//! stand, and a wrong sign at 0.6 m/s is recoverable.
//!
//! # What this module does NOT do
//!
//! It never issues the posture commands (Damp 1001, StandUp 1004, StandDown
//! 1005, RecoveryStand 1006): standing the robot up is a deliberate, watched
//! step the operator does with the vendor remote before teleop, and a node
//! that could stand a robot up on a stray frame would be the wrong shape.

#![forbid(unsafe_code)]

use cerulion_go2_dds::messages::{
    Request, RequestHeader, RequestIdentity, RequestLease, RequestPolicy,
};

/// Unitree sport-mode API id for `Move` (velocity in the JSON parameter).
/// Re-exported from the DDS support lib so the two cannot drift.
pub const SPORT_API_ID_MOVE: i64 = cerulion_go2_dds::messages::SPORT_API_ID_MOVE;

/// Unitree sport-mode API id for `StopMove` (zero motion, still standing).
pub const SPORT_API_ID_STOP_MOVE: i64 = 1003;

/// How often an idle (all-zero) command stream re-sends `StopMove`: 200 ms,
/// the mux's own zero-keepalive cadence (5 Hz).
pub const STOP_KEEPALIVE_PERIOD_NS: u64 = 200_000_000;

/// Saturation limits the driver clamps to, in m/s and rad/s. DUPLICATES of
/// the joystick crate's canonical constants (pinned in lockstep by
/// `tests/const_consistency_test.rs`), duplicated so this crate carries no
/// gilrs edge.
pub const MAX_VX: f64 = 0.6;
pub const MAX_VY: f64 = 0.4;
pub const MAX_VYAW: f64 = 1.0;

/// A velocity command: forward, left, and yaw-left rates.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Velocity {
    pub vx: f64,
    pub vy: f64,
    pub vyaw: f64,
}

impl Velocity {
    /// The all-zero command.
    pub const ZERO: Self = Self {
        vx: 0.0,
        vy: 0.0,
        vyaw: 0.0,
    };

    /// True when every component is finite (the only commands the driver
    /// forwards as motion).
    pub fn is_finite(self) -> bool {
        self.vx.is_finite() && self.vy.is_finite() && self.vyaw.is_finite()
    }

    /// True when any component is nonzero.
    pub fn is_nonzero(self) -> bool {
        self.vx != 0.0 || self.vy != 0.0 || self.vyaw != 0.0
    }

    /// Clamp each component to the driver's saturation limit and normalize a
    /// negative zero to a plain zero (so the JSON never carries `-0`).
    pub fn clamped(self) -> Self {
        fn clamp(v: f64, limit: f64) -> f64 {
            let c = v.clamp(-limit, limit);
            if c == 0.0 {
                0.0
            } else {
                c
            }
        }
        Self {
            vx: clamp(self.vx, MAX_VX),
            vy: clamp(self.vy, MAX_VY),
            vyaw: clamp(self.vyaw, MAX_VYAW),
        }
    }
}

/// One sport request the driver sends.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum SportCommand {
    /// api 1008 with the (already clamped) velocity as the JSON parameter.
    Move(Velocity),
    /// api 1003, empty parameter.
    StopMove,
}

impl SportCommand {
    /// The sport API id this command carries.
    pub const fn api_id(self) -> i64 {
        match self {
            Self::Move(_) => SPORT_API_ID_MOVE,
            Self::StopMove => SPORT_API_ID_STOP_MOVE,
        }
    }

    /// The request's `parameter` string: the `{"x":..,"y":..,"z":..}` JSON
    /// the vendor SDK sends for `Move`, empty for `StopMove`. Floats render
    /// through Rust's shortest round-trip `Display` (never exponent form, a
    /// whole number renders without a fraction), which the robot's JSON
    /// parser accepts for a double.
    pub fn parameter(self) -> String {
        match self {
            Self::Move(v) => format!("{{\"x\":{},\"y\":{},\"z\":{}}}", v.vx, v.vy, v.vyaw),
            Self::StopMove => String::new(),
        }
    }
}

/// The driver's carried state: what the previous command was, when the
/// last `StopMove` went out, and the next request identity.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct DriverState {
    /// Whether the previously decided command was a `Move`.
    pub prev_was_move: bool,
    /// Node-clock time of the last `StopMove` sent, if any.
    pub last_stop_sent_ns: Option<u64>,
    /// The identity every request carries; increments once per request SENT.
    pub next_request_id: i64,
}

impl Default for DriverState {
    fn default() -> Self {
        Self {
            prev_was_move: false,
            last_stop_sent_ns: None,
            next_request_id: 1,
        }
    }
}

/// The outcome of one tick's decision.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Decision {
    /// The request to send this tick, if any.
    pub command: Option<SportCommand>,
    /// The state to carry into the next tick.
    pub state: DriverState,
    /// True when the input carried a non-finite component and was treated
    /// as all zero.
    pub rejected_non_finite: bool,
}

/// Decide what one tick sends. Pure: see the module docs for the contract.
pub fn decide(now_ns: u64, cmd: Velocity, state: DriverState) -> Decision {
    let rejected_non_finite = !cmd.is_finite();
    let cmd = if rejected_non_finite {
        Velocity::ZERO
    } else {
        cmd.clamped()
    };

    if cmd.is_nonzero() {
        return Decision {
            command: Some(SportCommand::Move(cmd)),
            state: DriverState {
                prev_was_move: true,
                next_request_id: state.next_request_id + 1,
                ..state
            },
            rejected_non_finite,
        };
    }

    let keepalive_due = match state.last_stop_sent_ns {
        None => true,
        Some(last) => now_ns.saturating_sub(last) >= STOP_KEEPALIVE_PERIOD_NS,
    };
    if state.prev_was_move || keepalive_due {
        Decision {
            command: Some(SportCommand::StopMove),
            state: DriverState {
                prev_was_move: false,
                last_stop_sent_ns: Some(now_ns),
                next_request_id: state.next_request_id + 1,
            },
            rejected_non_finite,
        }
    } else {
        Decision {
            command: None,
            state,
            rejected_non_finite,
        }
    }
}

/// Build the wire struct for one command. `id` is the request identity
/// ([`DriverState::next_request_id`] BEFORE the decision incremented it).
/// Lease 0, priority 0, a reply requested, no binary payload: the vendor
/// SDK's shape for a sport request.
pub fn build_request(id: i64, command: SportCommand) -> Request {
    Request {
        header: RequestHeader {
            identity: RequestIdentity {
                id,
                api_id: command.api_id(),
            },
            lease: RequestLease { id: 0 },
            policy: RequestPolicy {
                priority: 0,
                noreply: false,
            },
        },
        parameter: command.parameter(),
        binary: Vec::new(),
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use cerulion_go2_dds::cdr::{decode_request, encode_request};

    const MS: u64 = 1_000_000;

    fn v(vx: f64, vy: f64, vyaw: f64) -> Velocity {
        Velocity { vx, vy, vyaw }
    }

    #[test]
    fn a_nonzero_command_moves_every_tick_and_burns_one_id_each() {
        let mut state = DriverState::default();
        for tick in 0..3u64 {
            let d = decide(tick * 20 * MS, v(0.5, 0.0, 0.25), state);
            assert_eq!(d.command, Some(SportCommand::Move(v(0.5, 0.0, 0.25))));
            assert!(!d.rejected_non_finite);
            assert!(d.state.prev_was_move);
            assert_eq!(d.state.next_request_id, state.next_request_id + 1);
            state = d.state;
        }
        assert_eq!(state.next_request_id, 4);
    }

    #[test]
    fn a_zero_after_motion_stops_immediately_then_keeps_alive_at_five_hz() {
        // Hand oracle over 12 beats of 20 ms: one Move, then eleven zeros.
        // Beat 1 stops on the transition (t = 20 ms); the next stop is due
        // when 200 ms have elapsed since it, i.e. at t = 220 ms (beat 11).
        let mut state = DriverState::default();
        let mut sent: Vec<(u64, SportCommand)> = Vec::new();
        for beat in 0..12u64 {
            let now = beat * 20 * MS;
            let cmd = if beat == 0 {
                v(0.3, 0.0, 0.0)
            } else {
                Velocity::ZERO
            };
            let d = decide(now, cmd, state);
            if let Some(c) = d.command {
                sent.push((now, c));
            }
            state = d.state;
        }
        assert_eq!(
            sent,
            vec![
                (0, SportCommand::Move(v(0.3, 0.0, 0.0))),
                (20 * MS, SportCommand::StopMove),
                (220 * MS, SportCommand::StopMove),
            ]
        );
        // Three requests were sent, so three identities were burned.
        assert_eq!(state.next_request_id, 4);
        assert!(!state.prev_was_move);
        assert_eq!(state.last_stop_sent_ns, Some(220 * MS));
    }

    #[test]
    fn the_first_ever_zero_sends_one_stop_then_waits_for_the_keepalive() {
        let d0 = decide(0, Velocity::ZERO, DriverState::default());
        assert_eq!(d0.command, Some(SportCommand::StopMove));
        let d1 = decide(20 * MS, Velocity::ZERO, d0.state);
        assert_eq!(d1.command, None);
        assert_eq!(d1.state, d0.state, "a silent tick carries state unchanged");
        // Exactly at the boundary the keepalive fires (inclusive).
        let d2 = decide(STOP_KEEPALIVE_PERIOD_NS, Velocity::ZERO, d1.state);
        assert_eq!(d2.command, Some(SportCommand::StopMove));
        // One tick short of it, nothing.
        let d3 = decide(STOP_KEEPALIVE_PERIOD_NS - 1, Velocity::ZERO, d0.state);
        assert_eq!(d3.command, None);
    }

    #[test]
    fn velocities_are_clamped_to_the_teleop_limits_and_negative_zero_is_normalized() {
        let d = decide(0, v(5.0, -3.0, 9.0), DriverState::default());
        assert_eq!(
            d.command,
            Some(SportCommand::Move(v(MAX_VX, -MAX_VY, MAX_VYAW)))
        );
        let c = v(-0.0, 0.2, -0.0).clamped();
        assert!(c.vx.is_sign_positive() && c.vyaw.is_sign_positive());
        assert_eq!(
            SportCommand::Move(c).parameter(),
            "{\"x\":0,\"y\":0.2,\"z\":0}"
        );
    }

    #[test]
    fn a_non_finite_command_is_treated_as_zero_and_counted() {
        let moving = decide(0, v(0.4, 0.0, 0.0), DriverState::default());
        for bad in [
            v(f64::NAN, 0.0, 0.0),
            v(0.0, f64::INFINITY, 0.0),
            v(0.0, 0.0, f64::NEG_INFINITY),
        ] {
            let d = decide(20 * MS, bad, moving.state);
            assert!(d.rejected_non_finite);
            // It behaves exactly as an all-zero after motion: a stop.
            assert_eq!(d.command, Some(SportCommand::StopMove));
        }
        // A finite command is never flagged.
        assert!(!decide(0, v(0.1, 0.1, 0.1), DriverState::default()).rejected_non_finite);
    }

    #[test]
    fn the_json_parameter_matches_the_vendor_shape_exactly() {
        assert_eq!(
            SportCommand::Move(v(0.5, 0.0, -0.25)).parameter(),
            "{\"x\":0.5,\"y\":0,\"z\":-0.25}"
        );
        assert_eq!(
            SportCommand::Move(v(0.6, -0.4, 1.0)).parameter(),
            "{\"x\":0.6,\"y\":-0.4,\"z\":1}"
        );
        assert_eq!(SportCommand::StopMove.parameter(), "");
        assert_eq!(SportCommand::Move(Velocity::ZERO).api_id(), 1008);
        assert_eq!(SportCommand::StopMove.api_id(), 1003);
    }

    #[test]
    fn build_request_round_trips_through_the_shipped_cdr_codec() {
        let req = build_request(7, SportCommand::Move(v(0.1, 0.0, 0.0)));
        assert_eq!(req.header.identity.id, 7);
        assert_eq!(req.header.identity.api_id, SPORT_API_ID_MOVE);
        assert_eq!(req.header.lease.id, 0);
        assert_eq!(req.header.policy.priority, 0);
        assert!(!req.header.policy.noreply);
        assert_eq!(req.parameter, "{\"x\":0.1,\"y\":0,\"z\":0}");
        assert!(req.binary.is_empty());
        // The struct the writer serializes is exactly what the shipped codec
        // encodes and decodes, byte for byte.
        let bytes = encode_request(&req).expect("encode");
        assert_eq!(decode_request(&bytes).expect("decode"), req);

        let stop = build_request(8, SportCommand::StopMove);
        assert_eq!(stop.header.identity.api_id, SPORT_API_ID_STOP_MOVE);
        assert_eq!(stop.parameter, "");
        let bytes = encode_request(&stop).expect("encode");
        assert_eq!(decode_request(&bytes).expect("decode"), stop);
    }

    #[test]
    fn the_policy_is_deterministic_across_two_runs() {
        let script = [
            v(0.2, 0.0, 0.0),
            v(0.2, 0.1, 0.0),
            Velocity::ZERO,
            Velocity::ZERO,
            v(f64::NAN, 0.0, 0.0),
            v(0.0, 0.0, 0.5),
            Velocity::ZERO,
        ];
        let run = || {
            let mut state = DriverState::default();
            let mut out = Vec::new();
            for (i, cmd) in script.iter().enumerate() {
                let d = decide(i as u64 * 20 * MS, *cmd, state);
                out.push((d.command, d.state, d.rejected_non_finite));
                state = d.state;
            }
            out
        };
        let a = run();
        let b = run();
        assert_eq!(a, b);
        // And equal to the hand oracle of what was sent.
        let sent: Vec<Option<SportCommand>> = a.iter().map(|(c, _, _)| *c).collect();
        assert_eq!(
            sent,
            vec![
                Some(SportCommand::Move(v(0.2, 0.0, 0.0))),
                Some(SportCommand::Move(v(0.2, 0.1, 0.0))),
                Some(SportCommand::StopMove),
                None,
                // The NaN frame lands 40 ms after that stop: treated as a
                // zero, and the keepalive is not due, so nothing is sent.
                None,
                Some(SportCommand::Move(v(0.0, 0.0, 0.5))),
                Some(SportCommand::StopMove),
            ]
        );
    }
}
