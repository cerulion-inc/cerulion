// SPDX-License-Identifier: AGPL-3.0-only
//! Pure per-iteration DECISION policy for the joystick hardware pump.
//!
//! The gilrs helper thread ([`crate::pump`]) is a thin I/O shell: each closure
//! call it (a) performs exactly one blocking observation (gilrs init at
//! startup, then one bounded `next_event_blocking`), (b) translates it into a
//! [`PumpInput`], and (c) asks [`PumpState::step`] what to DO. Everything that
//! decides behavior — connected seeding, ring cadence, keepalive, the liveness
//! watchdog, disconnect handling — lives HERE, pure and std-only, so the
//! safety behaviors are pinned by the oracle tests below instead of
//! living untestably inside a hardware thread.
//!
//! # The decisions this module owns
//!
//! - **Startup**: rings exactly once, and seeds `connected`
//!   from the shell's device enumeration (`gilrs.gamepads().next().is_some()`)
//!   — the vendored gilrs enumerates a pad that was connected BEFORE launch
//!   WITHOUT emitting `EventType::Connected`, so waiting for the event would
//!   leave the 20 Hz deadman keepalive dead for pre-connected pads.
//! - **Event evidence refresh**: ANY gilrs event (mapped or not) is evidence
//!   of a live pad link — it refreshes the liveness window AND sets
//!   `connected` (belt-and-suspenders over the startup seed; a `Disconnected`
//!   fold immediately overrides it back to false).
//! - **Keepalive**: while the deadman is held and a pad is
//!   connected, ring at least every [`KEEPALIVE_NS`] (~20 Hz). Evaluated on
//!   EVERY iteration — including unmapped-event iterations — so a pad
//!   streaming gyro/trigger jitter can never starve the keepalive (a check
//!   made only on the poll-timeout arm would let that stream starve it).
//! - **Liveness watchdog** (defense-in-depth): while the
//!   deadman latch is held, if NO gilrs event of ANY kind arrives within
//!   [`LIVENESS_TIMEOUT_NS`] (inclusive boundary), the link is SUSPECT — a
//!   BT-stalled radio would otherwise keep republishing the last non-zero
//!   command at 20 Hz, defeating the mux's 250 ms staleness gate until the
//!   kernel's BT supervision timeout finally emits `Disconnected` seconds
//!   later. The policy pushes one `Deadman(false)` (⇒ the node publishes a
//!   zero and disarms — fresh RB evidence is required to re-arm) and reports
//!   [`PumpWarn::LivenessExpired`] for the shell to log loudly. Trade-off,
//!   documented in BRINGUP.md: modern pads stream sensor/axis events
//!   continuously, so a healthy link never trips this; a genuinely silent,
//!   perfectly-still pad with RB held > 2 s trips a safe zero (acceptable —
//!   release and re-press RB to re-arm).

// The pure module is unsafe-free by construction (the crate root can't carry
// the forbid — the `#[cerulion_node]` macro in lib.rs expands FFI `unsafe`).
#![forbid(unsafe_code)]

use crate::mapping::PadEvent;

/// Deadman keepalive period: while RB is held (and a pad is connected) the
/// pump rings at least this often (~20 Hz) so the downstream mux keeps seeing
/// a fresh command. INCLUSIVE at the boundary (`elapsed >= KEEPALIVE_NS`
/// rings).
pub const KEEPALIVE_NS: u64 = 50_000_000; // 50 ms → 20 Hz

/// Liveness watchdog window: with the deadman latched, a
/// gap of this long with NO gilrs event of ANY kind marks the link suspect —
/// one zero is published and the deadman latch is cleared. INCLUSIVE at the
/// boundary. See the module docs for the documented trade-off.
pub const LIVENESS_TIMEOUT_NS: u64 = 2_000_000_000; // 2 s

/// One observation by the pump shell, handed to [`PumpState::step`].
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum PumpInput {
    /// The first closure call. `device_present` is the shell's gilrs
    /// enumeration result (`gamepads().next().is_some()`; `false` when gilrs
    /// init failed entirely) — the pre-connected seed.
    Startup {
        /// Whether a gamepad was already enumerated at init.
        device_present: bool,
    },
    /// A gilrs event arrived. `mapped` is its [`PadEvent`] translation, or
    /// `None` for an event the mapping ignores (gyro, analog triggers,
    /// unmapped buttons, ...). BOTH kinds count as link evidence.
    Event {
        /// The translated pad event, if the mapping recognises it.
        mapped: Option<PadEvent>,
    },
    /// The bounded poll timed out with no event.
    Timeout,
}

/// A loud condition the shell must log (the policy itself never logs — it is
/// pure and tracing-free).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PumpWarn {
    /// The liveness watchdog tripped: deadman latched but no gilrs event of
    /// any kind within [`LIVENESS_TIMEOUT_NS`]. The decision carrying this
    /// also pushes `Deadman(false)` and rings (one zero publish + disarm).
    LivenessExpired,
}

/// What the shell must DO for one iteration.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct PumpDecision {
    /// Push this event into the node's inbox BEFORE ringing.
    pub push: Option<PadEvent>,
    /// Ring the doorbell (⇒ the node ticks ⇒ publishes).
    pub ring: bool,
    /// Log this loudly (tracing lives in the shell).
    pub warn: Option<PumpWarn>,
}

impl PumpDecision {
    /// The do-nothing decision (no push, no ring, no warn).
    const QUIET: Self = Self {
        push: None,
        ring: false,
        warn: None,
    };
}

/// The pump's folded control state + the per-iteration decision function.
/// All-default start (`false`/`0`) IS the correct fresh state: un-started,
/// disarmed, disconnected-until-seeded, both anchors at 0 (re-anchored by
/// the mandatory `Startup` input before anything can compare against them).
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct PumpState {
    started: bool,
    deadman: bool,
    connected: bool,
    /// Time of the last RING (any cause) — the keepalive anchor.
    last_ring_ns: u64,
    /// Time of the last gilrs event of ANY kind — the liveness anchor.
    last_event_ns: u64,
}

impl PumpState {
    /// Fresh, un-started state.
    pub fn new() -> Self {
        Self::default()
    }

    /// Whether [`step`](Self::step) has seen the `Startup` input yet (the
    /// shell uses this to route its first call).
    pub fn started(&self) -> bool {
        self.started
    }

    /// Fold a mapped [`PadEvent`] into the deadman/connected tracking
    /// (mirrors [`crate::mapping::PadState::apply`]'s safety semantics:
    /// `Disconnected` also disarms).
    fn fold(&mut self, event: PadEvent) {
        match event {
            PadEvent::Deadman(held) => self.deadman = held,
            PadEvent::Connected => self.connected = true,
            PadEvent::Disconnected => {
                self.connected = false;
                self.deadman = false;
            }
            PadEvent::Axis(..) => {}
        }
    }

    /// Decide one pump iteration. Pure: a function of `(self, input, now_ns)`
    /// with the state transition applied in place. `now_ns` is the shell's
    /// monotonic clock (ns since pump start).
    ///
    /// Decision order (safety first):
    /// 1. `Startup` → seed `connected`, anchor both clocks, ring the one
    ///    startup zero.
    /// 2. A MAPPED event → fold + push + ring (real input activity; also
    ///    refreshes both anchors).
    /// 3. Liveness expiry (deadman latched, no event within
    ///    [`LIVENESS_TIMEOUT_NS`]) → push `Deadman(false)` + ring + warn.
    ///    Checked before the keepalive so a suspect link zeroes instead of
    ///    republishing its stale command.
    /// 4. Keepalive due (deadman + connected, no ring within
    ///    [`KEEPALIVE_NS`]) → ring. Evaluated on EVERY non-ringing iteration
    ///    (unmapped events included).
    /// 5. Otherwise quiet.
    pub fn step(&mut self, input: PumpInput, now_ns: u64) -> PumpDecision {
        match input {
            PumpInput::Startup { device_present } => {
                self.started = true;
                // Seed from enumeration — a pad connected before
                // launch never emits EventType::Connected.
                self.connected = device_present;
                self.last_ring_ns = now_ns;
                self.last_event_ns = now_ns;
                return PumpDecision {
                    push: None,
                    ring: true, // the single mandatory startup zero
                    warn: None,
                };
            }
            PumpInput::Event { mapped } => {
                // ANY event is link evidence: refresh the liveness anchor and
                // (belt-and-suspenders over the startup seed) mark the link
                // connected — folded Disconnected below immediately overrides.
                self.last_event_ns = now_ns;
                self.connected = true;
                if let Some(event) = mapped {
                    self.fold(event);
                    self.last_ring_ns = now_ns;
                    return PumpDecision {
                        push: Some(event),
                        ring: true,
                        warn: None,
                    };
                }
                // Unmapped: fall through to the shared cadence checks.
            }
            PumpInput::Timeout => {}
        }

        // 3. Liveness watchdog (inclusive boundary). Only reachable on
        // Timeout iterations — an Event iteration just refreshed the anchor.
        if self.deadman && now_ns.saturating_sub(self.last_event_ns) >= LIVENESS_TIMEOUT_NS {
            self.deadman = false; // fresh RB evidence required to re-arm
            self.last_ring_ns = now_ns;
            return PumpDecision {
                push: Some(PadEvent::Deadman(false)),
                ring: true, // publishes the safety zero
                warn: Some(PumpWarn::LivenessExpired),
            };
        }

        // 4. Keepalive — every iteration that hasn't already rung.
        if self.deadman
            && self.connected
            && now_ns.saturating_sub(self.last_ring_ns) >= KEEPALIVE_NS
        {
            self.last_ring_ns = now_ns;
            return PumpDecision {
                push: None,
                ring: true,
                warn: None,
            };
        }

        PumpDecision::QUIET
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::mapping::PadAxis;

    const MS: u64 = 1_000_000;

    /// Shorthand decisions for hand oracles.
    const QUIET: PumpDecision = PumpDecision::QUIET;
    const RING: PumpDecision = PumpDecision {
        push: None,
        ring: true,
        warn: None,
    };
    fn ring_push(event: PadEvent) -> PumpDecision {
        PumpDecision {
            push: Some(event),
            ring: true,
            warn: None,
        }
    }

    // 1 — startup rings exactly once (device present), then idle timeouts are
    // quiet (deadman not held). The binding startup-zero + no-idle contract at
    // the policy layer.
    #[test]
    fn startup_rings_once_then_idle_quiet() {
        let mut p = PumpState::new();
        assert!(!p.started());
        assert_eq!(
            p.step(
                PumpInput::Startup {
                    device_present: true
                },
                0
            ),
            RING
        );
        assert!(p.started());
        for k in 1..=10u64 {
            assert_eq!(
                p.step(PumpInput::Timeout, k * 20 * MS),
                QUIET,
                "idle timeout {k} must not ring"
            );
        }
    }

    // 2 — pre-connected pin: a pad connected BEFORE launch (seeded via enumeration,
    // NO EventType::Connected ever arrives) still gets the 20 Hz deadman
    // keepalive. Without the seed, `connected` stays false and the keepalive is dead.
    #[test]
    fn high1_pre_connected_pad_keepalive_without_connected_event() {
        let mut p = PumpState::new();
        p.step(
            PumpInput::Startup {
                device_present: true,
            },
            0,
        );
        // RB pressed at t=100ms (a mapped event — rings).
        assert_eq!(
            p.step(
                PumpInput::Event {
                    mapped: Some(PadEvent::Deadman(true))
                },
                100 * MS
            ),
            ring_push(PadEvent::Deadman(true))
        );
        // 20ms later: inside the keepalive window → quiet.
        assert_eq!(p.step(PumpInput::Timeout, 120 * MS), QUIET);
        // 50ms after the ring (inclusive boundary) → keepalive RINGS, despite
        // never having seen a Connected event (the pre-connected pin).
        assert_eq!(p.step(PumpInput::Timeout, 150 * MS), RING);
        // And again a full period later.
        assert_eq!(p.step(PumpInput::Timeout, 180 * MS), QUIET);
        assert_eq!(p.step(PumpInput::Timeout, 200 * MS), RING);
    }

    // 3 — startup WITHOUT a device: rings the startup zero; a (hypothetical)
    // deadman event still rings (event-driven), but keepalive stays off while
    // un-connected... EXCEPT the event itself is link evidence, so the event
    // marks the link connected and keepalive follows. Pins the any-event
    // evidence-refresh rule.
    #[test]
    fn startup_no_device_event_evidence_marks_connected() {
        let mut p = PumpState::new();
        assert_eq!(
            p.step(
                PumpInput::Startup {
                    device_present: false
                },
                0
            ),
            RING
        );
        // No pad, no events: quiet forever.
        assert_eq!(p.step(PumpInput::Timeout, 50 * MS), QUIET);
        // An event arrives after all (pad connected mid-run without a
        // Connected event reaching us) — evidence refresh marks connected.
        assert_eq!(
            p.step(
                PumpInput::Event {
                    mapped: Some(PadEvent::Deadman(true))
                },
                100 * MS
            ),
            ring_push(PadEvent::Deadman(true))
        );
        assert_eq!(
            p.step(PumpInput::Timeout, 150 * MS),
            RING,
            "keepalive armed"
        );
    }

    // 4 — keepalive-starvation pin: a pad streaming UNMAPPED events (gyro/trigger jitter)
    // at 10ms intervals must NOT starve the keepalive. Hand oracle over the
    // full decision vector: unmapped events at 10..=100ms; keepalive rings at
    // 50ms and 100ms (inclusive 50ms boundary from the last ring), quiet
    // otherwise. With the elapsed check only on the Timeout arm this vector
    // would be all-QUIET after the arm.
    #[test]
    fn finding5_unmapped_event_stream_does_not_starve_keepalive() {
        let mut p = PumpState::new();
        p.step(
            PumpInput::Startup {
                device_present: true,
            },
            0,
        );
        p.step(
            PumpInput::Event {
                mapped: Some(PadEvent::Deadman(true)),
            },
            0,
        ); // armed at t=0 (rings; keepalive anchor = 0)

        let mut got = Vec::new();
        for k in 1..=10u64 {
            got.push(p.step(PumpInput::Event { mapped: None }, k * 10 * MS));
        }
        // HAND ORACLE: rings exactly at t=50ms and t=100ms.
        let want: Vec<PumpDecision> = (1..=10u64)
            .map(|k| if k == 5 || k == 10 { RING } else { QUIET })
            .collect();
        assert_eq!(
            got, want,
            "keepalive must fire through an unmapped-event stream"
        );
    }

    // 5 — watchdog pin: liveness expiry. Deadman latched, pad goes fully
    // silent; keepalive republishes on 50ms timeouts until the 2s liveness
    // window (inclusive) — then ONE Deadman(false) push + ring + warn, then
    // quiet. A fresh RB press re-arms.
    #[test]
    fn finding2_liveness_expiry_zeroes_disarms_warns_once_then_rearms() {
        let mut p = PumpState::new();
        p.step(
            PumpInput::Startup {
                device_present: true,
            },
            0,
        );
        p.step(
            PumpInput::Event {
                mapped: Some(PadEvent::Deadman(true)),
            },
            0,
        ); // armed; liveness anchor = 0

        // Timeouts every 50ms up to 1950ms: keepalive rings each time (the
        // stale-command republish the watchdog exists to BOUND).
        for k in 1..=39u64 {
            assert_eq!(
                p.step(PumpInput::Timeout, k * 50 * MS),
                RING,
                "keepalive ring at t={}ms",
                k * 50
            );
        }
        // t=2000ms: liveness window (inclusive) → the watchdog preempts the
        // keepalive: one zero + disarm + warn.
        assert_eq!(
            p.step(PumpInput::Timeout, 2_000 * MS),
            PumpDecision {
                push: Some(PadEvent::Deadman(false)),
                ring: true,
                warn: Some(PumpWarn::LivenessExpired),
            },
            "liveness expiry must zero + disarm + warn"
        );
        // Disarmed: subsequent timeouts are QUIET (no keepalive, no re-warn).
        for k in 1..=10u64 {
            assert_eq!(p.step(PumpInput::Timeout, (2_000 + k * 50) * MS), QUIET);
        }
        // Fresh RB evidence re-arms; keepalive resumes.
        assert_eq!(
            p.step(
                PumpInput::Event {
                    mapped: Some(PadEvent::Deadman(true))
                },
                3_000 * MS
            ),
            ring_push(PadEvent::Deadman(true))
        );
        assert_eq!(p.step(PumpInput::Timeout, 3_050 * MS), RING);
    }

    // 6 — liveness boundary is INCLUSIVE: 1ns under the window still
    // keepalives; exactly at the window trips the watchdog.
    #[test]
    fn liveness_boundary_inclusive() {
        let mut p = PumpState::new();
        p.step(
            PumpInput::Startup {
                device_present: true,
            },
            0,
        );
        p.step(
            PumpInput::Event {
                mapped: Some(PadEvent::Deadman(true)),
            },
            0,
        );
        // 1ns under: NOT liveness — it's a keepalive ring (elapsed >= 50ms).
        let under = p.step(PumpInput::Timeout, LIVENESS_TIMEOUT_NS - 1);
        assert_eq!(under, RING, "1ns under the window is still keepalive");
        // Exactly at the window: watchdog trips.
        let at = p.step(PumpInput::Timeout, LIVENESS_TIMEOUT_NS);
        assert_eq!(at.warn, Some(PumpWarn::LivenessExpired));
        assert_eq!(at.push, Some(PadEvent::Deadman(false)));
    }

    // 7 — a healthy streaming pad NEVER trips liveness: axis events every
    // 100ms for 5s with the deadman held — no LivenessExpired anywhere.
    #[test]
    fn healthy_streaming_pad_never_trips_liveness() {
        let mut p = PumpState::new();
        p.step(
            PumpInput::Startup {
                device_present: true,
            },
            0,
        );
        p.step(
            PumpInput::Event {
                mapped: Some(PadEvent::Deadman(true)),
            },
            0,
        );
        for k in 1..=50u64 {
            let d = p.step(
                PumpInput::Event {
                    mapped: Some(PadEvent::Axis(PadAxis::LeftY, 0.5)),
                },
                k * 100 * MS,
            );
            assert_eq!(
                d.warn,
                None,
                "healthy link must never warn (t={}ms)",
                k * 100
            );
            assert!(d.ring, "mapped events always ring");
        }
    }

    // 8 — disconnect: push + ring, then keepalive is OFF (disarmed +
    // disconnected).
    #[test]
    fn disconnect_rings_once_then_keepalive_off() {
        let mut p = PumpState::new();
        p.step(
            PumpInput::Startup {
                device_present: true,
            },
            0,
        );
        p.step(
            PumpInput::Event {
                mapped: Some(PadEvent::Deadman(true)),
            },
            0,
        );
        assert_eq!(
            p.step(
                PumpInput::Event {
                    mapped: Some(PadEvent::Disconnected)
                },
                100 * MS
            ),
            ring_push(PadEvent::Disconnected)
        );
        for k in 1..=5u64 {
            assert_eq!(p.step(PumpInput::Timeout, (100 + k * 50) * MS), QUIET);
        }
    }

    // 9 — DETERMINISM: an identical (input, now) script folds to an identical
    // decision vector twice, and equals the hand oracle.
    #[test]
    fn policy_is_deterministic() {
        let script: &[(PumpInput, u64)] = &[
            (
                PumpInput::Startup {
                    device_present: true,
                },
                0,
            ),
            (
                PumpInput::Event {
                    mapped: Some(PadEvent::Deadman(true)),
                },
                10 * MS,
            ),
            (PumpInput::Event { mapped: None }, 30 * MS),
            (PumpInput::Timeout, 60 * MS),
            (PumpInput::Timeout, 80 * MS),
            (
                PumpInput::Event {
                    mapped: Some(PadEvent::Deadman(false)),
                },
                90 * MS,
            ),
            (PumpInput::Timeout, 200 * MS),
        ];
        let run = || {
            let mut p = PumpState::new();
            script
                .iter()
                .map(|&(input, now)| p.step(input, now))
                .collect::<Vec<_>>()
        };
        let a = run();
        let b = run();
        assert_eq!(a, b, "identical scripts must produce identical decisions");
        // HAND ORACLE: startup ring; deadman ring; unmapped quiet (20ms since
        // ring < 50); timeout at 60 rings (50ms since the 10ms ring); timeout
        // at 80 quiet (20ms since); release ring; timeout at 200 quiet
        // (disarmed).
        let want = vec![
            RING,
            ring_push(PadEvent::Deadman(true)),
            QUIET,
            RING,
            QUIET,
            ring_push(PadEvent::Deadman(false)),
            QUIET,
        ];
        assert_eq!(a, want, "the decision vector must equal the hand oracle");
    }
}
