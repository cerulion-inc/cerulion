// SPDX-License-Identifier: AGPL-3.0-only
//! Hardware `gilrs` gamepad pump SHELL for the joystick teleop node.
//!
//! This is the I/O half of the injectable event-source seam. ALL behavior —
//! connected seeding, ring cadence, keepalive, the liveness watchdog,
//! disconnect handling — lives in the PURE [`crate::pump_policy`] module
//! (oracle-tested); this file only performs the blocking
//! observations and applies the returned [`PumpDecision`]s (push → inbox,
//! ring → doorbell, warn → tracing).
//!
//! Compiled on every platform (gilrs is pure-Rust with Linux/macOS/Windows
//! backends) but only ever CONSTRUCTED on the live path — the node's
//! `external_source()` calls [`spawn_gilrs_blocking`], which the deterministic
//! polled/replay path never invokes (Principle #7). Every CI test drives the
//! node with scripted [`PadEvent`]s pushed directly into the shared inbox (or
//! the policy with scripted [`PumpInput`]s), so this shell never runs without
//! hardware.
//!
//! # Threading model
//!
//! [`spawn_gilrs_blocking`] returns an [`ExternalSource::Blocking`] closure.
//! The runtime drives it on a single dedicated helper thread; each `true`
//! return rings the node's doorbell (⇒ one tick ⇒ one publish). The
//! closure OWNS the `gilrs::Gilrs` handle (gilrs is `Send`, so the closure is
//! `Send`) and translates each iteration into ONE [`PumpState::step`] call. A
//! Bluetooth drop pushes `Disconnected` (zeroing the command once + a loud
//! warn) and keeps polling — the same `Gilrs` handle re-emits `Connected` on
//! reconnect (the node never exits).

use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

use cerulion_core::prelude::ExternalSource;
use gilrs::{Axis, Button, EventType, Gilrs};

use crate::mapping::{PadAxis, PadEvent};
use crate::pump_policy::{PumpDecision, PumpInput, PumpState, PumpWarn};

/// Bounded blocking-poll timeout: how long `next_event_blocking` waits before
/// returning `None` so the policy can re-evaluate keepalive/liveness and the
/// helper can observe an eventual runtime-drop shutdown between calls.
const POLL_TIMEOUT: Duration = Duration::from_millis(20);
/// Backoff sleep when no gamepad subsystem is available (gilrs init failed):
/// keeps the helper from spinning while it stays quiet (device-less).
const NO_DEVICE_BACKOFF: Duration = Duration::from_millis(50);

/// Build the [`ExternalSource::Blocking`] gamepad pump feeding `inbox`.
///
/// See [`crate::pump_policy`] for the full decision contract (startup zero,
/// keepalive, liveness watchdog, no-idle-publish).
pub fn spawn_gilrs_blocking(inbox: Arc<Mutex<Vec<PadEvent>>>) -> ExternalSource {
    // Per-closure state (owned by the helper thread for its whole life).
    let mut gilrs: Option<Gilrs> = None;
    let mut policy = PumpState::new();
    let base = Instant::now();

    ExternalSource::Blocking(Box::new(move || -> bool {
        // ---- Startup: init gilrs, seed `connected` from ENUMERATION
        // (a pad connected before launch never emits
        // EventType::Connected), and let the policy ring the one startup
        // zero (device present or not).
        if !policy.started() {
            let device_present = match Gilrs::new() {
                Ok(g) => {
                    let present = g.gamepads().next().is_some();
                    if present {
                        tracing::info!(
                            "gamepad already connected at launch — `connected` seeded \
                             from enumeration (no Connected event expected)"
                        );
                    } else {
                        tracing::info!(
                            "no gamepad at launch — `connected` seeded false; will \
                             arm on a Connected event"
                        );
                    }
                    gilrs = Some(g);
                    present
                }
                Err(e) => {
                    tracing::warn!(
                        error = %e,
                        "gilrs init failed — joystick node running device-less \
                         (startup zero still published; will stay quiet)"
                    );
                    false
                }
            };
            let now_ns = base.elapsed().as_nanos() as u64;
            let decision = policy.step(PumpInput::Startup { device_present }, now_ns);
            return apply(decision, &inbox);
        }

        // ---- Device-less: stay quiet (no fake data — Principle #13). A small
        // backoff avoids a hot spin. A device-less start is a genuine
        // no-gamepad host; reconnect handling needs a live gilrs context.
        let Some(g) = gilrs.as_mut() else {
            std::thread::sleep(NO_DEVICE_BACKOFF);
            return false;
        };

        // ---- One bounded blocking observation → one policy step.
        // (`gilrs::Event { id, event: EventType, time }` — we read `.event`.)
        let input = match g.next_event_blocking(Some(POLL_TIMEOUT)) {
            Some(ev) => PumpInput::Event {
                mapped: translate(ev.event),
            },
            None => PumpInput::Timeout,
        };
        let now_ns = base.elapsed().as_nanos() as u64;
        let decision = policy.step(input, now_ns);
        apply(decision, &inbox)
    }))
}

/// Apply one [`PumpDecision`]: log any warn, push any event into the node's
/// inbox (with the connect/disconnect operator logs), return the ring.
fn apply(decision: PumpDecision, inbox: &Arc<Mutex<Vec<PadEvent>>>) -> bool {
    if let Some(warn) = decision.warn {
        match warn {
            PumpWarn::LivenessExpired => {
                tracing::warn!(
                    timeout_ms = crate::pump_policy::LIVENESS_TIMEOUT_NS / 1_000_000,
                    "gamepad link SUSPECT — deadman held but no pad events within \
                     the liveness window: publishing zero and clearing the deadman \
                     latch (release and re-press RB to re-arm)"
                );
            }
        }
    }
    if let Some(event) = decision.push {
        match event {
            PadEvent::Connected => tracing::info!("gamepad connected"),
            PadEvent::Disconnected => tracing::warn!(
                "gamepad disconnected (BT drop / unplug) — publishing zero and \
                 scanning for reconnect (node stays alive)"
            ),
            _ => {}
        }
        // POISON RECOVERY (applies to every inbox lock in this crate): the
        // inbox is a Vec of plain POD events — a panic while it was held can
        // only leave a valid Vec (an element is either fully pushed or not;
        // nothing is ever torn), so recovering the guard is sound. And for a
        // SAFETY inbox, delivering Disconnected / Deadman(false) after a
        // panic strictly beats dropping them: an `if let Ok` here would
        // silently drop every post-poison event while the doorbell kept
        // ringing, leaving the node ticking on stale state forever.
        inbox
            .lock()
            .unwrap_or_else(std::sync::PoisonError::into_inner)
            .push(event);
    }
    decision.ring
}

/// Translate a raw gilrs [`EventType`] into a [`PadEvent`], or `None` for an
/// event the mapping ignores. PURE — all state tracking lives in
/// [`PumpState`].
fn translate(event: EventType) -> Option<PadEvent> {
    match event {
        EventType::AxisChanged(Axis::LeftStickY, v, _) => {
            Some(PadEvent::Axis(PadAxis::LeftY, v as f64))
        }
        EventType::AxisChanged(Axis::LeftStickX, v, _) => {
            Some(PadEvent::Axis(PadAxis::LeftX, v as f64))
        }
        EventType::AxisChanged(Axis::RightStickX, v, _) => {
            Some(PadEvent::Axis(PadAxis::RightX, v as f64))
        }
        // RB shoulder = gilrs RightTrigger (digital bumper, distinct from the
        // analog RightTrigger2 / RT).
        EventType::ButtonPressed(Button::RightTrigger, _) => Some(PadEvent::Deadman(true)),
        EventType::ButtonReleased(Button::RightTrigger, _) => Some(PadEvent::Deadman(false)),
        EventType::Connected => Some(PadEvent::Connected),
        EventType::Disconnected => Some(PadEvent::Disconnected),
        // Every other EventType (ButtonRepeated, ButtonChanged analog
        // triggers, Dropped queue-overflow marker, force-feedback, other
        // axes/buttons, and any future #[non_exhaustive] variant) is ignored —
        // the policy still counts it as link evidence via `Event{mapped:None}`.
        _ => None,
    }
}
