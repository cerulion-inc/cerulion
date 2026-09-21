// SPDX-License-Identifier: AGPL-3.0-only
//! The control-lease / deadman / e-stop state machine (PURE).
//!
//! One actuation lease holder at a time, always visible. A holder must renew
//! within the deadman window or the deadman fires and the robot enters the
//! safe frame. A higher-role challenger may steal the lease. And e-stop is a
//! PERMISSION FLOOR — **any** paired session may engage it, it is never
//! lease-gated, and it always wins (no actuation while engaged).
//!
//! This module is deterministic: every operation takes an explicit `now_ns`
//! rather than reading a clock, so the whole matrix is oracle-testable.
//!
//! **ROBOT_CONFIRM**: the deadman window and the safe-frame contents are
//! placeholders confirmed on-robot at integration time — see
//! [`crate::constants::LEASE_DEADMAN_WINDOW_MS_ROBOT_CONFIRM`] and
//! [`crate::constants::SAFE_FRAME_DESCRIPTION_ROBOT_CONFIRM`]. `cerud` owns the
//! *permission* floor only; it does not yet emit any actuation.

use crate::constants::{
    LEASE_DEADMAN_WINDOW_MS_ROBOT_CONFIRM, SAFE_FRAME_DESCRIPTION_ROBOT_CONFIRM,
};

/// A lease-holding role. Steal-by-role uses [`LeaseRole::rank`]: a strictly
/// higher rank may steal from a lower one; equal ranks may not steal.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum LeaseRole {
    /// A normal operator (teleop / app-driven control).
    Operator,
    /// A supervisor (may steal from an operator).
    Supervisor,
    /// A safety authority (may steal from anyone).
    Safety,
}

impl LeaseRole {
    /// The steal-precedence rank (higher wins).
    pub fn rank(&self) -> u8 {
        match self {
            LeaseRole::Operator => 1,
            LeaseRole::Supervisor => 2,
            LeaseRole::Safety => 3,
        }
    }
}

/// The current lease holder.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct LeaseHolder {
    /// The holding session identity.
    pub session: String,
    /// The holder's role.
    pub role: LeaseRole,
    /// When the lease was first acquired (ns).
    pub acquired_at_ns: u64,
    /// When the holder last renewed (ns). The deadman measures from here.
    pub last_renew_ns: u64,
}

/// The e-stop permission floor state.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum EstopState {
    /// Not engaged; actuation permitted (subject to a live lease).
    Clear,
    /// Engaged by `by`; no actuation until cleared.
    Engaged { by: String },
}

/// The safe frame the deadman/e-stop drives the robot into (a placeholder
/// marker — its contents are ROBOT_CONFIRM).
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct SafeFrame {
    /// A human-readable description (ROBOT_CONFIRM placeholder).
    pub description: &'static str,
}

impl SafeFrame {
    /// The placeholder safe frame.
    pub fn placeholder() -> Self {
        SafeFrame {
            description: SAFE_FRAME_DESCRIPTION_ROBOT_CONFIRM,
        }
    }
}

/// The outcome of an [`ControlLease::acquire`] call.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum AcquireOutcome {
    /// The lease was free and is now held by the caller.
    Granted,
    /// The caller already held it; the acquire renewed it.
    Renewed,
    /// The caller's role outranked the holder and stole the lease; the named
    /// session is the party that was displaced.
    Stolen { from: String },
    /// The lease is held by an equal-or-higher role; the caller is refused.
    Busy { held_by: String },
}

/// The event a [`ControlLease::poll_deadman`] may emit.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct DeadmanFired {
    /// The session whose missed renewal tripped the deadman.
    pub lapsed_session: String,
    /// The safe frame the robot enters.
    pub safe_frame: SafeFrame,
}

/// An observable snapshot of the lease state (Principle #3: the holder is
/// always visible, independent of execution).
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct LeaseSnapshot {
    pub holder: Option<LeaseHolder>,
    pub estop: EstopState,
    pub deadman_window_ns: u64,
}

/// The pure control-lease / deadman / e-stop state machine.
#[derive(Debug, Clone)]
pub struct ControlLease {
    holder: Option<LeaseHolder>,
    estop: EstopState,
    deadman_window_ns: u64,
}

impl ControlLease {
    /// A fresh lease with the given deadman window (ns).
    pub fn new(deadman_window_ns: u64) -> Self {
        ControlLease {
            holder: None,
            estop: EstopState::Clear,
            deadman_window_ns,
        }
    }

    /// A fresh lease using the ROBOT_CONFIRM default window.
    pub fn with_default_window() -> Self {
        ControlLease::new(LEASE_DEADMAN_WINDOW_MS_ROBOT_CONFIRM * 1_000_000)
    }

    /// Acquire (or renew, or steal) the lease.
    ///
    /// - Free → `Granted` (with the caller's `role`).
    /// - Held by the caller → `Renewed` (bumps the deadman clock; **the stored
    ///   role is unchanged** — see below).
    /// - Held by a strictly-lower role → `Stolen` (with the challenger's `role`).
    /// - Held by an equal-or-higher role → `Busy` (state unchanged).
    ///
    /// **No self-escalation (security).** A same-session re-acquire NEVER changes
    /// the holder's stored `role`, even if the call carries a higher (or lower)
    /// `role`. Role is fixed at grant/steal time; the only ways it can change are
    /// a fresh acquire *after releasing* (a new grant) or a steal by a
    /// strictly-higher **different** session. This lease is a pure state machine
    /// with no authorization context, so it must never be an escalation vector: a
    /// session that already holds the lease cannot silently raise its own
    /// steal-precedence (or any role-gated outcome such as `may_actuate`) by
    /// re-acquiring with a higher role. Caller-role authorization lives in the
    /// ops-server / authorizer, not here.
    pub fn acquire(&mut self, session: &str, role: LeaseRole, now_ns: u64) -> AcquireOutcome {
        match &self.holder {
            None => {
                self.holder = Some(LeaseHolder {
                    session: session.to_string(),
                    role,
                    acquired_at_ns: now_ns,
                    last_renew_ns: now_ns,
                });
                AcquireOutcome::Granted
            }
            Some(h) if h.session == session => {
                // RENEW in place: keep the EXISTING role (no self-escalation) and
                // the original acquire time; only bump the deadman clock. The
                // call's `role` argument is deliberately ignored on this arm.
                let acquired_at = h.acquired_at_ns;
                let existing_role = h.role;
                self.holder = Some(LeaseHolder {
                    session: session.to_string(),
                    role: existing_role,
                    acquired_at_ns: acquired_at,
                    last_renew_ns: now_ns,
                });
                AcquireOutcome::Renewed
            }
            Some(h) if role.rank() > h.role.rank() => {
                let from = h.session.clone();
                self.holder = Some(LeaseHolder {
                    session: session.to_string(),
                    role,
                    acquired_at_ns: now_ns,
                    last_renew_ns: now_ns,
                });
                AcquireOutcome::Stolen { from }
            }
            Some(h) => AcquireOutcome::Busy {
                held_by: h.session.clone(),
            },
        }
    }

    /// Renew the lease. Only the current holder may renew; anyone else gets an
    /// error (never a silent no-op).
    pub fn renew(&mut self, session: &str, now_ns: u64) -> Result<(), LeaseViolation> {
        match &mut self.holder {
            Some(h) if h.session == session => {
                h.last_renew_ns = now_ns;
                Ok(())
            }
            Some(h) => Err(LeaseViolation::NotHolder {
                caller: session.to_string(),
                held_by: h.session.clone(),
            }),
            None => Err(LeaseViolation::NoHolder {
                caller: session.to_string(),
            }),
        }
    }

    /// Release the lease. Only the current holder may release.
    pub fn release(&mut self, session: &str) -> Result<(), LeaseViolation> {
        match &self.holder {
            Some(h) if h.session == session => {
                self.holder = None;
                Ok(())
            }
            Some(h) => Err(LeaseViolation::NotHolder {
                caller: session.to_string(),
                held_by: h.session.clone(),
            }),
            None => Err(LeaseViolation::NoHolder {
                caller: session.to_string(),
            }),
        }
    }

    /// Poll the deadman. If a holder exists and has not renewed within the
    /// window, the deadman fires: the safe frame is entered and the lease is
    /// dropped. A second poll returns `None` (it already fired).
    pub fn poll_deadman(&mut self, now_ns: u64) -> Option<DeadmanFired> {
        let fired = match &self.holder {
            Some(h) => now_ns.saturating_sub(h.last_renew_ns) > self.deadman_window_ns,
            None => false,
        };
        if fired {
            let lapsed_session = self.holder.take().map(|h| h.session).unwrap_or_default();
            Some(DeadmanFired {
                lapsed_session,
                safe_frame: SafeFrame::placeholder(),
            })
        } else {
            None
        }
    }

    /// Engage e-stop. The PERMISSION FLOOR: any session may engage it, it is
    /// never lease-gated, and it always wins. Idempotent (re-engaging keeps
    /// the original engager). Returns the safe frame that is now in force.
    pub fn engage_estop(&mut self, session: &str) -> SafeFrame {
        if let EstopState::Clear = self.estop {
            self.estop = EstopState::Engaged {
                by: session.to_string(),
            };
        }
        SafeFrame::placeholder()
    }

    /// Clear e-stop, returning to normal (subject to a live lease). Any paired
    /// session may clear it in this skeleton; a stricter clear policy (only
    /// the engager / a safety role) is not implemented.
    pub fn clear_estop(&mut self) {
        self.estop = EstopState::Clear;
    }

    /// Whether actuation is permitted RIGHT NOW: a live (unexpired) holder AND
    /// e-stop clear. E-stop always dominates — an engaged e-stop forbids
    /// actuation regardless of the lease.
    pub fn may_actuate(&self, now_ns: u64) -> bool {
        if !matches!(self.estop, EstopState::Clear) {
            return false;
        }
        match &self.holder {
            Some(h) => now_ns.saturating_sub(h.last_renew_ns) <= self.deadman_window_ns,
            None => false,
        }
    }

    /// The current e-stop state.
    pub fn estop(&self) -> &EstopState {
        &self.estop
    }

    /// The current holder, if any.
    pub fn holder(&self) -> Option<&LeaseHolder> {
        self.holder.as_ref()
    }

    /// An observable snapshot.
    pub fn snapshot(&self) -> LeaseSnapshot {
        LeaseSnapshot {
            holder: self.holder.clone(),
            estop: self.estop.clone(),
            deadman_window_ns: self.deadman_window_ns,
        }
    }
}

/// A lease operation was attempted by a party not entitled to it.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum LeaseViolation {
    /// The caller is not the holder (someone else holds it).
    NotHolder { caller: String, held_by: String },
    /// There is no holder to operate on.
    NoHolder { caller: String },
}

impl std::fmt::Display for LeaseViolation {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            LeaseViolation::NotHolder { caller, held_by } => write!(
                f,
                "session '{caller}' is not the lease holder (held by '{held_by}')"
            ),
            LeaseViolation::NoHolder { caller } => {
                write!(f, "session '{caller}' operated on a lease with no holder")
            }
        }
    }
}

impl std::error::Error for LeaseViolation {}
