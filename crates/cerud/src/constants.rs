// SPDX-License-Identifier: AGPL-3.0-only
//! Well-known ports, env-var names, and the ROBOT-CONFIRM safety placeholders.

use crate::error::{CerudError, CerudResult};

/// The well-known TCP/UDP port the Cerulion ops service listens on in
/// production.
///
/// 7684 sits one above the gateway's 7683. It is DELIBERATELY **not** 7447
/// (zenoh's own router port) and **not** the bare gRPC default 50051 — the
/// decision prohibits both. Overridable via
/// [`CERULION_OPS_PORT_ENV`].
///
/// Note: this chunk ships the Unix-domain-socket dev transport; the port is
/// the production seam the later iroh transport binds. It is a constant here
/// so the value is pinned and greppable from day one.
pub const CERULION_OPS_PORT: u16 = 7684;

/// Environment variable overriding [`CERULION_OPS_PORT`].
pub const CERULION_OPS_PORT_ENV: &str = "CERULION_OPS_PORT";

/// Default on-robot state root for deploy bundles + receipts.
/// The bundle dir layout is `<STATE_ROOT>/bundles/<hash>/` with a
/// `<STATE_ROOT>/current` symlink (see [`crate::deploy`]).
pub const DEFAULT_STATE_ROOT: &str = "/var/lib/cerulion";

/// The control-lease **deadman window**: if the lease holder does not renew
/// within this window, the deadman fires and the robot enters the safe frame.
///
/// **ROBOT_CONFIRM** — 500 ms is a PLACEHOLDER. The final window is confirmed
/// on-robot at integration time (it depends on the actuator control-loop rate
/// and the physical stopping distance). Do not treat this value as final; do
/// not bake it into any actuation contract without the on-robot confirmation.
pub const LEASE_DEADMAN_WINDOW_MS_ROBOT_CONFIRM: u64 = 500;

/// Human-readable description of the safe frame the deadman/e-stop drives the
/// robot into.
///
/// **ROBOT_CONFIRM** — the actual safe-frame CONTENTS (which actuators to
/// zero, which brakes to engage, the ramp profile) are a PLACEHOLDER confirmed
/// on-robot at integration time. `cerud` only owns the *permission* floor
/// (who may stop); it does not yet emit any actuation.
pub const SAFE_FRAME_DESCRIPTION_ROBOT_CONFIRM: &str =
    "zero all actuation setpoints and engage holding brakes (contents confirmed on-robot)";

/// Parse an effective ops port from an optional raw env-var value.
///
/// Pure (takes the raw string rather than reading the environment) so it is
/// oracle-testable without env mutation:
/// - `None` → the default [`CERULION_OPS_PORT`].
/// - `Some(valid nonzero u16)` → that port.
/// - `Some(malformed)` / `Some("0")` → a loud [`CerudError::Config`] (strict:
///   a bad override is refused, never silently ignored).
pub fn parse_ops_port(raw: Option<&str>) -> CerudResult<u16> {
    match raw {
        None => Ok(CERULION_OPS_PORT),
        Some(s) => {
            let trimmed = s.trim();
            let port: u16 = trimmed.parse().map_err(|e| {
                CerudError::Config(format!(
                    "{CERULION_OPS_PORT_ENV}='{s}' is not a valid port: {e}"
                ))
            })?;
            if port == 0 {
                return Err(CerudError::Config(format!(
                    "{CERULION_OPS_PORT_ENV}='{s}' must be a nonzero port"
                )));
            }
            Ok(port)
        }
    }
}

/// Resolve the effective ops port from the process environment.
pub fn resolve_ops_port() -> CerudResult<u16> {
    parse_ops_port(std::env::var(CERULION_OPS_PORT_ENV).ok().as_deref())
}
