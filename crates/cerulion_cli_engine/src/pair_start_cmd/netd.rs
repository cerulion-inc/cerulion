// SPDX-License-Identifier: AGPL-3.0-only
//! The production serving host and process environment of a run: the
//! serving hold on its own `cerulion-netd` connection, status probes on a
//! second one, the process clocks and the operating system's random source.

use std::path::{Path, PathBuf};
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Arc;
use std::time::{Duration, Instant, SystemTime};

use cerulion_netd::client::{NetdClient, ServingLease};

use super::run::{HeldServing, ServingHost, StartEnv};
use super::{hold_refusal, HoldRefusal, NetdProbe};

/// The robot state root this process resolves: `CERULION_STATE_ROOT`, else
/// `robot-state` under the login configuration directory
/// (`cerulion_discovery::robot_state::resolve`), as `cerulion-netd` resolves
/// it.
pub fn resolve_state_root() -> Option<PathBuf> {
    cerulion_discovery::robot_state::resolve()
}

/// How long one status probe may take, its connect included.
pub const PROBE_DEADLINE: Duration = Duration::from_secs(1);

/// The serving host backed by `cerulion-netd`. The lease lives as long as
/// this value: dropping it releases the hold.
#[derive(Debug, Default)]
pub struct NetdServing {
    lease: Option<ServingLease>,
    status: Option<NetdClient>,
}

impl ServingHost for NetdServing {
    fn authority(&mut self) -> Result<(), &'static str> {
        cerulion_netd::serving_login::require_serving_authority()
    }

    fn hold(&mut self, ttl_s: u64) -> Result<HeldServing, HoldRefusal> {
        let exe = std::env::current_exe().ok();
        let exe_dir = exe.as_deref().and_then(Path::parent);
        let client = NetdClient::connect_or_spawn().map_err(|e| hold_refusal(&e, None, exe_dir))?;
        // Read before the hold consumes the client.
        let protocol = client.daemon_protocol();
        let lease = client
            .hold_serving(ttl_s)
            .map_err(|e| hold_refusal(&e, Some(protocol), exe_dir))?;
        let held = HeldServing {
            root: lease.state_root().to_path_buf(),
            expires: lease.expires(),
        };
        self.lease = Some(lease);
        Ok(held)
    }

    fn probe(&mut self) -> NetdProbe {
        let deadline = Instant::now() + PROBE_DEADLINE;
        if self.status.is_none() {
            self.status = NetdClient::connect_existing_until(deadline).ok();
        }
        let Some(client) = self.status.as_mut() else {
            return NetdProbe::Unreachable;
        };
        match client.status_until(deadline) {
            Ok(status) => NetdProbe::Status {
                startup_error: status.robot_startup_error.is_some(),
            },
            Err(error) => {
                tracing::debug!(
                    error = %error,
                    "cerulion pair --start: a cerulion-netd status probe failed"
                );
                self.status = None;
                NetdProbe::Unreachable
            }
        }
    }
}

/// The process environment: `running` is cleared by the interrupt handler.
#[derive(Debug)]
pub struct ProcessEnv {
    running: Arc<AtomicBool>,
}

impl ProcessEnv {
    /// The environment whose interrupt clears `running`.
    pub fn new(running: Arc<AtomicBool>) -> Self {
        ProcessEnv { running }
    }
}

impl StartEnv for ProcessEnv {
    fn now(&mut self) -> Instant {
        Instant::now()
    }

    fn unix_now_s(&mut self) -> u64 {
        SystemTime::now()
            .duration_since(SystemTime::UNIX_EPOCH)
            .map_or(0, |since| since.as_secs())
    }

    fn sleep(&mut self, span: Duration) {
        std::thread::sleep(span);
    }

    fn interrupted(&mut self) -> bool {
        !self.running.load(Ordering::Relaxed)
    }

    fn draw_u32(&mut self) -> Result<u32, String> {
        getrandom::u32().map_err(|e| e.to_string())
    }

    fn nonce(&mut self) -> Result<[u8; 16], String> {
        let mut nonce = [0u8; 16];
        getrandom::fill(&mut nonce).map_err(|e| e.to_string())?;
        Ok(nonce)
    }
}
