// SPDX-License-Identifier: AGPL-3.0-only
//! Resource defaults for authenticated shell and exec sessions.

use std::time::Duration;

pub(super) const MAX_REAUTHORIZE_INTERVAL: Duration = Duration::from_secs(1);

/// Robot-configurable resource limits. These never classify command contents.
#[derive(Clone, Debug)]
pub struct SessionLimits {
    pub reauthorize_interval: Duration,
    pub io_timeout: Duration,
    pub max_recording_bytes: u64,
    pub max_stdin_bytes: u64,
    pub max_stdout_bytes: u64,
    pub max_stderr_bytes: u64,
    pub exec_timeout: Option<Duration>,
    pub shell_timeout: Option<Duration>,
    pub shell_idle_timeout: Option<Duration>,
}

impl Default for SessionLimits {
    fn default() -> Self {
        Self {
            reauthorize_interval: MAX_REAUTHORIZE_INTERVAL,
            io_timeout: Duration::from_secs(2),
            max_recording_bytes: 64 * 1024 * 1024,
            max_stdin_bytes: 1024 * 1024,
            max_stdout_bytes: 1024 * 1024,
            max_stderr_bytes: 1024 * 1024,
            // One minute bounds unattended short commands; operators can extend
            // or disable it for longer work through the exec timeout setting.
            exec_timeout: Some(Duration::from_secs(60)),
            // Eight hours permits a working session while bounding abandoned
            // shells; the robot operator can extend or disable this wall limit.
            shell_timeout: Some(Duration::from_secs(8 * 60 * 60)),
            // Thirty minutes without input permits breaks while retiring idle
            // terminals; the robot operator can extend or disable this limit.
            shell_idle_timeout: Some(Duration::from_secs(30 * 60)),
        }
    }
}
