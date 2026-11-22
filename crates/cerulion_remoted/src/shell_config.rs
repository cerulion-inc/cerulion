// SPDX-License-Identifier: AGPL-3.0-only
//! Reusable option parsing for robot-local shell resource settings.
//! The caller supplies the local environment setting explicitly.

use std::time::Duration;

use crate::{RemotedConfig, RemotedError};
use clap::Args;

/// Parsed shell settings that a caller can apply to daemon configuration.
#[derive(Args, Debug)]
pub struct ShellFlags {
    /// Disable shell and exec locally; remote owners cannot override off.
    #[arg(long, value_parser = ["on", "off"])]
    remote_shell: Option<String>,
    /// Deadline for shell storage, transport, and admission I/O, in positive seconds.
    #[arg(long, default_value_t = 2, value_parser = clap::value_parser!(u64).range(1..))]
    remote_shell_io_timeout_secs: u64,
    /// Maximum concurrent shell and exec sessions on this robot.
    #[arg(long, default_value_t = 4)]
    remote_shell_max_sessions: usize,
    /// Maximum concurrent sessions per authenticated device.
    #[arg(long, default_value_t = 2)]
    remote_shell_max_per_device: usize,
    /// Maximum bytes in each complete session recording.
    #[arg(long, default_value_t = 64 * 1024 * 1024)]
    remote_shell_recording_bytes: u64,
    /// Maximum stdin bytes accepted by one exec session.
    #[arg(long, default_value_t = 1024 * 1024)]
    remote_exec_stdin_bytes: u64,
    /// Maximum stdout bytes delivered by exec; output is recorded up to the recording limit.
    #[arg(long, default_value_t = 1024 * 1024)]
    remote_exec_stdout_bytes: u64,
    /// Maximum stderr bytes delivered by exec; output is recorded up to the recording limit.
    #[arg(long, default_value_t = 1024 * 1024)]
    remote_exec_stderr_bytes: u64,
    /// Exec wall timeout in seconds; zero disables this timeout.
    #[arg(long, default_value_t = 60)]
    remote_exec_timeout_secs: u64,
    /// Shell wall timeout in seconds; zero disables this timeout.
    #[arg(long, default_value_t = 8 * 60 * 60)]
    remote_shell_timeout_secs: u64,
    /// Shell input-idle timeout in seconds; zero disables this timeout.
    #[arg(long, default_value_t = 30 * 60)]
    remote_shell_idle_secs: u64,
}

impl ShellFlags {
    /// Validate every setting before updating the supplied configuration.
    ///
    /// Either the parsed flag or `environment` saying off prohibits access.
    /// Invalid environment text is refused, including non-Unicode values.
    /// This method does not read the environment or open a state file.
    pub fn apply(
        self,
        config: &mut RemotedConfig,
        environment: Option<&std::ffi::OsStr>,
    ) -> Result<(), RemotedError> {
        let off = hard_off_os(self.remote_shell.as_deref(), environment)?;
        if self.remote_shell_max_sessions == 0
            || self.remote_shell_max_per_device == 0
            || self.remote_shell_recording_bytes == 0
        {
            return Err(RemotedError::Config("shell concurrency and recording limits must be positive; use --remote-shell off to disable access".into()));
        }
        config.remote_shell_off = off;
        config.remote_shell_max_sessions = self.remote_shell_max_sessions;
        config.remote_shell_max_per_device = self.remote_shell_max_per_device;
        let limits = &mut config.remote_shell_limits;
        apply_io_timeout(&mut limits.io_timeout, self.remote_shell_io_timeout_secs);
        limits.max_recording_bytes = self.remote_shell_recording_bytes;
        limits.max_stdin_bytes = self.remote_exec_stdin_bytes;
        limits.max_stdout_bytes = self.remote_exec_stdout_bytes;
        limits.max_stderr_bytes = self.remote_exec_stderr_bytes;
        limits.exec_timeout = duration(self.remote_exec_timeout_secs);
        limits.shell_timeout = duration(self.remote_shell_timeout_secs);
        limits.shell_idle_timeout = duration(self.remote_shell_idle_secs);
        Ok(())
    }
}

fn apply_io_timeout(timeout: &mut Duration, seconds: u64) {
    *timeout = Duration::from_secs(seconds);
}

fn duration(seconds: u64) -> Option<Duration> {
    (seconds != 0).then(|| Duration::from_secs(seconds))
}

#[cfg(test)]
#[path = "shell_config/tests.rs"]
mod tests;

fn hard_off_os(flag: Option<&str>, env: Option<&std::ffi::OsStr>) -> Result<bool, RemotedError> {
    let env = env
        .map(|value| {
            value.to_str().ok_or_else(|| {
                RemotedError::Config(
                    "remote shell setting must be valid UTF-8 containing on or off".into(),
                )
            })
        })
        .transpose()?;
    hard_off(flag, env)
}

fn hard_off(flag: Option<&str>, env: Option<&str>) -> Result<bool, RemotedError> {
    let mut off = false;
    for value in [flag, env].into_iter().flatten() {
        match value.trim().to_ascii_lowercase().as_str() {
            "" | "on" => {}
            "off" => off = true,
            _ => {
                return Err(RemotedError::Config(
                    "remote shell setting must be on or off".into(),
                ))
            }
        }
    }
    Ok(off)
}
