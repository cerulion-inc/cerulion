// SPDX-License-Identifier: AGPL-3.0-only
//! Cleanup of ordinary jobs in a child-owned Unix session.
//!
//! The caller keeps the session leader unreaped until cleanup finishes, pinning
//! the numeric SID. Each backend captures a process identity BEFORE checking
//! its SID. Native handles bind signals atomically; the Linux fallback
//! rechecks SID and start time immediately before each numeric-PID signal.

mod linux;

use std::io;
use std::time::{Duration, Instant};

use rustix::process::{Pid, Signal};

// Keep repeated job scans within the cleanup share of the two-second revocation
// budget after a one-second sweep and a 500 ms authorization lookup.
const CLEANUP_TIMEOUT: Duration = Duration::from_millis(250);
// Give signaled jobs a scheduling slice to exit between scans, while permitting
// repeated identity captures within the 250 ms cleanup budget.
const CLEANUP_RESCAN_INTERVAL: Duration = Duration::from_millis(5);

use linux as platform;

use linux::session_id;

/// Process identity mechanism selected by the cleanup preflight.
#[derive(Clone, Copy)]
pub enum Mode {
    /// Signals remain bound to a pidfd, even after numeric PID reuse.
    Native,
    /// Recheck birth time and session before signalling on a kernel without pidfds.
    /// The last check cannot be atomic with the numeric-PID signal.
    Revalidated,
}

/// Pure decision used immediately before every fallback signal. The syscall
/// following it still has the explicitly documented check-to-signal race.
fn same_process(
    expected_sid: Pid,
    actual_sid: Pid,
    expected_start: [u64; 2],
    actual_start: [u64; 2],
) -> bool {
    expected_sid == actual_sid && expected_start == actual_start
}

enum Identity {
    Native(platform::Identity),
    Revalidated(platform::FallbackIdentity),
}

impl Identity {
    fn capture(mode: Mode, pid: Pid) -> io::Result<Self> {
        match mode {
            Mode::Native => platform::Identity::capture(pid).map(Self::Native),
            Mode::Revalidated => platform::FallbackIdentity::capture(pid).map(Self::Revalidated),
        }
    }

    fn running(&self) -> io::Result<bool> {
        match self {
            Self::Native(identity) => identity.running(),
            Self::Revalidated(identity) => identity.running(),
        }
    }

    fn signal(&self, _sid: Pid, signal: Signal) -> io::Result<()> {
        match self {
            Self::Native(identity) => identity.signal(signal),
            Self::Revalidated(identity) => identity.signal_if_matches(_sid, signal).map(|_| ()),
        }
    }
}

/// Check non-reaping waits, process enumeration and identity support before spawn.
///
/// `force_fallback` exercises the revalidation path even when pidfds are available.
/// Otherwise only an unavailable pidfd syscall permits that path; permission or
/// other native capability failures are returned to the caller.
pub fn preflight(force_fallback: bool) -> io::Result<Mode> {
    let check = || {
        // The current process cannot be its own child. ECHILD proves these
        // non-reaping wait flags are supported and the syscall is permitted.
        // Reject before spawn if a sandbox/older OS denies the waiter API.
        loop {
            match rustix::process::waitid(
                rustix::process::WaitId::Pid(rustix::process::getpid()),
                rustix::process::WaitIdOptions::EXITED
                    | rustix::process::WaitIdOptions::NOWAIT
                    | rustix::process::WaitIdOptions::NOHANG,
            ) {
                Err(rustix::io::Errno::INTR) => continue,
                Err(rustix::io::Errno::CHILD) => break,
                Err(error) => return Err(io::Error::from(error)),
                Ok(_) => return Err(io::Error::other("unexpected self waitid result")),
            }
        }
        {
            let native_error = if force_fallback {
                io::Error::new(
                    io::ErrorKind::Unsupported,
                    "forced by the session test seam",
                )
            } else {
                match platform::preflight() {
                    Ok(()) => return Ok(Mode::Native),
                    Err(error) if platform::fallback_allowed(&error) => error,
                    Err(error) => return Err(error),
                }
            };
            let own = platform::FallbackIdentity::capture(rustix::process::getpid())?;
            own.running()?;
            rustix::process::getsid(None)?;
            platform::pids()?;
            // One warning per session, without command, environment, or PID data.
            tracing::warn!(reason = %native_error,
                "remote session uses process cleanup fallback; identity can change between revalidation and signal");
            Ok(Mode::Revalidated)
        }
    };
    check().map_err(|error| {
        io::Error::new(
            error.kind(),
            format!("safe remote-session process cleanup unavailable: {error}"),
        )
    })
}

/// Repeated scans include ordinary children forked during the preceding scan.
/// This is bounded lifecycle cleanup, not containment of a hostile process that
/// deliberately escapes its Unix session or continually forks.
///
/// The caller must own this session and retain its unreaped leader until this
/// call returns. Obtain `mode` with [`preflight`] before starting the child.
/// This blocking operation belongs outside an async executor thread.
pub fn terminate_session(sid: Pid, mode: Mode) -> io::Result<()> {
    let deadline = Instant::now() + CLEANUP_TIMEOUT;
    let mut empty_scans = 0;
    let mut first_error = None;
    loop {
        let mut found = false;
        for pid in platform::pids()? {
            if Instant::now() >= deadline {
                return Err(first_error.unwrap_or_else(|| {
                    io::Error::new(
                        io::ErrorKind::TimedOut,
                        "remote-session process cleanup deadline exceeded",
                    )
                }));
            }
            // Capture before the SID check: native handles remain bound to
            // this process; fallback captures supply the baseline revalidated
            // immediately before signaling, with the documented residual race.
            let captured = Identity::capture(mode, pid);
            let actual_sid = match session_id(pid) {
                Ok(actual_sid) => actual_sid,
                Err(error) => {
                    // A vanished process needs no signal. An unreadable or
                    // malformed Linux stat must not certify an empty session.
                    if cfg!(target_os = "linux")
                        && !matches!(error.raw_os_error(), Some(code) if code == rustix::io::Errno::NOENT.raw_os_error() || code == rustix::io::Errno::SRCH.raw_os_error())
                    {
                        first_error.get_or_insert(error);
                    }
                    continue;
                }
            };
            if actual_sid != Some(sid) {
                continue;
            }
            let identity = match captured {
                Ok(identity) => identity,
                Err(error) => {
                    // Processes owned by other users may be inaccessible;
                    // inability to capture one of our session's jobs is an
                    // actual cleanup failure, never an empty-scan success.
                    found = true;
                    if error.raw_os_error() != Some(rustix::io::Errno::SRCH.raw_os_error()) {
                        first_error.get_or_insert(error);
                    }
                    continue;
                }
            };
            match identity.running() {
                Ok(false) => continue,
                Ok(true) => {}
                Err(error) => {
                    first_error.get_or_insert(error);
                }
            }
            found = true;
            match identity.signal(sid, Signal::KILL) {
                Ok(()) => {}
                // A subsequent scan captures the remaining jobs; ESRCH alone
                // is never proof that this session is empty.
                Err(error)
                    if error.raw_os_error() == Some(rustix::io::Errno::SRCH.raw_os_error()) => {}
                Err(error) => {
                    first_error.get_or_insert(error);
                }
            }
        }
        empty_scans = if found { 0 } else { empty_scans + 1 };
        if empty_scans >= 2 {
            return first_error.map_or(Ok(()), Err);
        }
        std::thread::sleep(CLEANUP_RESCAN_INTERVAL);
    }
}

#[cfg(test)]
mod tests;
