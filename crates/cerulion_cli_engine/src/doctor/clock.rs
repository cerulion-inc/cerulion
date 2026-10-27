// SPDX-License-Identifier: AGPL-3.0-only
//! `clock.sync`: whether the kernel's clock is synchronised, read from the
//! kernel itself.
//!
//! Calls `ntp_adjtime` once with `modes = 0`, which reads the kernel's clock
//! discipline state and changes nothing. The time daemons that steer the clock
//! (chrony, ntpd and systemd-timesyncd on Linux, timed on macOS) set that
//! state through the same interface, so the reading does not depend on which
//! one runs. `STA_UNSYNC` in `status` means no daemon has marked the clock
//! synchronised; `maxerror` and `esterror` are the kernel's bound on and
//! estimate of the clock's error in microseconds, and `offset` is the last
//! measured offset (nanoseconds under `STA_NANO`, else microseconds).

// The decision compiles and is tested on every host; only a Linux or macOS
// build calls it from `run`.
#![cfg_attr(not(any(target_os = "linux", target_os = "macos")), allow(dead_code))]

use super::{Check, Context, Outcome, Reading, Status};

pub(super) const SYNC: Check = Check {
    id: "clock.sync",
    why: "wire stamps and the latency Cerulion reports between machines compare two clocks, \
          so an unsynchronised clock shifts every cross-host time and ordering by its error.",
    network: false,
    run,
};

/// The kernel clock state as read.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(super) struct ClockState {
    /// `STA_UNSYNC` was set.
    pub(super) unsynchronised: bool,
    /// `maxerror`, microseconds.
    pub(super) max_error_us: i64,
    /// `esterror`, microseconds.
    pub(super) est_error_us: i64,
    /// The last measured offset, nanoseconds.
    pub(super) offset_ns: i64,
}

/// The platform the fix is written for.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(super) enum FixPlatform {
    Linux,
    MacOs,
}

/// The decision: `state` is the reading, or the `errno` of a failed call.
pub(super) fn decide(state: Result<ClockState, i32>, platform: FixPlatform) -> Outcome {
    let s = match state {
        Ok(s) => s,
        Err(errno) => {
            return Outcome::new(
                Status::Warn,
                format!("could not read the kernel clock state (ntp_adjtime errno {errno})"),
            )
            .with_fix(match platform {
                FixPlatform::Linux => "run `timedatectl` to see whether the clock is synchronised",
                FixPlatform::MacOs => {
                    "open System Settings, General, Date & Time to see whether the time is set \
                     automatically"
                }
            })
            .reading("errno", Reading::Int(u64::from(errno.unsigned_abs())));
        }
    };
    let base = |status, summary: String| {
        Outcome::new(status, summary)
            .reading("synchronised", Reading::Bool(!s.unsynchronised))
            .reading("max_error_us", Reading::Int(s.max_error_us.unsigned_abs()))
            .reading("est_error_us", Reading::Int(s.est_error_us.unsigned_abs()))
            .reading("offset_ns", Reading::Text(s.offset_ns.to_string()))
    };
    if s.unsynchronised {
        base(
            Status::Warn,
            "the kernel marks the clock unsynchronised (STA_UNSYNC is set)".to_string(),
        )
        .with_fix(match platform {
            FixPlatform::Linux => {
                "start a time daemon: `sudo timedatectl set-ntp true` (systemd-timesyncd), or \
                 `sudo systemctl enable --now chronyd` where chrony is installed; phc2sys \
                 clears the flag once its servo locks, so if phc2sys steers this clock from PTP, \
                 check its state (`pmc`, the phc2sys log) instead of starting a second daemon"
            }
            FixPlatform::MacOs => {
                "turn on \"Set time and date automatically\" in System Settings, General, Date \
                 & Time"
            }
        })
    } else {
        base(
            Status::Pass,
            format!(
                "synchronised; the kernel bounds the error at {} us (estimate {} us), last \
                 offset {} ns",
                s.max_error_us, s.est_error_us, s.offset_ns
            ),
        )
    }
}

/// The one `ntp_adjtime` call: a read with `modes = 0`.
#[cfg(any(target_os = "linux", target_os = "macos"))]
// The `timex` fields are `c_long`: `i64::from` is the identity on 64-bit
// targets and widens on 32-bit ones.
#[allow(clippy::useless_conversion)]
fn read() -> Result<ClockState, i32> {
    // SAFETY: `timex` is a plain C struct of integers, for which all zeroes
    // is a valid value; `tx` is a live, writable, correctly sized local the
    // call fills in, and `modes = 0` (from the zeroing) makes the call a read
    // that changes no kernel state.
    let (rc, tx) = unsafe {
        let mut tx: libc::timex = std::mem::zeroed();
        let rc = libc::ntp_adjtime(&mut tx);
        (rc, tx)
    };
    if rc < 0 {
        return Err(std::io::Error::last_os_error().raw_os_error().unwrap_or(0));
    }
    let offset = i64::from(tx.offset);
    Ok(ClockState {
        unsynchronised: tx.status & libc::STA_UNSYNC != 0,
        max_error_us: i64::from(tx.maxerror),
        est_error_us: i64::from(tx.esterror),
        offset_ns: if tx.status & libc::STA_NANO != 0 {
            offset
        } else {
            offset.saturating_mul(1000)
        },
    })
}

#[cfg(any(target_os = "linux", target_os = "macos"))]
fn run(_ctx: &Context) -> Outcome {
    let platform = if cfg!(target_os = "macos") {
        FixPlatform::MacOs
    } else {
        FixPlatform::Linux
    };
    decide(read(), platform)
}

#[cfg(not(any(target_os = "linux", target_os = "macos")))]
fn run(_ctx: &Context) -> Outcome {
    Outcome::not_applicable("this check reads the Linux and macOS kernel clock interface")
}

#[cfg(test)]
mod tests {
    use super::*;

    const SYNCED: ClockState = ClockState {
        unsynchronised: false,
        max_error_us: 77_614,
        est_error_us: 500,
        offset_ns: -6_446,
    };

    #[test]
    fn a_synchronised_clock_passes_with_its_error_bound() {
        let o = decide(Ok(SYNCED), FixPlatform::Linux);
        assert_eq!(o.status, Status::Pass);
        assert_eq!(
            o.summary,
            "synchronised; the kernel bounds the error at 77614 us (estimate 500 us), last \
             offset -6446 ns"
        );
        assert_eq!(o.fix, None);
        assert_eq!(o.readings.get("max_error_us"), Some(&Reading::Int(77_614)));
        assert_eq!(o.readings.get("synchronised"), Some(&Reading::Bool(true)));
    }

    #[test]
    fn an_unsynchronised_clock_warns_with_the_platform_fix() {
        let unsynced = ClockState {
            unsynchronised: true,
            ..SYNCED
        };
        let linux = decide(Ok(unsynced), FixPlatform::Linux);
        assert_eq!(linux.status, Status::Warn);
        assert_eq!(
            linux.summary,
            "the kernel marks the clock unsynchronised (STA_UNSYNC is set)"
        );
        assert!(linux.fix.unwrap().contains("sudo timedatectl set-ntp true"));
        let mac = decide(Ok(unsynced), FixPlatform::MacOs);
        assert!(mac
            .fix
            .unwrap()
            .contains("\"Set time and date automatically\""));
    }

    #[test]
    fn a_failed_read_warns_with_the_errno() {
        let o = decide(Err(1), FixPlatform::Linux);
        assert_eq!(o.status, Status::Warn);
        assert_eq!(
            o.summary,
            "could not read the kernel clock state (ntp_adjtime errno 1)"
        );
        assert_eq!(o.readings.get("errno"), Some(&Reading::Int(1)));
    }

    /// The read itself: unprivileged and with `modes = 0`, the call succeeds
    /// and returns a bound that is not negative.
    #[cfg(any(target_os = "linux", target_os = "macos"))]
    #[test]
    fn the_kernel_clock_state_reads_on_this_host() {
        let state = read().expect("ntp_adjtime with modes = 0 succeeds unprivileged");
        assert!(state.max_error_us >= 0, "{state:?}");
        assert!(state.est_error_us >= 0, "{state:?}");
    }
}
