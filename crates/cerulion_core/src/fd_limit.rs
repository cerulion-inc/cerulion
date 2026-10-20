// SPDX-License-Identifier: AGPL-3.0-only
//! The open-file limit (`RLIMIT_NOFILE`) of a process that hosts nodes.
//!
//! iceoryx2 keeps a file open for every port it creates (the port's lifetime
//! tag) next to the files behind each service, so the number of files a graph
//! opens while it starts grows with its ports. The process that starts a graph
//! holds every topic of the graph while it starts, so the soft limit of 1024
//! that most Linux logins hand out stops a graph of about 40 chained nodes.
//! iceoryx2 reports the exhausted limit as a port or service creation error
//! (`UnableToCreatePortTag`, `ResourceCreationFailed`, `InternalFailure`),
//! never as `EMFILE`.
//!
//! [`raise_open_file_limit`] raises the soft limit once per process. Every
//! transport constructor calls it before it creates the iceoryx2 node, so each
//! process that hosts nodes runs on the raised limit, and the processes it
//! starts afterwards inherit it. The soft limit goes up to the hard limit on
//! Linux, at most to [`LINUX_RAISE_CAP`]. Elsewhere iceoryx2 waits with
//! `select()`, which cannot watch a descriptor numbered `FD_SETSIZE` or
//! higher, so the soft limit stops at `FD_SETSIZE`, and on macOS also at
//! `kern.maxfilesperproc`, above which `setrlimit` fails with `EINVAL`. A
//! limit is never lowered.
//!
//! [`open_file_shortfall`] is the check `graph run` makes before it starts
//! anything: a graph whose estimated need is above the limit in force (and,
//! where iceoryx2 waits with `select()`, above `FD_SETSIZE`) is refused with
//! a message that names the limit and the fix.

use std::fmt;

/// Files the process that starts a graph holds open per output port at the
/// peak of the start burst (the topic's services and its ports' tags), rounded
/// up from the 14 that iceoryx2 0.10 opens.
pub const OPEN_FILES_PER_OUTPUT: u64 = 15;

/// Files the starting process holds open per input port at the peak of the
/// start burst (the subscriber's port tags and its trigger listener), rounded
/// up from the 22 that iceoryx2 0.10 opens.
pub const OPEN_FILES_PER_INPUT: u64 = 23;

/// Files the starting process holds before any port (the iceoryx2 node, the
/// runtime's own services, logs and sockets), rounded up from the 22 that
/// iceoryx2 0.10 opens.
pub const OPEN_FILES_BASE: u64 = 64;

/// The highest soft limit the raise sets on Linux. A hard limit above it
/// (container runtimes set up to 2^30) is left unused, so the processes this
/// one starts never inherit a soft limit that makes a loop over every
/// possible descriptor take seconds.
pub const LINUX_RAISE_CAP: u64 = 1 << 20;

/// A process's `RLIMIT_NOFILE` pair. An unlimited value (`RLIM_INFINITY`)
/// reads as [`u64::MAX`].
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct OpenFileLimit {
    /// The soft limit: the most files the process may hold open now.
    pub soft: u64,
    /// The hard limit: the most an unprivileged process may raise `soft` to.
    pub hard: u64,
}

/// What [`raise_open_file_limit`] found and did.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum RaiseOutcome {
    /// The soft limit was raised from `from` to the limit in `now`.
    Raised {
        /// The soft limit before the raise.
        from: u64,
        /// The limits after the raise.
        now: OpenFileLimit,
    },
    /// The soft limit was already at or above the ceiling this process may
    /// raise it to, so nothing changed.
    AlreadyAtCeiling(OpenFileLimit),
    /// `setrlimit` refused the raise to `target`; the limits are unchanged.
    Refused {
        /// The limits, unchanged.
        limit: OpenFileLimit,
        /// The soft limit that was asked for.
        target: u64,
        /// The `errno` `setrlimit` returned.
        errno: i32,
    },
    /// `getrlimit` failed with this `errno`, so the limit is unknown.
    Unreadable {
        /// The `errno` `getrlimit` returned.
        errno: i32,
    },
}

impl RaiseOutcome {
    /// The limits in force after the call, when they could be read.
    #[must_use]
    pub fn limit(&self) -> Option<OpenFileLimit> {
        match *self {
            Self::Raised { now, .. } => Some(now),
            Self::AlreadyAtCeiling(limit) | Self::Refused { limit, .. } => Some(limit),
            Self::Unreadable { .. } => None,
        }
    }
}

/// The soft limit to raise to, or `None` when `limit.soft` already reaches the
/// ceiling. The ceiling is `limit.hard`, lowered to `platform_cap` when there
/// is one. Never returns a value at or below `limit.soft`.
#[must_use]
pub fn raise_target(limit: OpenFileLimit, platform_cap: Option<u64>) -> Option<u64> {
    let ceiling = platform_cap.map_or(limit.hard, |cap| cap.min(limit.hard));
    (ceiling > limit.soft).then_some(ceiling)
}

/// The ports of a graph the estimate is made from.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct GraphPorts {
    /// The graph's node count (reported, not counted).
    pub nodes: usize,
    /// Output ports summed over the nodes.
    pub outputs: usize,
    /// Input ports summed over the nodes.
    pub inputs: usize,
}

/// The files a graph with these ports is estimated to open in the process
/// that starts it: [`OPEN_FILES_BASE`] plus [`OPEN_FILES_PER_OUTPUT`] per
/// output and [`OPEN_FILES_PER_INPUT`] per input.
#[must_use]
pub fn open_files_needed(ports: GraphPorts) -> u64 {
    let count = |n: usize| u64::try_from(n).unwrap_or(u64::MAX);
    OPEN_FILES_BASE
        .saturating_add(OPEN_FILES_PER_OUTPUT.saturating_mul(count(ports.outputs)))
        .saturating_add(OPEN_FILES_PER_INPUT.saturating_mul(count(ports.inputs)))
}

/// A graph that needs more open files than this process may hold.
/// Its `Display` is the refusal the user reads.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct OpenFileShortfall {
    /// The graph's ports.
    pub ports: GraphPorts,
    /// [`open_files_needed`] for `ports`.
    pub needed: u64,
    /// The limits after [`raise_open_file_limit`].
    pub limit: OpenFileLimit,
    /// The `select()` ceiling on platforms where iceoryx2 waits with
    /// `select()`; `None` on Linux.
    pub select_ceiling: Option<u64>,
}

/// `Some` when a graph with `ports` needs more open files than `limit.soft`,
/// or than `select_ceiling` where there is one (the platform's `select()`
/// ceiling, [`select_ceiling`] on this build): a descriptor numbered at or
/// above it cannot be waited on, whatever the soft limit allows.
#[must_use]
pub fn open_file_shortfall(
    ports: GraphPorts,
    limit: OpenFileLimit,
    select_ceiling: Option<u64>,
) -> Option<OpenFileShortfall> {
    let needed = open_files_needed(ports);
    let usable = select_ceiling.map_or(limit.soft, |cap| cap.min(limit.soft));
    (needed > usable).then_some(OpenFileShortfall {
        ports,
        needed,
        limit,
        select_ceiling,
    })
}

/// A limit as the refusal prints it: [`u64::MAX`] is `unlimited`.
struct ShownLimit(u64);

impl fmt::Display for ShownLimit {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        if self.0 == u64::MAX {
            f.write_str("unlimited")
        } else {
            write!(f, "{}", self.0)
        }
    }
}

impl fmt::Display for OpenFileShortfall {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let Self {
            ports,
            needed,
            limit,
            select_ceiling,
        } = *self;
        write!(
            f,
            "this graph has {nodes} nodes with {outputs} output and {inputs} input ports, and \
             starting it opens about {needed} files in this process (iceoryx2 keeps a file open \
             for every port), but the open-file limit (RLIMIT_NOFILE) is {soft} with a hard \
             limit of {hard}. ",
            nodes = ports.nodes,
            outputs = ports.outputs,
            inputs = ports.inputs,
            soft = ShownLimit(limit.soft),
            hard = ShownLimit(limit.hard),
        )?;
        match select_ceiling {
            Some(cap) if needed > cap => write!(
                f,
                "On this platform iceoryx2 waits with select(), which cannot watch a file \
                 descriptor numbered {cap} or higher, so a graph that needs more than {cap} \
                 files does not start here, whatever the limit. Run this graph on Linux."
            ),
            Some(_) if needed > limit.hard => write!(
                f,
                "Raise the hard limit to at least {needed} before the run: `ulimit -Hn {needed}` \
                 in a root shell, or `sudo launchctl limit maxfiles {needed} unlimited` on macOS."
            ),
            None if needed > limit.hard => write!(
                f,
                "Raise the hard limit to at least {needed} before the run: `ulimit -Hn {needed}` \
                 in a root shell, `LimitNOFILE={needed}` in a systemd unit, a `nofile` line in \
                 /etc/security/limits.conf, or `--ulimit nofile={needed}:{needed}` for a \
                 container."
            ),
            _ => write!(
                f,
                "The hard limit allows it but the soft limit was not raised that far; run \
                 `ulimit -Sn {needed}` in the shell that starts the graph."
            ),
        }
    }
}

/// The `select()` ceiling of this build: `FD_SETSIZE` on the unix targets
/// other than Linux, where iceoryx2 waits with `select()`; `None` on Linux,
/// where it waits with epoll, and off unix, where there is no
/// `RLIMIT_NOFILE`.
#[must_use]
pub fn select_ceiling() -> Option<u64> {
    #[cfg(any(target_os = "linux", not(unix)))]
    {
        None
    }
    #[cfg(all(unix, not(target_os = "linux")))]
    {
        u64::try_from(libc::FD_SETSIZE).ok()
    }
}

/// The cap below the hard limit on this build: [`LINUX_RAISE_CAP`] on Linux,
/// [`select_ceiling`] elsewhere, lowered on macOS to `kern.maxfilesperproc`.
#[cfg(unix)]
fn platform_cap() -> Option<u64> {
    #[cfg(target_os = "linux")]
    {
        Some(LINUX_RAISE_CAP)
    }
    #[cfg(target_os = "macos")]
    {
        let cap = select_ceiling();
        match max_files_per_proc() {
            Some(per_proc) => Some(cap.map_or(per_proc, |c| c.min(per_proc))),
            None => cap,
        }
    }
    #[cfg(all(not(target_os = "macos"), not(target_os = "linux")))]
    {
        select_ceiling()
    }
}

/// `kern.maxfilesperproc`, or `None` when the sysctl cannot be read.
#[cfg(target_os = "macos")]
fn max_files_per_proc() -> Option<u64> {
    let mut value: libc::c_int = 0;
    let mut len = std::mem::size_of::<libc::c_int>();
    // SAFETY: the name is a NUL-terminated literal; `value` and `len` are
    // locals that outlive the call, `len` holds the size of `value`, and no
    // new value is written (null pointer, zero length).
    let rc = unsafe {
        libc::sysctlbyname(
            c"kern.maxfilesperproc".as_ptr(),
            (&raw mut value).cast(),
            &raw mut len,
            std::ptr::null_mut(),
            0,
        )
    };
    if rc != 0 {
        return None;
    }
    u64::try_from(value).ok()
}

/// Reads this process's `RLIMIT_NOFILE`, or the `errno` of the failed read.
#[cfg(unix)]
pub fn current_open_file_limit() -> Result<OpenFileLimit, i32> {
    let mut rl = libc::rlimit {
        rlim_cur: 0,
        rlim_max: 0,
    };
    // SAFETY: `getrlimit` writes the two limits into `rl`, a local we own.
    if unsafe { libc::getrlimit(libc::RLIMIT_NOFILE, &mut rl) } != 0 {
        return Err(last_errno());
    }
    Ok(OpenFileLimit {
        soft: rlim_to_u64(rl.rlim_cur),
        hard: rlim_to_u64(rl.rlim_max),
    })
}

#[cfg(unix)]
fn last_errno() -> i32 {
    std::io::Error::last_os_error().raw_os_error().unwrap_or(0)
}

// `rlim_t` is `u64` on the 64-bit targets and narrower on some 32-bit ones,
// and `RLIM_INFINITY` differs per platform (`u64::MAX` on Linux, `i64::MAX`
// on macOS); both conversions map it to and from `u64::MAX`.
#[cfg(unix)]
#[allow(clippy::useless_conversion)] // the identity on 64-bit targets
fn rlim_to_u64(value: libc::rlim_t) -> u64 {
    if value == libc::RLIM_INFINITY {
        u64::MAX
    } else {
        u64::from(value)
    }
}

#[cfg(unix)]
#[allow(clippy::useless_conversion, clippy::unnecessary_fallible_conversions)] // the identity on 64-bit targets
fn u64_to_rlim(value: u64) -> libc::rlim_t {
    if value == u64::MAX {
        libc::RLIM_INFINITY
    } else {
        libc::rlim_t::try_from(value).unwrap_or(libc::RLIM_INFINITY)
    }
}

/// Raises this process's soft `RLIMIT_NOFILE` to the ceiling described in the
/// module docs. The first call does the work and logs the outcome once (debug
/// on success, warn when the raise is refused or the limit is unreadable);
/// every later call returns the first call's outcome. Thread-safe.
#[cfg(unix)]
pub fn raise_open_file_limit() -> RaiseOutcome {
    static OUTCOME: std::sync::OnceLock<RaiseOutcome> = std::sync::OnceLock::new();
    *OUTCOME.get_or_init(|| {
        let outcome = raise_once();
        log_outcome(&outcome);
        outcome
    })
}

#[cfg(unix)]
fn raise_once() -> RaiseOutcome {
    let limit = match current_open_file_limit() {
        Ok(limit) => limit,
        Err(errno) => return RaiseOutcome::Unreadable { errno },
    };
    let Some(target) = raise_target(limit, platform_cap()) else {
        return RaiseOutcome::AlreadyAtCeiling(limit);
    };
    let rl = libc::rlimit {
        rlim_cur: u64_to_rlim(target),
        rlim_max: u64_to_rlim(limit.hard),
    };
    // SAFETY: `setrlimit` reads `rl`, a local we own, and changes only this
    // process's limit.
    if unsafe { libc::setrlimit(libc::RLIMIT_NOFILE, &rl) } != 0 {
        return RaiseOutcome::Refused {
            limit,
            target,
            errno: last_errno(),
        };
    }
    RaiseOutcome::Raised {
        from: limit.soft,
        now: current_open_file_limit().unwrap_or(OpenFileLimit {
            soft: target,
            hard: limit.hard,
        }),
    }
}

#[cfg(unix)]
fn log_outcome(outcome: &RaiseOutcome) {
    match *outcome {
        RaiseOutcome::Raised { from, now } => tracing::debug!(
            from,
            soft = now.soft,
            hard = now.hard,
            "raised the open-file soft limit (RLIMIT_NOFILE)"
        ),
        RaiseOutcome::AlreadyAtCeiling(limit) => tracing::debug!(
            soft = limit.soft,
            hard = limit.hard,
            "open-file soft limit (RLIMIT_NOFILE) already at its ceiling"
        ),
        RaiseOutcome::Refused {
            limit,
            target,
            errno,
        } => tracing::warn!(
            soft = limit.soft,
            hard = limit.hard,
            target,
            errno,
            "setrlimit refused to raise the open-file soft limit (RLIMIT_NOFILE); a large \
             graph may fail to create its ports. Fix: raise it with `ulimit -Sn` before the run"
        ),
        RaiseOutcome::Unreadable { errno } => tracing::warn!(
            errno,
            "getrlimit could not read the open-file limit (RLIMIT_NOFILE); it was not raised"
        ),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn lim(soft: u64, hard: u64) -> OpenFileLimit {
        OpenFileLimit { soft, hard }
    }

    /// `groups` chains of camera -> detector -> tracker plus an imu: four
    /// nodes, four outputs and two inputs per group.
    fn chains(groups: usize) -> GraphPorts {
        GraphPorts {
            nodes: 4 * groups,
            outputs: 4 * groups,
            inputs: 2 * groups,
        }
    }

    #[test]
    fn raise_target_goes_to_the_hard_limit_under_the_cap() {
        assert_eq!(
            raise_target(lim(1024, 524_288), Some(1 << 20)),
            Some(524_288)
        );
        assert_eq!(raise_target(lim(1024, 524_288), None), Some(524_288));
    }

    #[test]
    fn raise_target_stops_at_the_platform_cap() {
        assert_eq!(raise_target(lim(256, u64::MAX), Some(1024)), Some(1024));
        assert_eq!(raise_target(lim(256, 512), Some(1024)), Some(512));
        assert_eq!(
            raise_target(lim(1024, 1_073_741_816), Some(1_048_576)),
            Some(1_048_576)
        );
    }

    #[test]
    fn raise_target_never_lowers_or_repeats_a_limit() {
        assert_eq!(raise_target(lim(524_288, 524_288), None), None);
        // A soft limit set above the cap by the user stays where it is.
        assert_eq!(raise_target(lim(1_048_576, u64::MAX), Some(1024)), None);
        assert_eq!(raise_target(lim(1024, 1024), Some(10_240)), None);
    }

    #[test]
    fn needed_files_are_sixty_four_plus_fifteen_per_output_and_twenty_three_per_input() {
        let ports = |outputs, inputs| GraphPorts {
            nodes: 0,
            outputs,
            inputs,
        };
        assert_eq!(open_files_needed(ports(0, 0)), 64);
        assert_eq!(open_files_needed(ports(1, 0)), 79);
        assert_eq!(open_files_needed(ports(0, 1)), 87);
        assert_eq!(open_files_needed(ports(80, 0)), 1264);
        assert_eq!(open_files_needed(chains(10)), 1124);
        assert_eq!(open_files_needed(chains(20)), 2184);
        assert_eq!(open_files_needed(ports(usize::MAX, 1)), u64::MAX);
    }

    #[test]
    fn a_graph_within_the_soft_limit_has_no_shortfall() {
        assert_eq!(open_file_shortfall(chains(5), lim(1024, 1024), None), None);
        assert_eq!(
            open_file_shortfall(chains(20), lim(524_288, 524_288), None),
            None
        );
        // Exactly at the limit is enough.
        assert_eq!(open_file_shortfall(chains(10), lim(1124, 1124), None), None);
    }

    #[test]
    fn a_hard_limit_below_the_need_names_both_limits_and_the_hard_limit_fixes() {
        let s = open_file_shortfall(chains(20), lim(1024, 1024), None).expect("shortfall");
        assert_eq!(s.needed, 2184);
        assert_eq!(
            s.to_string(),
            "this graph has 80 nodes with 80 output and 40 input ports, and starting it opens \
             about 2184 files in this process (iceoryx2 keeps a file open for every port), but \
             the open-file limit (RLIMIT_NOFILE) is 1024 with a hard limit of 1024. Raise the \
             hard limit to at least 2184 before the run: `ulimit -Hn 2184` in a root shell, \
             `LimitNOFILE=2184` in a systemd unit, a `nofile` line in /etc/security/limits.conf, \
             or `--ulimit nofile=2184:2184` for a container."
        );
    }

    #[test]
    fn a_soft_limit_left_below_a_sufficient_hard_limit_names_the_soft_fix() {
        let s = open_file_shortfall(chains(10), lim(1024, 524_288), None).expect("shortfall");
        assert_eq!(
            s.to_string(),
            "this graph has 40 nodes with 40 output and 20 input ports, and starting it opens \
             about 1124 files in this process (iceoryx2 keeps a file open for every port), but \
             the open-file limit (RLIMIT_NOFILE) is 1024 with a hard limit of 524288. The hard \
             limit allows it but the soft limit was not raised that far; run `ulimit -Sn 1124` \
             in the shell that starts the graph."
        );
    }

    #[test]
    fn a_need_above_the_select_ceiling_is_refused_even_under_a_high_soft_limit() {
        let need_1124 = chains(10);
        for soft in [1024, 10_240] {
            let s = open_file_shortfall(need_1124, lim(soft, u64::MAX), Some(1024))
                .expect("above the select() ceiling");
            assert!(
                s.to_string().ends_with(
                    "with a hard limit of unlimited. On this platform iceoryx2 waits with \
                     select(), which cannot watch a file descriptor numbered 1024 or higher, so \
                     a graph that needs more than 1024 files does not start here, whatever the \
                     limit. Run this graph on Linux."
                ),
                "{s}"
            );
        }
    }

    #[test]
    fn a_need_under_the_select_ceiling_but_above_a_low_hard_limit_names_the_macos_fix() {
        let s = open_file_shortfall(chains(5), lim(256, 256), Some(1024)).expect("shortfall");
        assert_eq!(s.needed, 594);
        assert!(
            s.to_string().ends_with(
                "Raise the hard limit to at least 594 before the run: `ulimit -Hn 594` in a root \
                 shell, or `sudo launchctl limit maxfiles 594 unlimited` on macOS."
            ),
            "{s}"
        );
    }

    #[test]
    fn the_outcome_reports_the_limit_in_force() {
        let now = lim(4096, 4096);
        assert_eq!(RaiseOutcome::Raised { from: 1024, now }.limit(), Some(now));
        assert_eq!(RaiseOutcome::AlreadyAtCeiling(now).limit(), Some(now));
        assert_eq!(
            RaiseOutcome::Refused {
                limit: now,
                target: 8192,
                errno: 1
            }
            .limit(),
            Some(now)
        );
        assert_eq!(RaiseOutcome::Unreadable { errno: 14 }.limit(), None);
    }
}
