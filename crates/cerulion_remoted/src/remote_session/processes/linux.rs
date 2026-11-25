// SPDX-License-Identifier: AGPL-3.0-only
//! Linux pidfds bind cleanup signals to a process rather than a reusable PID.

use std::io;
use std::os::fd::OwnedFd;

use rustix::event::{poll, PollFd, PollFlags, Timespec};
use rustix::process::{pidfd_open, pidfd_send_signal, Pid, PidfdFlags, Signal};

pub(super) struct Identity(OwnedFd);

pub(super) fn fallback_allowed(error: &io::Error) -> bool {
    error.raw_os_error() == Some(rustix::io::Errno::NOSYS.raw_os_error())
}

pub(super) struct FallbackIdentity {
    pid: Pid,
    sid: Option<Pid>,
    start: [u64; 2],
    live: bool,
}

impl FallbackIdentity {
    pub(super) fn capture(pid: Pid) -> io::Result<Self> {
        let path = format!("/proc/{}/stat", pid.as_raw_nonzero());
        parse_stat(pid, &std::fs::read(path)?)
    }

    pub(super) fn running(&self) -> io::Result<bool> {
        Ok(self.live)
    }

    pub(super) fn signal_if_matches(&self, sid: Pid, signal: Signal) -> io::Result<bool> {
        let current = match Self::capture(self.pid) {
            Ok(current) => current,
            Err(error) if matches!(error.raw_os_error(), Some(code) if code == rustix::io::Errno::NOENT.raw_os_error() || code == rustix::io::Errno::SRCH.raw_os_error()) => {
                return Ok(false)
            }
            Err(error) => return Err(error),
        };
        if !current.live || current.sid != Some(sid) {
            return Ok(false);
        }
        let actual_sid = match session_id(self.pid) {
            Ok(Some(sid)) => sid,
            Ok(None) => return Ok(false),
            Err(error) if matches!(error.raw_os_error(), Some(code) if code == rustix::io::Errno::NOENT.raw_os_error() || code == rustix::io::Errno::SRCH.raw_os_error()) => {
                return Ok(false)
            }
            Err(error) => return Err(error),
        };
        if !super::same_process(sid, actual_sid, self.start, current.start) {
            return Ok(false);
        }
        // The final session/start-time check cannot be atomic with kill on old
        // kernels. The leader remains pinned, but this residual race remains.
        rustix::process::kill_process(self.pid, signal)?;
        Ok(true)
    }
}

/// Kernel threads have SID zero, which is not a user-session identifier.
/// Decode it without calling an API that represents every result as a nonzero PID.
pub(super) fn session_id(pid: Pid) -> io::Result<Option<Pid>> {
    Ok(FallbackIdentity::capture(pid)?.sid)
}

fn parse_stat(pid: Pid, bytes: &[u8]) -> io::Result<FallbackIdentity> {
    let invalid = || {
        io::Error::new(
            io::ErrorKind::InvalidData,
            "invalid Linux process stat record",
        )
    };
    let first_space = bytes
        .iter()
        .position(|byte| *byte == b' ')
        .ok_or_else(invalid)?;
    let reported_pid = std::str::from_utf8(&bytes[..first_space])
        .ok()
        .and_then(|value| value.parse::<i32>().ok());
    if reported_pid != Some(pid.as_raw_nonzero().get()) {
        return Err(invalid());
    }
    // comm may contain ')' and non-UTF8 bytes (including a truncated multibyte
    // name). Only the ASCII fields after its final ')' are decoded.
    let end = bytes
        .iter()
        .rposition(|byte| *byte == b')')
        .ok_or_else(invalid)?;
    let fields: Vec<&[u8]> = bytes[end + 1..]
        .split(u8::is_ascii_whitespace)
        .filter(|field| !field.is_empty())
        .collect();
    let state = fields
        .first()
        .filter(|field| field.len() == 1)
        .ok_or_else(invalid)?[0];
    let number = |index: usize| {
        fields
            .get(index)
            .and_then(|field| std::str::from_utf8(field).ok())
            .and_then(|value| value.parse::<u64>().ok())
            .ok_or_else(invalid)
    };
    let raw_sid = i32::try_from(number(3)?).map_err(|_| invalid())?;
    let sid = Pid::from_raw(raw_sid);
    // A zombie thread-group leader can still have running worker threads.
    // num_threads (field 20) includes the leader; retain those groups for cleanup.
    let live = match state {
        b'Z' => number(17)? > 1,
        b'X' => false,
        _ => true,
    };
    Ok(FallbackIdentity {
        pid,
        sid,
        start: [number(19)?, 0],
        live,
    })
}

impl Identity {
    pub(super) fn capture(pid: Pid) -> io::Result<Self> {
        pidfd_open(pid, PidfdFlags::empty())
            .map(Self)
            .map_err(Into::into)
    }

    pub(super) fn signal(&self, signal: Signal) -> io::Result<()> {
        pidfd_send_signal(&self.0, signal).map_err(Into::into)
    }

    pub(super) fn running(&self) -> io::Result<bool> {
        let mut fds = [PollFd::new(&self.0, PollFlags::IN)];
        let timeout = Timespec {
            tv_sec: 0,
            tv_nsec: 0,
        };
        loop {
            match poll(&mut fds, Some(&timeout)) {
                Err(rustix::io::Errno::INTR) => continue,
                result => {
                    result?;
                    break;
                }
            }
        }
        Ok(fds[0].revents().is_empty())
    }
}

pub(super) fn preflight() -> io::Result<()> {
    Identity::capture(rustix::process::getpid())?
        .running()
        .map_err(required_capability)?;
    // A pipe cannot name a process, so this probes the actual cleanup signal
    // syscall without delivering a signal. Its expected rejection is EBADF;
    // sandbox denials must refuse native cleanup before the child is spawned.
    let (pipe, _) = io::pipe().map_err(required_capability)?;
    signal_probe(pidfd_send_signal(&pipe, Signal::KILL))?;
    // /proc is also required for enumerating the other job-control groups.
    pids().map_err(required_capability)?;
    Ok(())
}

// Only pidfd open/send ENOSYS permits fallback. Preserve other failures' kinds,
// but wrap their errno so denied readiness or pipe setup cannot select fallback.
fn required_capability(error: io::Error) -> io::Error {
    io::Error::new(error.kind(), error)
}

fn signal_probe(result: rustix::io::Result<()>) -> io::Result<()> {
    match result {
        Err(rustix::io::Errno::BADF) => Ok(()),
        Err(error) => Err(error.into()),
        Ok(()) => Err(io::Error::other(
            "unexpected pidfd signal result for a pipe",
        )),
    }
}

pub(super) fn pids() -> io::Result<Vec<Pid>> {
    let mut pids = Vec::new();
    for entry in std::fs::read_dir("/proc")? {
        let entry = entry?;
        if let Some(pid) = entry
            .file_name()
            .to_str()
            .and_then(|name| name.parse().ok())
            .and_then(Pid::from_raw)
        {
            pids.push(pid);
        }
    }
    Ok(pids)
}

#[cfg(test)]
mod tests {
    use super::*;

    struct Child(std::process::Child);

    impl Drop for Child {
        fn drop(&mut self) {
            let _ = self.0.kill();
            let _ = self.0.wait();
        }
    }

    #[test]
    fn signal_probe_refuses_permission_errors_and_unexpected_success() {
        for _ in 0..2 {
            signal_probe(Err(rustix::io::Errno::BADF)).unwrap();
            for errno in [
                rustix::io::Errno::PERM,
                rustix::io::Errno::ACCESS,
                rustix::io::Errno::INVAL,
                rustix::io::Errno::NOSYS,
            ] {
                let error = signal_probe(Err(errno)).unwrap_err();
                assert_eq!(error.raw_os_error(), Some(errno.raw_os_error()));
                assert_eq!(fallback_allowed(&error), errno == rustix::io::Errno::NOSYS);
                let required = required_capability(io::Error::from(errno));
                assert_eq!(required.kind(), error.kind());
                assert!(!fallback_allowed(&required));
            }
            let error = signal_probe(Ok(())).unwrap_err();
            assert_eq!(error.kind(), io::ErrorKind::Other);
            assert!(!fallback_allowed(&error));
        }
    }

    #[test]
    fn signal_capability_probe_cannot_deliver_to_a_process() {
        let (pipe, _) = io::pipe().unwrap();
        assert_eq!(
            pidfd_send_signal(&pipe, Signal::KILL),
            Err(rustix::io::Errno::BADF)
        );
    }

    #[test]
    fn fallback_stat_accepts_non_utf8_and_parentheses_in_comm() {
        let pid = Pid::from_raw(42).unwrap();
        let record = b"42 (name) with \xff) S 1 42 17 0 0 0 0 0 0 0 0 0 0 0 20 0 1 0 424242 0\n";
        let identity = parse_stat(pid, record).unwrap();
        assert_eq!(identity.sid, Pid::from_raw(17));
        assert_eq!(identity.start, [424242, 0]);
        assert!(identity.live);
    }

    #[test]
    fn fallback_stat_keeps_zombie_leader_live_while_workers_remain() {
        // Literal proc stat fields: state=3, session=6, num_threads=20, starttime=22.
        let cases: &[(&[u8], bool)] = &[
            (
                b"42 (job) Z 1 42 17 0 0 0 0 0 0 0 0 0 0 0 20 0 1 0 424242 0\n",
                false,
            ),
            (
                b"42 (job) Z 1 42 17 0 0 0 0 0 0 0 0 0 0 0 20 0 2 0 424242 0\n",
                true,
            ),
            (
                b"42 (job) Z 1 42 17 0 0 0 0 0 0 0 0 0 0 0 20 0 7 0 424242 0\n",
                true,
            ),
            (
                b"42 (job) X 1 42 17 0 0 0 0 0 0 0 0 0 0 0 20 0 1 0 424242 0\n",
                false,
            ),
            (
                b"42 (job) S 1 42 17 0 0 0 0 0 0 0 0 0 0 0 20 0 1 0 424242 0\n",
                true,
            ),
        ];
        let pid = Pid::from_raw(42).unwrap();
        for &(record, expected_running) in cases {
            let identity = parse_stat(pid, record).unwrap();
            assert_eq!(identity.sid, Pid::from_raw(17));
            assert_eq!(identity.start, [424242, 0]);
            assert_eq!(identity.running().unwrap(), expected_running);
        }
    }

    #[test]
    fn kernel_thread_stat_has_no_user_session() {
        // Independent proc-stat vector: PID 2, session 0, one live kernel thread.
        let record = b"2 (kthreadd) S 0 0 0 0 -1 2129984 0 0 0 0 0 147 0 0 20 0 1 0 22 0\n";
        let identity = parse_stat(Pid::from_raw(2).unwrap(), record).unwrap();
        assert_eq!(identity.sid, None);
        assert!(identity.live);
        assert_eq!(identity.start, [22, 0]);
        assert_ne!(identity.sid, Pid::from_raw(17));
    }

    #[test]
    fn malformed_session_ids_refuse_before_cleanup_classification() {
        for sid in ["-1", "2147483648", "18446744073709551616", "invalid"] {
            let record = format!("42 (job) S 1 42 {sid} 0 0 0 0 0 0 0 0 0 0 0 20 0 1 0 424242 0\n");
            let error = parse_stat(Pid::from_raw(42).unwrap(), record.as_bytes())
                .err()
                .expect("invalid session IDs must refuse classification");
            assert_eq!(error.kind(), io::ErrorKind::InvalidData);
        }
    }

    #[test]
    fn session_scan_handles_kernel_threads_and_still_finds_the_current_process() {
        let own = rustix::process::getpid();
        let expected_sid = rustix::process::getsid(None).unwrap();
        // PID namespaces need not expose the native kernel process. On a native
        // host this independent prefix identifies its zero parent/group/session.
        let kernel_pid_two = std::fs::read("/proc/2/stat")
            .ok()
            .and_then(|bytes| bytes.strip_prefix(b"2 (kthreadd) ").map(<[u8]>::to_vec))
            .is_some_and(|tail| {
                let fields = tail.split(u8::is_ascii_whitespace).collect::<Vec<_>>();
                fields
                    .get(1..4)
                    .is_some_and(|ids| ids.iter().all(|id| *id == b"0"))
            });
        let mut saw_own = false;
        let mut saw_kernel_pid_two = false;
        for pid in pids().unwrap() {
            let result = session_id(pid);
            if kernel_pid_two && pid.as_raw_nonzero().get() == 2 {
                assert_eq!(result.unwrap(), None);
                saw_kernel_pid_two = true;
            } else if pid == own {
                assert_eq!(result.unwrap(), Some(expected_sid));
                saw_own = true;
            } else if let Err(error) = result {
                // Concurrent exit or a restricted unrelated process is expected.
                assert!(
                    matches!(
                        error.kind(),
                        io::ErrorKind::NotFound | io::ErrorKind::PermissionDenied
                    ) || error.raw_os_error() == Some(rustix::io::Errno::SRCH.raw_os_error())
                );
            }
        }
        assert!(saw_own);
        if kernel_pid_two {
            assert!(saw_kernel_pid_two);
            eprintln!("classified native kernel PID 2 as having no user session");
        } else {
            eprintln!("native kernel PID 2 is not exposed by this process namespace");
        }
    }

    #[test]
    fn fallback_zombie_stat_rejects_invalid_thread_count() {
        let records: &[&[u8]] = &[
            b"42 (job) Z 1 42 17 0 0 0 0 0 0 0 0 0 0 0 20 0 invalid 0 424242 0\n",
            b"42 (job) Z 1 42 17 0 0 0 0 0 0 0 0 0 0 0 20 0 -1 0 424242 0\n",
            b"42 (job) Z 1 42 17 0 0 0 0 0 0 0 0 0 0 0 20 0 18446744073709551616 0 424242 0\n",
        ];
        for record in records {
            let error = parse_stat(Pid::from_raw(42).unwrap(), record)
                .err()
                .expect("invalid thread count must refuse cleanup classification");
            assert_eq!(error.kind(), io::ErrorKind::InvalidData);
        }
    }

    #[test]
    fn fallback_mismatch_skips_a_real_live_child() {
        let mut child = Child(
            std::process::Command::new("/bin/sleep")
                .arg("10")
                .spawn()
                .unwrap(),
        );
        let pid = Pid::from_raw(child.0.id() as i32).unwrap();
        let sid = rustix::process::getsid(Some(pid)).unwrap();
        let wrong_sid = Pid::from_raw(if sid.as_raw_nonzero().get() == 1 {
            2
        } else {
            1
        })
        .unwrap();
        let mut identity = FallbackIdentity::capture(pid).unwrap();
        assert!(!identity.signal_if_matches(wrong_sid, Signal::KILL).unwrap());
        assert!(child.0.try_wait().unwrap().is_none());
        identity.start[0] ^= 1;
        assert!(!identity.signal_if_matches(sid, Signal::KILL).unwrap());
        assert!(child.0.try_wait().unwrap().is_none());
        // A fresh matching identity really signals, so the skipped calls above
        // are not explained by a broken or no-op signaling implementation.
        assert!(FallbackIdentity::capture(pid)
            .unwrap()
            .signal_if_matches(sid, Signal::KILL)
            .unwrap());
        child.0.wait().unwrap();
    }

    #[test]
    fn reaped_child_identity_cannot_signal_a_reused_pid() {
        match preflight() {
            Ok(()) => {}
            Err(error) if fallback_allowed(&error) => return,
            Err(error) => panic!("native cleanup capability failed unexpectedly: {error}"),
        }
        let mut child = std::process::Command::new("/bin/sleep")
            .arg("10")
            .spawn()
            .unwrap();
        let pid = Pid::from_raw(child.id() as i32).unwrap();
        let identity = Identity::capture(pid).unwrap();
        assert!(identity.running().unwrap());
        child.kill().unwrap();
        child.wait().unwrap();
        assert!(!identity.running().unwrap());
        assert_eq!(
            identity.signal(Signal::KILL).unwrap_err().raw_os_error(),
            Some(rustix::io::Errno::SRCH.raw_os_error())
        );
    }
}
