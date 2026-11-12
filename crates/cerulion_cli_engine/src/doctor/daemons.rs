// SPDX-License-Identifier: AGPL-3.0-only
//! `daemon.netd` and `daemon.remoted`: the two daemons that carry this
//! machine's remote traffic.
//!
//! `daemon.netd` connects to the netd control socket
//! ([`cerulion_netd::default_socket_path`]) with
//! [`cerulion_netd::NetdClient::connect_existing_at`], which never starts a
//! daemon, and asks for its `status` once, with no retry, a request that
//! registers nothing. The open connection restarts a desk daemon's idle grace
//! and is counted among the daemon's connections, so the check reports the
//! connections other than its own. A daemon that closes the connection during
//! its idle exit is reported as not running.
//!
//! `daemon.remoted` looks for `cerulion-remoted` beside this `cerulion` and,
//! on Linux, for a running process whose `argv[0]` names it, read from
//! `/proc/<pid>/cmdline`. remoted has no local status socket, so the check
//! reports and does not judge.

// The netd decision compiles and is tested on every host; only a Unix build
// calls it from `run_netd`.
#![cfg_attr(not(unix), allow(dead_code))]

use std::path::Path;

use super::{Check, Context, Outcome, Reading, Status};

pub(super) const NETD: Check = Check {
    id: "daemon.netd",
    why: "cerulion-netd carries every remote topic of this machine: topic commands and the \
          graph gateway demand and mirror through it, and a stale or unreachable daemon \
          leaves them without remote topics.",
    network: false,
    run: run_netd,
};

pub(super) const REMOTED: Check = Check {
    id: "daemon.remoted",
    why: "cerulion-remoted serves this robot to paired desks over the remote plane; without \
          it running, `cerulion connect` from a desk off the LAN cannot reach this machine.",
    network: false,
    run: run_remoted,
};

/// What the netd probe found.
#[derive(Debug, Clone, PartialEq, Eq)]
pub(super) enum NetdReading {
    /// Nothing listens at the socket.
    NotRunning { socket: String },
    /// Connecting failed for another reason (permissions, a bad path).
    ConnectFailed { socket: String, error: String },
    /// A daemon answered the connection but not the status request as this
    /// client expects (another protocol version, a closed connection).
    StatusFailed {
        socket: String,
        /// The pid in the socket's pidfile, when there is one.
        pid: Option<u32>,
        protocol: Option<u32>,
        error: String,
    },
    /// The status answer.
    Running {
        socket: String,
        protocol: u32,
        /// Connections other than the check's own.
        other_connections: usize,
        demands: usize,
        lan_access: Option<String>,
    },
}

pub(super) fn decide_netd(r: &NetdReading) -> Outcome {
    match r {
        NetdReading::NotRunning { socket } => Outcome::new(
            Status::Info,
            format!(
                "cerulion-netd is not running at {socket}; topic commands and remote graph \
                 runs start it when they need it"
            ),
        )
        .reading("socket", Reading::Text(socket.clone()))
        .reading("running", Reading::Bool(false)),
        NetdReading::ConnectFailed { socket, error } => Outcome::new(
            Status::Warn,
            format!("could not connect to {socket}: {error}"),
        )
        .with_fix(format!(
            "check that this user owns the socket and its directory (`ls -ld {socket}`), or set \
             CERULION_NETD_SOCK to a short path this user owns"
        ))
        .reading("socket", Reading::Text(socket.clone())),
        NetdReading::StatusFailed {
            socket,
            pid,
            protocol,
            error,
        } => {
            let mut o = Outcome::new(
                Status::Warn,
                format!("cerulion-netd at {socket} answered, but not its status: {error}"),
            )
            .with_fix(match pid {
                Some(pid) => format!(
                    "stop that daemon (`kill {pid}`); the next topic command starts the one \
                     installed with this cerulion"
                ),
                None => format!(
                    "no netd pidfile names a running cerulion-netd for that socket: find what \
                     holds it with `lsof {socket}`"
                ),
            })
            .reading("socket", Reading::Text(socket.clone()));
            if let Some(p) = protocol {
                o = o.reading("protocol", Reading::Int(u64::from(*p)));
            }
            if let Some(p) = pid {
                o = o.reading("pid", Reading::Int(u64::from(*p)));
            }
            o
        }
        NetdReading::Running {
            socket,
            protocol,
            other_connections,
            demands,
            lan_access,
        } => {
            let posture = lan_access
                .as_deref()
                .map_or(String::new(), |p| format!(", LAN access {p}"));
            let mut o = Outcome::new(
                Status::Pass,
                format!(
                    "cerulion-netd is running (protocol {protocol}): {other_connections} other \
                     connection{}, {demands} demanded topic{}{posture}",
                    if *other_connections == 1 { "" } else { "s" },
                    if *demands == 1 { "" } else { "s" },
                ),
            )
            .reading("socket", Reading::Text(socket.clone()))
            .reading("running", Reading::Bool(true))
            .reading("protocol", Reading::Int(u64::from(*protocol)))
            .reading(
                "other_connections",
                Reading::Int(u64::try_from(*other_connections).unwrap_or(u64::MAX)),
            )
            .reading(
                "demands",
                Reading::Int(u64::try_from(*demands).unwrap_or(u64::MAX)),
            );
            if let Some(p) = lan_access {
                o = o.reading("lan_access", Reading::Text(p.clone()));
            }
            o
        }
    }
}

/// What the remoted look found. `running` is `None` where processes cannot
/// be listed (no `/proc`).
pub(super) fn decide_remoted(installed: Option<&str>, running: Option<usize>) -> Outcome {
    let installed_text = installed.map_or_else(
        || "cerulion-remoted is not installed beside cerulion".to_string(),
        |p| format!("cerulion-remoted is installed at {p}"),
    );
    let running_text = match running {
        Some(0) => "; no cerulion-remoted process is running".to_string(),
        Some(1) => "; 1 cerulion-remoted process is running".to_string(),
        Some(n) => format!("; {n} cerulion-remoted processes are running"),
        None => String::new(),
    };
    let mut o = Outcome::new(Status::Info, format!("{installed_text}{running_text}"))
        .reading("installed", Reading::Bool(installed.is_some()));
    if let Some(n) = running {
        o = o.reading(
            "running",
            Reading::Int(u64::try_from(n).unwrap_or(u64::MAX)),
        );
    }
    o
}

/// Whether `cmdline` (the NUL-separated `/proc/<pid>/cmdline`) starts with an
/// `argv[0]` whose file name is `cerulion-remoted`.
pub(super) fn cmdline_is_remoted(cmdline: &[u8]) -> bool {
    cmdline_argv0_is(cmdline, "cerulion-remoted")
}

/// Whether `cmdline` (the NUL-separated `/proc/<pid>/cmdline`) starts with an
/// `argv[0]` whose file name is `name`.
pub(super) fn cmdline_argv0_is(cmdline: &[u8], name: &str) -> bool {
    let argv0 = cmdline.split(|b| *b == 0).next().unwrap_or_default();
    Path::new(&*String::from_utf8_lossy(argv0))
        .file_name()
        .is_some_and(|n| n == name)
}

/// The pidfile's `pid` when `proc_root/<pid>/cmdline` names `cerulion-netd`
/// as `argv[0]`, else `None`. A pidfile a crashed netd left behind can name a
/// pid the kernel has since given to another process, so the `kill` fix is
/// offered only for a pid read back as netd.
pub(super) fn verified_netd_pid(proc_root: &Path, pid: Option<u32>) -> Option<u32> {
    let pid = pid?;
    let cmdline = std::fs::read(proc_root.join(pid.to_string()).join("cmdline")).ok()?;
    cmdline_argv0_is(&cmdline, "cerulion-netd").then_some(pid)
}

/// Processes under `proc_root` whose `argv[0]` is `cerulion-remoted`, or
/// `None` when `proc_root` cannot be listed.
pub(super) fn count_remoted(proc_root: &Path) -> Option<usize> {
    let entries = std::fs::read_dir(proc_root).ok()?;
    Some(
        entries
            .filter_map(Result::ok)
            .filter(|e| {
                e.file_name()
                    .to_str()
                    .is_some_and(|n| n.bytes().all(|b| b.is_ascii_digit()))
            })
            .filter(|e| {
                std::fs::read(e.path().join("cmdline")).is_ok_and(|c| cmdline_is_remoted(&c))
            })
            .count(),
    )
}

#[cfg(unix)]
fn run_netd(_ctx: &Context) -> Outcome {
    use cerulion_netd::client::{closed_while_exiting, is_not_running};
    use cerulion_netd::ClientError;
    let socket = cerulion_netd::default_socket_path();
    let shown = socket.display().to_string();
    // `/proc` exists only on Linux; elsewhere the pid stays unverified and
    // the fix names `lsof` instead of `kill`.
    let pid = verified_netd_pid(
        Path::new("/proc"),
        cerulion_hygiene::read_pidfile(&cerulion_hygiene::pidfile_for(&socket)),
    );
    // A read that timed out is a daemon that did not answer in time, which
    // the client's own error text does not say.
    let describe = |e: &ClientError| match e {
        ClientError::Io(io)
            if matches!(
                io.kind(),
                std::io::ErrorKind::TimedOut | std::io::ErrorKind::WouldBlock
            ) =>
        {
            format!("it did not answer within the client's round-trip bound ({io})")
        }
        other => other.to_string(),
    };
    let reading = match cerulion_netd::NetdClient::connect_existing_at(socket) {
        Err(ClientError::Connect { source, .. }) if is_not_running(&source) => {
            NetdReading::NotRunning { socket: shown }
        }
        Err(e) if closed_while_exiting(&e) => NetdReading::NotRunning { socket: shown },
        Err(e @ ClientError::Connect { .. }) => NetdReading::ConnectFailed {
            socket: shown,
            error: e.to_string(),
        },
        Err(e) => NetdReading::StatusFailed {
            socket: shown,
            pid,
            protocol: None,
            error: describe(&e),
        },
        Ok(mut client) => {
            let protocol = client.daemon_protocol();
            match client.status() {
                Ok(s) => running(shown, protocol, &s),
                Err(e) if closed_while_exiting(&e) => NetdReading::NotRunning { socket: shown },
                Err(e) => NetdReading::StatusFailed {
                    socket: shown,
                    pid,
                    protocol: Some(protocol),
                    error: describe(&e),
                },
            }
        }
    };
    decide_netd(&reading)
}

/// The reading for a `status` answer received on the check's own
/// connection, which the daemon counts among its connections.
#[cfg(unix)]
fn running(
    socket: String,
    protocol: u32,
    s: &cerulion_netd::protocol::StatusResponse,
) -> NetdReading {
    NetdReading::Running {
        socket,
        protocol,
        other_connections: s.active_connections.saturating_sub(1),
        demands: s.demands.len(),
        lan_access: s.lan_access.map(|p| p.as_str().to_string()),
    }
}

#[cfg(not(unix))]
fn run_netd(_ctx: &Context) -> Outcome {
    Outcome::not_applicable("cerulion-netd is a Unix daemon")
}

fn run_remoted(_ctx: &Context) -> Outcome {
    let installed = std::env::current_exe()
        .ok()
        .and_then(|exe| exe.parent().map(|d| d.join("cerulion-remoted")))
        .filter(|p| p.exists())
        .map(|p| p.display().to_string());
    let running = if cfg!(target_os = "linux") {
        count_remoted(Path::new("/proc"))
    } else {
        None
    };
    decide_remoted(installed.as_deref(), running)
}

#[cfg(test)]
mod tests {
    use super::*;

    const SOCK: &str = "/run/user/1000/cerulion/netd.sock";

    #[test]
    fn a_running_netd_passes_with_its_counts() {
        let o = decide_netd(&NetdReading::Running {
            socket: SOCK.to_string(),
            protocol: 7,
            other_connections: 2,
            demands: 3,
            lan_access: Some("paired".to_string()),
        });
        assert_eq!(o.status, Status::Pass);
        assert_eq!(
            o.summary,
            "cerulion-netd is running (protocol 7): 2 other connections, 3 demanded topics, LAN \
             access paired"
        );
        let one = decide_netd(&NetdReading::Running {
            socket: SOCK.to_string(),
            protocol: 7,
            other_connections: 1,
            demands: 1,
            lan_access: None,
        });
        assert_eq!(
            one.summary,
            "cerulion-netd is running (protocol 7): 1 other connection, 1 demanded topic"
        );
        assert_eq!(o.readings.get("protocol"), Some(&Reading::Int(7)));
    }

    #[cfg(unix)]
    #[test]
    fn the_status_answer_counts_connections_other_than_the_checks_own() {
        let status = cerulion_netd::protocol::StatusResponse {
            robot_startup_error: None,
            id: 1,
            demands: vec![cerulion_netd::protocol::DemandEntry {
                robot: "go2".to_string(),
                topic: "/scan".to_string(),
                local_topic: None,
                refcount: 1,
                plane: None,
            }],
            active_connections: 3,
            idle: false,
            connect_endpoints: None,
            lan_access: Some(cerulion_core::LanAccess::Open),
            egress: None,
        };
        assert_eq!(
            running(SOCK.to_string(), 7, &status),
            NetdReading::Running {
                socket: SOCK.to_string(),
                protocol: 7,
                other_connections: 2,
                demands: 1,
                lan_access: Some("open".to_string()),
            }
        );
        let alone = cerulion_netd::protocol::StatusResponse {
            robot_startup_error: None,
            active_connections: 0,
            demands: Vec::new(),
            ..status
        };
        assert!(matches!(
            running(SOCK.to_string(), 7, &alone),
            NetdReading::Running {
                other_connections: 0,
                ..
            }
        ));
    }

    #[test]
    fn a_netd_that_is_not_running_is_info() {
        let o = decide_netd(&NetdReading::NotRunning {
            socket: SOCK.to_string(),
        });
        assert_eq!(o.status, Status::Info);
        assert_eq!(o.fix, None);
        assert_eq!(o.readings.get("running"), Some(&Reading::Bool(false)));
    }

    #[test]
    fn an_unreachable_or_mismatched_netd_warns_with_a_fix() {
        let refused = decide_netd(&NetdReading::ConnectFailed {
            socket: SOCK.to_string(),
            error: "permission denied".to_string(),
        });
        assert_eq!(refused.status, Status::Warn);
        assert!(refused.fix.unwrap().contains("CERULION_NETD_SOCK"));
        let mismatched = decide_netd(&NetdReading::StatusFailed {
            socket: SOCK.to_string(),
            pid: Some(4242),
            protocol: Some(3),
            error: "bad banner".to_string(),
        });
        assert_eq!(mismatched.status, Status::Warn);
        assert_eq!(
            mismatched.fix.as_deref(),
            Some(
                "stop that daemon (`kill 4242`); the next topic command starts the one installed \
                 with this cerulion"
            )
        );
        assert_eq!(mismatched.readings.get("protocol"), Some(&Reading::Int(3)));
        let foreign = decide_netd(&NetdReading::StatusFailed {
            socket: SOCK.to_string(),
            pid: None,
            protocol: None,
            error: "not a netd banner".to_string(),
        });
        assert_eq!(
            foreign.fix.as_deref(),
            Some(
                "no netd pidfile names a running cerulion-netd for that socket: find what holds \
                 it with `lsof /run/user/1000/cerulion/netd.sock`"
            )
        );
    }

    #[test]
    fn remoted_is_reported_and_never_judged() {
        let o = decide_remoted(Some("/usr/bin/cerulion-remoted"), Some(1));
        assert_eq!(o.status, Status::Info);
        assert_eq!(
            o.summary,
            "cerulion-remoted is installed at /usr/bin/cerulion-remoted; 1 cerulion-remoted \
             process is running"
        );
        let none = decide_remoted(None, None);
        assert_eq!(none.status, Status::Info);
        assert_eq!(
            none.summary,
            "cerulion-remoted is not installed beside cerulion"
        );
        assert_eq!(none.readings.get("running"), None);
    }

    #[test]
    fn the_cmdline_match_reads_argv0_only() {
        assert!(cmdline_is_remoted(b"/usr/bin/cerulion-remoted\0--flag\0"));
        assert!(cmdline_is_remoted(b"cerulion-remoted"));
        assert!(!cmdline_is_remoted(b"/usr/bin/vim\0cerulion-remoted\0"));
        assert!(!cmdline_is_remoted(b"/usr/bin/cerulion-remoted-helper\0"));
        assert!(!cmdline_is_remoted(b""));
    }

    #[test]
    fn count_remoted_reads_numbered_entries() {
        let proc_root = tempfile::tempdir().unwrap();
        for (pid, cmd) in [
            ("10", &b"/usr/bin/cerulion-remoted\0"[..]),
            ("11", &b"/bin/sh\0"[..]),
            ("self", &b"/usr/bin/cerulion-remoted\0"[..]),
        ] {
            let d = proc_root.path().join(pid);
            std::fs::create_dir(&d).unwrap();
            std::fs::write(d.join("cmdline"), cmd).unwrap();
        }
        assert_eq!(count_remoted(proc_root.path()), Some(1));
        assert_eq!(count_remoted(&proc_root.path().join("absent")), None);
    }

    #[test]
    fn the_kill_fix_names_only_a_pid_read_back_as_netd() {
        let proc_root = tempfile::tempdir().unwrap();
        for (pid, cmd) in [
            ("20", &b"/usr/lib/cerulion/cerulion-netd\0--socket\0x\0"[..]),
            ("21", &b"/usr/sbin/sshd\0"[..]),
        ] {
            let d = proc_root.path().join(pid);
            std::fs::create_dir(&d).unwrap();
            std::fs::write(d.join("cmdline"), cmd).unwrap();
        }
        let root = proc_root.path();
        assert_eq!(verified_netd_pid(root, Some(20)), Some(20));
        // A stale pidfile whose pid now belongs to another process.
        assert_eq!(verified_netd_pid(root, Some(21)), None);
        // A pid with no process at all.
        assert_eq!(verified_netd_pid(root, Some(22)), None);
        assert_eq!(verified_netd_pid(root, None), None);
    }
}
