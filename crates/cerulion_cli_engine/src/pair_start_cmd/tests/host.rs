// SPDX-License-Identifier: AGPL-3.0-only
//! The production serving host, [`NetdServing`], against an in-process
//! `cerulion-netd`: a thread on a temporary control socket that writes the
//! banner and answers each request line with a hand-written reply. No process
//! is spawned.

use std::io::{BufRead as _, BufReader, Write as _};
use std::os::unix::net::{UnixListener, UnixStream};
use std::path::{Path, PathBuf};
use std::sync::mpsc;
use std::time::Duration;

use super::super::run::ServingHost as _;
use super::super::{HoldRefusal, NetdProbe, NetdServing};

const WAIT: Duration = Duration::from_secs(5);

/// The control-socket variable pointed at a temporary socket for the guard's
/// life, under the crate's environment lock.
struct SocketEnv {
    previous: Option<std::ffi::OsString>,
    _lock: std::sync::MutexGuard<'static, ()>,
}

impl SocketEnv {
    fn set(socket: &Path) -> Self {
        let lock = crate::test_env::env_lock();
        let previous = std::env::var_os(cerulion_netd::hygiene::SOCKET_ENV);
        std::env::set_var(cerulion_netd::hygiene::SOCKET_ENV, socket);
        SocketEnv {
            previous,
            _lock: lock,
        }
    }
}

impl Drop for SocketEnv {
    fn drop(&mut self) {
        match self.previous.take() {
            Some(previous) => std::env::set_var(cerulion_netd::hygiene::SOCKET_ENV, previous),
            None => std::env::remove_var(cerulion_netd::hygiene::SOCKET_ENV),
        }
    }
}

/// One accepted connection: the banner for `protocol` written, its lines
/// read through `reader`.
struct Conn {
    stream: UnixStream,
    reader: BufReader<UnixStream>,
}

impl Conn {
    fn accept(listener: &UnixListener, protocol: u32) -> Self {
        let (mut stream, _) = listener.accept().unwrap();
        stream.set_read_timeout(Some(WAIT)).unwrap();
        let hello = cerulion_netd::Hello {
            protocol,
            ..cerulion_netd::Hello::new()
        };
        writeln!(stream, "{}", hello.to_json_line()).unwrap();
        let reader = BufReader::new(stream.try_clone().unwrap());
        Conn { stream, reader }
    }

    /// The next request line, or `None` once the client closed.
    fn request(&mut self) -> Option<serde_json::Value> {
        let mut line = String::new();
        match self.reader.read_line(&mut line).unwrap() {
            0 => None,
            _ => Some(serde_json::from_str(&line).unwrap()),
        }
    }

    fn reply(&mut self, line: &str) {
        writeln!(self.stream, "{line}").unwrap();
    }
}

fn socket_dir() -> (tempfile::TempDir, PathBuf) {
    let dir = tempfile::tempdir().unwrap();
    let socket = dir.path().join("netd.sock");
    (dir, socket)
}

#[test]
fn a_hold_keeps_its_connection_until_the_host_is_dropped() {
    let (_dir, socket) = socket_dir();
    let listener = UnixListener::bind(&socket).unwrap();
    let (said, heard) = mpsc::channel();
    let netd = std::thread::spawn(move || {
        let mut conn = Conn::accept(&listener, cerulion_netd::protocol::PROTOCOL_VERSION);
        let request = conn.request().expect("a hold request");
        assert_eq!(request["method"], "hold_serving");
        assert_eq!(request["ttl_s"], 790);
        let id = request["id"].as_u64().unwrap();
        conn.reply(&format!(
            r#"{{"id":{id},"state_root":"/srv/robot","ttl_s":790}}"#
        ));
        said.send("held").unwrap();
        // The lease is the connection: it lasts until the client closes it.
        assert!(conn.request().is_none(), "nothing more is asked");
        said.send("closed").unwrap();
    });
    let env = SocketEnv::set(&socket);
    let mut host = NetdServing::default();
    let held = host.hold(790).unwrap();
    assert_eq!(held.root, PathBuf::from("/srv/robot"));
    assert_eq!(heard.recv_timeout(WAIT), Ok("held"));
    assert!(
        heard.recv_timeout(Duration::from_millis(300)).is_err(),
        "the hold is kept while the host lives"
    );
    drop(host);
    assert_eq!(heard.recv_timeout(WAIT), Ok("closed"));
    drop(env);
    netd.join().unwrap();
}

#[test]
fn a_netd_older_than_the_hold_verb_is_named_by_the_protocol_its_banner_gave() {
    let (_dir, socket) = socket_dir();
    let listener = UnixListener::bind(&socket).unwrap();
    let netd = std::thread::spawn(move || {
        let mut conn = Conn::accept(&listener, 11);
        assert!(
            conn.request().is_none(),
            "nothing is asked of an older netd"
        );
    });
    let env = SocketEnv::set(&socket);
    let mut host = NetdServing::default();
    assert_eq!(host.hold(790), Err(HoldRefusal::TooOld { daemon: 11 }));
    drop(host);
    drop(env);
    netd.join().unwrap();
}

#[test]
fn a_probe_reports_the_startup_error_and_reconnects_after_a_failed_status() {
    let (_dir, socket) = socket_dir();
    let listener = UnixListener::bind(&socket).unwrap();
    let status = |id: u64, startup_error: Option<&str>| {
        let error = startup_error
            .map(|e| format!(r#","robot_startup_error":"{e}""#))
            .unwrap_or_default();
        format!(r#"{{"id":{id},"demands":[],"active_connections":2,"idle":false{error}}}"#)
    };
    let netd = std::thread::spawn(move || {
        let mut accepted = 0;
        // The first connection answers two probes, then closes before the
        // third is answered.
        let mut first = Conn::accept(&listener, cerulion_netd::protocol::PROTOCOL_VERSION);
        accepted += 1;
        for startup_error in [Some("remote server startup failed"), None] {
            let request = first.request().expect("a status request");
            assert_eq!(request["method"], "status");
            first.reply(&status(request["id"].as_u64().unwrap(), startup_error));
        }
        drop(first);
        // The next probe connects again.
        let mut second = Conn::accept(&listener, cerulion_netd::protocol::PROTOCOL_VERSION);
        accepted += 1;
        let request = second.request().expect("a status request");
        assert_eq!(request["method"], "status");
        second.reply(&status(request["id"].as_u64().unwrap(), None));
        assert!(second.request().is_none());
        accepted
    });
    let env = SocketEnv::set(&socket);
    let mut host = NetdServing::default();
    assert_eq!(
        host.probe(),
        NetdProbe::Status {
            startup_error: true
        }
    );
    assert_eq!(
        host.probe(),
        NetdProbe::Status {
            startup_error: false
        }
    );
    assert_eq!(host.probe(), NetdProbe::Unreachable);
    assert_eq!(
        host.probe(),
        NetdProbe::Status {
            startup_error: false
        }
    );
    drop(host);
    drop(env);
    assert_eq!(netd.join().unwrap(), 2, "a failed connection is not reused");
}

#[test]
fn a_probe_with_no_netd_listening_is_unreachable() {
    let (_dir, socket) = socket_dir();
    let _env = SocketEnv::set(&socket);
    let mut host = NetdServing::default();
    assert_eq!(host.probe(), NetdProbe::Unreachable);
    assert_eq!(host.probe(), NetdProbe::Unreachable);
}
