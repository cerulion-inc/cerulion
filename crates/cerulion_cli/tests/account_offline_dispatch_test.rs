// SPDX-License-Identifier: AGPL-3.0-only
//! Real CLI offline/completion dispatch: no account request or netd spawn.
//! HTTP and executable sentinels have positive controls; neither impersonates
//! a working service. Each child owns an isolated home, cwd and socket path.

#![cfg(unix)]

mod mp_support;

use std::io::{Read, Write};
use std::net::{Shutdown, SocketAddr, TcpListener, TcpStream};
use std::os::unix::fs::PermissionsExt;
use std::path::PathBuf;
use std::process::{Command, Output, Stdio};
use std::sync::atomic::{AtomicBool, AtomicUsize, Ordering};
use std::sync::Arc;
use std::thread::JoinHandle;
use std::time::Duration;

use cerulion_cli_engine::auth;
use iceoryx2::prelude::SemanticString;
use mp_support::ChildGuard;

struct HttpSentinel {
    address: SocketAddr,
    accepted: Arc<AtomicUsize>,
    stop: Arc<AtomicBool>,
    worker: Option<JoinHandle<()>>,
}

impl HttpSentinel {
    fn start() -> Self {
        let listener = TcpListener::bind("127.0.0.1:0").unwrap();
        listener.set_nonblocking(true).unwrap();
        let address = listener.local_addr().unwrap();
        let accepted = Arc::new(AtomicUsize::new(0));
        let stop = Arc::new(AtomicBool::new(false));
        let count = Arc::clone(&accepted);
        let shutdown = Arc::clone(&stop);
        let worker = std::thread::spawn(move || loop {
            match listener.accept() {
                Ok((mut stream, _)) => {
                    count.fetch_add(1, Ordering::SeqCst);
                    // On macOS an accepted socket inherits the listener's
                    // non-blocking mode, so the read timeout would not apply.
                    // Read the request through the end of its headers before
                    // the reply: request bytes that are unread at close, or
                    // that arrive after it, reset the connection and fail the
                    // client's read.
                    stream.set_nonblocking(false).unwrap();
                    stream
                        .set_read_timeout(Some(Duration::from_millis(100)))
                        .unwrap();
                    stream
                        .set_write_timeout(Some(Duration::from_millis(100)))
                        .unwrap();
                    let mut request = Vec::new();
                    let mut chunk = [0; 2048];
                    while request.len() < chunk.len()
                        && !request.windows(4).any(|end| end == b"\r\n\r\n")
                    {
                        match stream.read(&mut chunk) {
                            Ok(0) | Err(_) => break,
                            Ok(read) => request.extend_from_slice(&chunk[..read]),
                        }
                    }
                    let _ = stream.write_all(b"HTTP/1.1 503 Service Unavailable\r\nContent-Length: 0\r\nConnection: close\r\n\r\n");
                    let _ = stream.shutdown(Shutdown::Write);
                }
                Err(error) if error.kind() == std::io::ErrorKind::WouldBlock => {
                    // Drain already queued connections before stopping.
                    if shutdown.load(Ordering::SeqCst) {
                        break;
                    }
                    std::thread::sleep(Duration::from_millis(2));
                }
                Err(error) => panic!("HTTP sentinel accept: {error}"),
            }
        });
        let sentinel = Self {
            address,
            accepted,
            stop,
            worker: Some(worker),
        };
        let mut client = TcpStream::connect(address).unwrap();
        client
            .set_read_timeout(Some(Duration::from_secs(2)))
            .unwrap();
        client
            .write_all(b"GET /positive-control HTTP/1.1\r\nHost: localhost\r\n\r\n")
            .unwrap();
        let mut response = String::new();
        client.read_to_string(&mut response).unwrap();
        assert!(response.starts_with("HTTP/1.1 503 "));
        assert_eq!(sentinel.accepted.swap(0, Ordering::SeqCst), 1);
        sentinel
    }

    fn finish(&mut self) -> usize {
        self.stop.store(true, Ordering::SeqCst);
        if let Some(worker) = self.worker.take() {
            worker.join().unwrap();
        }
        self.accepted.load(Ordering::SeqCst)
    }
}

impl Drop for HttpSentinel {
    fn drop(&mut self) {
        self.finish();
    }
}

struct Fixture {
    root: tempfile::TempDir,
    home: PathBuf,
    account: PathBuf,
    socket: PathBuf,
    spawn_marker: PathBuf,
    spawn_sentinel: PathBuf,
    http: HttpSentinel,
}

impl Fixture {
    fn new() -> Self {
        let root = tempfile::tempdir_in("/tmp").unwrap();
        let home = root.path().join("home");
        let account = home.join(".cerulion");
        std::fs::create_dir_all(&account).unwrap();
        let now = auth::now_unix_ns();
        auth::write_to(
            &account.join("auth.json"),
            &auth::AuthState {
                account_id: "AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA".into(),
                session_token: "offline-dispatch-session".into(),
                refresh_token: "offline-dispatch-refresh".into(),
                expires_at_ns: now + 600_000_000_000,
                logged_in_ever: true,
                role: None,
            },
        )
        .unwrap();
        std::fs::write(account.join("desk.key"), [0x42; 32]).unwrap();
        std::fs::set_permissions(
            account.join("desk.key"),
            std::fs::Permissions::from_mode(0o600),
        )
        .unwrap();
        // Local completion must remain useful while account networking is forbidden.
        std::fs::write(account.join("peers.json"), serde_json::to_vec(&serde_json::json!({
            "v": 1, "peers": [{"robot": "cached_robot", "locator": "tcp/127.0.0.1:9", "last_seen": now / 1_000_000_000}]
        })).unwrap()).unwrap();
        // Project-local iceoryx config isolates the empty local listing from the desk.
        std::fs::create_dir(root.path().join("config")).unwrap();
        let prefix = format!(
            "offline_{}_",
            root.path().file_name().unwrap().to_str().unwrap()
        );
        std::fs::write(
            root.path().join("config/iceoryx2.toml"),
            format!(
                "[global]\nroot-path = {:?}\nprefix = {prefix:?}\n",
                root.path().join("iox").to_str().unwrap()
            ),
        )
        .unwrap();
        let config_path = root.path().join("config/iceoryx2.toml");
        let parsed = iceoryx2::config::Config::from_file(
            &iceoryx2::prelude::FilePath::new(config_path.to_str().unwrap().as_bytes()).unwrap(),
        )
        .unwrap();
        assert_eq!(
            parsed.global.root_path().as_bytes(),
            root.path().join("iox").to_str().unwrap().as_bytes()
        );
        assert_eq!(parsed.global.prefix.as_bytes(), prefix.as_bytes());
        let spawn_marker = root.path().join("spawned");
        let spawn_sentinel = root.path().join("spawn-sentinel");
        std::fs::write(
            &spawn_sentinel,
            "#!/bin/sh\n: > \"$CERULION_TEST_SPAWN_MARKER\"\nexit 78\n",
        )
        .unwrap();
        std::fs::set_permissions(&spawn_sentinel, std::fs::Permissions::from_mode(0o700)).unwrap();
        let status = Command::new(&spawn_sentinel)
            .env("CERULION_TEST_SPAWN_MARKER", &spawn_marker)
            .status()
            .unwrap();
        assert_eq!(status.code(), Some(78));
        assert!(spawn_marker.is_file());
        std::fs::remove_file(&spawn_marker).unwrap();
        Self {
            socket: root.path().join("netd.sock"),
            root,
            home,
            account,
            spawn_marker,
            spawn_sentinel,
            http: HttpSentinel::start(),
        }
    }

    fn command(&self) -> Command {
        let mut command = Command::new(env!("CARGO_BIN_EXE_cerulion"));
        command
            .env_clear()
            .current_dir(self.root.path())
            .env("HOME", &self.home)
            .env("CERULION_HOME", &self.account)
            .env(
                "CERULION_ACCOUNT_SERVICE",
                format!("http://{}", self.http.address),
            )
            .env("CERULION_NETD_SOCK", &self.socket)
            .env("CERULION_NETD_SOCKET", &self.socket)
            .env("CERULION_NETD_BIN", &self.spawn_sentinel)
            .env("CERULION_TEST_SPAWN_MARKER", &self.spawn_marker)
            .env("NO_COLOR", "1")
            .env("RUST_LOG", "error")
            .env("IOX2_LOG_LEVEL", "error")
            .stdin(Stdio::null());
        command
    }

    fn run(&mut self, mut command: Command) -> Output {
        let stdout = self.root.path().join("stdout");
        let stderr = self.root.path().join("stderr");
        let mut child = ChildGuard::single_process(
            command
                .stdout(std::fs::File::create(&stdout).unwrap())
                .stderr(std::fs::File::create(&stderr).unwrap())
                .spawn()
                .unwrap(),
        );
        let status = child
            .wait_bounded(Duration::from_secs(10))
            .unwrap_or_else(|| {
                panic!(
                    "CLI deadline: {}",
                    std::fs::read_to_string(&stderr).unwrap()
                )
            });
        let out = Output {
            status,
            stdout: std::fs::read(stdout).unwrap(),
            stderr: std::fs::read(stderr).unwrap(),
        };
        assert_eq!(self.http.finish(), 0, "child contacted the account service");
        assert!(!self.spawn_marker.exists(), "child attempted to spawn netd");
        assert!(!self.socket.exists(), "child created a daemon socket");
        out
    }
}

#[test]
fn offline_topic_list_never_contacts_account_service_or_spawns_netd() {
    let mut fixture = Fixture::new();
    let mut command = fixture.command();
    command.args(["topic", "list", "--no-network"]);
    let out = fixture.run(command);
    assert_eq!(
        out.status.code(),
        Some(0),
        "{}",
        String::from_utf8_lossy(&out.stderr)
    );
    assert_eq!(out.stdout, b"No active local topics.\n");
    assert!(
        out.stderr.is_empty(),
        "{}",
        String::from_utf8_lossy(&out.stderr)
    );
}

#[test]
fn robot_completion_is_local_and_silent_under_hostile_logging() {
    let mut fixture = Fixture::new();
    let mut command = fixture.command();
    command
        .env("COMPLETE", "bash")
        .env("_CLAP_IFS", "\u{0b}")
        .env("_CLAP_COMPLETE_INDEX", "3")
        .env("RUST_LOG", "trace")
        .env("IOX2_LOG_LEVEL", "notalevel")
        .args(["--", "cerulion", "viz", "--robot", ""]);
    let out = fixture.run(command);
    assert_eq!(
        out.status.code(),
        Some(0),
        "{}",
        String::from_utf8_lossy(&out.stderr)
    );
    assert!(
        out.stderr.is_empty(),
        "{}",
        String::from_utf8_lossy(&out.stderr)
    );
    let stdout = String::from_utf8(out.stdout).unwrap();
    let candidates: Vec<_> = stdout
        .split('\u{0b}')
        .filter(|value| !value.is_empty())
        .collect();
    assert_eq!(candidates, ["cached_robot"]);
}
