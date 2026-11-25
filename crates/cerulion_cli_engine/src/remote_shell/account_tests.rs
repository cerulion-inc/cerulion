// SPDX-License-Identifier: AGPL-3.0-only
//! Real owner-only account HTTP, followed by the engine's helper spawn.
//! The helper records argv and exits; it makes no claim of robot reachability.

#![cfg(unix)]

use super::*;
use std::ffi::{OsStr, OsString};
use std::os::unix::fs::PermissionsExt;
use std::path::PathBuf;
use std::sync::mpsc;
use std::time::Duration;

use base64::engine::general_purpose::URL_SAFE_NO_PAD;
use base64::Engine;
use cerulion_accountd::{
    AppState, CapturingEmailSender, Clock, ServiceConfig, UnconfiguredResolver,
};
use cerulion_pairing::format::{PublicKey, RobotId};

struct EnvGuard(&'static str, Option<OsString>);
impl EnvGuard {
    fn set(name: &'static str, value: Option<&OsStr>) -> Self {
        let old = std::env::var_os(name);
        match value {
            Some(value) => std::env::set_var(name, value),
            None => std::env::remove_var(name),
        }
        Self(name, old)
    }
}
impl Drop for EnvGuard {
    fn drop(&mut self) {
        match self.1.take() {
            Some(value) => std::env::set_var(self.0, value),
            None => std::env::remove_var(self.0),
        }
    }
}

struct Server {
    stop: Option<tokio::sync::oneshot::Sender<()>>,
    done: mpsc::Receiver<()>,
    thread: Option<std::thread::JoinHandle<()>>,
    url: String,
}
impl Server {
    fn start(app: Arc<AppState>) -> Self {
        let (ready, address) = mpsc::channel();
        let (stop, stopped) = tokio::sync::oneshot::channel();
        let (finished, done) = mpsc::channel();
        let thread = std::thread::spawn(move || {
            let runtime = tokio::runtime::Builder::new_multi_thread()
                .worker_threads(2)
                .enable_all()
                .build()
                .unwrap();
            runtime.block_on(async move {
                let listener = tokio::net::TcpListener::bind("127.0.0.1:0").await.unwrap();
                ready.send(listener.local_addr().unwrap()).unwrap();
                tokio::select! {
                    result = cerulion_accountd::serve(listener, app) => result.unwrap(),
                    _ = stopped => {},
                }
            });
            runtime.shutdown_timeout(Duration::from_secs(2));
            let _ = finished.send(());
        });
        Self {
            stop: Some(stop),
            done,
            thread: Some(thread),
            url: format!(
                "http://{}",
                address.recv_timeout(Duration::from_secs(5)).unwrap()
            ),
        }
    }
}
impl Drop for Server {
    fn drop(&mut self) {
        if let Some(stop) = self.stop.take() {
            let _ = stop.send(());
        }
        if self.done.recv_timeout(Duration::from_secs(5)).is_ok() {
            self.thread.take().unwrap().join().unwrap();
        } else if !std::thread::panicking() {
            panic!("account server did not stop within five seconds");
        }
    }
}

struct Fixture {
    // Restore process environment before deleting its paths.
    _env: Vec<EnvGuard>,
    _server: Server,
    home: tempfile::TempDir,
    app: Arc<AppState>,
    state: auth::AuthState,
    robot_id: [u8; 32],
    robot_key: [u8; 32],
    marker: PathBuf,
}
impl Fixture {
    fn new() -> Self {
        let now = auth::now_unix_ns();
        let app = Arc::new(
            AppState::dev(
                ServiceConfig::default(),
                Clock::System,
                Arc::new(CapturingEmailSender::new()),
                Arc::new(UnconfiguredResolver),
            )
            .unwrap(),
        );
        let owner = app
            .db
            .upsert_user_by_identity("email", "shell-owner@example.test", None, 0, now)
            .unwrap();
        let robot_key = ed25519_dalek::SigningKey::from_bytes(&[0x71; 32])
            .verifying_key()
            .to_bytes();
        let robot = app
            .db
            .register_robot(&owner.account_id, "owned-robot", &robot_key, None, now)
            .unwrap();
        let token = "shell-account-session";
        let expiry = now + 600_000_000_000;
        app.db
            .insert_session(
                &owner.user_id,
                &cerulion_accountd::hash_token(token),
                &cerulion_accountd::hash_token("shell-account-refresh"),
                expiry,
                expiry,
                now,
            )
            .unwrap();
        let server = Server::start(Arc::clone(&app));
        let home = tempfile::tempdir_in("/tmp").unwrap();
        let state = auth::AuthState {
            account_id: URL_SAFE_NO_PAD.encode(owner.account_id),
            session_token: token.into(),
            refresh_token: "shell-account-refresh".into(),
            expires_at_ns: expiry,
            logged_in_ever: true,
            role: None,
        };
        auth::write_to(&home.path().join("auth.json"), &state).unwrap();
        std::fs::write(home.path().join("desk.key"), [0x73; 32]).unwrap();
        std::fs::set_permissions(
            home.path().join("desk.key"),
            std::fs::Permissions::from_mode(0o600),
        )
        .unwrap();
        let helper = home.path().join("helper");
        let marker = home.path().join("argv");
        std::fs::write(
            &helper,
            "#!/bin/sh\nprintf '%s\\0' \"$@\" > \"$CERULION_TEST_SPAWN_MARKER\"\nexit 78\n",
        )
        .unwrap();
        std::fs::set_permissions(&helper, std::fs::Permissions::from_mode(0o700)).unwrap();
        let env = vec![
            EnvGuard::set("CERULION_HOME", Some(home.path().as_os_str())),
            EnvGuard::set("CERULION_ACCOUNT_SERVICE", Some(OsStr::new(&server.url))),
            EnvGuard::set("CERULION_CONNECTD_BIN", Some(helper.as_os_str())),
            EnvGuard::set("CERULION_TEST_SPAWN_MARKER", Some(marker.as_os_str())),
            EnvGuard::set("CERULION_NETWORK", None),
            EnvGuard::set("CERULION_NETD_NETWORK", None),
            EnvGuard::set(
                crate::account_cmd::EPOCH_DIR_ENV,
                Some(home.path().join("epochs").as_os_str()),
            ),
        ];
        Self {
            _env: env,
            _server: server,
            home,
            app,
            state,
            robot_id: robot.robot_id,
            robot_key,
            marker,
        }
    }

    fn route(&self) -> String {
        cerulion_netd::account_access::robot_route(&self.robot_id)
    }

    fn assert_no_pairing_or_helper(&self) {
        assert!(!self.marker.exists());
        assert!(!self.home.path().join("device-chain.json").exists());
        assert!(!self.home.path().join("robots.toml").exists());
        assert!(!self.home.path().join("shell-witnesses").exists());
    }
}

fn spawn_bounded(prepared: PreparedSession) -> CliResult<i32> {
    spawn_bounded_with_running(prepared, Arc::new(AtomicBool::new(true)))
}

fn spawn_bounded_with_running(
    prepared: PreparedSession,
    running: Arc<AtomicBool>,
) -> CliResult<i32> {
    let watchdog_running = Arc::clone(&running);
    let (cancel, cancelled) = mpsc::channel::<()>();
    let watchdog = std::thread::spawn(move || {
        if matches!(
            cancelled.recv_timeout(Duration::from_secs(10)),
            Err(mpsc::RecvTimeoutError::Timeout)
        ) {
            watchdog_running.store(false, std::sync::atomic::Ordering::SeqCst);
        }
    });
    let result = prepared.spawn(running);
    drop(cancel);
    watchdog.join().unwrap();
    result
}

fn value<'a>(args: &'a [String], flag: &str) -> Option<&'a str> {
    args.iter()
        .position(|arg| arg == flag)
        .map(|index| args[index + 1].as_str())
}

#[test]
fn account_name_and_explicit_id_forward_public_context_without_reading_proof() {
    let _env_lock = crate::test_env::env_lock();
    let fixture = Fixture::new();
    for selector in ["owned-robot".to_owned(), fixture.route()] {
        let command = [
            "printf",
            "%s",
            "a b",
            "",
            "--no-network",
            "--owner-account",
            "literal-command-value",
            "--robot-name",
            "literal-command-route",
        ]
        .map(str::to_owned)
        .to_vec();
        let prepared = prepare(&selector, &Operation::Exec(command.clone()), true).unwrap();
        let args = &prepared.plan.argv;
        assert_eq!(
            value(args, "--eid"),
            Some(hex::encode(fixture.robot_key).as_str())
        );
        assert_eq!(value(args, "--robot-name"), Some(fixture.route().as_str()));
        let selected = prepared.account.as_ref().unwrap();
        assert_eq!(selected.robot.robot_id, RobotId(fixture.robot_id));
        assert_eq!(
            selected.robot.endpoint,
            account_robots::RobotEndpoint::Known(PublicKey(fixture.robot_key))
        );
        let account: [u8; 32] = URL_SAFE_NO_PAD
            .decode(&fixture.state.account_id)
            .unwrap()
            .try_into()
            .unwrap();
        assert_eq!(
            value(args, "--owner-account"),
            Some(hex::encode(account).as_str())
        );
        let desk = ed25519_dalek::SigningKey::from_bytes(&[0x73; 32])
            .verifying_key()
            .to_bytes();
        assert_eq!(
            value(args, "--owner-desk-key"),
            Some(hex::encode(desk).as_str())
        );
        assert_eq!(
            value(args, "--owner-config-dir"),
            fixture.home.path().to_str()
        );
        assert_eq!(
            value(args, "--owner-store-lock"),
            fixture.home.path().join(".studio-auth.lock").to_str()
        );
        let separator = args.iter().position(|arg| arg == "--").unwrap();
        assert_eq!(
            args[..separator]
                .iter()
                .filter(|arg| arg.as_str() == "--robot-name")
                .count(),
            1
        );
        assert_eq!(&args[separator + 1..], command);
        for flag in [
            "--owner-account",
            "--owner-desk-key",
            "--owner-config-dir",
            "--owner-store-lock",
        ] {
            assert!(args.iter().position(|arg| arg == flag).unwrap() < separator);
        }
        let expected = args.join("\0") + "\0";
        assert_eq!(spawn_bounded(prepared).unwrap(), 78);
        assert_eq!(std::fs::read(&fixture.marker).unwrap(), expected.as_bytes());
        std::fs::remove_file(&fixture.marker).unwrap();
        fixture.assert_no_pairing_or_helper();
    }
}

#[test]
fn pinned_alias_and_raw_eid_stay_local_while_account_routes_replace_reverse_pins() {
    let _env_lock = crate::test_env::env_lock();
    let fixture = Fixture::new();
    let foreign = hex::encode([0x31; 32]);
    std::fs::write(
        fixture.home.path().join("robots.toml"),
        format!(
            "[robots]\nowned-robot = {foreign:?}\nexisting-epoch-name = {:?}\n",
            hex::encode(fixture.robot_key)
        ),
    )
    .unwrap();
    let unavailable = EnvGuard::set(
        "CERULION_ACCOUNT_SERVICE",
        Some(OsStr::new("http://127.0.0.1:1")),
    );
    for selector in ["owned-robot", foreign.as_str()] {
        let prepared = prepare(selector, &Operation::Shell, false).unwrap();
        assert_eq!(value(&prepared.plan.argv, "--eid"), Some(foreign.as_str()));
        assert_eq!(
            value(&prepared.plan.argv, "--robot-name"),
            Some("owned-robot")
        );
        assert_eq!(value(&prepared.plan.argv, "--owner-account"), None);
        assert!(prepared.account.is_none());
    }
    drop(unavailable);
    let prepared = prepare(
        &fixture.route(),
        &Operation::Policy {
            enabled: Some(false),
        },
        false,
    )
    .unwrap();
    assert_eq!(
        value(&prepared.plan.argv, "--eid"),
        Some(hex::encode(fixture.robot_key).as_str())
    );
    assert_eq!(
        value(&prepared.plan.argv, "--robot-name"),
        Some(fixture.route().as_str())
    );
    assert_eq!(
        prepared
            .plan
            .argv
            .iter()
            .filter(|arg| arg.as_str() == "--robot-name")
            .count(),
        1
    );
    assert!(
        std::fs::read_to_string(fixture.home.path().join("robots.toml"))
            .unwrap()
            .contains("existing-epoch-name")
    );
    assert!(prepared.account.is_some());
    assert!(!fixture.marker.exists());
}

#[test]
fn duplicate_account_names_and_cached_lan_names_refuse_without_fallback() {
    let _env_lock = crate::test_env::env_lock();
    for duplicate in [false, true] {
        let fixture = Fixture::new();
        if duplicate {
            let account: [u8; 32] = URL_SAFE_NO_PAD
                .decode(&fixture.state.account_id)
                .unwrap()
                .try_into()
                .unwrap();
            let key = ed25519_dalek::SigningKey::from_bytes(&[0x72; 32])
                .verifying_key()
                .to_bytes();
            fixture
                .app
                .db
                .register_robot(&account, "owned-robot", &key, None, auth::now_unix_ns())
                .unwrap();
        } else {
            std::fs::write(fixture.home.path().join("peers.json"), serde_json::to_vec(&serde_json::json!({
                "v":1,"peers":[{"robot":"owned-robot","locator":"tcp/127.0.0.1:9","last_seen":peer_cache::current_unix_secs()}]
            })).unwrap()).unwrap();
        }
        let error = prepare("owned-robot", &Operation::Shell, false)
            .err()
            .unwrap()
            .to_string();
        assert!(
            error.contains("ambiguous") && error.contains(&fixture.route()),
            "{error}"
        );
        fixture.assert_no_pairing_or_helper();
        assert!(prepare(&fixture.route(), &Operation::Shell, false).is_ok());
    }
}

#[test]
fn interrupted_account_lookup_does_not_start_the_prepared_command() {
    let _env_lock = crate::test_env::env_lock();
    let fixture = Fixture::new();
    let prepared = prepare(
        "owned-robot",
        &Operation::Exec(vec!["/bin/true".into()]),
        false,
    )
    .unwrap();
    assert_eq!(
        prepared.spawn(Arc::new(AtomicBool::new(false))).unwrap(),
        130
    );
    fixture.assert_no_pairing_or_helper();
}

#[test]
fn account_lookup_refusal_and_changed_login_never_spawn_a_helper() {
    let _env_lock = crate::test_env::env_lock();
    let fixture = Fixture::new();
    fixture
        .app
        .db
        .revoke_session_by_token_hash(&cerulion_accountd::hash_token(&fixture.state.session_token))
        .unwrap();
    let error = prepare("owned-robot", &Operation::Shell, false)
        .err()
        .unwrap()
        .to_string();
    assert!(error.contains("refused the session"), "{error}");
    fixture.assert_no_pairing_or_helper();
    drop(fixture);
    for change in ["session", "logout", "key"] {
        let fixture = Fixture::new();
        let prepared = prepare("owned-robot", &Operation::Shell, false).unwrap();
        if change == "key" {
            std::fs::write(fixture.home.path().join("identity/device.key"), [0x74; 32]).unwrap();
        } else {
            let mut state = fixture.state.clone();
            state.session_token = if change == "logout" {
                String::new()
            } else {
                "replacement-session".into()
            };
            auth::write_to(&fixture.home.path().join("auth.json"), &state).unwrap();
        }
        assert!(spawn_bounded(prepared).is_err(), "accepted {change}");
        fixture.assert_no_pairing_or_helper();
    }
}

#[test]
fn a_refused_epoch_refresh_keeps_the_prepared_identity_and_real_helper_exit() {
    let _env_lock = crate::test_env::env_lock();
    let fixture = Fixture::new();
    let auth_before = std::fs::read(fixture.home.path().join("auth.json")).unwrap();
    let prepared = prepare(
        "owned-robot",
        &Operation::Exec(vec!["printf".into(), "hello".into()]),
        false,
    )
    .unwrap();
    fixture
        .app
        .db
        .revoke_session_by_token_hash(&cerulion_accountd::hash_token(&fixture.state.session_token))
        .unwrap();
    assert_eq!(spawn_bounded(prepared).unwrap(), 78);
    let recorded = std::fs::read(&fixture.marker).unwrap();
    assert!(recorded.starts_with(b"remote-exec\0"));
    assert!(recorded.ends_with(b"--\0printf\0hello\0"));
    assert_eq!(
        std::fs::read(fixture.home.path().join("auth.json")).unwrap(),
        auth_before
    );
}

#[test]
fn missing_endpoint_is_a_terminal_refusal_and_unknown_name_does_not_fall_back() {
    let rows = [account_robots::AccountRobot {
        robot_id: RobotId([0x41; 32]),
        hostname: "legacy-robot".into(),
        endpoint: account_robots::RobotEndpoint::Unknown {
            reason:
                "the account service did not provide this robot's endpoint key; presence is unknown",
        },
    }];
    let error = account_endpoint(&rows, "legacy-robot", &Default::default())
        .unwrap_err()
        .to_string();
    assert_eq!(error, "cannot open the selected account robot: the account service did not provide this robot's endpoint key; presence is unknown");
    let mut known = rows[0].clone();
    known.endpoint = account_robots::RobotEndpoint::Known(PublicKey([0x42; 32]));
    let error = account_endpoint(&[known], "missing\u{1b}[31m\nrobot", &Default::default())
        .unwrap_err()
        .to_string();
    assert!(error.contains("neither a pinned alias nor an owned account robot"));
    assert!(!error.contains(['\u{1b}', '\n']));
}

#[test]
fn invalid_account_paths_and_unreadable_pin_stores_are_refused() {
    use std::os::unix::ffi::OsStrExt;

    let _env_lock = crate::test_env::env_lock();
    assert_eq!(
        context_path(Path::new(OsStr::from_bytes(b"/tmp/\xff")))
            .unwrap_err()
            .to_string(),
        "the account directory path is not valid UTF-8"
    );
    let fixture = Fixture::new();
    assert!(!local_selector("unconfigured-robot").unwrap());
    std::fs::create_dir(fixture.home.path().join("robots.toml")).unwrap();
    assert!(local_selector("unconfigured-robot").is_err());
    assert!(!fixture.marker.exists());
}

#[test]
fn account_routes_match_a_literal_lowercase_hex_oracle() {
    let id = std::array::from_fn(|index| index as u8);
    let route = "account:000102030405060708090a0b0c0d0e0f101112131415161718191a1b1c1d1e1f";
    assert_eq!(cerulion_netd::account_access::robot_route(&id), route);
    let rows = [account_robots::AccountRobot {
        robot_id: RobotId(id),
        hostname: "literal-route-robot".into(),
        endpoint: account_robots::RobotEndpoint::Known(PublicKey([0x42; 32])),
    }];
    assert_eq!(
        account_endpoint(&rows, route, &Default::default())
            .unwrap()
            .eid,
        "4242424242424242424242424242424242424242424242424242424242424242"
    );
    for invalid in [
        "account:AAECAwQFBgcICQoLDA0ODxAREhMUFRYXGBkaGxwdHh8",
        "account:000102030405060708090A0B0C0D0E0F101112131415161718191A1B1C1D1E1F",
    ] {
        assert!(account_endpoint(&rows, invalid, &Default::default()).is_err());
    }
    let mut malformed = ConnectPlan {
        bin: "cerulion-connectd".into(),
        argv: vec!["connect".into(), "--robot-name".into()],
    };
    assert!(set_account_epoch_route(&mut malformed, &rows[0]).is_err());
}

// Bound owned loopback request/response setup and shutdown if a regression stops HTTP progress.
const ACCESS_GATE_BOUND: Duration = Duration::from_secs(5);

struct AccessGate {
    url: String,
    address: std::net::SocketAddr,
    stop: Arc<AtomicBool>,
    requests: mpsc::Receiver<()>,
    done: mpsc::Receiver<Result<Option<u16>, String>>,
    thread: Option<std::thread::JoinHandle<()>>,
}

impl AccessGate {
    fn start(upstream: &str, path: String, on_access: impl FnOnce() + Send + 'static) -> Self {
        use std::io::{Read, Write};
        let upstream: std::net::SocketAddr =
            upstream.strip_prefix("http://").unwrap().parse().unwrap();
        let listener = std::net::TcpListener::bind("127.0.0.1:0").unwrap();
        let address = listener.local_addr().unwrap();
        let stop = Arc::new(AtomicBool::new(false));
        let stopped = Arc::clone(&stop);
        let (requested, requests) = mpsc::channel();
        let (finished, done) = mpsc::channel();
        let thread = std::thread::spawn(move || {
            let result = (|| -> Result<Option<u16>, String> {
                let (mut client, _) = listener.accept().map_err(|e| e.to_string())?;
                if stopped.load(Ordering::SeqCst) {
                    return Ok(None);
                }
                let deadline = std::time::Instant::now() + ACCESS_GATE_BOUND;
                let remaining = || {
                    deadline
                        .checked_duration_since(std::time::Instant::now())
                        .filter(|duration| !duration.is_zero())
                        .ok_or_else(|| "access gate exceeded its request deadline".to_owned())
                };
                let mut header = Vec::new();
                while !header.ends_with(b"\r\n\r\n") {
                    if header.len() >= 16 * 1024 {
                        return Err("access request headers exceeded the fixture bound".into());
                    }
                    client
                        .set_read_timeout(Some(remaining()?))
                        .map_err(|e| e.to_string())?;
                    let mut byte = [0];
                    client.read_exact(&mut byte).map_err(|e| e.to_string())?;
                    header.push(byte[0]);
                }
                let header = std::str::from_utf8(&header).map_err(|e| e.to_string())?;
                let mut lines = header.split("\r\n");
                if lines.next() != Some(format!("GET {path} HTTP/1.1").as_str()) {
                    return Err("the engine did not fetch the selected robot's access route".into());
                }
                requested.send(()).map_err(|e| e.to_string())?;
                // The real request has left the engine. Mutate only now, before forwarding it.
                on_access();
                let mut server = std::net::TcpStream::connect_timeout(&upstream, remaining()?)
                    .map_err(|e| e.to_string())?;
                server
                    .set_write_timeout(Some(remaining()?))
                    .map_err(|e| e.to_string())?;
                let mut forwarded = format!("GET {path} HTTP/1.1\r\n");
                for line in lines.filter(|line| !line.is_empty()) {
                    if !line
                        .split_once(':')
                        .is_some_and(|(name, _)| name.eq_ignore_ascii_case("connection"))
                    {
                        forwarded.push_str(line);
                        forwarded.push_str("\r\n");
                    }
                }
                forwarded.push_str("Connection: close\r\n\r\n");
                server
                    .write_all(forwarded.as_bytes())
                    .map_err(|e| e.to_string())?;
                let mut response = Vec::new();
                loop {
                    server
                        .set_read_timeout(Some(remaining()?))
                        .map_err(|e| e.to_string())?;
                    let mut chunk = [0; 4096];
                    let count = server.read(&mut chunk).map_err(|e| e.to_string())?;
                    if count == 0 {
                        break;
                    }
                    response.extend_from_slice(&chunk[..count]);
                    if response.len() > 1024 * 1024 {
                        return Err("account response exceeded the fixture bound".into());
                    }
                }
                let status = std::str::from_utf8(&response)
                    .map_err(|e| e.to_string())?
                    .split_whitespace()
                    .nth(1)
                    .ok_or("account response had no status")?
                    .parse::<u16>()
                    .map_err(|e| e.to_string())?;
                client
                    .set_write_timeout(Some(remaining()?))
                    .map_err(|e| e.to_string())?;
                client.write_all(&response).map_err(|e| e.to_string())?;
                Ok(Some(status))
            })();
            let _ = finished.send(result);
        });
        Self {
            url: format!("http://{address}"),
            address,
            stop,
            requests,
            done,
            thread: Some(thread),
        }
    }

    fn finish(mut self) -> Option<u16> {
        let result = self
            .done
            .recv_timeout(ACCESS_GATE_BOUND * 3)
            .expect("access gate did not finish");
        self.thread.take().unwrap().join().unwrap();
        result.expect("access gate did not forward the real account response")
    }
}

impl Drop for AccessGate {
    fn drop(&mut self) {
        if self.thread.is_none() {
            return;
        }
        self.stop.store(true, Ordering::SeqCst);
        // Wake accept when cancellation correctly prevented any request.
        let _ = std::net::TcpStream::connect_timeout(&self.address, ACCESS_GATE_BOUND);
        let completed = self.done.recv_timeout(ACCESS_GATE_BOUND * 3);
        if completed.is_ok() || matches!(completed, Err(mpsc::RecvTimeoutError::Disconnected)) {
            let joined = self.thread.take().unwrap().join();
            if !std::thread::panicking() {
                joined.expect("access gate thread panicked");
            }
        } else if !std::thread::panicking() {
            panic!("access gate did not stop within its owned I/O bounds");
        }
    }
}

#[test]
fn engine_network_refusal_precedes_account_http_identity_and_helper() {
    let _env_lock = crate::test_env::env_lock();
    for switch in ["CERULION_NETWORK", "CERULION_NETD_NETWORK"] {
        let fixture = Fixture::new();
        let auth_before = std::fs::read(fixture.home.path().join("auth.json")).unwrap();
        let key_before = std::fs::read(fixture.home.path().join("desk.key")).unwrap();
        let gate = AccessGate::start(&fixture._server.url, "/v1/robots".into(), || {});
        let service = EnvGuard::set("CERULION_ACCOUNT_SERVICE", Some(OsStr::new(&gate.url)));
        let blocked = EnvGuard::set(switch, Some(OsStr::new("off")));
        let command = ["printf", "%s", "a b", "", "--no-network"]
            .map(str::to_owned)
            .to_vec();
        for operation in [
            Operation::Shell,
            Operation::Exec(command.clone()),
            Operation::Policy { enabled: None },
            Operation::Policy {
                enabled: Some(true),
            },
            Operation::Policy {
                enabled: Some(false),
            },
        ] {
            // Bypass CLI dispatch: removing prepare's check must make this fail.
            assert!(
                matches!(
                    prepare("owned-robot", &operation, false),
                    Err(CliError::RemoteShellNetworkRefused(_))
                ),
                "prepare did not refuse {operation:?} under {switch}=off"
            );
            assert!(
                matches!(
                    run(
                        "owned-robot".into(),
                        operation.clone(),
                        false,
                        Arc::new(AtomicBool::new(true)),
                    ),
                    Err(CliError::RemoteShellNetworkRefused(_))
                ),
                "run did not refuse {operation:?} under {switch}=off"
            );
            assert!(matches!(
                gate.requests.try_recv(),
                Err(mpsc::TryRecvError::Empty)
            ));
            fixture.assert_no_pairing_or_helper();
            assert!(!fixture.home.path().join("identity").exists());
            assert!(!fixture.home.path().join("epochs").exists());
            assert_eq!(
                std::fs::read(fixture.home.path().join("auth.json")).unwrap(),
                auth_before
            );
            assert_eq!(
                std::fs::read(fixture.home.path().join("desk.key")).unwrap(),
                key_before
            );
        }

        // The same live HTTP sentinel and seeded identity must work when enabled.
        drop(blocked);
        let prepared = prepare("owned-robot", &Operation::Exec(command), false).unwrap();
        gate.requests.recv_timeout(ACCESS_GATE_BOUND).unwrap();
        assert_eq!(gate.finish(), Some(200));
        drop(service); // Epoch refresh uses the existing account server directly.
        assert_eq!(spawn_bounded(prepared).unwrap(), 78);
        let recorded = std::fs::read(&fixture.marker).unwrap();
        assert!(recorded.ends_with(b"\0"));
        let argv = std::str::from_utf8(&recorded)
            .unwrap()
            .split_terminator('\0')
            .collect::<Vec<_>>();
        assert_eq!(argv[0], "remote-exec");
        let separator = argv.iter().position(|arg| *arg == "--").unwrap();
        assert_eq!(
            &argv[separator + 1..],
            &["printf", "%s", "a b", "", "--no-network"]
        );
        std::fs::remove_file(&fixture.marker).unwrap();
        fixture.assert_no_pairing_or_helper();
    }
}

#[test]
fn cancellation_before_epoch_sync_issues_no_access_request_or_helper() {
    let _env_lock = crate::test_env::env_lock();
    let fixture = Fixture::new();
    let prepared = prepare("owned-robot", &Operation::Shell, false).unwrap();
    let gate = AccessGate::start(
        &fixture._server.url,
        format!(
            "/v1/robots/{}/access",
            URL_SAFE_NO_PAD.encode(fixture.robot_id)
        ),
        || panic!("cancelled invocation issued epoch HTTP"),
    );
    let _service = EnvGuard::set("CERULION_ACCOUNT_SERVICE", Some(OsStr::new(&gate.url)));
    assert_eq!(
        prepared.spawn(Arc::new(AtomicBool::new(false))).unwrap(),
        130
    );
    assert!(matches!(
        gate.requests.try_recv(),
        Err(mpsc::TryRecvError::Empty)
    ));
    fixture.assert_no_pairing_or_helper();
}

#[test]
fn identity_and_cancellation_changes_during_real_epoch_http_prevent_helper_spawn() {
    let _env_lock = crate::test_env::env_lock();
    for change in ["session", "account", "key", "cancel", "session-and-refusal"] {
        let fixture = Fixture::new();
        let prepared = prepare("owned-robot", &Operation::Shell, false).unwrap();
        let running = Arc::new(AtomicBool::new(true));
        let changed_running = Arc::clone(&running);
        let auth_path = fixture.home.path().join("auth.json");
        let key_path = fixture.home.path().join("identity/device.key");
        let mut changed_state = fixture.state.clone();
        let original_token = fixture.state.session_token.clone();
        let app = Arc::clone(&fixture.app);
        let gate = AccessGate::start(
            &fixture._server.url,
            format!(
                "/v1/robots/{}/access",
                URL_SAFE_NO_PAD.encode(fixture.robot_id)
            ),
            move || match change {
                "cancel" => changed_running.store(false, Ordering::SeqCst),
                "key" => std::fs::write(key_path, [0x74; 32]).unwrap(),
                "account" => {
                    changed_state.account_id = URL_SAFE_NO_PAD.encode([0x75; 32]);
                    auth::write_to(&auth_path, &changed_state).unwrap();
                }
                _ => {
                    changed_state.session_token = "replacement-session".into();
                    auth::write_to(&auth_path, &changed_state).unwrap();
                    if change == "session-and-refusal" {
                        app.db
                            .revoke_session_by_token_hash(&cerulion_accountd::hash_token(
                                &original_token,
                            ))
                            .unwrap();
                    }
                }
            },
        );
        let _service = EnvGuard::set("CERULION_ACCOUNT_SERVICE", Some(OsStr::new(&gate.url)));
        let result = spawn_bounded_with_running(prepared, running);
        assert_eq!(
            gate.finish(),
            Some(if change == "session-and-refusal" {
                401
            } else {
                200
            })
        );
        if change == "cancel" {
            assert_eq!(result.unwrap(), 130);
        } else {
            assert!(result.is_err(), "accepted changed {change}");
        }
        fixture.assert_no_pairing_or_helper();
    }
}

#[test]
fn no_epoch_and_refused_refresh_still_spawn_with_the_canonical_cache_key() {
    let _env_lock = crate::test_env::env_lock();
    for refuse in [false, true] {
        let fixture = Fixture::new();
        let prepared = prepare(
            "owned-robot",
            &Operation::Exec(vec!["printf".into(), "hello".into()]),
            false,
        )
        .unwrap();
        let cache = crate::account_cmd::resolve_epoch_cache_path(&fixture.route()).unwrap();
        let previous = if refuse {
            fixture
                .app
                .db
                .add_revocation(
                    &fixture.robot_id,
                    cerulion_accountd::RevocationTarget::Device([0x76; 32]),
                    auth::now_unix_ns(),
                )
                .unwrap();
            assert_eq!(
                crate::account_cmd::sync_robot_epoch_to_cache(
                    &fixture.state.session_token,
                    &URL_SAFE_NO_PAD.encode(fixture.robot_id),
                    &fixture.route()
                )
                .unwrap(),
                Some(1)
            );
            Some(std::fs::read(&cache).unwrap())
        } else {
            None
        };
        let app = Arc::clone(&fixture.app);
        let token = fixture.state.session_token.clone();
        let gate = AccessGate::start(
            &fixture._server.url,
            format!(
                "/v1/robots/{}/access",
                URL_SAFE_NO_PAD.encode(fixture.robot_id)
            ),
            move || {
                if refuse {
                    app.db
                        .revoke_session_by_token_hash(&cerulion_accountd::hash_token(&token))
                        .unwrap();
                }
            },
        );
        let _service = EnvGuard::set("CERULION_ACCOUNT_SERVICE", Some(OsStr::new(&gate.url)));
        assert_eq!(spawn_bounded(prepared).unwrap(), 78);
        assert_eq!(gate.finish(), Some(if refuse { 401 } else { 200 }));
        let args = std::fs::read(&fixture.marker).unwrap();
        let args = std::str::from_utf8(&args)
            .unwrap()
            .split_terminator('\0')
            .map(str::to_owned)
            .collect::<Vec<_>>();
        assert_eq!(value(&args, "--robot-name"), Some(fixture.route().as_str()));
        match previous {
            Some(bytes) => assert_eq!(std::fs::read(cache).unwrap(), bytes),
            None => assert!(!cache.exists()),
        }
        std::fs::remove_file(&fixture.marker).unwrap();
        fixture.assert_no_pairing_or_helper();
    }
}
