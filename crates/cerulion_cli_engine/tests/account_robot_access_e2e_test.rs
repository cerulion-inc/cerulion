// SPDX-License-Identifier: AGPL-3.0-only
//! Actual account HTTP and netd UDS boundaries. The injected plane makes no
//! claim of robot reachability; real TLS reachability is covered by netd tests.

#![cfg(unix)]

use std::sync::{mpsc, Arc};

use base64::engine::general_purpose::URL_SAFE_NO_PAD;
use base64::Engine;
use cerulion_accountd::{
    AppState, CapturingEmailSender, Clock, ServiceConfig, UnconfiguredResolver,
};
use cerulion_cli_engine::{account_robot_access, auth};
use serial_test::serial;

struct EnvGuard(&'static str, Option<std::ffi::OsString>);
impl EnvGuard {
    fn set(name: &'static str, value: impl AsRef<std::ffi::OsStr>) -> Self {
        let old = std::env::var_os(name);
        std::env::set_var(name, value);
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
    stop: Option<mpsc::Sender<()>>,
    thread: Option<std::thread::JoinHandle<()>>,
    url: String,
}
impl Server {
    fn start(state: Arc<AppState>) -> Self {
        let (ready, address) = mpsc::channel();
        let (stop, stopped) = mpsc::channel();
        let thread = std::thread::spawn(move || {
            tokio::runtime::Builder::new_multi_thread()
                .enable_all()
                .build()
                .unwrap()
                .block_on(async move {
                    let listener = tokio::net::TcpListener::bind("127.0.0.1:0").await.unwrap();
                    ready.send(listener.local_addr().unwrap()).unwrap();
                    let shutdown = tokio::task::spawn_blocking(move || stopped.recv());
                    tokio::select! {
                        result = cerulion_accountd::serve(listener, state) => result.unwrap(),
                        result = shutdown => { result.unwrap().unwrap(); },
                    }
                });
        });
        Self {
            stop: Some(stop),
            thread: Some(thread),
            url: format!("http://{}", address.recv().unwrap()),
        }
    }
}
impl Drop for Server {
    fn drop(&mut self) {
        let _ = self.stop.take().unwrap().send(());
        self.thread.take().unwrap().join().unwrap();
    }
}

struct Fixture {
    _env: Vec<EnvGuard>,
    _server: Server,
    home: tempfile::TempDir,
    robot_id: [u8; 32],
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
            .upsert_user_by_identity("email", "account-access@example.test", None, 0, now)
            .unwrap();
        let key = ed25519_dalek::SigningKey::from_bytes(&[0x71; 32])
            .verifying_key()
            .to_bytes();
        let robot = app
            .db
            .register_robot(&owner.account_id, "owned-robot", &key, None, now)
            .unwrap();
        let token = "account-access-session-fixture";
        let refresh = "account-access-refresh-fixture";
        let expiry = now + 600_000_000_000;
        app.db
            .insert_session(
                &owner.user_id,
                &cerulion_accountd::hash_token(token),
                &cerulion_accountd::hash_token(refresh),
                expiry,
                expiry,
                now,
            )
            .unwrap();
        let server = Server::start(app);
        let home = tempfile::tempdir_in("/tmp").unwrap();
        let env = vec![
            EnvGuard::set("CERULION_HOME", home.path()),
            EnvGuard::set("CERULION_ACCOUNT_SERVICE", &server.url),
            EnvGuard::set("CERULION_NETD_SOCK", home.path().join("netd.sock")),
        ];
        auth::write_to(
            &home.path().join("auth.json"),
            &auth::AuthState {
                account_id: URL_SAFE_NO_PAD.encode(owner.account_id),
                session_token: token.into(),
                refresh_token: refresh.into(),
                expires_at_ns: expiry,
                logged_in_ever: true,
                role: None,
            },
        )
        .unwrap();
        std::fs::write(home.path().join("desk.key"), [0x73; 32]).unwrap();
        Self {
            _env: env,
            _server: server,
            home,
            robot_id: robot.robot_id,
        }
    }
    fn socket(&self) -> std::path::PathBuf {
        self.home.path().join("netd.sock")
    }
}

#[derive(Default)]
struct AccessSpy(std::sync::Mutex<Vec<cerulion_netd::account_access::AccountAccessRequest>>);
impl cerulion_netd::MirrorPlane for AccessSpy {
    fn ensure_mirror(
        &self,
        _: &cerulion_netd::TopicKey,
        _: u64,
    ) -> Result<(), cerulion_netd::MirrorError> {
        panic!("listing and preparing a viewer target must never demand")
    }
    fn release_mirror(&self, _: &cerulion_netd::TopicKey) -> cerulion_netd::MirrorRelease {
        panic!("listing and preparing a target hold no mirrors")
    }
    fn account_access(
        &self,
        action: &cerulion_netd::account_access::AccountAccessRequest,
    ) -> Result<cerulion_netd::account_access::AccountAccessReply, String> {
        use cerulion_netd::account_access::{
            AccountAccessReply, AccountAccessRequest, RobotPresence,
        };
        self.0.lock().unwrap().push(action.clone());
        match action {
            AccountAccessRequest::Install { snapshot } => {
                let wire = serde_json::to_string(snapshot).unwrap();
                assert!(!wire.contains("account-access-session-fixture"));
                assert!(!wire.contains("account-access-refresh-fixture"));
                assert!(!wire.contains("private_key") && !wire.contains("desk_seed"));
                assert!(snapshot.owner_chain.is_none());
                Ok(AccountAccessReply::Installed {
                    robot_count: snapshot.robots.len(),
                })
            }
            AccountAccessRequest::Probe {
                robot_id,
                budget_ms,
            } => {
                assert!((1..=500).contains(budget_ms));
                Ok(AccountAccessReply::Presence {
                    robot_id: *robot_id,
                    presence: RobotPresence::Unknown {
                        reason: "test plane made no reachability observation".into(),
                    },
                })
            }
            _ => panic!("listing and target preparation must not fetch metadata or pair"),
        }
    }
}

#[test]
#[serial]
fn actual_http_directory_installs_public_membership_and_keeps_viewer_daemon_alive() {
    use cerulion_netd::account_access::{robot_route, AccountAccessRequest, RobotPresence};
    let fixture = Fixture::new();
    let spy = Arc::new(AccessSpy::default());
    let mut daemon = cerulion_netd::start(
        fixture.socket(),
        spy.clone(),
        cerulion_netd::NetdConfig::default(),
    )
    .unwrap();
    let listing = account_robot_access::list().unwrap();
    assert!(listing.diagnostic.is_none());
    assert_eq!(listing.rows.len(), 1);
    assert_eq!(listing.rows[0].robot.robot_id.0, fixture.robot_id);
    assert_eq!(listing.rows[0].robot.hostname, "owned-robot");
    assert!(matches!(
        listing.rows[0].presence,
        RobotPresence::Unknown { .. }
    ));
    assert!(
        account_robot_access::render_rows(&listing.rows).contains("(account directory)  unknown:")
    );
    let route = robot_route(&fixture.robot_id);
    let target = account_robot_access::prepare_viz_target(&route, &[], &[]).unwrap();
    assert_eq!(target.route, route);
    assert!(target.diagnostics.is_empty());
    assert!(
        daemon.active_connections() >= 1,
        "target holds the daemon until vizd takes its own demand"
    );
    let actions = spy.0.lock().unwrap();
    assert_eq!(actions.len(), 3);
    assert!(matches!(&actions[0], AccountAccessRequest::Install { .. }));
    assert!(
        matches!(&actions[1], AccountAccessRequest::Probe { robot_id, .. } if *robot_id == fixture.robot_id)
    );
    assert!(matches!(&actions[2], AccountAccessRequest::Install { .. }));
    drop(actions);
    assert_eq!(daemon.active_demand_count(), 0);
    drop(target);
    daemon.shutdown();
}

#[test]
#[serial]
fn uncooperative_local_peer_cannot_spend_the_whole_list_budget_on_one_probe() {
    use cerulion_netd::account_access::{
        AccountAccessReply, AccountAccessRequest, AccountAccessResponse,
    };
    use cerulion_netd::protocol::{Hello, Request, Response};
    use std::io::{BufRead, BufReader, Read, Write};
    use std::os::unix::net::UnixListener;
    use std::time::{Duration, Instant};
    let fixture = Fixture::new();
    let listener = UnixListener::bind(fixture.socket()).unwrap();
    let worker = std::thread::spawn(move || {
        let (mut stream, _) = listener.accept().unwrap();
        stream
            .set_read_timeout(Some(Duration::from_secs(6)))
            .unwrap();
        writeln!(stream, "{}", Hello::new().to_json_line()).unwrap();
        let mut reader = BufReader::new(stream.try_clone().unwrap());
        let mut line = String::new();
        reader.read_line(&mut line).unwrap();
        let Request::AccountAccess {
            id,
            action: AccountAccessRequest::Install { snapshot },
        } = serde_json::from_str(&line).unwrap()
        else {
            panic!("expected install")
        };
        let response = Response::AccountAccess(AccountAccessResponse {
            id,
            account_access: AccountAccessReply::Installed {
                robot_count: snapshot.robots.len(),
            },
        });
        writeln!(stream, "{}", response.to_json_line()).unwrap();
        line.clear();
        reader.read_line(&mut line).unwrap();
        let Request::AccountAccess {
            action: AccountAccessRequest::Probe { budget_ms, .. },
            ..
        } = serde_json::from_str(&line).unwrap()
        else {
            panic!("expected probe")
        };
        assert!((1..=500).contains(&budget_ms));
        let started = Instant::now();
        let mut remainder = Vec::new();
        reader.read_to_end(&mut remainder).unwrap();
        assert!(remainder.is_empty());
        started.elapsed()
    });
    let listing = account_robot_access::list().unwrap();
    let closed_after = worker.join().unwrap();
    assert!(listing.diagnostic.is_some());
    assert!(
        closed_after < Duration::from_secs(2),
        "ignored per-robot budget kept the IPC connection open for {closed_after:?}"
    );
    assert!(matches!(
        listing.rows[0].presence,
        cerulion_netd::account_access::RobotPresence::Unknown { .. }
    ));
}
