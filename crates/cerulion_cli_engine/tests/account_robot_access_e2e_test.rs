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

#[test]
#[serial]
fn a_blackholed_account_service_bounds_the_listing_and_names_the_gap() {
    use std::time::{Duration, Instant};
    // Accepts every connection and never answers.
    let listener = std::net::TcpListener::bind("127.0.0.1:0").unwrap();
    let address = listener.local_addr().unwrap();
    let held = Arc::new(std::sync::Mutex::new(Vec::new()));
    let holder = Arc::clone(&held);
    std::thread::spawn(move || {
        for stream in listener.incoming().flatten() {
            holder.lock().unwrap().push(stream);
        }
    });
    let home = tempfile::tempdir().unwrap();
    let _home = EnvGuard::set("CERULION_HOME", home.path());
    let _service = EnvGuard::set("CERULION_ACCOUNT_SERVICE", format!("http://{address}"));
    let expiry = auth::now_unix_ns() + 600_000_000_000;
    auth::write_to(
        &home.path().join("auth.json"),
        &auth::AuthState {
            account_id: "00112233-4455-4677-8899-aabbccddeeff".into(),
            session_token: "blackhole-session".into(),
            refresh_token: "blackhole-refresh".into(),
            expires_at_ns: expiry,
            logged_in_ever: true,
            role: None,
        },
    )
    .unwrap();
    std::fs::write(home.path().join("desk.key"), [0x74; 32]).unwrap();

    let started = Instant::now();
    let mut err = Vec::new();
    let rows = account_robot_access::listing_rows("cerulion topic list", &mut err).unwrap();
    let elapsed = started.elapsed();

    assert_eq!(rows, "", "no account rows are invented");
    assert_eq!(
        String::from_utf8(err).unwrap(),
        "cerulion topic list: account robots not listed: account robot catalog unavailable: \
         the account service did not answer within 2000 ms\n"
    );
    // The listing waited out its 2 s budget, and no longer than a second past it.
    assert!(elapsed >= Duration::from_secs(2), "{elapsed:?}");
    assert!(elapsed < Duration::from_secs(3), "{elapsed:?}");
    assert!(
        !held.lock().unwrap().is_empty(),
        "the request reached the blackholed service"
    );
}

#[test]
#[serial]
fn a_listing_with_a_stale_session_refreshes_nothing_and_says_so() {
    use std::time::{Duration, Instant};
    // Counts every connection; a refresh or a catalog request would arrive here.
    let listener = std::net::TcpListener::bind("127.0.0.1:0").unwrap();
    let address = listener.local_addr().unwrap();
    let held = Arc::new(std::sync::Mutex::new(Vec::new()));
    let holder = Arc::clone(&held);
    std::thread::spawn(move || {
        for stream in listener.incoming().flatten() {
            holder.lock().unwrap().push(stream);
        }
    });
    let home = tempfile::tempdir().unwrap();
    let _home = EnvGuard::set("CERULION_HOME", home.path());
    let _service = EnvGuard::set("CERULION_ACCOUNT_SERVICE", format!("http://{address}"));
    let auth_path = home.path().join("auth.json");
    auth::write_to(
        &auth_path,
        &auth::AuthState {
            account_id: "00112233-4455-4677-8899-aabbccddeeff".into(),
            session_token: "stale-session".into(),
            refresh_token: "single-use-refresh".into(),
            expires_at_ns: 1,
            logged_in_ever: true,
            role: None,
        },
    )
    .unwrap();
    std::fs::write(home.path().join("desk.key"), [0x75; 32]).unwrap();
    let auth_before = std::fs::read(&auth_path).unwrap();

    let started = Instant::now();
    let mut err = Vec::new();
    let rows = account_robot_access::listing_rows("cerulion topic list", &mut err).unwrap();

    assert_eq!(rows, "");
    assert_eq!(
        String::from_utf8(err).unwrap(),
        "cerulion topic list: account robots not listed: account robot catalog unavailable: \
         the session is stale; any account command (or `cerulion login`) renews it\n"
    );
    assert!(started.elapsed() < Duration::from_secs(1));
    std::thread::sleep(Duration::from_millis(200));
    assert!(
        held.lock().unwrap().is_empty(),
        "nothing reached the account service"
    );
    assert_eq!(
        std::fs::read(&auth_path).unwrap(),
        auth_before,
        "tokens untouched"
    );
}

#[test]
#[serial]
fn a_machine_that_never_signed_in_lists_no_account_line_and_creates_nothing() {
    let home = tempfile::tempdir().unwrap();
    let config = home.path().join("never-signed-in");
    let _home = EnvGuard::set("CERULION_HOME", &config);
    let _service = EnvGuard::set("CERULION_ACCOUNT_SERVICE", "http://127.0.0.1:1");
    let mut err = Vec::new();
    assert_eq!(
        account_robot_access::listing_rows("cerulion topic list", &mut err).unwrap(),
        ""
    );
    assert!(err.is_empty());
    assert!(!config.exists(), "no config directory is created");
}

#[test]
#[serial]
fn a_listing_refreshes_the_listed_robots_revocation_epoch_into_the_dialed_routes_cache() {
    use cerulion_cli_engine::account_cmd::{revoke_robot_access, RevokeTarget};
    use cerulion_netd::account_access::robot_route;
    let fixture = Fixture::new();
    let spy = Arc::new(AccessSpy::default());
    let mut daemon = cerulion_netd::start(
        fixture.socket(),
        spy.clone(),
        cerulion_netd::NetdConfig::default(),
    )
    .unwrap();
    // The owner revokes one desk device at the account service.
    let revoked = [0x5a_u8; 32];
    revoke_robot_access(
        "account-access-session-fixture",
        &URL_SAFE_NO_PAD.encode(fixture.robot_id),
        &RevokeTarget::Device(URL_SAFE_NO_PAD.encode(revoked)),
    )
    .unwrap();
    // The cache netd pushes for this route: <config>/epochs/<route with ':' as '_'>.epoch.
    let route = robot_route(&fixture.robot_id);
    let cache = fixture
        .home
        .path()
        .join("epochs")
        .join(format!("{}.epoch", route.replace(':', "_")));
    assert!(!cache.exists());

    let listing = account_robot_access::list().unwrap();
    assert!(listing.diagnostic.is_none(), "{:?}", listing.diagnostic);

    let artifact = std::fs::read_to_string(&cache).expect("the listed robot's epoch is cached");
    let wire: cerulion_pairing::verify::EpochSyncWire =
        postcard::from_bytes(&URL_SAFE_NO_PAD.decode(artifact.trim()).unwrap()).unwrap();
    let epoch = &wire.signed_epoch.epoch_data;
    assert_eq!(epoch.robot.0, fixture.robot_id);
    assert_eq!(
        epoch.revoked_devices,
        vec![cerulion_pairing::format::PublicKey(revoked)]
    );
    // No cache is written for any other name.
    let names: Vec<_> = std::fs::read_dir(fixture.home.path().join("epochs"))
        .unwrap()
        .map(|entry| entry.unwrap().file_name())
        .collect();
    assert_eq!(names.len(), 1, "{names:?}");

    // `viz --robot` refreshes its selected robot before the dial too.
    let second = [0x5c_u8; 32];
    revoke_robot_access(
        "account-access-session-fixture",
        &URL_SAFE_NO_PAD.encode(fixture.robot_id),
        &RevokeTarget::Device(URL_SAFE_NO_PAD.encode(second)),
    )
    .unwrap();
    std::fs::remove_file(&cache).unwrap();
    let target = account_robot_access::prepare_viz_target(&route, &[], &[]).unwrap();
    assert!(target.diagnostics.is_empty(), "{:?}", target.diagnostics);
    let artifact = std::fs::read_to_string(&cache).expect("the selected robot's epoch is cached");
    let wire: cerulion_pairing::verify::EpochSyncWire =
        postcard::from_bytes(&URL_SAFE_NO_PAD.decode(artifact.trim()).unwrap()).unwrap();
    assert_eq!(wire.signed_epoch.epoch_data.revoked_devices.len(), 2);
    drop(target);
    daemon.shutdown();
}
