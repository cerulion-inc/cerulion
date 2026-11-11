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

#[test]
#[serial]
fn a_stale_directory_refuses_before_starting_a_network_daemon() {
    let fixture = Fixture::new();
    let directory = cerulion_cli_engine::account_robots::fetch().unwrap();
    let path = fixture.home.path().join("auth.json");
    let mut state = auth::load_from(&path).state().unwrap().clone();
    state.session_token = "changed-session-after-fetch".into();
    auth::write_to(&path, &state).unwrap();
    let error = account_robot_access::install_directory(&directory).unwrap_err();
    assert!(error.to_string().contains("login changed"), "{error}");
    assert!(
        !fixture.socket().exists(),
        "stale membership must not start a daemon"
    );
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
    fn install_paired_robots(
        &self,
        robots: &[cerulion_netd::account_access::PairedRobot],
        _: &mut cerulion_netd::DemandRegistry,
    ) -> Result<(), String> {
        self.0.lock().unwrap().push(
            cerulion_netd::account_access::AccountAccessRequest::InstallPaired {
                robots: robots.to_vec(),
            },
        );
        Ok(())
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
                if let Some(chain) = &snapshot.owner_chain {
                    let proof: cerulion_pairing::verify::OwnerCertificatePresentationWire =
                        postcard::from_bytes(chain).unwrap();
                    assert_eq!(
                        proof.device_cert.cert.device_key.0,
                        ed25519_dalek::SigningKey::from_bytes(&[0x73; 32])
                            .verifying_key()
                            .to_bytes()
                    );
                    assert_eq!(
                        proof.device_cert.cert.account.0,
                        snapshot.pairing_account_id
                    );
                }
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
            AccountAccessRequest::Discover { .. } => {
                let actions = self.0.lock().unwrap();
                let snapshot = actions
                    .iter()
                    .rev()
                    .find_map(|action| match action {
                        AccountAccessRequest::Install { snapshot } => Some(snapshot),
                        _ => None,
                    })
                    .unwrap();
                let mut robots: Vec<_> = snapshot
                    .robots
                    .iter()
                    .map(|robot| cerulion_netd::account_access::RobotCatalog {
                        route: cerulion_netd::account_access::robot_route(&robot.robot_id),
                        name: robot.hostname.clone(),
                        endpoint_key: robot.endpoint_key,
                        trust: cerulion_netd::account_access::RobotTrustSource::Account,
                        presence: RobotPresence::Unknown {
                            reason: "test plane made no reachability observation".into(),
                        },
                        path: cerulion_netd::account_access::RobotPath::Unknown,
                        catalog: None,
                    })
                    .collect();
                if let Some(pins) = actions.iter().rev().find_map(|action| match action {
                    AccountAccessRequest::InstallPaired { robots } => Some(robots),
                    _ => None,
                }) {
                    for pin in pins {
                        if !robots
                            .iter()
                            .any(|robot| robot.endpoint_key == Some(pin.endpoint_key))
                        {
                            robots.push(cerulion_netd::account_access::RobotCatalog {
                                route: pin.name.clone(),
                                name: pin.name.clone(),
                                endpoint_key: Some(pin.endpoint_key),
                                trust: cerulion_netd::account_access::RobotTrustSource::Paired,
                                presence: RobotPresence::Unknown {
                                    reason: "test plane made no reachability observation".into(),
                                },
                                path: cerulion_netd::account_access::RobotPath::Unknown,
                                catalog: None,
                            });
                        }
                    }
                }
                Ok(AccountAccessReply::Discovery { robots })
            }
            _ => panic!("unexpected account operation"),
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
    assert_eq!(actions.len(), 5);
    assert!(matches!(&actions[0], AccountAccessRequest::Install { .. }));
    assert!(
        matches!(&actions[1], AccountAccessRequest::Probe { robot_id, .. } if *robot_id == fixture.robot_id)
    );
    assert!(matches!(&actions[2], AccountAccessRequest::Install { .. }));
    assert!(matches!(
        &actions[3],
        AccountAccessRequest::InstallPaired { .. }
    ));
    assert!(matches!(&actions[4], AccountAccessRequest::Discover { .. }));
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
fn read_only_directory_listing_does_not_spend_a_refresh_token() {
    use std::time::{Duration, Instant};
    // The read-only worker cannot rotate tokens. Automatic discovery refreshes
    // separately on its calling thread before using this listing helper.
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

#[test]
#[serial]
fn a_listing_waits_out_a_busy_controller_and_installs() {
    use cerulion_netd::account_access::{
        AccountAccessReply, AccountAccessRequest, AccountAccessResponse, CONTROLLER_BUSY,
    };
    use cerulion_netd::protocol::{ErrorResponse, Hello, Request, Response};
    use std::io::{BufRead, BufReader, Write};
    use std::os::unix::net::UnixListener;
    use std::time::Duration;
    let fixture = Fixture::new();
    let listener = UnixListener::bind(fixture.socket()).unwrap();
    let worker = std::thread::spawn(move || {
        let (mut stream, _) = listener.accept().unwrap();
        stream
            .set_read_timeout(Some(Duration::from_secs(6)))
            .unwrap();
        writeln!(stream, "{}", Hello::new().to_json_line()).unwrap();
        let mut reader = BufReader::new(stream.try_clone().unwrap());
        let mut installs = 0;
        loop {
            let mut line = String::new();
            if !matches!(reader.read_line(&mut line), Ok(n) if n > 0) {
                return installs;
            }
            let Ok(Request::AccountAccess {
                id,
                action: AccountAccessRequest::Install { snapshot },
            }) = serde_json::from_str(&line)
            else {
                return installs;
            };
            installs += 1;
            let response = if installs == 1 {
                Response::Error(ErrorResponse {
                    id: Some(id),
                    error: CONTROLLER_BUSY.into(),
                    robot: None,
                    topic: None,
                })
            } else {
                Response::AccountAccess(AccountAccessResponse {
                    id,
                    account_access: AccountAccessReply::Installed {
                        robot_count: snapshot.robots.len(),
                    },
                })
            };
            writeln!(stream, "{}", response.to_json_line()).unwrap();
        }
    });
    let listing = account_robot_access::list().unwrap();
    let installs = worker.join().unwrap();
    assert_eq!(installs, 2, "the busy install was retried");
    assert!(
        !listing
            .diagnostic
            .as_deref()
            .is_some_and(|text| text.contains(CONTROLLER_BUSY)),
        "{:?}",
        listing.diagnostic
    );
    assert_eq!(listing.rows.len(), 1);
}

#[test]
#[serial]
fn automatic_listing_unions_owned_and_paired_robots_and_preserves_the_cached_directory() {
    let fixture = Fixture::new();
    let paired_key = ed25519_dalek::SigningKey::from_bytes(&[0x75; 32])
        .verifying_key()
        .to_bytes();
    std::fs::write(
        fixture.home.path().join("robots.toml"),
        format!("[robots]\npaired = \"{}\"\n", hex::encode(paired_key)),
    )
    .unwrap();
    let spy = Arc::new(AccessSpy::default());
    let mut daemon =
        cerulion_netd::start(fixture.socket(), spy, cerulion_netd::NetdConfig::default()).unwrap();
    let listing = account_robot_access::automatic::discover().unwrap();
    assert!(listing.diagnostics.is_empty(), "{:?}", listing.diagnostics);
    assert_eq!(
        listing
            .robots
            .iter()
            .map(|robot| robot.name.as_str())
            .collect::<Vec<_>>(),
        ["owned-robot", "paired"]
    );
    assert_eq!(
        listing.robots[0].trust,
        cerulion_netd::account_access::RobotTrustSource::Account
    );
    assert_eq!(
        listing.robots[1].trust,
        cerulion_netd::account_access::RobotTrustSource::Paired
    );
    drop(listing);
    let offline = EnvGuard::set("CERULION_ACCOUNT_SERVICE", "http://127.0.0.1:9");
    let cached = account_robot_access::automatic::discover().unwrap();
    assert_eq!(
        cached
            .robots
            .iter()
            .map(|robot| robot.name.as_str())
            .collect::<Vec<_>>(),
        ["owned-robot", "paired"]
    );
    assert!(!cached.diagnostics.is_empty());
    drop(cached);
    let killed = EnvGuard::set("CERULION_NETWORK", "off");
    let local = account_robot_access::automatic::discover().unwrap();
    assert!(local.robots.is_empty() && local.diagnostics.is_empty());
    drop(local);
    drop(killed);
    drop(offline);
    assert_eq!(daemon.active_demand_count(), 0);
    daemon.shutdown();
}

#[test]
#[serial]
fn a_broken_saved_pairing_does_not_block_local_or_account_discovery() {
    let fixture = Fixture::new();
    std::fs::write(
        fixture.home.path().join("robots.toml"),
        "[robots]\ninvalid = \"not-a-key\"\n",
    )
    .unwrap();
    assert!(account_robot_access::automatic::paired_robots().is_err());
    let spy = Arc::new(AccessSpy::default());
    let mut daemon =
        cerulion_netd::start(fixture.socket(), spy, cerulion_netd::NetdConfig::default()).unwrap();
    let listing = account_robot_access::automatic::discover().unwrap();
    assert_eq!(listing.robots.len(), 1);
    assert_eq!(listing.robots[0].name, "owned-robot");
    assert!(!listing.diagnostics.is_empty());
    drop(listing);
    let path = fixture.home.path().join("auth.json");
    std::fs::remove_file(path).unwrap();
    let local = account_robot_access::automatic::discover().unwrap();
    assert!(local.robots.is_empty());
    assert!(!local.diagnostics.is_empty());
    drop(local);
    assert_eq!(daemon.active_demand_count(), 0);
    daemon.shutdown();
}

#[test]
#[serial]
fn paired_posture_lists_and_reads_owned_topics_without_a_lan_catalog_request() {
    use cerulion_cli_engine::topic_cmd::{self, introspect, TopicScope};
    use cerulion_core::{CatalogEntry, CatalogProvenance};
    use cerulion_netd::{MirrorPlane, TopicKey};
    struct ReadPlane {
        account: AccessSpy,
        topic: String,
        hash: u64,
        demands: std::sync::atomic::AtomicUsize,
    }
    impl MirrorPlane for ReadPlane {
        fn local_channel(&self, key: &TopicKey) -> String {
            key.local_topic()
        }
        fn ensure_mirror(
            &self,
            key: &TopicKey,
            hash: u64,
        ) -> Result<(), cerulion_netd::MirrorError> {
            assert!(key.observer);
            assert!(key.robot.starts_with("account:"));
            assert_eq!(key.topic, self.topic);
            assert_eq!(hash, self.hash);
            self.demands
                .fetch_add(1, std::sync::atomic::Ordering::SeqCst);
            Ok(())
        }
        fn release_mirror(&self, _: &TopicKey) -> cerulion_netd::MirrorRelease {
            cerulion_netd::MirrorRelease::Retired
        }
        fn paired_catalog(&self, _: &str) -> Option<Result<cerulion_core::CatalogReply, String>> {
            panic!("paired default uses the authenticated discovery result, without a LAN catalog request")
        }
        fn install_paired_robots(
            &self,
            robots: &[cerulion_netd::account_access::PairedRobot],
            reg: &mut cerulion_netd::DemandRegistry,
        ) -> Result<(), String> {
            self.account.install_paired_robots(robots, reg)
        }
        fn account_access(
            &self,
            action: &cerulion_netd::account_access::AccountAccessRequest,
        ) -> Result<cerulion_netd::account_access::AccountAccessReply, String> {
            let mut reply = self.account.account_access(action)?;
            if let cerulion_netd::account_access::AccountAccessReply::Discovery { robots } =
                &mut reply
            {
                for robot in robots {
                    robot.catalog =
                        Some(cerulion_core::transport::cerulion_q::build_catalog_reply(
                            &robot.route,
                            [CatalogEntry {
                                topic: self.topic.clone(),
                                schema_name: Some("geometry_msgs/Vector3".into()),
                                schema_hash: Some(self.hash),
                                provenance: CatalogProvenance::Runtime,
                                producer_count: None,
                                liveness: None,
                            }],
                        ));
                }
            }
            Ok(reply)
        }
    }
    let fixture = Fixture::new();
    let _paired = EnvGuard::set("CERULION_LAN_ACCESS", "paired");
    let topic = format!("/account_observer/{}", std::process::id());
    let schema = cerulion_core::codegen::parse_rosmsg(
        "float64 x\nfloat64 y\nfloat64 z\n",
        "Vector3",
        Some("geometry_msgs"),
    )
    .unwrap();
    let (walker, warnings) = cerulion_core::codegen::FrameWalker::new(vec![schema]);
    assert!(warnings.is_empty());
    let hash = walker.schema_hash_for("geometry_msgs/Vector3").unwrap();
    let plane = Arc::new(ReadPlane {
        account: AccessSpy::default(),
        topic: topic.clone(),
        hash,
        demands: std::sync::atomic::AtomicUsize::new(0),
    });
    let mut daemon = cerulion_netd::start(
        fixture.socket(),
        plane.clone(),
        cerulion_netd::NetdConfig::default(),
    )
    .unwrap();
    let mut out = Vec::new();
    let mut err = Vec::new();
    introspect::print_topic_list(
        false,
        introspect::TypeColumn::Shown,
        topic_cmd::remote_discovery_options(false, vec![], vec![]),
        false,
        None,
        &mut out,
        &mut err,
    )
    .unwrap();
    assert!(err.is_empty(), "{}", String::from_utf8_lossy(&err));
    let out = String::from_utf8(out).unwrap();
    assert!(out.contains("owned-robot"));
    assert!(out.contains(&format!(
        "{topic}  [geometry_msgs/Vector3]  robot=owned-robot"
    )));
    let channel = TopicKey::observed(
        cerulion_netd::account_access::robot_route(&fixture.robot_id),
        &topic,
    )
    .local_topic();
    let (stop, stopped) = mpsc::channel::<()>();
    let (ready, started) = mpsc::channel();
    let producer = std::thread::spawn(move || {
        let manager = cerulion_core::TransportManager::get_or_init().unwrap();
        let mut publisher = manager
            .create_publisher_simple(&channel, cerulion_core::wire::MaxSliceLen::const_new(256))
            .unwrap();
        let header = cerulion_core::WireHeader {
            schema_hash: hash,
            total_size: (cerulion_core::WireHeader::SIZE + 24) as u32,
            offset_table_offset: (cerulion_core::WireHeader::SIZE + 24) as u32,
            offset_table_count: 0,
            sequence: 17,
            timestamp_ns: 123_456_789,
            step_tag: cerulion_core::wire::StepTag::UNTAGGED,
        };
        let mut frame = vec![0; cerulion_core::WireHeader::SIZE + 24];
        header.write_to_buf(&mut frame);
        for (index, value) in [1.0_f64, 2.0, 3.0].into_iter().enumerate() {
            let offset = cerulion_core::WireHeader::SIZE + index * 8;
            frame[offset..offset + 8].copy_from_slice(&value.to_le_bytes());
        }
        ready.send(()).unwrap();
        while matches!(stopped.try_recv(), Err(mpsc::TryRecvError::Empty)) {
            publisher.publish_raw(&frame).unwrap();
            std::thread::sleep(std::time::Duration::from_millis(10));
        }
    });
    started
        .recv_timeout(std::time::Duration::from_secs(5))
        .unwrap();
    let info = topic_cmd::topic_info_with_selection(
        &topic,
        None,
        TopicScope::Automatic,
        Some("owned-robot"),
    )
    .unwrap();
    assert!(info.contains("geometry_msgs/Vector3"));
    assert!(info.contains("Last sequence: 17"));
    assert!(info.contains("Last timestamp: 123456789ns"));
    assert_eq!(plane.demands.load(std::sync::atomic::Ordering::SeqCst), 1);
    let released = std::time::Instant::now() + std::time::Duration::from_secs(5);
    while daemon.active_demand_count() != 0 {
        assert!(
            std::time::Instant::now() < released,
            "closed observer connection must release its demand"
        );
        std::thread::sleep(std::time::Duration::from_millis(10));
    }
    drop(stop);
    producer.join().unwrap();
    daemon.shutdown();
}

fn expire_session(fixture: &Fixture) -> auth::AuthState {
    let path = fixture.home.path().join("auth.json");
    let mut state = auth::load_from(&path).state().unwrap().clone();
    state.expires_at_ns = 1;
    auth::write_to(&path, &state).unwrap();
    state
}

#[test]
#[serial]
fn automatic_discovery_rotates_an_expired_session_before_listing_robots() {
    let fixture = Fixture::new();
    let stale = expire_session(&fixture);
    let mut daemon = cerulion_netd::start(
        fixture.socket(),
        Arc::new(AccessSpy::default()),
        cerulion_netd::NetdConfig::default(),
    )
    .unwrap();
    let listing = account_robot_access::automatic::discover().unwrap();
    assert!(listing.diagnostics.is_empty(), "{:?}", listing.diagnostics);
    assert!(!listing.serves_cached_directory());
    assert_eq!(listing.robots.len(), 1);
    assert_eq!(listing.robots[0].name, "owned-robot");
    assert_eq!(
        listing.robots[0].route,
        cerulion_netd::account_access::robot_route(&fixture.robot_id)
    );
    let loaded = auth::load_from(&fixture.home.path().join("auth.json"));
    let rotated = loaded.state().unwrap();
    assert_ne!(rotated.session_token, stale.session_token);
    assert_ne!(rotated.refresh_token, stale.refresh_token);
    assert!(rotated.session_is_valid(auth::now_unix_ns()));
    assert_eq!(rotated.account_id, stale.account_id);
    drop(listing);
    daemon.shutdown();
}

#[test]
#[serial]
fn automatic_discovery_bounds_a_blackholed_refresh_and_serves_the_cache() {
    use std::time::{Duration, Instant};
    let fixture = Fixture::new();
    cerulion_cli_engine::account_robots::fetch()
        .unwrap()
        .cache_for_offline()
        .unwrap();
    let stale = expire_session(&fixture);
    let listener = std::net::TcpListener::bind("127.0.0.1:0").unwrap();
    let _service = EnvGuard::set(
        "CERULION_ACCOUNT_SERVICE",
        format!("http://{}", listener.local_addr().unwrap()),
    );
    let (stop, stopped) = mpsc::channel();
    let (connected, received) = mpsc::channel();
    let blackhole = std::thread::spawn(move || {
        let (stream, _) = listener.accept().unwrap();
        connected.send(()).unwrap();
        stopped.recv_timeout(Duration::from_secs(5)).unwrap();
        drop(stream);
    });
    let mut daemon = cerulion_netd::start(
        fixture.socket(),
        Arc::new(AccessSpy::default()),
        cerulion_netd::NetdConfig::default(),
    )
    .unwrap();
    let started = Instant::now();
    let listing = account_robot_access::automatic::discover().unwrap();
    let elapsed = started.elapsed();
    assert!(elapsed >= Duration::from_secs(2), "{elapsed:?}");
    assert!(elapsed < Duration::from_secs(3), "{elapsed:?}");
    received.recv_timeout(Duration::from_secs(1)).unwrap();
    assert!(listing.serves_cached_directory());
    assert_eq!(listing.robots.len(), 1);
    assert_eq!(listing.robots[0].name, "owned-robot");
    assert!(listing
        .diagnostics
        .iter()
        .any(|row| row.contains("unreachable")));
    assert!(listing
        .diagnostics
        .iter()
        .any(|row| row.contains("revocation")));
    let loaded = auth::load_from(&fixture.home.path().join("auth.json"));
    assert_eq!(
        loaded.state().unwrap(),
        &stale,
        "no token rotation was abandoned"
    );
    drop(listing);
    stop.send(()).unwrap();
    blackhole.join().unwrap();
    daemon.shutdown();
}

#[test]
#[serial]
#[tracing_test::traced_test]
fn local_topic_echo_on_an_expired_session_desk_delivers_without_warns() {
    use cerulion_cli_engine::topic_cmd::{self, TopicScope};
    use std::sync::atomic::{AtomicBool, Ordering};
    use std::time::Duration;
    let fixture = Fixture::new();
    cerulion_cli_engine::account_robots::fetch()
        .unwrap()
        .cache_for_offline()
        .unwrap();
    let stale = expire_session(&fixture);
    let _service = EnvGuard::set("CERULION_ACCOUNT_SERVICE", "http://127.0.0.1:9");
    let mut daemon = cerulion_netd::start(
        fixture.socket(),
        Arc::new(AccessSpy::default()),
        cerulion_netd::NetdConfig::default(),
    )
    .unwrap();
    let topic = format!("/expired_session_echo_{}", std::process::id());
    let walker = vector3_walker();
    let hash = walker.schema_hash_for("geometry_msgs/Vector3").unwrap();
    let running = Arc::new(AtomicBool::new(true));
    struct StopAfterDelivery {
        bytes: Vec<u8>,
        running: Arc<AtomicBool>,
    }
    impl std::io::Write for StopAfterDelivery {
        fn write(&mut self, bytes: &[u8]) -> std::io::Result<usize> {
            self.bytes.extend_from_slice(bytes);
            if self.bytes.windows(4).any(|window| window == b"3.25") {
                self.running.store(false, Ordering::SeqCst);
            }
            Ok(bytes.len())
        }
        fn flush(&mut self) -> std::io::Result<()> {
            Ok(())
        }
    }
    let writer_running = running.clone();
    let (ready, begun) = mpsc::channel();
    let producing = running.clone();
    let producer_topic = topic.clone();
    let producer = std::thread::spawn(move || {
        let manager = cerulion_core::TransportManager::get_or_init().unwrap();
        let mut publisher = manager
            .create_publisher_simple(
                &producer_topic,
                cerulion_core::wire::MaxSliceLen::const_new(256),
            )
            .unwrap();
        let header = cerulion_core::WireHeader {
            schema_hash: hash,
            total_size: 64,
            offset_table_offset: 64,
            offset_table_count: 0,
            sequence: 19,
            timestamp_ns: 123_456_789,
            step_tag: cerulion_core::wire::StepTag::UNTAGGED,
        };
        let mut frame = [0; 64];
        header.write_to_buf(&mut frame);
        for (index, value) in [1.5_f64, -2.5, 3.25].into_iter().enumerate() {
            let offset = 40 + index * 8;
            frame[offset..offset + 8].copy_from_slice(&value.to_le_bytes());
        }
        ready.send(()).unwrap();
        let deadline = std::time::Instant::now() + Duration::from_secs(5);
        while producing.load(Ordering::SeqCst) && std::time::Instant::now() < deadline {
            publisher.publish_raw(&frame).unwrap();
            std::thread::sleep(Duration::from_millis(10));
        }
        producing.store(false, Ordering::SeqCst);
    });
    begun.recv_timeout(Duration::from_secs(5)).unwrap();
    let mut writer = StopAfterDelivery {
        bytes: Vec::new(),
        running: writer_running,
    };
    let echoed = topic_cmd::topic_echo_with_selection(
        &topic,
        None,
        running,
        &mut writer,
        16,
        TopicScope::Automatic,
        None,
    );
    producer.join().unwrap();
    echoed.unwrap();
    let output = String::from_utf8(writer.bytes).unwrap();
    assert!(output.contains("x: 1.5"), "{output}");
    assert!(output.contains("y: -2.5"), "{output}");
    assert!(output.contains("z: 3.25"), "{output}");
    logs_assert(|lines| {
        let warnings: Vec<_> = lines
            .iter()
            .filter(|line| line.contains(" WARN "))
            .collect();
        if warnings.is_empty() {
            Ok(())
        } else {
            Err(format!("unexpected warnings: {warnings:?}"))
        }
    });
    let mut out = Vec::new();
    let mut err = Vec::new();
    topic_cmd::introspect::print_topic_list(
        false,
        topic_cmd::introspect::TypeColumn::Hidden,
        topic_cmd::remote_discovery_options(false, vec![], vec![]),
        false,
        None,
        &mut out,
        &mut err,
    )
    .unwrap();
    let diagnostic = String::from_utf8(err).unwrap();
    assert_eq!(diagnostic.lines().count(), 1, "{diagnostic}");
    assert!(diagnostic.contains("unreachable"), "{diagnostic}");
    assert!(String::from_utf8(out).unwrap().contains("owned-robot"));
    let loaded = auth::load_from(&fixture.home.path().join("auth.json"));
    assert_eq!(loaded.state().unwrap(), &stale);
    let explicit = topic_cmd::topic_info_with_selection(
        &topic,
        None,
        TopicScope::Automatic,
        Some("owned-robot"),
    )
    .unwrap_err();
    assert!(
        explicit.to_string().contains("unknown reachability"),
        "{explicit}"
    );
    daemon.shutdown();
}

fn vector3_walker() -> cerulion_core::codegen::FrameWalker {
    let schema = cerulion_core::codegen::parse_rosmsg(
        "float64 x\nfloat64 y\nfloat64 z\n",
        "Vector3",
        Some("geometry_msgs"),
    )
    .unwrap();
    let (walker, warnings) = cerulion_core::codegen::FrameWalker::new(vec![schema]);
    assert!(warnings.is_empty());
    walker
}
