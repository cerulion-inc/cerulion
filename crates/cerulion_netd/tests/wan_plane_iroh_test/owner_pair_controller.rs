// SPDX-License-Identifier: AGPL-3.0-only
//! Account snapshots and lifecycle over the real netd socket and robot transport.

use super::*;
use base64::{engine::general_purpose::URL_SAFE_NO_PAD, Engine};
use cerulion_core::transport::demand_authorizer::AllowAllAuthorizer;
use cerulion_netd::account_access::{
    robot_route, AccountAccessReply, AccountAccessRequest, AccountRobot, AccountSnapshot,
    RobotPresence,
};
use cerulion_netd::account_controller::{AccountControllerConfig, AccountWanController};
use cerulion_netd::{NetdClient, NetdConfig};

struct Login {
    directory: tempfile::TempDir,
    seed: [u8; 32],
}

impl Login {
    fn new() -> Self {
        let login = Self {
            directory: tempfile::tempdir().unwrap(),
            seed: unique_secret(),
        };
        std::fs::write(
            login
                .directory
                .path()
                .join(cerulion_discovery::robot_state::AUTH_STORE_LOCK_FILE),
            [],
        )
        .unwrap();
        login.write(&proof(login.seed, OWNER));
        login
    }

    fn write(&self, proof: &OwnerCertificatePresentationWire) {
        let lock = std::fs::OpenOptions::new()
            .read(true)
            .write(true)
            .open(
                self.directory
                    .path()
                    .join(cerulion_discovery::robot_state::AUTH_STORE_LOCK_FILE),
            )
            .unwrap();
        lock.lock().unwrap();
        let encode = |bytes: Vec<u8>| URL_SAFE_NO_PAD.encode(bytes);
        let leaf = encode(postcard::to_stdvec(&proof.device_cert).unwrap());
        let issuer = encode(postcard::to_stdvec(&proof.intermediate).unwrap());
        std::fs::write(
            self.directory.path().join("auth.json"),
            serde_json::to_vec(&serde_json::json!({
                "account_id": URL_SAFE_NO_PAD.encode(OWNER.0), "session_token":"expired",
                "refresh_token":"refresh", "expires_at_ns":1, "logged_in_ever":true,
            }))
            .unwrap(),
        )
        .unwrap();
        std::fs::write(self.directory.path().join("desk.key"), self.seed).unwrap();
        std::fs::write(self.directory.path().join("device.cert"), &leaf).unwrap();
        std::fs::write(
            self.directory.path().join("device-chain.json"),
            serde_json::to_vec(&serde_json::json!({
                "device_cert":leaf, "intermediate":issuer,
            }))
            .unwrap(),
        )
        .unwrap();
    }

    fn snapshot(&self, robots: &[(RobotId, &OwnerRobot)]) -> AccountSnapshot {
        let identity = cerulion_netd::identity_snapshot::load_at(self.directory.path()).unwrap();
        AccountSnapshot {
            auth_account_id: identity.auth_account_id().into(),
            pairing_account_id: identity.account().0,
            device_key: identity.device_key().0,
            owner_chain: identity.owner_chain_wire().map(Vec::from),
            robots: robots
                .iter()
                .map(|(id, robot)| AccountRobot {
                    robot_id: id.0,
                    hostname: "shared-display-label".into(),
                    endpoint_key: Some(*robot.fixture.eid.as_bytes()),
                })
                .collect(),
        }
    }
}

struct Producers {
    stop: Arc<AtomicBool>,
    handles: Vec<std::thread::JoinHandle<()>>,
}
impl Producers {
    fn new() -> Self {
        Self {
            stop: Arc::new(AtomicBool::new(false)),
            handles: Vec::new(),
        }
    }
    fn start(&mut self, robot: &OwnerRobot, topic: &str) {
        self.handles.push(spawn_producer(
            robot.fixture.manager.clone(),
            topic.into(),
            self.stop.clone(),
        ));
        let deadline = Instant::now() + DIAL_TIMEOUT;
        while !robot
            .fixture
            .manager
            .list_topics()
            .unwrap()
            .contains(&topic.to_owned())
        {
            assert!(
                Instant::now() < deadline,
                "producer topic was not registered"
            );
            std::thread::sleep(Duration::from_millis(5));
        }
    }
}
impl Drop for Producers {
    fn drop(&mut self) {
        self.stop.store(true, Ordering::Relaxed);
        for handle in self.handles.drain(..) {
            let _ = handle.join();
        }
    }
}

struct Desk {
    daemon: cerulion_netd::daemon::RunningNetd,
    manager: Arc<TransportManager>,
    socket: std::path::PathBuf,
    _directory: tempfile::TempDir,
}
impl Desk {
    fn start(
        login: &Login,
        robots: &[(RobotId, &OwnerRobot)],
        network: bool,
        serving: bool,
    ) -> Self {
        let manager = test_manager("account_controller");
        let direct = robots
            .iter()
            .map(|(id, robot)| {
                (
                    (id.0, *robot.fixture.eid.as_bytes()),
                    robot.fixture.addrs.clone(),
                )
            })
            .collect();
        let controller = Arc::new(
            AccountWanController::new(
                manager.clone(),
                WanRegistry::new(HashMap::new(), unique_secret(), RelayConfig::Disabled),
                Arc::new(AllowAllAuthorizer),
                AccountControllerConfig {
                    config_home: Some(login.directory.path().to_owned()),
                    network_enabled: network,
                    serving_gateway: serving,
                    epoch_dir: None,
                    trusted_direct: direct,
                },
            )
            .unwrap(),
        );
        let directory = tempfile::tempdir_in("/tmp").unwrap();
        let socket = directory.path().join("netd.sock");
        let daemon = cerulion_netd::start(
            socket.clone(),
            controller,
            NetdConfig {
                account_identity_watch: true,
                ..NetdConfig::default()
            },
        )
        .unwrap();
        Self {
            daemon,
            manager,
            socket,
            _directory: directory,
        }
    }
    fn client(&self) -> NetdClient {
        NetdClient::connect_existing_at(self.socket.clone()).unwrap()
    }
    fn install(&self, snapshot: AccountSnapshot) {
        let count = snapshot.robots.len();
        assert_eq!(
            self.client()
                .account_access_once(AccountAccessRequest::Install {
                    snapshot: Box::new(snapshot)
                })
                .unwrap(),
            AccountAccessReply::Installed { robot_count: count }
        );
    }
}
impl Drop for Desk {
    fn drop(&mut self) {
        self.daemon.shutdown();
    }
}

fn pairs(robot: &OwnerRobot) -> usize {
    robot
        .receipts()
        .iter()
        .filter(|receipt| receipt["verb"] == "pair" && receipt["outcome"]["outcome"] == "ok")
        .count()
}

#[test]
fn account_install_probe_query_refresh_and_remove_cross_real_daemon_and_deliver() {
    let a = OwnerRobot::start_with_robot_id("controller_a", RobotId([5; 32]), None, None);
    let b = OwnerRobot::start_with_robot_id("controller_b", RobotId([6; 32]), None, None);
    let login = Login::new();
    let desk = Desk::start(
        &login,
        &[(RobotId([5; 32]), &a), (RobotId([6; 32]), &b)],
        true,
        false,
    );
    let mut producers = Producers::new();
    let topic_a = format!("/controller/a/{}", unique_id());
    let topic_b = format!("/controller/b/{}", unique_id());
    producers.start(&a, &topic_a);
    producers.start(&b, &topic_b);
    desk.install(login.snapshot(&[(RobotId([5; 32]), &a), (RobotId([6; 32]), &b)]));
    assert!(a.receipts().is_empty() && b.receipts().is_empty());
    let mut a_client = desk.client();
    assert_eq!(
        a_client
            .account_access_once(AccountAccessRequest::Probe {
                robot_id: [5; 32],
                budget_ms: 4000
            })
            .unwrap(),
        AccountAccessReply::Presence {
            robot_id: [5; 32],
            presence: RobotPresence::Online
        }
    );
    assert_eq!(a.binding(&login.seed), None);
    assert!(a.receipts().is_empty());
    let AccountAccessReply::Catalog { catalog, .. } = a_client
        .account_access_once(AccountAccessRequest::Catalog { robot_id: [5; 32] })
        .unwrap()
    else {
        panic!("catalog reply required")
    };
    assert_eq!(catalog.entries.len(), 1);
    assert_eq!(catalog.entries[0].topic, topic_a);
    assert_eq!(catalog.entries[0].schema_hash, Some(ORACLE_SCHEMA_HASH));
    assert_eq!(pairs(&a), 1);
    a_client
        .demand(&robot_route(&[5; 32]), &topic_a, ORACLE_SCHEMA_HASH)
        .unwrap();
    let mut b_client = desk.client();
    b_client
        .demand(&robot_route(&[6; 32]), &topic_b, ORACLE_SCHEMA_HASH)
        .unwrap();
    assert_oracle_and_gap_free(&collect_frames(&desk.manager, &topic_a, 3));
    assert_oracle_and_gap_free(&collect_frames(&desk.manager, &topic_b, 3));

    let mut refreshed = proof(login.seed, OWNER).as_ref().clone();
    refreshed.device_cert.cert.issued_at_ns += 1;
    refreshed.device_cert = refreshed.device_cert.cert.clone().sign(&signing_key(10));
    login.write(&refreshed);
    desk.install(login.snapshot(&[(RobotId([5; 32]), &a), (RobotId([6; 32]), &b)]));
    assert_eq!(desk.daemon.active_demand_count(), 2);
    assert_oracle_and_gap_free(&collect_frames(&desk.manager, &topic_a, 3));
    assert_oracle_and_gap_free(&collect_frames(&desk.manager, &topic_b, 3));
    assert_eq!(pairs(&a), 1);
    assert_eq!(pairs(&b), 1);

    desk.install(login.snapshot(&[(RobotId([6; 32]), &b)]));
    assert_eq!(desk.daemon.active_demand_count(), 1);
    assert!(gather_absent(&desk.manager, &topic_a));
    assert_oracle_and_gap_free(&collect_frames(&desk.manager, &topic_b, 3));
    assert!(a_client
        .demand(&robot_route(&[5; 32]), &topic_a, ORACLE_SCHEMA_HASH)
        .is_err());
    desk.install(login.snapshot(&[(RobotId([5; 32]), &a), (RobotId([6; 32]), &b)]));
    let mut new_a = desk.client();
    new_a
        .demand(&robot_route(&[5; 32]), &topic_a, ORACLE_SCHEMA_HASH)
        .unwrap();
    drop(a_client);
    assert_oracle_and_gap_free(&collect_frames(&desk.manager, &topic_a, 3));
    assert_oracle_and_gap_free(&collect_frames(&desk.manager, &topic_b, 3));
    assert_eq!(desk.daemon.active_demand_count(), 2);
}

#[test]
fn key_change_and_logout_retire_live_holds_before_reenrollment() {
    let robot = OwnerRobot::start("controller_key_change", None, None);
    let mut login = Login::new();
    let desk = Desk::start(&login, &[(ROBOT, &robot)], true, false);
    let mut producers = Producers::new();
    let topic = format!("/controller/key/{}", unique_id());
    producers.start(&robot, &topic);
    desk.install(login.snapshot(&[(ROBOT, &robot)]));
    let mut old = desk.client();
    old.demand(&robot_route(&ROBOT.0), &topic, ORACLE_SCHEMA_HASH)
        .unwrap();
    assert_oracle_and_gap_free(&collect_frames(&desk.manager, &topic, 3));
    let old_seed = login.seed;
    login.seed = unique_secret();
    login.write(&proof(login.seed, OWNER));
    let deadline = Instant::now() + Duration::from_secs(3);
    while desk.daemon.active_demand_count() != 0 {
        assert!(
            Instant::now() < deadline,
            "watcher did not retire old identity holds"
        );
        std::thread::sleep(Duration::from_millis(20));
    }
    assert!(gather_absent(&desk.manager, &topic));
    assert!(old
        .demand(&robot_route(&ROBOT.0), &topic, ORACLE_SCHEMA_HASH)
        .is_err());
    desk.install(login.snapshot(&[(ROBOT, &robot)]));
    let mut new = desk.client();
    new.demand(&robot_route(&ROBOT.0), &topic, ORACLE_SCHEMA_HASH)
        .unwrap();
    drop(old);
    assert_oracle_and_gap_free(&collect_frames(&desk.manager, &topic, 3));
    assert_eq!(robot.binding(&old_seed), Some(OWNER));
    assert_eq!(robot.binding(&login.seed), Some(OWNER));
    assert_eq!(pairs(&robot), 2);
    std::fs::remove_file(login.directory.path().join("auth.json")).unwrap();
    let deadline = Instant::now() + Duration::from_secs(3);
    while desk.daemon.active_demand_count() != 0 {
        assert!(
            Instant::now() < deadline,
            "watcher did not retire logged-out holds"
        );
        std::thread::sleep(Duration::from_millis(20));
    }
    assert!(gather_absent(&desk.manager, &topic));
    assert!(new
        .demand(&robot_route(&ROBOT.0), &topic, ORACLE_SCHEMA_HASH)
        .is_err());
}

#[test]
fn failed_additional_topic_retires_held_robot_topics_before_reuse() {
    let robot = OwnerRobot::start("controller_failed_topic", None, None);
    let login = Login::new();
    let desk = Desk::start(&login, &[(ROBOT, &robot)], true, false);
    let mut producers = Producers::new();
    let topic = format!("/controller/recover/{}", unique_id());
    producers.start(&robot, &topic);
    desk.install(login.snapshot(&[(ROBOT, &robot)]));
    let mut first = desk.client();
    first
        .demand(&robot_route(&ROBOT.0), &topic, ORACLE_SCHEMA_HASH)
        .unwrap();
    assert_oracle_and_gap_free(&collect_frames(&desk.manager, &topic, 3));
    let mut second = desk.client();
    assert!(second
        .demand(
            &robot_route(&ROBOT.0),
            "/controller/not-published",
            ORACLE_SCHEMA_HASH
        )
        .is_err());
    let restored = second
        .demand(&robot_route(&ROBOT.0), &topic, ORACLE_SCHEMA_HASH)
        .unwrap();
    assert!(
        restored.mirror_created,
        "retired reader must not classify as AlreadyMirrored"
    );
    drop(first);
    assert_oracle_and_gap_free(&collect_frames(&desk.manager, &topic, 3));
    assert_eq!(desk.daemon.active_demand_count(), 1);
    assert_eq!(pairs(&robot), 1);
}

#[test]
fn new_demand_after_actual_reader_death_reopens_same_identity_and_delivers() {
    let login = Login::new();
    let mut robot = OwnerRobot::start(
        "controller_natural_death",
        None,
        Some((public_of(&login.seed), OWNER)),
    );
    // This one fixture captures an actual accepted connection so it can close
    // the transport while retaining its endpoint, key and direct addresses.
    // Authentication and wire serving still use production handlers and the
    // real durable owner store/index. Other cases cover full owner enrollment.
    robot.fixture._serve.abort();
    robot.fixture.rt.block_on(async {
        let _ = (&mut robot.fixture._serve).await;
    });
    let authz = Arc::new(PairingAuthorizer::new(
        TrustStore::load(&robot.config.store_file, MAC).unwrap(),
        DeviceAccountIndex::load(&robot.config.index_file, MAC).unwrap(),
    ));
    let plane = Arc::new(WirePlane::with_manager(
        "natural-death-robot",
        robot.fixture.manager.clone(),
    ));
    let accepted_connections = Arc::new(std::sync::Mutex::new(Vec::new()));
    let connections = accepted_connections.clone();
    let endpoint = robot.fixture.endpoint.clone();
    robot.fixture._serve = robot.fixture.rt.spawn(async move {
        while let Ok(Some(accepted)) = accept_one(&endpoint).await {
            connections
                .lock()
                .unwrap()
                .push(accepted.connection.clone());
            handle_accepted_with_wire(accepted, authz.clone(), None, Some(plane.clone())).await;
        }
    });
    let desk = Desk::start(&login, &[(ROBOT, &robot)], true, false);
    let mut producers = Producers::new();
    let topic = format!("/controller/natural/{}", unique_id());
    producers.start(&robot, &topic);
    desk.install(login.snapshot(&[(ROBOT, &robot)]));
    let mut old = desk.client();
    old.demand(&robot_route(&ROBOT.0), &topic, ORACLE_SCHEMA_HASH)
        .unwrap();
    assert_oracle_and_gap_free(&collect_frames(&desk.manager, &topic, 3));
    {
        let connections = accepted_connections.lock().unwrap();
        assert_eq!(connections.len(), 1);
        connections[0].close(0u32.into(), b"test connection interruption");
    }
    assert!(
        gather_absent(&desk.manager, &topic),
        "actual stream death must complete before re-demand"
    );
    assert_eq!(
        desk.daemon.active_demand_count(),
        1,
        "this regression begins with the old consumer's stale hold"
    );
    let mut new = desk.client();
    let reply = new
        .demand(&robot_route(&ROBOT.0), &topic, ORACLE_SCHEMA_HASH)
        .unwrap();
    assert!(
        reply.mirror_created,
        "a dead reader cannot satisfy AlreadyMirrored"
    );
    drop(old);
    assert_oracle_and_gap_free(&collect_frames(&desk.manager, &topic, 3));
    assert_eq!(desk.daemon.active_demand_count(), 1);
    assert_eq!(
        accepted_connections.lock().unwrap().len(),
        2,
        "a fresh authenticated connection carried the new reader"
    );
    assert_eq!(robot.binding(&login.seed), Some(OWNER));
    assert!(
        robot.receipts().is_empty(),
        "this pre-enrolled recovery fixture never runs owner admission"
    );
}

#[test]
fn rekey_retires_only_changed_robot_and_preserves_other_robot_delivery() {
    let a = OwnerRobot::start_with_robot_id("controller_rekey_old", RobotId([5; 32]), None, None);
    let replacement =
        OwnerRobot::start_with_robot_id("controller_rekey_new", RobotId([5; 32]), None, None);
    let b = OwnerRobot::start_with_robot_id("controller_rekey_keep", RobotId([6; 32]), None, None);
    let login = Login::new();
    let desk = Desk::start(
        &login,
        &[
            (RobotId([5; 32]), &a),
            (RobotId([5; 32]), &replacement),
            (RobotId([6; 32]), &b),
        ],
        true,
        false,
    );
    let mut producers = Producers::new();
    let topic_a = format!("/controller/rekey/a/{}", unique_id());
    let topic_b = format!("/controller/rekey/b/{}", unique_id());
    producers.start(&a, &topic_a);
    producers.start(&b, &topic_b);
    desk.install(login.snapshot(&[(RobotId([5; 32]), &a), (RobotId([6; 32]), &b)]));
    let mut old = desk.client();
    old.demand(&robot_route(&[5; 32]), &topic_a, ORACLE_SCHEMA_HASH)
        .unwrap();
    let mut keep = desk.client();
    keep.demand(&robot_route(&[6; 32]), &topic_b, ORACLE_SCHEMA_HASH)
        .unwrap();
    assert_oracle_and_gap_free(&collect_frames(&desk.manager, &topic_a, 3));
    assert_oracle_and_gap_free(&collect_frames(&desk.manager, &topic_b, 3));
    let manager = replacement.fixture.manager.clone();
    let topic = topic_a.clone();
    let stop = producers.stop.clone();
    producers.handles.push(std::thread::spawn(move || {
        let mut publisher = manager
            .create_publisher_simple(&topic, MaxSliceLen::const_new(256))
            .unwrap();
        let mut sequence = 100_000;
        while !stop.load(Ordering::Relaxed) {
            publisher.publish_raw(&oracle_frame(sequence)).unwrap();
            sequence += 1;
            std::thread::sleep(Duration::from_millis(5));
        }
    }));
    desk.install(login.snapshot(&[(RobotId([5; 32]), &replacement), (RobotId([6; 32]), &b)]));
    assert_eq!(desk.daemon.active_demand_count(), 1);
    assert_oracle_and_gap_free(&collect_frames(&desk.manager, &topic_b, 3));
    let mut new = desk.client();
    new.demand(&robot_route(&[5; 32]), &topic_a, ORACLE_SCHEMA_HASH)
        .unwrap();
    drop(old);
    let frames = collect_frames(&desk.manager, &topic_a, 3);
    assert!(
        frames[0].0 >= 100_000,
        "actual bytes identify the replacement producer"
    );
    assert_oracle_and_gap_free(&frames);
    assert_oracle_and_gap_free(&collect_frames(&desk.manager, &topic_b, 3));
    assert_eq!(replacement.binding(&login.seed), Some(OWNER));
    assert_eq!(pairs(&b), 1);
}
