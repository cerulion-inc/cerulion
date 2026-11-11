// SPDX-License-Identifier: AGPL-3.0-only
//! Build cerulion_cli and cerulion_remoted first.
//! Real socket, owner proof, shared endpoint and retained subscriber transitions.

use super::*;
use cerulion_core::transport::subscriber::DataOnlySubscriber;
use cerulion_netd::protocol::ServingPlane;
use std::path::Path;

struct EnvGuard(&'static str, Option<std::ffi::OsString>);
impl EnvGuard {
    fn set(key: &'static str, value: impl AsRef<std::ffi::OsStr>) -> Self {
        let prior = std::env::var_os(key);
        std::env::set_var(key, value);
        Self(key, prior)
    }
}
impl Drop for EnvGuard {
    fn drop(&mut self) {
        if let Some(value) = &self.1 {
            std::env::set_var(self.0, value);
        } else {
            std::env::remove_var(self.0);
        }
    }
}

fn cached_local_robot(root: &Path, seed: [u8; 32]) {
    use std::os::unix::fs::DirBuilderExt;
    let dir = root.join("remoted");
    std::fs::DirBuilder::new()
        .recursive(true)
        .mode(0o700)
        .create(&dir)
        .unwrap();
    std::fs::write(dir.join("device_key"), seed).unwrap();
    std::fs::write(dir.join("trust_store.mac_key"), MAC).unwrap();
    let public = DeviceIdentity::from_seed(&seed).public_key();
    let mut store = TrustStore::provision(
        RobotId([0xe1; 32]),
        public,
        RootSet::new(
            vec![PublicKey(signing_key(1).verifying_key().to_bytes())],
            1,
        )
        .unwrap(),
        CHASSIS,
        T_NOW,
    )
    .unwrap()
    .with_path(dir.join("trust_store"));
    store
        .claim(OWNER, CHASSIS, PrincipalKind::Human, T_NOW)
        .unwrap();
    store.save(MAC).unwrap();
    let mut index = DeviceAccountIndex::new().with_path(dir.join("device_index.json"));
    index.bind(public, OWNER);
    index.save(MAC).unwrap();
}

fn read_after(tap: &mut DataOnlySubscriber, after: Option<u32>) -> (u32, Vec<u8>) {
    let deadline = Instant::now() + Duration::from_secs(20);
    let budget = tap.max_borrowed_samples().max(1);
    let mut batch = Vec::with_capacity(budget);
    loop {
        tap.drain_owned(budget, &mut batch).unwrap();
        for sample in batch.drain(..) {
            let sequence = sample.wire_header().unwrap().sequence;
            if after.is_none_or(|last| sequence > last) {
                let bytes = sample.payload().to_vec();
                assert_eq!(bytes, oracle_frame(sequence));
                return (sequence, bytes);
            }
        }
        assert!(
            Instant::now() < deadline,
            "retained subscriber received no new oracle frame"
        );
        std::thread::sleep(Duration::from_millis(3));
    }
}

#[test]
fn serving_transition_restores_canonical_and_observer_pins_on_the_same_subscribers() {
    // The supervisor resolves its real child beside current_exe. Reproduce the
    // installed binary layout privately; the parent creates no transport session.
    let directory = tempfile::tempdir_in("/tmp").unwrap();
    let current = std::env::current_exe().unwrap();
    let remoted = current
        .parent()
        .unwrap()
        .parent()
        .unwrap()
        .join("cerulion-remoted");
    for (source, name) in [
        (&current, "transition-test"),
        (&remoted, "cerulion-remoted"),
    ] {
        assert!(
            source.is_file(),
            "required executable is absent: {}",
            source.display()
        );
        std::fs::copy(source, directory.path().join(name)).unwrap_or_else(|error| {
            panic!(
                "could not copy required executable {}: {error}",
                source.display()
            )
        });
    }
    let output = std::process::Command::new(directory.path().join("transition-test"))
        .args([
            "--ignored",
            "--exact",
            "owner_pair::controller::transition::actual_child_fixture",
            "--test-threads=1",
            "--nocapture",
        ])
        .output()
        .unwrap();
    assert!(
        output.status.success(),
        "actual-child fixture failed: {}\\n{}",
        String::from_utf8_lossy(&output.stdout),
        String::from_utf8_lossy(&output.stderr)
    );
}

#[test]
#[ignore = "subprocess fixture for the real installed sibling layout"]
fn actual_child_fixture() {
    let _ = tracing_subscriber::fmt()
        .with_env_filter("cerulion_netd=debug,cerulion_remoted=debug")
        .with_test_writer()
        .try_init();
    let robot = OwnerRobot::start_with_robot_id("transition", ROBOT, None, None);
    let login = Login::new();
    let _home = EnvGuard::set("CERULION_HOME", login.directory.path());
    let _state = EnvGuard::set("CERULION_STATE_ROOT", login.directory.path());
    let desk = Desk::start(&login, &[(ROBOT, &robot)], true, false);
    let topic = format!("/wan/transition/{}", unique_id());
    let mut producers = Producers::new();
    producers.start(&robot, &topic);
    desk.install(login.snapshot(&[(ROBOT, &robot)]));
    let route = robot_route(&ROBOT.0);
    let mut canonical = desk.client();
    canonical
        .demand(&route, &topic, ORACLE_SCHEMA_HASH)
        .unwrap();
    let mut observer = desk.client();
    let local = observer
        .observe(&route, &topic, ORACLE_SCHEMA_HASH)
        .unwrap()
        .local_topic;
    let mut canonical_tap = desk.manager.create_data_only_subscriber(&topic).unwrap();
    let mut observer_tap = desk.manager.create_data_only_subscriber(&local).unwrap();
    let mut a = read_after(&mut canonical_tap, None).0;
    let mut b = read_after(&mut observer_tap, None).0;
    let runtime = tokio::runtime::Runtime::new().unwrap();
    for _ in 0..2 {
        desk.controller.begin_serving().unwrap();
        let endpoint = runtime.block_on(disabled_endpoint(login.seed));
        let socket = cerulion_netd::shared_endpoint::socket_path(login.directory.path());
        let server = cerulion_netd::shared_endpoint::OutboundServer::start(
            socket,
            endpoint.clone(),
            runtime.handle().clone(),
            login.seed,
            desk.manager.clone(),
        )
        .unwrap();
        desk.controller.serving_ready().unwrap();
        assert!(desk.controller.holds_outbound_readers());
        assert_eq!(desk.daemon.active_demand_count(), 2);
        let rows = canonical.status().unwrap().demands;
        assert_eq!(rows.len(), 2);
        assert!(rows
            .iter()
            .all(|row| row.refcount == 1 && row.plane == Some(ServingPlane::Iroh)));
        a = read_after(&mut canonical_tap, Some(a)).0;
        b = read_after(&mut observer_tap, Some(b)).0;
        // Failed child spawn takes the same end hook before any endpoint exists;
        // successful stop takes it only after this owned endpoint is fully gone.
        drop(server);
        runtime.block_on(endpoint.close());
        desk.controller.end_serving();
        assert!(!desk.controller.holds_outbound_readers());
        assert_eq!(desk.daemon.active_demand_count(), 2);
        a = read_after(&mut canonical_tap, Some(a)).0;
        b = read_after(&mut observer_tap, Some(b)).0;
        assert_eq!(canonical.status().unwrap().demands.len(), 2);
        assert_eq!(observer.status().unwrap().demands.len(), 2);
    }
    // Exercise the actual child constructor with persisted claimed authority.
    // Its begin/ready/end hooks must restore these same retained subscribers.
    cached_local_robot(login.directory.path(), login.seed);
    std::fs::create_dir_all(login.directory.path().join("config")).unwrap();
    let ix = desk.manager.iox_config();
    std::fs::write(
        login.directory.path().join("config/iceoryx2.toml"),
        format!(
            "[global]\nroot-path = {:?}\nprefix = {:?}\n",
            ix.global.root_path().to_string(),
            ix.global.prefix.to_string()
        ),
    )
    .unwrap();
    // iceoryx2 resolves the project config from cwd, independently of CERULION_HOME.
    // The real child inherits this private directory and loads the same namespace.
    let previous_cwd = std::env::current_dir().unwrap();
    struct RestoreCwd(std::path::PathBuf);
    impl Drop for RestoreCwd {
        fn drop(&mut self) {
            std::env::set_current_dir(&self.0).unwrap();
        }
    }
    let _cwd = RestoreCwd(previous_cwd);
    std::env::set_current_dir(login.directory.path()).unwrap();
    let _socket = EnvGuard::set("CERULION_NETD_SOCK", &desk.socket);
    let _relay = EnvGuard::set("CERULION_NETD_RELAY_DISABLED", "1");
    let gateway = Arc::new(cerulion_netd::GatewayEgressPlane::new(desk.manager.clone()));
    let endpoint_held = Arc::new(AtomicBool::new(false));
    let mut supervisor = cerulion_netd::robot_supervisor::RobotSupervisor::start(
        Arc::new(AtomicBool::new(true)),
        gateway,
        desk.controller.clone(),
        endpoint_held.clone(),
    )
    .unwrap();
    let deadline = Instant::now() + Duration::from_secs(15);
    while !cerulion_netd::shared_endpoint::socket_path(login.directory.path()).exists() {
        assert!(
            Instant::now() < deadline,
            "actual serving child did not become ready"
        );
        std::thread::sleep(Duration::from_millis(20));
    }
    a = read_after(&mut canonical_tap, Some(a)).0;
    b = read_after(&mut observer_tap, Some(b)).0;
    assert!(endpoint_held.load(Ordering::Acquire));
    supervisor.shutdown();
    assert!(!endpoint_held.load(Ordering::Acquire));
    a = read_after(&mut canonical_tap, Some(a)).0;
    b = read_after(&mut observer_tap, Some(b)).0;
    assert_eq!(desk.daemon.active_demand_count(), 2);

    // A failed spawn needs no shared endpoint and restores the original readers.
    desk.controller.begin_serving().unwrap();
    desk.controller.end_serving();
    read_after(&mut canonical_tap, Some(a));
    read_after(&mut observer_tap, Some(b));
    assert_eq!(desk.daemon.active_demand_count(), 2);
}

#[test]
fn failed_transition_with_removed_identity_pushes_named_loss_to_every_held_reader() {
    let robot = OwnerRobot::start_with_robot_id("transition_loss", ROBOT, None, None);
    let login = Login::new();
    let _home = EnvGuard::set("CERULION_HOME", login.directory.path());
    let _state = EnvGuard::set("CERULION_STATE_ROOT", login.directory.path());
    let desk = Desk::start(&login, &[(ROBOT, &robot)], true, false);
    let topic = format!("/wan/transition_loss/{}", unique_id());
    let mut producers = Producers::new();
    producers.start(&robot, &topic);
    desk.install(login.snapshot(&[(ROBOT, &robot)]));
    let route = robot_route(&ROBOT.0);
    let mut observer = desk.client();
    observer
        .observe(&route, &topic, ORACLE_SCHEMA_HASH)
        .unwrap();
    desk.controller.begin_serving().unwrap();
    std::fs::remove_file(login.directory.path().join("desk.key")).unwrap();
    assert!(desk.controller.serving_ready().is_err());
    desk.controller.end_serving();
    let deadline = Instant::now() + Duration::from_secs(5);
    loop {
        match observer.status() {
            Err(cerulion_netd::ClientError::MirrorLost(event)) => {
                assert_eq!(event.robot, route);
                assert_eq!(event.topic, topic);
                break;
            }
            Ok(_) => {
                assert!(
                    Instant::now() < deadline,
                    "held reader was silently retired"
                );
                std::thread::sleep(Duration::from_millis(20));
            }
            Err(error) => panic!("expected named mirror loss, got {error}"),
        }
    }
    assert_eq!(desk.daemon.active_demand_count(), 0);
}

#[test]
fn many_stalled_restorations_finish_within_one_budget_and_notify_every_holder() {
    use std::io::{Read, Write};
    use std::os::unix::net::UnixListener;
    let robot = OwnerRobot::start_with_robot_id("deadline", ROBOT, None, None);
    let login = Login::new();
    let _home = EnvGuard::set("CERULION_HOME", login.directory.path());
    let _state = EnvGuard::set("CERULION_STATE_ROOT", login.directory.path());
    let desk = Desk::start(&login, &[(ROBOT, &robot)], true, false);
    let route = robot_route(&ROBOT.0);
    let topics: Vec<_> = (0..8).map(|pin| format!("/wan/deadline/{pin}")).collect();
    let mut producers = Producers::new();
    for topic in &topics {
        producers.start(&robot, topic);
    }
    desk.install(login.snapshot(&[(ROBOT, &robot)]));
    let mut holders = Vec::new();
    for topic in &topics {
        let mut client = desk.client();
        let channel = client
            .observe(&route, topic, ORACLE_SCHEMA_HASH)
            .unwrap()
            .local_topic;
        assert_oracle_and_gap_free(&collect_frames(&desk.manager, &channel, 1));
        holders.push(client);
    }
    desk.controller.begin_serving().unwrap();
    let socket = cerulion_netd::shared_endpoint::socket_path(login.directory.path());
    std::fs::create_dir_all(socket.parent().unwrap()).unwrap();
    let listener = UnixListener::bind(&socket).unwrap();
    let server = std::thread::spawn(move || {
        let (mut stream, _) = listener.accept().unwrap();
        stream
            .set_read_timeout(Some(Duration::from_secs(9)))
            .unwrap();
        let read_request = |stream: &mut std::os::unix::net::UnixStream| {
            let mut size = [0; 4];
            stream.read_exact(&mut size).unwrap();
            let size = u32::from_le_bytes(size) as usize;
            assert!(size < 256 * 1024);
            let mut bytes = vec![0; size];
            stream.read_exact(&mut bytes).unwrap();
            serde_json::from_slice::<serde_json::Value>(&bytes).unwrap()
        };
        let sync = read_request(&mut stream);
        assert_eq!(sync["operation"]["op"], "sync");
        let answer = b"{\"Ok\":{\"Retired\":[]}}";
        stream
            .write_all(&(answer.len() as u32).to_le_bytes())
            .unwrap();
        stream.write_all(answer).unwrap();
        let ensure = read_request(&mut stream);
        assert_eq!(ensure["operation"]["op"], "ensure");
        assert_eq!(ensure["operation"]["topic"], "/wan/deadline/0");
        assert_eq!(ensure["operation"]["observer"], true);
        assert_eq!(ensure["operation"]["schema_hash"], ORACLE_SCHEMA_HASH);
        // Stall the real socket after its first admitted restoration request.
        // A client deadline must close it before the fixture's independent cap.
        let mut next = [0; 1];
        match stream.read(&mut next) {
            Ok(0) => true,
            Err(error)
                if matches!(
                    error.kind(),
                    std::io::ErrorKind::TimedOut | std::io::ErrorKind::WouldBlock
                ) =>
            {
                false
            }
            other => panic!("unexpected continuation after stalled restoration: {other:?}"),
        }
    });
    let started = Instant::now();
    assert!(desk.controller.serving_ready().is_err());
    assert!(
        started.elapsed() < Duration::from_secs(7),
        "restoration exceeded its total budget and cleanup margin"
    );
    assert!(
        server.join().unwrap(),
        "the client did not enforce its deadline"
    );
    let loss_deadline = Instant::now() + Duration::from_secs(4);
    for (topic, holder) in topics.iter().zip(&mut holders) {
        loop {
            match holder.status() {
                Err(cerulion_netd::ClientError::MirrorLost(event)) => {
                    assert_eq!(event.robot, route);
                    assert_eq!(&event.topic, topic);
                    break;
                }
                Ok(_) => {
                    assert!(
                        Instant::now() < loss_deadline,
                        "a stalled pin was silently retired"
                    );
                    std::thread::sleep(Duration::from_millis(20));
                }
                Err(error) => panic!("expected named loss, got {error}"),
            }
        }
    }
    assert_eq!(desk.daemon.active_demand_count(), 0);
    desk.controller.end_serving();
}

#[test]
fn stopped_reader_release_does_not_block_another_robot_or_leave_a_stale_row() {
    let mut observed = Vec::new();
    for _ in 0..2 {
        let first = OwnerRobot::start_with_robot_id("release_dead", ROBOT, None, None);
        let second_id = RobotId([0xb2; 32]);
        let second = OwnerRobot::start_with_robot_id("release_next", second_id, None, None);
        let login = Login::new();
        let desk = Desk::start(
            &login,
            &[(ROBOT, &first), (second_id, &second)],
            true,
            false,
        );
        let mut producers = Producers::new();
        producers.start(&first, "/wan/release/dead");
        producers.start(&second, "/wan/release/next");
        desk.install(login.snapshot(&[(ROBOT, &first), (second_id, &second)]));
        let first_route = robot_route(&ROBOT.0);
        let second_route = robot_route(&second_id.0);
        let mut holding = desk.client();
        let local = holding
            .observe(&first_route, "/wan/release/dead", ORACLE_SCHEMA_HASH)
            .unwrap()
            .local_topic;
        let mut tap = desk.manager.create_data_only_subscriber(&local).unwrap();
        read_after(&mut tap, None);
        first.fixture.rt.block_on(first.fixture.endpoint.close());
        let deadline = Instant::now() + Duration::from_secs(5);
        while tap.publisher_count() != 0 {
            assert!(Instant::now() < deadline, "closed robot reader did not end");
            std::thread::sleep(Duration::from_millis(10));
        }
        drop(holding);
        let mut next = desk.client();
        loop {
            match next.observe(&second_route, "/wan/release/next", ORACLE_SCHEMA_HASH) {
                Ok(reply) => {
                    let mut tap = desk
                        .manager
                        .create_data_only_subscriber(&reply.local_topic)
                        .unwrap();
                    read_after(&mut tap, None);
                    break;
                }
                Err(_) if Instant::now() < deadline => {
                    std::thread::sleep(Duration::from_millis(10))
                }
                Err(error) => panic!("second robot remained blocked: {error}"),
            }
        }
        let rows = next.status().unwrap().demands;
        assert_eq!(rows.len(), 1);
        assert_eq!(rows[0].robot, second_route);
        assert_eq!(rows[0].refcount, 1);
        observed.push((rows.len(), rows[0].topic.clone(), rows[0].refcount));
    }
    assert_eq!(observed[0], observed[1]);
}

#[test]
fn unreachable_transition_retires_every_pin_and_allows_a_second_robot() {
    let mut outcomes = Vec::new();
    for stop_before_ready in [true, false] {
        for _ in 0..2 {
            let first = OwnerRobot::start_with_robot_id("restore_dead", ROBOT, None, None);
            let second_id = RobotId([0xb3; 32]);
            let second = OwnerRobot::start_with_robot_id("restore_next", second_id, None, None);
            let login = Login::new();
            let _home = EnvGuard::set("CERULION_HOME", login.directory.path());
            let _state = EnvGuard::set("CERULION_STATE_ROOT", login.directory.path());
            let desk = Desk::start(
                &login,
                &[(ROBOT, &first), (second_id, &second)],
                true,
                false,
            );
            let mut producers = Producers::new();
            producers.start(&first, "/wan/restore/dead");
            producers.start(&second, "/wan/restore/next");
            desk.install(login.snapshot(&[(ROBOT, &first), (second_id, &second)]));
            let mut holder = desk.client();
            let first_route = robot_route(&ROBOT.0);
            holder
                .observe(&first_route, "/wan/restore/dead", ORACLE_SCHEMA_HASH)
                .unwrap();
            desk.controller.begin_serving().unwrap();
            first.fixture.rt.block_on(first.fixture.endpoint.close());
            let runtime = tokio::runtime::Runtime::new().unwrap();
            let endpoint = runtime.block_on(disabled_endpoint(login.seed));
            let server = cerulion_netd::shared_endpoint::OutboundServer::start(
                cerulion_netd::shared_endpoint::socket_path(login.directory.path()),
                endpoint.clone(),
                runtime.handle().clone(),
                login.seed,
                desk.manager.clone(),
            )
            .unwrap();
            if stop_before_ready {
                assert!(desk.controller.serving_ready().is_err());
            }
            drop(server);
            runtime.block_on(endpoint.close());
            desk.controller.end_serving();
            let deadline = Instant::now() + Duration::from_secs(2);
            loop {
                match holder.status() {
                    Err(cerulion_netd::ClientError::MirrorLost(loss)) => {
                        assert_eq!(loss.robot, first_route);
                        assert_eq!(loss.topic, "/wan/restore/dead");
                        break;
                    }
                    Ok(_) if Instant::now() < deadline => {
                        std::thread::sleep(Duration::from_millis(10))
                    }
                    other => panic!("failed restoration did not notify its holder: {other:?}"),
                }
            }
            assert_eq!(desk.daemon.active_demand_count(), 0);
            let mut next = desk.client();
            let route = robot_route(&second_id.0);
            let channel = next
                .observe(&route, "/wan/restore/next", ORACLE_SCHEMA_HASH)
                .unwrap()
                .local_topic;
            let mut tap = desk.manager.create_data_only_subscriber(&channel).unwrap();
            read_after(&mut tap, None);
            outcomes.push((stop_before_ready, desk.daemon.active_demand_count()));
        }
    }
    assert_eq!(outcomes, [(true, 1), (true, 1), (false, 1), (false, 1)]);
}

#[test]
fn stalled_first_demand_cannot_poison_a_healthy_observer_heartbeat() {
    struct StalledPlane {
        controller: Arc<AccountWanController>,
        started: std::sync::mpsc::SyncSender<()>,
    }
    impl MirrorPlane for StalledPlane {
        fn prepare_demand(
            &self,
            key: &TopicKey,
            registry: &mut cerulion_netd::registry::DemandRegistry,
        ) -> Result<(), String> {
            self.controller.prepare_demand(key, registry)
        }
        fn ensure_mirror(&self, key: &TopicKey, hash: u64) -> Result<(), MirrorError> {
            if key.topic == "/wan/stall" {
                self.started.send(()).unwrap();
                std::thread::sleep(Duration::from_secs(6));
                Err(MirrorError::Iroh {
                    key: key.clone(),
                    reason: "stalled fixture refuses the new demand".into(),
                })
            } else {
                self.controller.ensure_mirror(key, hash)
            }
        }
        fn release_mirror(&self, key: &TopicKey) -> MirrorRelease {
            self.controller.release_mirror(key)
        }
        fn local_channel(&self, key: &TopicKey) -> String {
            self.controller.local_channel(key)
        }
    }
    let robot = OwnerRobot::start_with_robot_id("healthy", ROBOT, None, None);
    let login = Login::new();
    let desk = Desk::start(&login, &[(ROBOT, &robot)], true, false);
    let mut producers = Producers::new();
    producers.start(&robot, "/wan/healthy");
    desk.install(login.snapshot(&[(ROBOT, &robot)]));
    let socket = desk._directory.path().join("stall.sock");
    let (started, ready) = std::sync::mpsc::sync_channel(1);
    let mut daemon = cerulion_netd::start(
        socket.clone(),
        Arc::new(StalledPlane {
            controller: desk.controller.clone(),
            started,
        }),
        NetdConfig::default(),
    )
    .unwrap();
    let mut holding = NetdClient::connect_existing_at(socket.clone()).unwrap();
    let route = robot_route(&ROBOT.0);
    let channel = holding
        .observe(&route, "/wan/healthy", ORACLE_SCHEMA_HASH)
        .unwrap()
        .local_topic;
    let mut tap = desk.manager.create_data_only_subscriber(&channel).unwrap();
    let mut sequence = read_after(&mut tap, None).0;
    let worker = std::thread::spawn(move || {
        NetdClient::connect_existing_at(socket)
            .unwrap()
            .demand("offline", "/wan/stall", ORACLE_SCHEMA_HASH)
            .unwrap_err()
    });
    ready.recv_timeout(Duration::from_secs(2)).unwrap();
    let deadline = Instant::now() + Duration::from_millis(6200);
    let mut busy = 0;
    while Instant::now() < deadline {
        holding.poll_pushed_losses().unwrap();
        assert!(holding.take_mirror_losses().is_empty());
        if holding
            .probe_status_until(Instant::now() + Duration::from_millis(70))
            .is_err()
        {
            busy += 1;
        }
        sequence = read_after(&mut tap, Some(sequence)).0;
        std::thread::sleep(Duration::from_millis(100));
    }
    assert!(
        busy > 0,
        "the observer never encountered the stalled registry"
    );
    worker.join().unwrap();
    let rows = holding.status().unwrap().demands;
    assert_eq!(rows.len(), 1);
    assert_eq!(rows[0].robot, route);
    assert_eq!(rows[0].topic, "/wan/healthy");
    assert_eq!(rows[0].refcount, 1);
    assert_eq!(daemon.active_demand_count(), 1);
    drop(holding);
    daemon.shutdown();
}
