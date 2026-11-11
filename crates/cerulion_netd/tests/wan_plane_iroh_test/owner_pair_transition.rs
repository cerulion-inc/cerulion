// SPDX-License-Identifier: AGPL-3.0-only
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
