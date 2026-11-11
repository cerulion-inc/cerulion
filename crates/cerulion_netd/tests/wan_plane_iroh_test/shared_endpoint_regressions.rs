// SPDX-License-Identifier: AGPL-3.0-only
//! Run serially: serving discovery temporarily selects its owned state root.

use super::*;
use cerulion_core::transport::demand_authorizer::AllowAllAuthorizer;
use cerulion_netd::account_access::{
    AccountAccessReply, AccountAccessRequest, PairedRobot, RobotPresence,
};
use cerulion_netd::account_controller::{AccountControllerConfig, AccountWanController};
use cerulion_netd::registry::DemandRegistry;
use cerulion_netd::shared_endpoint::OutboundServer;

#[test]
fn shared_endpoint_operation_errors_preserve_canonical_and_observer_delivery() {
    let seed = unique_secret();
    let rt = tokio::runtime::Runtime::new().unwrap();
    let endpoint = rt.block_on(disabled_endpoint(seed));
    let remote = RobotFixture::start("errors_source", Arc::new(claimed_paired(public_of(&seed))));
    let manager = test_manager("errors_host");
    let directory = tempfile::tempdir().unwrap();
    let socket = directory.path().join("outbound.sock");
    let service = OutboundServer::start(
        socket.clone(),
        endpoint.clone(),
        rt.handle().clone(),
        seed,
        manager.clone(),
    )
    .unwrap();
    let mut peers = HashMap::from([(
        "producer".into(),
        WanRobot {
            eid: remote.eid,
            direct_addrs: remote.addrs.clone(),
        },
    )]);
    peers.insert(
        "offline".into(),
        WanRobot {
            eid: EndpointId::from_bytes(&public_of(&unique_secret())).unwrap(),
            direct_addrs: Vec::new(),
        },
    );
    let plane = IrohMirrorPlane::new_shared(
        manager.clone(),
        Arc::new(WanRegistry::new(peers, seed, RelayConfig::Disabled)),
        socket,
    )
    .unwrap();
    let topic = format!("/shared/errors/{}", unique_id());
    let stop = Arc::new(AtomicBool::new(false));
    let producer = spawn_producer(remote.manager.clone(), topic.clone(), stop.clone());
    let canonical = TopicKey::new("producer", &topic);
    let observed = TopicKey::observed("producer", &topic);
    plane.ensure_mirror(&canonical, ORACLE_SCHEMA_HASH).unwrap();
    plane.ensure_mirror(&observed, ORACLE_SCHEMA_HASH).unwrap();
    assert_eq!(service.active_clients().unwrap(), 1);
    for operation in 0..3 {
        match operation {
            0 => assert!(plane
                .query_catalog("offline", Duration::from_millis(500))
                .is_err()),
            1 => assert!(plane
                .ensure_mirror(
                    &TopicKey::new("producer", "/shared/not-produced"),
                    ORACLE_SCHEMA_HASH
                )
                .is_err()),
            2 => assert!(plane
                .query_schema("producer", "/__cerulion", Duration::from_millis(500))
                .unwrap_err()
                .contains("is a network mirror of another robot; read it on its source robot")),
            _ => unreachable!(),
        }
        assert!(
            plane.has_reader(&canonical).unwrap(),
            "operation {operation} lost canonical ingress"
        );
        assert!(
            plane.has_reader(&observed).unwrap(),
            "operation {operation} lost observer ingress"
        );
        assert_oracle_and_gap_free(&collect_frames(&manager, &canonical.local_topic(), 3));
        assert_oracle_and_gap_free(&collect_frames(&manager, &observed.local_topic(), 3));
        let deadline = Instant::now() + Duration::from_secs(2);
        while service.active_clients().unwrap() != 1 {
            assert!(
                Instant::now() < deadline,
                "a metadata connection remained live"
            );
            std::thread::sleep(Duration::from_millis(2));
        }
    }
    assert_eq!(plane.release_mirror(&canonical), MirrorRelease::Retired);
    assert_eq!(plane.release_mirror(&observed), MirrorRelease::Retired);
    stop.store(true, Ordering::Relaxed);
    producer.join().unwrap();
    drop(plane);
    drop(service);
    rt.block_on(endpoint.close());
}

struct StateRoot(std::ffi::OsString, Option<std::ffi::OsString>);
impl StateRoot {
    fn select(path: &std::path::Path) -> Self {
        let name = std::ffi::OsString::from("CERULION_STATE_ROOT");
        let previous = std::env::var_os(&name);
        std::env::set_var(&name, path);
        Self(name, previous)
    }
}
impl Drop for StateRoot {
    fn drop(&mut self) {
        match &self.1 {
            Some(value) => std::env::set_var(&self.0, value),
            None => std::env::remove_var(&self.0),
        }
    }
}

#[test]
fn shared_endpoint_serving_discovery_lists_two_online_and_one_offline_without_losing_a_reader() {
    let directory = tempfile::tempdir_in("/tmp").unwrap();
    // Select before creating worker threads; restore after every server is reaped.
    let _root = StateRoot::select(directory.path());
    let mut runs = Vec::new();
    for _ in 0..2 {
        let seed = unique_secret();
        let rt = tokio::runtime::Runtime::new().unwrap();
        let endpoint = rt.block_on(disabled_endpoint(seed));
        let a = RobotFixture::start("discover_a", Arc::new(claimed_paired(public_of(&seed))));
        let b = RobotFixture::start("discover_b", Arc::new(claimed_paired(public_of(&seed))));
        let manager = test_manager("discover_owner");
        let service = OutboundServer::start(
            cerulion_netd::shared_endpoint::socket_path(directory.path()),
            endpoint.clone(),
            rt.handle().clone(),
            seed,
            manager.clone(),
        )
        .unwrap();
        let peers: HashMap<String, WanRobot> = HashMap::from([
            (
                "a".into(),
                WanRobot {
                    eid: a.eid,
                    direct_addrs: a.addrs.clone(),
                },
            ),
            (
                "b".into(),
                WanRobot {
                    eid: b.eid,
                    direct_addrs: b.addrs.clone(),
                },
            ),
            (
                "offline".into(),
                WanRobot {
                    eid: EndpointId::from_bytes(&public_of(&unique_secret())).unwrap(),
                    direct_addrs: Vec::new(),
                },
            ),
        ]);
        let pins = peers
            .iter()
            .map(|(name, peer)| PairedRobot {
                name: name.clone(),
                endpoint_key: *peer.eid.as_bytes(),
            })
            .collect::<Vec<_>>();
        let controller = AccountWanController::new(
            manager.clone(),
            WanRegistry::new(peers, seed, RelayConfig::Disabled),
            Arc::new(AllowAllAuthorizer),
            AccountControllerConfig {
                config_home: None,
                network_enabled: true,
                serving_gateway: true,
                epoch_dir: None,
                trusted_direct: HashMap::new(),
            },
        )
        .unwrap();
        controller
            .install_paired_robots(&pins, &mut DemandRegistry::new(Instant::now()))
            .unwrap();
        let stop = Arc::new(AtomicBool::new(false));
        let pa = spawn_producer(a.manager.clone(), "/shared/discover/a".into(), stop.clone());
        let pb = spawn_producer(b.manager.clone(), "/shared/discover/b".into(), stop.clone());
        let key = TopicKey::observed("a", "/shared/discover/a");
        controller.ensure_mirror(&key, ORACLE_SCHEMA_HASH).unwrap();
        let AccountAccessReply::Discovery { robots } = controller
            .account_access(&AccountAccessRequest::Discover { budget_ms: 3900 })
            .unwrap()
        else {
            panic!("missing discovery reply");
        };
        assert_eq!(robots.len(), 3);
        let mut rows = Vec::new();
        for (name, topic) in [("a", "/shared/discover/a"), ("b", "/shared/discover/b")] {
            let row = robots.iter().find(|row| row.route == name).unwrap();
            assert_eq!(row.presence, RobotPresence::Online);
            assert_eq!(row.path, cerulion_netd::account_access::RobotPath::Direct);
            let catalog = row.catalog.as_ref().unwrap();
            assert_eq!(catalog.error, None);
            assert_eq!(catalog.entries.len(), 1);
            assert_eq!(catalog.entries[0].topic, topic);
            assert_eq!(catalog.entries[0].schema_hash, Some(ORACLE_SCHEMA_HASH));
            rows.push((
                name.to_owned(),
                topic.to_owned(),
                catalog.entries[0].schema_hash,
            ));
        }
        assert!(matches!(
            robots
                .iter()
                .find(|row| row.route == "offline")
                .unwrap()
                .presence,
            RobotPresence::NotReached { .. }
        ));
        assert_oracle_and_gap_free(&collect_frames(&manager, &key.local_topic(), 3));
        runs.push(rows);
        assert_eq!(controller.release_mirror(&key), MirrorRelease::Retired);
        stop.store(true, Ordering::Relaxed);
        pa.join().unwrap();
        pb.join().unwrap();
        drop(controller);
        drop(service);
        rt.block_on(endpoint.close());
    }
    let expected = vec![
        (
            "a".to_owned(),
            "/shared/discover/a".to_owned(),
            Some(ORACLE_SCHEMA_HASH),
        ),
        (
            "b".to_owned(),
            "/shared/discover/b".to_owned(),
            Some(ORACLE_SCHEMA_HASH),
        ),
    ];
    assert_eq!(runs[0], expected);
    assert_eq!(runs[1], expected);
    assert_eq!(runs[0], runs[1]);
}
