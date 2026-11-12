// SPDX-License-Identifier: AGPL-3.0-only
//! Real signed owner proof crosses shared control and preserves both reader modes.

use super::*;

#[test]
fn shared_owner_certificate_sync_retains_canonical_and_observer_delivery() {
    let seed = unique_secret();
    let robot = OwnerRobot::start("shared_owner", None, None);
    let manager = test_manager("shared_owner_desk");
    let runtime = tokio::runtime::Runtime::new().unwrap();
    let endpoint = runtime.block_on(disabled_endpoint(seed));
    let directory = tempfile::tempdir().unwrap();
    let socket = directory.path().join("outbound.sock");
    let server = cerulion_netd::shared_endpoint::OutboundServer::start(
        socket.clone(),
        endpoint.clone(),
        runtime.handle().clone(),
        seed,
        manager.clone(),
    )
    .unwrap();
    let peers = HashMap::from([(
        "robot".to_owned(),
        WanRobot {
            eid: robot.fixture.eid,
            direct_addrs: robot.fixture.addrs.clone(),
        },
    )]);
    let registry = Arc::new(
        WanRegistry::new(peers.clone(), seed, RelayConfig::Disabled).with_account(Some(OWNER.0)),
    );
    let plane = IrohMirrorPlane::new_shared(manager.clone(), registry.clone(), socket).unwrap();
    // Owner proof may appear on an existing same-account control connection.
    assert!(plane.update_membership(peers.clone()).unwrap().is_empty());
    registry
        .replace_owner_certificate(Some(proof(seed, OWNER)))
        .unwrap();
    assert!(plane.update_membership(peers.clone()).unwrap().is_empty());
    let topic = format!("/owner/shared/{}", unique_id());
    let stop = Arc::new(AtomicBool::new(false));
    let producer = spawn_producer(robot.fixture.manager.clone(), topic.clone(), stop.clone());
    let canonical = TopicKey::new("robot", &topic);
    let observer = TopicKey::observed("robot", &topic);
    plane.ensure_mirror(&canonical, ORACLE_SCHEMA_HASH).unwrap();
    plane.ensure_mirror(&observer, ORACLE_SCHEMA_HASH).unwrap();
    // A fresh metadata connection must admit the proof on its first request too.
    let catalog = plane
        .query_catalog("robot", Duration::from_millis(500))
        .unwrap();
    assert!(catalog.entries.iter().any(|entry| entry.topic == topic));
    assert_oracle_and_gap_free(&collect_frames(&manager, &canonical.local_topic(), 3));
    assert_oracle_and_gap_free(&collect_frames(&manager, &observer.local_topic(), 3));
    let mut invalid = (*proof(seed, OWNER)).clone();
    invalid.version = 999;
    registry
        .replace_owner_certificate(Some(Arc::new(invalid)))
        .unwrap();
    assert!(plane
        .update_membership(peers.clone())
        .unwrap_err()
        .contains("version"));
    assert_oracle_and_gap_free(&collect_frames(&manager, &canonical.local_topic(), 3));
    assert_oracle_and_gap_free(&collect_frames(&manager, &observer.local_topic(), 3));
    registry
        .replace_owner_certificate(Some(proof(seed, OWNER)))
        .unwrap();
    assert!(plane.update_membership(peers).unwrap().is_empty());
    assert!(plane.has_reader(&canonical).unwrap());
    assert!(plane.has_reader(&observer).unwrap());
    assert_oracle_and_gap_free(&collect_frames(&manager, &canonical.local_topic(), 3));
    assert_oracle_and_gap_free(&collect_frames(&manager, &observer.local_topic(), 3));
    assert_eq!(robot.binding(&seed), Some(OWNER));
    assert_eq!(server.active_clients().unwrap(), 1);
    assert_eq!(plane.release_mirror(&canonical), MirrorRelease::Retired);
    assert_eq!(plane.release_mirror(&observer), MirrorRelease::Retired);
    stop.store(true, Ordering::Relaxed);
    producer.join().unwrap();
    drop(plane);
    drop(server);
    runtime
        .block_on(async { tokio::time::timeout(Duration::from_secs(5), endpoint.close()).await })
        .expect("owner shared endpoint closes on its owning runtime");
}
