// SPDX-License-Identifier: AGPL-3.0-only
//! Account replacement retires transport state and its observable provenance.

use super::*;

#[test]
fn account_invalidation_clears_provenance_epoch_state_and_frees_the_writer_slot() {
    let robot = OwnerRobot::start("account_retirement", None, None);
    let (registry, plane, desk) = robot.plane(unique_secret(), OWNER);
    let topic = format!("/retirement/{}", unique_id());
    let stop = Arc::new(AtomicBool::new(false));
    let producer = spawn_producer(robot.fixture.manager.clone(), topic.clone(), stop.clone());
    let key = TopicKey::new("robot", &topic);
    plane.ensure_mirror(&key, ORACLE_SCHEMA_HASH).unwrap();
    assert_oracle_and_gap_free(&collect_frames(&desk, &topic, 3));
    assert_eq!(gather_origin(&desk, &topic).as_deref(), Some("robot"));
    assert!(plane.last_push_outcome("robot").is_some());

    plane.invalidate_account().unwrap();
    assert!(registry.owner_certificate().unwrap().is_none());
    assert_eq!(plane.connection_count(), 0);
    assert_eq!(plane.reader_count(), 0);
    assert_eq!(plane.last_push_outcome("robot"), None);
    assert!(desk
        .gather_mirror_provenance(Duration::from_millis(300))
        .unwrap()
        .iter()
        .all(|entry| entry.topic != topic));

    // The prior reader's injector must already have dropped: this is the same
    // single-writer slot a replacement account plane will need.
    let replacement = desk
        .create_ingress_injector(&topic, ORACLE_SCHEMA_HASH, MaxSliceLen::const_new(256))
        .unwrap();
    let mut receiver = desk.create_data_only_subscriber(&topic).unwrap();
    assert!(matches!(
        replacement.reinject_raw(&oracle_frame(7)),
        cerulion_core::transport::network::ReinjectOutcome::Injected { .. }
    ));
    let mut delivered = Vec::new();
    assert_eq!(receiver.drain_owned(1, &mut delivered).unwrap(), 1);
    assert_eq!(delivered[0].payload(), oracle_frame(7));
    drop(delivered);
    drop(receiver);
    drop(replacement);
    assert!(plane.ensure_mirror(&key, ORACLE_SCHEMA_HASH).is_err());
    assert!(plane.query_catalog("robot", DIAL_TIMEOUT).is_err());
    assert!(plane.probe_robot("robot", DIAL_TIMEOUT).is_err());
    stop.store(true, Ordering::Relaxed);
    producer.join().unwrap();
}

#[test]
fn retiring_metadata_only_robot_clears_diagnostics_without_removing_membership() {
    let robot = OwnerRobot::start("metadata_retirement", None, None);
    let (registry, plane, _) = robot.plane(unique_secret(), OWNER);
    let expected_peer = registry.get("robot").unwrap();
    plane.query_catalog("robot", DIAL_TIMEOUT).unwrap();
    assert_eq!(plane.connection_count(), 0);
    assert_eq!(plane.reader_count(), 0);
    assert!(plane.last_push_outcome("robot").is_some());
    plane.retire_robot("robot").unwrap();
    assert_eq!(plane.last_push_outcome("robot"), None);
    assert_eq!(registry.get("robot").unwrap(), expected_peer);
    plane.retire_robot("robot").unwrap();
    plane.query_catalog("robot", DIAL_TIMEOUT).unwrap();
    assert!(plane.last_push_outcome("robot").is_some());
    assert_eq!(
        robot
            .receipts()
            .iter()
            .filter(|receipt| {
                receipt["verb"] == "pair" && receipt["outcome"]["outcome"] == "ok"
            })
            .count(),
        1
    );
    plane.invalidate_account().unwrap();
}
