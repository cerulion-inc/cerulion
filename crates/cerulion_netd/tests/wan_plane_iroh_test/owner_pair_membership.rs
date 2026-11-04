// SPDX-License-Identifier: AGPL-3.0-only
//! Membership refresh retires only selected peers while a live reader continues.
use super::*;

fn peer(robot: &OwnerRobot) -> WanRobot {
    WanRobot {
        eid: robot.fixture.eid,
        direct_addrs: robot.fixture.addrs.clone(),
    }
}

#[test]
fn membership_refresh_preserves_unchanged_readers_and_retires_changed_peers() {
    let a = OwnerRobot::start_with_robot_id("membership_a", RobotId([5; 32]), None, None);
    let b = OwnerRobot::start_with_robot_id("membership_b", RobotId([6; 32]), None, None);
    let replacement_a =
        OwnerRobot::start_with_robot_id("membership_replacement", RobotId([5; 32]), None, None);
    let seed = unique_secret();
    let initial = HashMap::from([("a".into(), peer(&a)), ("b".into(), peer(&b))]);
    let registry = Arc::new(
        WanRegistry::new(initial.clone(), seed, RelayConfig::Disabled).with_account(Some(OWNER.0)),
    );
    registry
        .replace_owner_certificate(Some(proof(seed, OWNER)))
        .unwrap();
    let desk = test_manager("membership_desk");
    let plane = IrohMirrorPlane::new(desk.clone(), registry.clone())
        .unwrap()
        .with_dial_timeout(DIAL_TIMEOUT);
    let topic_a = format!("/membership/a/{}", unique_id());
    let topic_b = format!("/membership/b/{}", unique_id());
    let stop = Arc::new(AtomicBool::new(false));
    let producer_a = spawn_producer(a.fixture.manager.clone(), topic_a.clone(), stop.clone());
    let producer_b = spawn_producer(b.fixture.manager.clone(), topic_b.clone(), stop.clone());
    let key_a = TopicKey::new("a", &topic_a);
    let key_b = TopicKey::new("b", &topic_b);
    plane.ensure_mirror(&key_a, ORACLE_SCHEMA_HASH).unwrap();
    plane.ensure_mirror(&key_b, ORACLE_SCHEMA_HASH).unwrap();
    assert_oracle_and_gap_free(&collect_frames(&desk, &topic_a, 3));
    assert_oracle_and_gap_free(&collect_frames(&desk, &topic_b, 3));
    assert_eq!(gather_origin(&desk, &topic_a).as_deref(), Some("a"));
    assert_eq!(gather_origin(&desk, &topic_b).as_deref(), Some("b"));
    assert!(plane.update_membership(initial).unwrap().is_empty());
    assert_eq!(plane.connection_count(), 2);
    assert_eq!(plane.reader_count(), 2);

    let updated = HashMap::from([("a".into(), peer(&replacement_a)), ("b".into(), peer(&b))]);
    assert_eq!(
        plane.update_membership(updated).unwrap(),
        vec![key_a.clone()]
    );
    assert_eq!(plane.connection_count(), 1);
    assert_eq!(plane.reader_count(), 1);
    assert_eq!(plane.last_push_outcome("a"), None);
    assert!(plane.last_push_outcome("b").is_some());
    let provenance = desk
        .gather_mirror_provenance(Duration::from_millis(300))
        .unwrap();
    assert!(provenance.iter().all(|entry| entry.topic != topic_a));
    assert!(provenance
        .iter()
        .any(|entry| entry.topic == topic_b && entry.origin_robot == "b"));
    assert_oracle_and_gap_free(&collect_frames(&desk, &topic_b, 3));

    // Distinct sequence range identifies the replacement robot's actual frames.
    let manager = replacement_a.fixture.manager.clone();
    let topic = topic_a.clone();
    let stop_replacement = stop.clone();
    let producer_replacement = std::thread::spawn(move || {
        let mut publisher = manager
            .create_publisher_simple(&topic, MaxSliceLen::const_new(256))
            .unwrap();
        let mut sequence = 100_000;
        while !stop_replacement.load(Ordering::Relaxed) {
            publisher.publish_raw(&oracle_frame(sequence)).unwrap();
            sequence += 1;
            std::thread::sleep(Duration::from_millis(5));
        }
    });
    plane.ensure_mirror(&key_a, ORACLE_SCHEMA_HASH).unwrap();
    let frames = collect_frames(&desk, &topic_a, 3);
    assert!(frames[0].0 >= 100_000);
    assert_oracle_and_gap_free(&frames);
    assert_eq!(replacement_a.binding(&seed), Some(OWNER));
    assert_oracle_and_gap_free(&collect_frames(&desk, &topic_b, 3));

    assert_eq!(
        plane.update_membership(HashMap::new()).unwrap(),
        vec![key_a.clone(), key_b.clone()]
    );
    assert_eq!(plane.connection_count(), 0);
    assert_eq!(plane.reader_count(), 0);
    assert_eq!(plane.last_push_outcome("b"), None);
    assert!(desk
        .gather_mirror_provenance(Duration::from_millis(300))
        .unwrap()
        .is_empty());
    assert!(plane.probe_robot("b", DIAL_TIMEOUT).is_err());
    assert!(plane
        .update_membership(HashMap::from([(" b ".into(), peer(&b))]))
        .is_err());
    assert_eq!(registry.robot_count().unwrap(), 0);
    plane
        .update_membership(HashMap::from([("b".into(), peer(&b))]))
        .unwrap();
    plane.ensure_mirror(&key_b, ORACLE_SCHEMA_HASH).unwrap();
    assert_oracle_and_gap_free(&collect_frames(&desk, &topic_b, 3));
    assert_eq!(
        b.receipts()
            .iter()
            .filter(|receipt| receipt["verb"] == "pair" && receipt["outcome"]["outcome"] == "ok")
            .count(),
        1
    );
    plane.invalidate_account().unwrap();
    assert!(plane.update_membership(HashMap::new()).is_err());
    stop.store(true, Ordering::Relaxed);
    producer_a.join().unwrap();
    producer_b.join().unwrap();
    producer_replacement.join().unwrap();
}
