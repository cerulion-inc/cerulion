// SPDX-License-Identifier: AGPL-3.0-only
//! Bounded presence and metadata over the production robot serving path.

use super::*;
use cerulion_netd::iroh_plane::RobotProbeOutcome;
use cerulion_netd::EpochPushOutcome;
use cerulion_pairing::verify::EpochSyncWire;

fn assert_one_pair_attempt(robot: &OwnerRobot, terminal_outcome: &str) {
    let receipts = robot.receipts();
    assert!(receipts.iter().all(|receipt| receipt["verb"] == "pair"));
    let outcomes: Vec<_> = receipts
        .iter()
        .map(|receipt| receipt["outcome"]["outcome"].as_str().unwrap())
        .collect();
    // Mutating ops append one intent receipt before the terminal receipt.
    assert_eq!(outcomes, ["intent", terminal_outcome]);
}

#[test]
fn tls_probe_does_not_enroll_or_create_a_reader() {
    let seed = unique_secret();
    let robot = OwnerRobot::start("probe_owner", None, None);
    let (_, plane, _) = robot.plane(seed, OWNER);
    assert_eq!(
        plane.probe_robot("robot", DIAL_TIMEOUT).unwrap(),
        RobotProbeOutcome::Online
    );
    assert_eq!(robot.binding(&seed), None);
    assert!(robot.receipts().is_empty());
    assert_eq!(plane.connection_count(), 0);
    assert_eq!(plane.reader_count(), 0);
    assert!(plane.probe_robot("missing", DIAL_TIMEOUT).is_err());
    robot.fixture.kill();
    assert!(matches!(
        plane
            .probe_robot("robot", Duration::from_millis(100))
            .unwrap(),
        RobotProbeOutcome::NotReached { .. }
    ));
    plane.invalidate_account().unwrap();
    assert!(plane.probe_robot("robot", DIAL_TIMEOUT).is_err());
    assert!(plane.query_catalog("robot", DIAL_TIMEOUT).is_err());
    assert!(plane.query_schema("robot", "/state", DIAL_TIMEOUT).is_err());
    assert!(robot.receipts().is_empty());
}

#[test]
fn fresh_catalog_pairs_once_and_queries_preserve_active_reader_delivery() {
    let seed = unique_secret();
    let robot = OwnerRobot::start("query_owner", None, None);
    let (_, plane, desk) = robot.plane(seed, OWNER);
    let topic = format!("/query/{}", unique_id());
    let stop = Arc::new(AtomicBool::new(false));
    let producer = spawn_producer(robot.fixture.manager.clone(), topic.clone(), stop.clone());
    let deadline = Instant::now() + DIAL_TIMEOUT;
    while !robot
        .fixture
        .manager
        .list_topics()
        .unwrap()
        .contains(&topic)
    {
        assert!(
            Instant::now() < deadline,
            "producer never registered its topic"
        );
        std::thread::sleep(Duration::from_millis(5));
    }

    let catalog = plane.query_catalog("robot", DIAL_TIMEOUT).unwrap();
    assert_eq!(catalog.robot, "robot");
    assert_eq!(catalog.version, CATALOG_WIRE_VERSION);
    assert_eq!(catalog.error, None);
    assert_eq!(catalog.entries.len(), 1);
    assert_eq!(catalog.entries[0].topic, topic);
    assert_eq!(catalog.entries[0].schema_hash, Some(ORACLE_SCHEMA_HASH));
    assert_eq!(robot.binding(&seed), Some(OWNER));
    assert_eq!(plane.connection_count(), 0);
    assert_eq!(plane.reader_count(), 0);

    let key = TopicKey::new("robot", &topic);
    plane.ensure_mirror(&key, ORACLE_SCHEMA_HASH).unwrap();
    assert_oracle_and_gap_free(&collect_frames(&desk, &topic, 3));
    assert_eq!(
        plane.probe_robot("robot", DIAL_TIMEOUT).unwrap(),
        RobotProbeOutcome::Online
    );
    let repeated = plane.query_catalog("robot", DIAL_TIMEOUT).unwrap();
    assert_eq!(repeated.entries[0].topic, topic);
    assert_eq!(repeated.entries[0].schema_hash, Some(ORACLE_SCHEMA_HASH));
    let schema = plane.query_schema("robot", &topic, DIAL_TIMEOUT).unwrap();
    assert_eq!(schema.robot, "robot");
    assert_eq!(schema.requested, topic);
    assert!(schema.docs.is_empty());
    assert!(schema
        .error
        .as_deref()
        .unwrap()
        .contains("no schema available"));
    assert_eq!(
        plane.query_catalog("robot", Duration::ZERO).unwrap_err(),
        "robot catalog query exceeded its time budget"
    );
    assert_eq!(plane.connection_count(), 1);
    assert_eq!(plane.reader_count(), 1);
    assert_oracle_and_gap_free(&collect_frames(&desk, &topic, 3));
    assert_one_pair_attempt(&robot, "ok");
    assert_eq!(plane.release_mirror(&key), MirrorRelease::Retired);
    stop.store(true, Ordering::Relaxed);
    producer.join().unwrap();
}

#[test]
fn foreign_owner_and_revoked_device_metadata_never_return_a_catalog() {
    let seed = unique_secret();
    let robot = OwnerRobot::start("query_foreign", None, None);
    let (_, plane, _) = robot.plane(seed, AccountId([99; 32]));
    assert!(plane.query_catalog("robot", DIAL_TIMEOUT).is_err());
    assert_eq!(robot.binding(&seed), None);
    assert_one_pair_attempt(&robot, "error");
    assert_eq!(plane.reader_count(), 0);

    let revoked_seed = unique_secret();
    let revoked = OwnerRobot::start(
        "query_revoked",
        Some(PublicKey(public_of(&revoked_seed))),
        Some((public_of(&revoked_seed), OWNER)),
    );
    let (_, revoked_plane, _) = revoked.plane(revoked_seed, OWNER);
    assert!(revoked_plane.query_catalog("robot", DIAL_TIMEOUT).is_err());
    assert!(revoked_plane
        .query_schema("robot", "/state", DIAL_TIMEOUT)
        .is_err());
    assert!(revoked.receipts().is_empty());
    assert_eq!(revoked_plane.connection_count(), 0);
}

#[test]
fn cached_self_revocation_is_applied_before_fresh_metadata_is_returned() {
    let seed = unique_secret();
    let robot = OwnerRobot::start("query_epoch", None, None);
    let cache = tempfile::tempdir().unwrap();
    let owner_proof = proof(seed, OWNER);
    let wire = EpochSyncWire::new(
        owner_proof.intermediate.clone(),
        AccessListEpoch {
            version: FORMAT_VERSION,
            robot: ROBOT,
            epoch: 5,
            revoked_accounts: vec![],
            revoked_devices: vec![PublicKey(public_of(&seed))],
            issued_at_ns: T_NOW,
            issuer_key: public(&signing_key(10)),
        }
        .sign(&signing_key(10)),
    );
    std::fs::write(
        cache
            .path()
            .join(cerulion_wireclient::config::epoch_cache_file_name("robot")),
        cerulion_wireclient::config::encode_epoch_cache(&wire).unwrap(),
    )
    .unwrap();
    let registry = Arc::new(
        one_robot_registry("robot", &robot.fixture, seed)
            .with_account(Some(OWNER.0))
            .with_epoch_dir(Some(cache.path().to_owned())),
    );
    registry
        .replace_owner_certificate(Some(owner_proof))
        .unwrap();
    let plane = IrohMirrorPlane::new(test_manager("query_epoch_desk"), registry)
        .unwrap()
        .with_dial_timeout(DIAL_TIMEOUT);
    assert!(
        plane.query_catalog("robot", DIAL_TIMEOUT).is_err(),
        "the admission catalog predates the self-revoking epoch and must not escape"
    );
    assert_eq!(
        plane.last_push_outcome("robot"),
        Some(EpochPushOutcome::Applied { epoch: 5 })
    );
    assert_eq!(robot.binding(&seed), Some(OWNER));
    assert_one_pair_attempt(&robot, "ok");
    assert_eq!(plane.connection_count(), 0);
    assert_eq!(plane.reader_count(), 0);
}

#[test]
fn presence_sends_no_control_bytes_and_total_query_timeout_closes_only_its_connection() {
    let rt = tokio::runtime::Runtime::new().unwrap();
    let endpoint = rt.block_on(disabled_endpoint(unique_secret()));
    let eid = endpoint.id();
    let addrs = loopback_sockets(&endpoint.bound_sockets());
    let server = rt.spawn(async move {
        let probe = accept_one(&endpoint).await.unwrap().unwrap();
        assert!(
            accept_frame_stream(&probe.connection).await.is_err(),
            "a TLS-only probe must close before opening a control stream"
        );
        let query = accept_one(&endpoint).await.unwrap().unwrap();
        let (_send, mut recv) = accept_frame_stream(&query.connection).await.unwrap();
        let bytes = read_frame(&mut recv, DEFAULT_MAX_FRAME_LEN).await.unwrap();
        assert!(matches!(
            serde_json::from_slice::<WireRequest>(&bytes).unwrap(),
            WireRequest::Catalog
        ));
        // Deliberately leave the admission request unanswered. The client's total
        // deadline must close this connection before the longer per-await limit.
        query.connection.closed().await;
    });
    let registry = Arc::new(WanRegistry::new(
        HashMap::from([(
            "robot".into(),
            WanRobot {
                eid,
                direct_addrs: addrs,
            },
        )]),
        unique_secret(),
        RelayConfig::Disabled,
    ));
    let plane = IrohMirrorPlane::new(test_manager("query_budget"), registry)
        .unwrap()
        .with_dial_timeout(Duration::from_secs(30));
    assert_eq!(
        plane.probe_robot("robot", DIAL_TIMEOUT).unwrap(),
        RobotProbeOutcome::Online
    );
    let error = plane
        .query_catalog("robot", Duration::from_secs(2))
        .unwrap_err();
    assert_eq!(error, "robot catalog query exceeded its time budget");
    assert_eq!(plane.connection_count(), 0);
    assert_eq!(plane.reader_count(), 0);
    rt.block_on(async {
        tokio::time::timeout(DIAL_TIMEOUT, server)
            .await
            .unwrap()
            .unwrap();
    });
}
