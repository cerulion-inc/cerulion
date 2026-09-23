// SPDX-License-Identifier: AGPL-3.0-only
//! Owner first-demand admission against the real ops and wire serving paths.

use super::*;
use cerulion_pairing::format::{
    AccessListEpoch, DeviceCert, IntermediateCert, Scope, Validity, FORMAT_VERSION,
};
use cerulion_pairing::verify::OwnerCertificatePresentationWire;
use cerulion_remoted::{RemotedClock, RemotedConfig};
use ed25519_dalek::SigningKey;

const MAC: &[u8] = b"owner-admission-test-store-key";
const ROBOT: RobotId = RobotId([5; 32]);

#[path = "owner_pair_queries.rs"]
mod queries;

#[path = "owner_pair_retirement.rs"]
mod retirement;

#[path = "owner_pair_membership.rs"]
mod membership;

fn signing_key(seed: u8) -> SigningKey {
    SigningKey::from_bytes(&[seed; 32])
}
fn public(key: &SigningKey) -> PublicKey {
    PublicKey(key.verifying_key().to_bytes())
}

fn proof(seed: [u8; 32], account: AccountId) -> Arc<OwnerCertificatePresentationWire> {
    let issuer = signing_key(10);
    let validity = Validity {
        not_before_ns: 0,
        not_after_ns: T_NOW * 100,
    };
    let intermediate = IntermediateCert {
        version: FORMAT_VERSION,
        intermediate_key: public(&issuer),
        validity,
        issued_at_ns: T_NOW / 2,
        max_scope: Scope::OWNER_FULL,
    }
    .sign_by_roots(&[&signing_key(1)]);
    let device_cert = DeviceCert {
        version: FORMAT_VERSION,
        device_key: PublicKey(public_of(&seed)),
        account,
        principal_kind: PrincipalKind::Human,
        scope: Scope::OWNER_FULL,
        validity,
        issued_at_ns: T_NOW / 2,
        issuer_key: public(&issuer),
    }
    .sign(&issuer);
    Arc::new(OwnerCertificatePresentationWire::new(
        intermediate,
        device_cert,
    ))
}

struct OwnerRobot {
    config: RemotedConfig,
    fixture: RobotFixture,
    _directory: tempfile::TempDir,
}

impl OwnerRobot {
    fn start(
        tag: &str,
        revoked_device: Option<PublicKey>,
        indexed_account: Option<([u8; 32], AccountId)>,
    ) -> Self {
        Self::start_with_robot_id(tag, ROBOT, revoked_device, indexed_account)
    }

    fn start_with_robot_id(
        tag: &str,
        robot_id: RobotId,
        revoked_device: Option<PublicKey>,
        indexed_account: Option<([u8; 32], AccountId)>,
    ) -> Self {
        let directory = tempfile::tempdir().unwrap();
        let config = RemotedConfig::from_state_root(directory.path(), RelayConfig::Disabled, false);
        std::fs::create_dir_all(config.store_file.parent().unwrap()).unwrap();
        std::fs::create_dir_all(&config.log_root).unwrap();
        let rt = tokio::runtime::Runtime::new().unwrap();
        let endpoint = rt.block_on(disabled_endpoint(unique_secret()));
        let manager = test_manager(tag);
        let mut store = TrustStore::provision(
            robot_id,
            PublicKey(*endpoint.id().as_bytes()),
            RootSet::new(vec![public(&signing_key(1))], 1).unwrap(),
            CHASSIS,
            T_NOW,
        )
        .unwrap()
        .with_path(&config.store_file);
        store
            .claim(OWNER, CHASSIS, PrincipalKind::Human, T_NOW)
            .unwrap();
        if let Some(device) = revoked_device {
            let epoch = AccessListEpoch {
                version: FORMAT_VERSION,
                robot: robot_id,
                epoch: 1,
                revoked_accounts: vec![],
                revoked_devices: vec![device],
                issued_at_ns: T_NOW,
                issuer_key: public(&signing_key(10)),
            }
            .sign(&signing_key(10));
            store
                .apply_epoch(&epoch, &proof([20; 32], OWNER).intermediate, T_NOW)
                .unwrap();
        }
        store.save(MAC).unwrap();
        let mut index = DeviceAccountIndex::new().with_path(&config.index_file);
        if let Some((key, account)) = indexed_account {
            index.bind(PublicKey(key), account);
        }
        index.save(MAC).unwrap();
        let serving_config = config.clone();
        let serving_endpoint = endpoint.clone();
        let serving_manager = manager.clone();
        let serve = rt.spawn(async move {
            cerulion_remoted::serve_endpoint(
                &serving_config,
                store,
                index,
                MAC.to_vec(),
                &serving_endpoint,
                RemotedClock::fixed(T_NOW),
                Some(Duration::from_millis(20)),
                Some(serving_manager),
                std::future::pending(),
            )
            .await
            .unwrap();
        });
        Self {
            fixture: RobotFixture {
                eid: endpoint.id(),
                addrs: loopback_sockets(&endpoint.bound_sockets()),
                rt,
                endpoint,
                manager,
                _serve: serve,
            },
            config,
            _directory: directory,
        }
    }

    fn receipts(&self) -> Vec<serde_json::Value> {
        std::fs::read_to_string(&self.config.receipt_file)
            .unwrap_or_default()
            .lines()
            .map(|line| serde_json::from_str(line).unwrap())
            .collect()
    }

    fn binding(&self, seed: &[u8; 32]) -> Option<AccountId> {
        DeviceAccountIndex::load(&self.config.index_file, MAC)
            .unwrap()
            .account_for(&PublicKey(public_of(seed)))
    }

    fn plane(
        &self,
        seed: [u8; 32],
        account: AccountId,
    ) -> (Arc<WanRegistry>, IrohMirrorPlane, Arc<TransportManager>) {
        let registry = Arc::new(
            one_robot_registry("robot", &self.fixture, seed).with_account(Some(account.0)),
        );
        registry
            .replace_owner_certificate(Some(proof(seed, account)))
            .unwrap();
        let manager = test_manager("owner_desk");
        let plane = IrohMirrorPlane::new(manager.clone(), registry.clone())
            .unwrap()
            .with_dial_timeout(DIAL_TIMEOUT);
        (registry, plane, manager)
    }
}

#[test]
fn first_owner_demand_pairs_once_delivers_data_and_invalidates_sessions() {
    let seed = unique_secret();
    let robot = OwnerRobot::start("owner_robot", None, None);
    let (registry, plane, desk) = robot.plane(seed, OWNER);
    let topic = format!("/owner/{}", unique_id());
    let stop = Arc::new(AtomicBool::new(false));
    let producer = spawn_producer(robot.fixture.manager.clone(), topic.clone(), stop.clone());
    let key = TopicKey::new("robot", &topic);
    plane.ensure_mirror(&key, ORACLE_SCHEMA_HASH).unwrap();
    let frames = collect_frames(&desk, &topic, 3);
    assert_oracle_and_gap_free(&frames);
    assert_eq!(robot.binding(&seed), Some(OWNER));
    let store = TrustStore::load(&robot.config.store_file, MAC).unwrap();
    let row = store.is_allowed(&OWNER, T_NOW).unwrap();
    assert_eq!(row.scope, Scope::OWNER_FULL);
    assert_eq!(row.source, cerulion_pairing::verify::PairingSource::Claim);
    assert_eq!(row.expires_at_ns, None);
    assert_eq!(plane.release_mirror(&key), MirrorRelease::Retired);
    plane.ensure_mirror(&key, ORACLE_SCHEMA_HASH).unwrap();
    assert_oracle_and_gap_free(&collect_frames(&desk, &topic, 3));
    let receipts = robot.receipts();
    assert_eq!(
        receipts
            .iter()
            .filter(|r| r["verb"] == "pair" && r["outcome"]["outcome"] == "ok")
            .count(),
        1
    );
    assert!(receipts.iter().all(|r| r["verb"] == "pair"));
    plane.invalidate_account().unwrap();
    assert!(registry.owner_certificate().unwrap().is_none());
    assert_eq!(plane.connection_count(), 0);
    assert_eq!(plane.reader_count(), 0);
    assert!(plane.ensure_mirror(&key, ORACLE_SCHEMA_HASH).is_err());
    stop.store(true, Ordering::Relaxed);
    producer.join().unwrap();
}

#[test]
fn non_owner_certificate_is_refused_without_a_binding_or_code_fallback() {
    let seed = unique_secret();
    let robot = OwnerRobot::start("foreign_owner", None, None);
    let (_, plane, _) = robot.plane(seed, AccountId([99; 32]));
    assert!(plane
        .ensure_mirror(&TopicKey::new("robot", "/denied"), ORACLE_SCHEMA_HASH)
        .is_err());
    assert_eq!(robot.binding(&seed), None);
    assert_eq!(plane.connection_count(), 0);
    assert_eq!(plane.reader_count(), 0);
    let receipts = robot.receipts();
    assert!(
        !receipts.is_empty(),
        "the owner proof reached the real pair verb"
    );
    assert!(receipts.iter().all(|r| r["verb"] == "pair"));
    assert_eq!(
        receipts
            .iter()
            .filter(|r| r["outcome"]["outcome"] == "error")
            .count(),
        1
    );
}

#[test]
fn revoked_device_and_missing_account_row_never_start_owner_pairing() {
    for missing_row in [false, true] {
        let seed = unique_secret();
        let robot = OwnerRobot::start(
            "denied_before_pair",
            (!missing_row).then_some(PublicKey(public_of(&seed))),
            missing_row.then_some((public_of(&seed), AccountId([98; 32]))),
        );
        let (_, plane, _) = robot.plane(seed, OWNER);
        assert!(plane
            .ensure_mirror(&TopicKey::new("robot", "/denied"), ORACLE_SCHEMA_HASH)
            .is_err());
        assert_eq!(plane.connection_count(), 0);
        assert_eq!(plane.reader_count(), 0);
        assert!(
            robot.receipts().is_empty(),
            "refused access must not attempt any ops verb"
        );
        assert_eq!(
            robot.binding(&seed),
            missing_row.then_some(AccountId([98; 32]))
        );
    }
}

#[test]
fn rejected_replacement_clears_old_proof_and_clones_share_invalidation() {
    let seed = unique_secret();
    let registry =
        WanRegistry::new(HashMap::new(), seed, RelayConfig::Disabled).with_account(Some(OWNER.0));
    registry
        .replace_owner_certificate(Some(proof(seed, OWNER)))
        .unwrap();
    let clone = registry.clone();
    assert!(registry
        .replace_owner_certificate(Some(proof([31; 32], OWNER)))
        .is_err());
    assert!(clone.owner_certificate().unwrap().is_none());
    registry
        .replace_owner_certificate(Some(proof(seed, OWNER)))
        .unwrap();
    assert!(registry
        .replace_owner_certificate(Some(proof(seed, AccountId([99; 32]))))
        .is_err());
    assert!(clone.owner_certificate().unwrap().is_none());
    registry
        .replace_owner_certificate(Some(proof(seed, OWNER)))
        .unwrap();
    let changed = registry.with_account(Some([99; 32]));
    assert!(changed.owner_certificate().is_err());
}

#[test]
fn peer_catalog_demand_and_transport_errors_are_sanitized_before_surfacing() {
    for phase in ["catalog", "demand", "transport"] {
        let rt = tokio::runtime::Runtime::new().unwrap();
        let endpoint = rt.block_on(disabled_endpoint(unique_secret()));
        let eid = endpoint.id();
        let addrs = loopback_sockets(&endpoint.bound_sockets());
        let server = rt.spawn(async move {
            let accepted = accept_one(&endpoint).await.unwrap().unwrap();
            let (mut send, mut recv) = accept_frame_stream(&accepted.connection).await.unwrap();
            let catalog = read_frame(&mut recv, DEFAULT_MAX_FRAME_LEN).await.unwrap();
            assert!(matches!(
                serde_json::from_slice::<WireRequest>(&catalog).unwrap(),
                WireRequest::Catalog
            ));
            if phase == "transport" {
                accepted
                    .connection
                    .close(0u32.into(), b"peer\x1b[2J\nclose");
                return;
            }
            if phase == "demand" {
                let reply = WireResponse::Catalog(CatalogReply {
                    version: CATALOG_WIRE_VERSION,
                    robot: "robot".into(),
                    entries: vec![],
                    error: None,
                });
                write_frame(&mut send, &serde_json::to_vec(&reply).unwrap())
                    .await
                    .unwrap();
                let demand = read_frame(&mut recv, DEFAULT_MAX_FRAME_LEN).await.unwrap();
                assert!(matches!(
                    serde_json::from_slice::<WireRequest>(&demand).unwrap(),
                    WireRequest::Demand { .. }
                ));
            }
            let reply = WireResponse::Error {
                topic: None,
                message: "peer\u{1b}[2J\nclose".into(),
            };
            write_frame(&mut send, &serde_json::to_vec(&reply).unwrap())
                .await
                .unwrap();
            let _ = send.finish();
            let _ = read_frame(&mut recv, DEFAULT_MAX_FRAME_LEN).await;
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
        let plane = IrohMirrorPlane::new(test_manager("peer_error"), registry)
            .unwrap()
            .with_dial_timeout(DIAL_TIMEOUT);
        let error = plane
            .ensure_mirror(&TopicKey::new("robot", "/denied"), ORACLE_SCHEMA_HASH)
            .unwrap_err();
        let MirrorError::Iroh { reason, .. } = error else {
            panic!("expected WAN error");
        };
        assert!(!reason.contains('\u{1b}'), "{phase}: {reason:?}");
        assert!(!reason.contains('\n'), "{phase}: {reason:?}");
        if phase != "transport" {
            assert!(reason.contains("peer"));
        }
        assert_eq!(plane.connection_count(), 0);
        assert_eq!(plane.reader_count(), 0);
        rt.block_on(async {
            tokio::time::timeout(DIAL_TIMEOUT, server)
                .await
                .unwrap()
                .unwrap();
        });
    }
}

#[test]
fn serving_machine_refuses_wan_before_constructing_a_client_runtime() {
    let manager = test_manager("serving_role");
    let seed = unique_secret();
    let registry = Arc::new(WanRegistry::new(
        HashMap::from([(
            "wan_robot".into(),
            WanRobot {
                eid: EndpointId::from_bytes(&public_of(&unique_secret())).unwrap(),
                direct_addrs: vec![],
            },
        )]),
        seed,
        RelayConfig::Disabled,
    ));
    let runtime = tokio::runtime::Runtime::new().unwrap();
    runtime.block_on(async {
        // Dropping an owned client runtime inside this async context would
        // panic. The serving variant must not construct or own one at all.
        let plane = DualMirrorPlane::for_serving_machine(
            GatewayMirrorPlane::new(manager.clone()), registry,
        );
        let wan = TopicKey::new("wan_robot", "/image");
        let lan = TopicKey::new("lan_robot", "/image");
        assert_eq!(plane.plane_for(&wan).unwrap(), Plane::Iroh);
        assert_eq!(plane.plane_for(&lan).unwrap(), Plane::Zenoh);
        let error = plane.ensure_mirror(&wan, ORACLE_SCHEMA_HASH).unwrap_err();
        let MirrorError::Iroh { key, reason } = error else { panic!("expected explicit WAN refusal") };
        assert_eq!(key, wan);
        assert_eq!(reason, "this machine serves the WAN plane; consuming other robots over the WAN from a serving machine is not supported in this version; use the LAN plane");
        assert_eq!(plane.release_mirror(&wan), MirrorRelease::Retired);
        drop(plane);
    });
}

#[path = "owner_pair_controller.rs"]
mod controller;
