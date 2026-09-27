// SPDX-License-Identifier: AGPL-3.0-only
use super::*;
use crate::account_access::{
    AccountAccessReply, AccountAccessRequest, AccountRobot, RobotPresence,
};
use crate::mirror::MirrorRelease;
use base64::{engine::general_purpose::URL_SAFE_NO_PAD, Engine};
use cerulion_core::transport::demand_authorizer::AllowAllAuthorizer;
use cerulion_core::transport::TransportConfig;
use cerulion_link::RelayConfig;
use cerulion_pairing::client::DeviceIdentity;

const ACCOUNT: [u8; 32] = [5; 32];
const SEED: [u8; 32] = [9; 32];

struct Login {
    home: tempfile::TempDir,
}
impl Login {
    fn new() -> Self {
        let login = Self {
            home: tempfile::tempdir().unwrap(),
        };
        std::fs::write(
            login
                .home
                .path()
                .join(cerulion_discovery::robot_state::AUTH_STORE_LOCK_FILE),
            [],
        )
        .unwrap();
        login.write(ACCOUNT, SEED);
        login
    }
    fn write(&self, account: [u8; 32], seed: [u8; 32]) {
        let lock = std::fs::OpenOptions::new()
            .read(true)
            .write(true)
            .open(
                self.home
                    .path()
                    .join(cerulion_discovery::robot_state::AUTH_STORE_LOCK_FILE),
            )
            .unwrap();
        lock.lock().unwrap();
        std::fs::write(self.home.path().join("desk.key"), seed).unwrap();
        std::fs::write(
            self.home.path().join("auth.json"),
            serde_json::to_vec(&serde_json::json!({
                "account_id": URL_SAFE_NO_PAD.encode(account), "session_token":"expired",
                "refresh_token":"refresh", "expires_at_ns":1, "logged_in_ever":true,
            }))
            .unwrap(),
        )
        .unwrap();
    }
    fn snapshot(&self, robots: Vec<AccountRobot>) -> AccountSnapshot {
        let identity = identity_snapshot::load_at(self.home.path()).unwrap();
        AccountSnapshot {
            auth_account_id: identity.auth_account_id().into(),
            pairing_account_id: identity.account().0,
            device_key: identity.device_key().0,
            owner_chain: identity.owner_chain_wire().map(Vec::from),
            robots,
        }
    }
    fn controller(&self, network_enabled: bool, serving_gateway: bool) -> AccountWanController {
        let manager = TransportManager::init_for_test(
            TransportConfig::default(),
            cerulion_core::testing::iceoryx_test_config(),
        )
        .unwrap();
        AccountWanController::new(
            manager,
            WanRegistry::new(HashMap::new(), [7; 32], RelayConfig::Disabled),
            Arc::new(AllowAllAuthorizer),
            AccountControllerConfig {
                config_home: Some(self.home.path().to_owned()),
                network_enabled,
                serving_gateway,
                epoch_dir: None,
                trusted_direct: HashMap::new(),
            },
        )
        .unwrap()
    }
}

fn robot(id: u8, endpoint: bool) -> AccountRobot {
    AccountRobot {
        robot_id: [id; 32],
        hostname: format!("robot-{id}"),
        endpoint_key: endpoint.then(|| DeviceIdentity::from_seed(&[id; 32]).public_key().0),
    }
}

#[test]
fn busy_grace_has_a_precise_monotonic_boundary() {
    let now = Instant::now();
    assert!(!busy_grace_elapsed(now, now));
    assert!(!busy_grace_elapsed(now, now + Duration::from_millis(4999)));
    assert!(busy_grace_elapsed(now, now + Duration::from_secs(5)));
    assert!(busy_grace_elapsed(now, now + Duration::from_secs(6)));
    assert!(!busy_grace_elapsed(now + Duration::from_secs(1), now));
}

#[test]
fn install_and_unknown_presence_never_construct_a_wan_plane() {
    let login = Login::new();
    let controller = login.controller(true, false);
    let snapshot = login.snapshot(vec![robot(1, false)]);
    let mut registry = DemandRegistry::new(Instant::now());
    assert!(matches!(
        controller
            .install_account_snapshot(&snapshot, &mut registry)
            .unwrap(),
        AccountAccessReply::Installed { robot_count: 1 }
    ));
    assert!(controller.state().unwrap().plane.is_none());
    let reply = controller
        .account_access(&AccountAccessRequest::Probe {
            robot_id: [1; 32],
            budget_ms: 100,
        })
        .unwrap();
    assert!(matches!(
        reply,
        AccountAccessReply::Presence {
            presence: RobotPresence::Unknown { .. },
            ..
        }
    ));
    assert!(controller.state().unwrap().plane.is_none());
    assert!(controller
        .account_access(&AccountAccessRequest::Catalog { robot_id: [1; 32] })
        .is_err());
    assert!(controller
        .prepare_demand(
            &TopicKey::new(account_access::robot_route(&[1; 32]), "/state"),
            &mut registry
        )
        .is_err());
    assert_eq!(registry.active_demand_count(), 0);
}

#[test]
fn resolved_network_and_serving_postures_refuse_before_plane_or_hold() {
    let login = Login::new();
    for (network, serving, refusal) in [
        (false, false, NETWORK_OFF),
        (true, true, crate::wan::SERVING_MACHINE_WAN_REFUSAL),
    ] {
        let controller = login.controller(network, serving);
        let mut registry = DemandRegistry::new(Instant::now());
        controller
            .install_account_snapshot(&login.snapshot(vec![robot(1, true)]), &mut registry)
            .unwrap();
        assert_eq!(
            controller
                .account_access(&AccountAccessRequest::Probe {
                    robot_id: [1; 32],
                    budget_ms: 100
                })
                .unwrap_err(),
            refusal
        );
        let key = TopicKey::new(account_access::robot_route(&[1; 32]), "/state");
        assert_eq!(
            controller.prepare_demand(&key, &mut registry).unwrap_err(),
            refusal
        );
        assert!(controller
            .ensure_mirror(&key, 1)
            .unwrap_err()
            .to_string()
            .contains(refusal));
        assert!(controller.state().unwrap().plane.is_none());
        assert_eq!(registry.active_demand_count(), 0);
        assert!(controller.pins().unwrap().is_empty());
    }
}

#[test]
fn public_snapshot_fields_are_compared_with_one_actual_login_transaction() {
    let login = Login::new();
    let controller = login.controller(false, false);
    let snapshot = login.snapshot(Vec::new());
    let mut registry = DemandRegistry::new(Instant::now());
    for field in 0..4 {
        let mut altered = snapshot.clone();
        match field {
            0 => altered.auth_account_id = URL_SAFE_NO_PAD.encode([8; 32]),
            1 => altered.pairing_account_id = [8; 32],
            2 => altered.device_key = [8; 32],
            3 => altered.owner_chain = Some(vec![8]),
            _ => unreachable!(),
        }
        assert_eq!(
            controller
                .install_account_snapshot(&altered, &mut registry)
                .unwrap_err(),
            "account robot snapshot does not match the current local login"
        );
        assert!(!controller.state().unwrap().account_mode);
    }
    controller
        .install_account_snapshot(&snapshot, &mut registry)
        .unwrap();
    let lock = std::fs::File::open(
        login
            .home
            .path()
            .join(cerulion_discovery::robot_state::AUTH_STORE_LOCK_FILE),
    )
    .unwrap();
    lock.lock().unwrap();
    assert_eq!(
        controller
            .install_account_snapshot(&snapshot, &mut registry)
            .unwrap_err(),
        IDENTITY_BUSY
    );
    assert!(controller.state().unwrap().identity.is_some());
}

#[test]
fn membership_preflight_and_retirement_preserve_tearing_generation_order() {
    // This is a pure registry/route state oracle. Real delivery is exercised by
    // the owner-pair controller integration fixture, not inferred from these pins.
    let login = Login::new();
    let controller = login.controller(true, false);
    let mut registry = DemandRegistry::new(Instant::now());
    let initial = login.snapshot(vec![robot(1, true), robot(2, true)]);
    controller
        .install_account_snapshot(&initial, &mut registry)
        .unwrap();
    let key = TopicKey::new(account_access::robot_route(&[1; 32]), "/state");
    let old_conn = registry.connect(Instant::now());
    registry.demand(old_conn, key.clone(), 1, Instant::now());
    registry.mark_mirror_present(&key);
    controller.pins().unwrap().insert(key.clone(), Route::Wan);
    registry.release(old_conn, &key, Instant::now());
    assert!(registry.claim_tearing(&key));
    let smaller = login.snapshot(vec![robot(2, true)]);
    assert!(controller
        .install_account_snapshot(&smaller, &mut registry)
        .unwrap_err()
        .contains("still being retired"));
    assert_eq!(
        controller
            .state()
            .unwrap()
            .snapshot
            .as_ref()
            .unwrap()
            .robots
            .len(),
        2
    );
    assert!(controller.pins().unwrap().contains_key(&key));
    assert_eq!(controller.release_mirror(&key), MirrorRelease::Retired);
    assert!(
        controller.pins().unwrap().contains_key(&key),
        "physical release cannot discard its generation pin"
    );
    assert!(registry.retire(&key));
    controller.mirror_retired(&key);
    assert!(!controller.pins().unwrap().contains_key(&key));
    controller
        .install_account_snapshot(&smaller, &mut registry)
        .unwrap();
    assert_eq!(
        controller
            .state()
            .unwrap()
            .snapshot
            .as_ref()
            .unwrap()
            .robots
            .len(),
        1
    );
}

#[test]
fn changed_identity_retires_all_old_holds_before_reinstall() {
    let login = Login::new();
    let controller = login.controller(true, false);
    let mut registry = DemandRegistry::new(Instant::now());
    controller
        .install_account_snapshot(&login.snapshot(vec![robot(1, true)]), &mut registry)
        .unwrap();
    let key = TopicKey::new(account_access::robot_route(&[1; 32]), "/state");
    let old = registry.connect(Instant::now());
    registry.demand(old, key.clone(), 1, Instant::now());
    registry.mark_mirror_present(&key);
    controller.pins().unwrap().insert(key.clone(), Route::Wan);
    login.write([6; 32], [10; 32]);
    assert_eq!(
        controller.prepare_demand(&key, &mut registry).unwrap_err(),
        NEED_INSTALL
    );
    assert_eq!(registry.active_demand_count(), 0);
    assert!(controller.pins().unwrap().is_empty());
    controller
        .install_account_snapshot(&login.snapshot(vec![robot(1, true)]), &mut registry)
        .unwrap();
    let new = registry.connect(Instant::now());
    registry.demand(new, key.clone(), 1, Instant::now());
    registry.mark_mirror_present(&key);
    registry.disconnect(old, Instant::now());
    assert_eq!(
        registry.active_demand_count(),
        1,
        "old connection cleanup cannot decrement the replacement generation"
    );
}

#[test]
fn wan_query_mutex_does_not_block_lan_admission_or_bad_route_refusal() {
    let login = Login::new();
    let controller = login.controller(true, false);
    let _query = controller.state().unwrap();
    let mut registry = DemandRegistry::new(Instant::now());
    assert!(controller
        .prepare_demand(&TopicKey::new("lan-robot", "/state"), &mut registry)
        .is_ok());
    assert!(controller
        .prepare_demand(&TopicKey::new("account:bad", "/state"), &mut registry)
        .unwrap_err()
        .contains("64 lowercase"));
    assert_eq!(
        controller
            .prepare_demand(
                &TopicKey::new(account_access::robot_route(&[1; 32]), "/state"),
                &mut registry
            )
            .unwrap_err(),
        BUSY
    );
}

#[test]
fn manual_registry_cannot_claim_the_reserved_account_namespace() {
    let login = Login::new();
    let manager = TransportManager::init_for_test(
        TransportConfig::default(),
        cerulion_core::testing::iceoryx_test_config(),
    )
    .unwrap();
    for name in [
        "account:bad".into(),
        account_access::robot_route(&[5; 32]),
        " account:bad ".into(),
    ] {
        let peer = WanRobot {
            eid: EndpointId::from_bytes(&DeviceIdentity::from_seed(&SEED).public_key().0).unwrap(),
            direct_addrs: Vec::new(),
        };
        let result = AccountWanController::new(
            manager.clone(),
            WanRegistry::new(HashMap::from([(name, peer)]), SEED, RelayConfig::Disabled),
            Arc::new(AllowAllAuthorizer),
            AccountControllerConfig {
                config_home: Some(login.home.path().to_owned()),
                network_enabled: true,
                serving_gateway: false,
                epoch_dir: None,
                trusted_direct: HashMap::new(),
            },
        );
        assert!(result.is_err());
    }
}

#[test]
fn poisoned_controller_refuses_wan_without_rerouting_it_to_lan() {
    let login = Login::new();
    let controller = login.controller(true, false);
    std::thread::scope(|scope| {
        assert!(scope
            .spawn(|| {
                let _guard = controller.state.lock().unwrap();
                panic!("poison the query state for the refusal oracle");
            })
            .join()
            .is_err());
    });
    let mut registry = DemandRegistry::new(Instant::now());
    let key = TopicKey::new(account_access::robot_route(&[1; 32]), "/state");
    assert_eq!(
        controller.prepare_demand(&key, &mut registry).unwrap_err(),
        "account robot controller state is poisoned"
    );
    assert!(controller.pins().unwrap().is_empty());
    assert_eq!(registry.active_demand_count(), 0);
    assert!(controller
        .prepare_demand(&TopicKey::new("lan-robot", "/state"), &mut registry)
        .is_ok());
}
