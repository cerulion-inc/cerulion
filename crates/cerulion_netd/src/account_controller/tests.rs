// SPDX-License-Identifier: AGPL-3.0-only
use super::*;

#[test]
fn restoration_deadline_bounds_attempts_and_refuses_the_deadline_boundary() {
    let start = Instant::now();
    let deadline = start + RESTORATION_BUDGET;
    let attempted: Vec<_> = [0, 2, 5, 7, 20]
        .into_iter()
        .enumerate()
        .filter_map(|(pin, elapsed)| {
            restoration_time_left(deadline, start + Duration::from_secs(elapsed)).map(|_| pin)
        })
        .collect();
    assert_eq!(attempted, [0, 1]);
    assert_eq!(
        restoration_time_left(deadline, start),
        Some(Duration::from_secs(5))
    );
    assert_eq!(
        restoration_time_left(deadline, start + Duration::from_secs(2)),
        Some(Duration::from_secs(3))
    );
    assert_eq!(restoration_time_left(deadline, deadline), None);
    assert_eq!(
        restoration_time_left(deadline, deadline + Duration::from_nanos(1)),
        None
    );
}
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

#[test]
fn serving_without_outbound_readers_needs_no_migration_connection() {
    for _ in 0..2 {
        let login = Login::new();
        let controller = login.controller(true, false);
        controller.begin_serving().unwrap();
        let started = Instant::now();
        assert!(controller.serving_ready().is_ok());
        let state = controller.state.lock().unwrap();
        assert!(state.plane.is_none());
        assert!(state.handover.is_none());
        assert!(!state.pending_all);
        assert!(started.elapsed() < Duration::from_millis(50));
    }
}

#[test]
fn busy_serving_without_wan_readers_keeps_lan_readers_and_reports_no_loss() {
    for _ in 0..2 {
        let login = Login::new();
        let controller = login.controller(true, false);
        let lan = TopicKey::new("lan", "/local");
        controller.pins().unwrap().insert(
            lan.clone(),
            Pin {
                route: Route::Lan,
                schema_hash: 7,
                live: true,
            },
        );
        controller.begin_serving().unwrap();
        let mut state = controller.state.lock().unwrap();
        assert!(controller.serving_ready().is_ok());
        assert!(controller.pending_keys(&state).unwrap().is_empty());
        let mut registry = DemandRegistry::new(Instant::now());
        controller
            .retire_pending(&mut state, &mut registry)
            .unwrap();
        assert!(state.handover.is_none());
        assert!(controller.pins().unwrap()[&lan].live);
    }
}

#[test]
fn serving_ready_cancels_each_wan_pin_without_waiting_for_a_busy_controller() {
    let login = Login::new();
    let controller = login.controller(true, false);
    let wan = TopicKey::observed("arm", "/vector");
    let lan = TopicKey::new("lan", "/local");
    {
        let mut pins = controller.pins().unwrap();
        for (key, route) in [(wan.clone(), Route::Wan), (lan.clone(), Route::Lan)] {
            pins.insert(
                key,
                Pin {
                    route,
                    schema_hash: 7,
                    live: true,
                },
            );
        }
    }
    let mut state = controller.state.lock().unwrap();
    state.serving_gateway = true;
    state.handover = Some(vec![(wan.clone(), 7)]);
    let started = Instant::now();
    let error = controller.serving_ready().unwrap_err();
    assert!(started.elapsed() < Duration::from_secs(1));
    assert!(
        error.contains("arm") && error.contains("/vector"),
        "{error}"
    );
    assert_eq!(
        controller.pending_keys(&state).unwrap(),
        BTreeSet::from([wan.clone()])
    );
    let pins = controller.pins().unwrap();
    assert!(!pins[&wan].live);
    assert!(pins[&lan].live);
    drop(pins);
    let mut registry = DemandRegistry::new(Instant::now());
    controller
        .retire_pending(&mut state, &mut registry)
        .unwrap();
    assert!(
        state.handover.is_none(),
        "cancellation must permit later re-demands"
    );
    assert!(controller.pins().unwrap().contains_key(&lan));
}

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
fn resolved_network_off_refuses_before_plane_or_hold() {
    let login = Login::new();
    for (network, serving, refusal) in [(false, false, NETWORK_OFF), (false, true, NETWORK_OFF)] {
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
    controller.pins().unwrap().insert(
        key.clone(),
        Pin {
            route: Route::Wan,
            schema_hash: 1,
            live: true,
        },
    );
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
    controller.pins().unwrap().insert(
        key.clone(),
        Pin {
            route: Route::Wan,
            schema_hash: 1,
            live: true,
        },
    );
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

#[test]
fn last_holder_release_during_controller_io_cannot_revive_a_migrating_pin() {
    let login = Login::new();
    let controller = login.controller(true, true);
    let now = Instant::now();
    let mut registry = DemandRegistry::new(now);
    controller
        .install_account_snapshot(&login.snapshot(vec![robot(1, true)]), &mut registry)
        .unwrap();
    let key = TopicKey::new(account_access::robot_route(&[1; 32]), "/released");
    let conn = registry.connect(now);
    registry.demand(conn, key.clone(), 7, now);
    registry.mark_mirror_present(&key);
    controller.pins().unwrap().insert(
        key.clone(),
        Pin {
            route: Route::Wan,
            schema_hash: 7,
            live: true,
        },
    );
    assert!(controller.holds_outbound_readers());
    let state = controller.state.lock().unwrap();
    registry.release(conn, &key, now);
    assert_eq!(controller.release_mirror(&key), MirrorRelease::Lingering);
    assert!(!controller.pins().unwrap()[&key].live);
    assert!(
        controller.holds_outbound_readers(),
        "busy must remain conservative"
    );
    drop(state);
    assert!(!controller.holds_outbound_readers());
    controller.refresh_identity(&mut registry).unwrap();
    assert!(controller.pins().unwrap().is_empty());
    assert!(registry.snapshot().is_empty());
    assert!(
        registry.take_mirror_losses(conn).is_empty(),
        "released holders receive no loss event"
    );
    assert!(
        controller.state().unwrap().plane.is_none(),
        "a released reader must not construct an endpoint"
    );
}

#[test]
fn a_non_live_pin_without_an_owned_reader_is_retired_on_identity_refresh() {
    for _ in 0..2 {
        let login = Login::new();
        let controller = login.controller(true, false);
        let key = TopicKey::observed("arm", "/absent");
        let runtime = tokio::runtime::Runtime::new().unwrap();
        let endpoint = runtime
            .block_on(cerulion_link::build_endpoint(
                cerulion_link::EndpointConfig::new(SEED).with_relay(RelayConfig::Disabled),
            ))
            .unwrap();
        let plane = IrohMirrorPlane::on_owned_endpoint(
            controller.manager.clone(),
            Arc::new(WanRegistry::new(
                HashMap::new(),
                SEED,
                RelayConfig::Disabled,
            )),
            endpoint.clone(),
            runtime.handle().clone(),
        )
        .unwrap();
        let now = Instant::now();
        let mut registry = DemandRegistry::new(now);
        let conn = registry.connect(now);
        registry.demand(conn, key.clone(), 7, now);
        registry.mark_mirror_present(&key);
        {
            let mut state = controller.state().unwrap();
            controller.sync_identity(&mut state).unwrap();
            state.plane = Some(plane);
        }
        controller.pins().unwrap().insert(
            key.clone(),
            Pin {
                route: Route::Wan,
                schema_hash: 7,
                live: false,
            },
        );
        controller.refresh_identity(&mut registry).unwrap();
        assert!(controller.pins().unwrap().is_empty());
        assert_eq!(registry.active_demand_count(), 0);
        assert_eq!(registry.take_mirror_losses(conn), BTreeSet::from([key]));
        drop(controller);
        runtime.block_on(endpoint.close());
    }
}
