// SPDX-License-Identifier: AGPL-3.0-only
//! Retirement retains ownership until real mirror provenance cleanup finishes.

use super::*;
use cerulion_core::TransportConfig;
use std::sync::mpsc;

#[test]
fn reader_health_distinguishes_absence_busy_and_invalidated_state_without_dialing() {
    let manager = TransportManager::init_for_test(
        TransportConfig::default(),
        cerulion_core::testing::iceoryx_test_config(),
    )
    .unwrap();
    let registry = Arc::new(WanRegistry::new(
        HashMap::new(),
        [0x31; 32],
        cerulion_link::RelayConfig::Disabled,
    ));
    let plane = IrohMirrorPlane::new(manager, registry).unwrap();
    let key = TopicKey::new("robot", "/state");
    assert!(!plane.has_reader(&key).unwrap());
    let held = plane.inner.try_lock().unwrap();
    assert!(plane.has_reader(&key).unwrap_err().contains("busy"));
    assert!(held.endpoint.is_none());
    drop(held);
    plane.invalidate_account().unwrap();
    assert!(plane
        .has_reader(&key)
        .unwrap_err()
        .contains("account changed"));
}

#[test]
fn delayed_old_cleanup_cannot_remove_replacement_provenance() {
    let runtime = tokio::runtime::Builder::new_multi_thread()
        .worker_threads(2)
        .enable_all()
        .build()
        .unwrap();
    let manager = TransportManager::init_for_test(
        TransportConfig {
            node_name: "netd_cleanup_order".into(),
            ..Default::default()
        },
        cerulion_core::testing::iceoryx_test_config(),
    )
    .unwrap();
    let topic = "/retirement/state";
    manager
        .register_mirror_provenance(topic, "old-owner")
        .unwrap();
    let inner = Arc::new(AsyncMutex::new(IrohState {
        endpoint: None,
        robots: HashMap::new(),
        account_invalidated: false,
    }));
    let (entered_tx, entered_rx) = mpsc::sync_channel(1);
    let (continue_tx, continue_rx) = mpsc::sync_channel(1);
    let old_inner = inner.clone();
    let old_manager = manager.clone();
    let old_cleanup = runtime.spawn(async move {
        let state = old_inner.lock().await;
        finish_retirement(state, || {
            entered_tx.send(()).unwrap();
            // This delay is entirely inside the test closure. Shipped cleanup
            // has no timing hook, channel, sleep or altered transport behavior.
            continue_rx.recv_timeout(Duration::from_secs(5)).unwrap();
            assert!(old_manager.unregister_mirror_provenance(topic).unwrap());
        });
    });
    entered_rx.recv_timeout(Duration::from_secs(5)).unwrap();

    // Try-lock is the ordering oracle: replacement cannot own the plane while
    // old cleanup is paused. No elapsed-time assertion decides the outcome.
    let old_cleanup_still_owns_state = inner.try_lock().is_err();
    let replacement_inner = inner.clone();
    let replacement_manager = manager.clone();
    let replacement = runtime.spawn(async move {
        let _state = replacement_inner.lock().await;
        replacement_manager
            .register_mirror_provenance(topic, "replacement-owner")
            .unwrap();
    });
    continue_tx.send(()).unwrap();
    runtime.block_on(async {
        old_cleanup.await.unwrap();
        replacement.await.unwrap();
    });
    assert!(old_cleanup_still_owns_state);
    let records = manager
        .gather_mirror_provenance(Duration::from_millis(300))
        .unwrap();
    let record = records.iter().find(|record| record.topic == topic).unwrap();
    assert_eq!(record.origin_robot, "replacement-owner");
}
