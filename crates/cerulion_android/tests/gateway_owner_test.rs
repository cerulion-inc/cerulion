// SPDX-License-Identifier: AGPL-3.0-only
//! Only one embedded runtime may own the process network gateway: a second
//! gateway start is refused without touching the owner's LAN posture or
//! bridge grants. Own binary: the manager and the runtime's process-lifetime
//! transport latch are process-global.

use cerulion_android::{EmbedError, EmbeddedRuntime, RuntimeState};
use cerulion_core::transport::TransportManager;
use cerulion_core::LanAccess;
use serde_json::json;
use std::sync::atomic::Ordering;
use tempfile::tempdir;

#[test]
fn a_second_gateway_runtime_is_refused_and_leaves_the_owner_serving() {
    let port = std::net::TcpListener::bind("127.0.0.1:0")
        .expect("bind ephemeral port")
        .local_addr()
        .expect("local_addr")
        .port();
    let config = |topic: &str| {
        json!({
            "listen": [format!("tcp/127.0.0.1:{port}")],
            "lan_access": "open",
            "mdns": false,
            "publish": [{"topic": topic, "schema": "std_msgs/String"}]
        })
        .to_string()
    };

    let owner_dir = tempdir().unwrap();
    let mut owner =
        EmbeddedRuntime::new(owner_dir.path().to_path_buf(), "owner_robot".into()).unwrap();
    owner.start(&config("/gateway_owner/a")).unwrap();
    let manager = TransportManager::get().expect("the owner created the process manager");
    let network = manager.network().expect("gateway network");
    assert_eq!(network.lan_access(), LanAccess::Open);
    // A bridge grant the owner's gateway serves: the refused start must
    // leave it enabled.
    let owner_flag = manager
        .bridge_manager()
        .register_topic("/gateway_owner/granted")
        .expect("register owner bridge");
    owner_flag.store(true, Ordering::Relaxed);

    let second_dir = tempdir().unwrap();
    let mut second =
        EmbeddedRuntime::new(second_dir.path().to_path_buf(), "owner_robot".into()).unwrap();
    let error = second
        .start(&config("/gateway_owner/b"))
        .expect_err("a second gateway runtime cannot share the owner's gateway");
    assert!(matches!(error, EmbedError::BadState(_)), "{error}");
    assert_eq!(
        network.lan_access(),
        LanAccess::Open,
        "a refused gateway start must not close the owner's LAN access"
    );
    assert!(
        !manager.bridge_manager().is_lan_closed().unwrap(),
        "a refused gateway start must not close the owner's bridges"
    );
    assert!(
        owner_flag.load(Ordering::Relaxed),
        "a refused gateway start must not disable the owner's bridge"
    );
    drop(second);
    assert_eq!(network.lan_access(), LanAccess::Open);
    assert!(owner_flag.load(Ordering::Relaxed));
    assert!(matches!(
        owner.status_handle().lock().unwrap().state,
        RuntimeState::Running
    ));

    // Once the owner stops, the gateway is free for another runtime.
    owner.stop().unwrap();
    let mut next =
        EmbeddedRuntime::new(second_dir.path().to_path_buf(), "owner_robot".into()).unwrap();
    next.start(&config("/gateway_owner/b")).unwrap();
    assert_eq!(network.lan_access(), LanAccess::Open);
    next.stop().unwrap();
}
