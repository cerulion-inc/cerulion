// SPDX-License-Identifier: AGPL-3.0-only
//! A runtime that shares but did not create the gateway must not revoke its
//! LAN posture on stop. Own binary: the manager and the runtime's
//! process-lifetime transport latch are process-global.

use cerulion_android::EmbeddedRuntime;
use cerulion_core::transport::network::NetworkConfig;
use cerulion_core::transport::{TransportConfig, TransportManager};
use cerulion_core::LanAccess;
use serde_json::json;
use tempfile::tempdir;

#[test]
fn a_listen_less_runtime_stop_keeps_the_gateway_owners_lan_posture() {
    // A gateway some other party created on the process manager (netd hosting
    // the shared plane): the embedded runtime only SHARES it, so its stop
    // must not close LAN access it does not own.
    let port = std::net::TcpListener::bind("127.0.0.1:0")
        .expect("bind ephemeral port")
        .local_addr()
        .expect("local_addr")
        .port();
    let manager = TransportManager::init(TransportConfig {
        node_name: "ownership_host".to_string(),
        network: Some(NetworkConfig {
            listen_endpoints: vec![format!("tcp/127.0.0.1:{port}")],
            robot_identity: Some("ownership_robot".to_string()),
            ..NetworkConfig::default()
        }),
        ..Default::default()
    })
    .expect("init shared manager");
    let network = manager.network().expect("gateway network");
    network.set_lan_access(LanAccess::Open);

    let temp = tempdir().unwrap();
    let mut local =
        EmbeddedRuntime::new(temp.path().to_path_buf(), "ownership_robot".into()).unwrap();
    local
        .start(
            &json!({
                "listen": [],
                "publish": [{"topic": "/ownership/local", "schema": "std_msgs/String"}]
            })
            .to_string(),
        )
        .unwrap();
    // An enabled bridge the owner registered: the sharer's stop must leave
    // its flag alone, not just the manager-wide closed bit.
    let owner_flag = manager
        .bridge_manager()
        .register_topic("/ownership/owner_topic")
        .expect("register owner bridge");
    owner_flag.store(true, std::sync::atomic::Ordering::Relaxed);
    local.stop().unwrap();
    assert_eq!(
        network.lan_access(),
        LanAccess::Open,
        "a local-only runtime's stop must not close the gateway it does not own"
    );
    assert!(
        !manager.bridge_manager().is_lan_closed().unwrap(),
        "a local-only runtime's stop must not close the owner's bridges"
    );
    assert!(
        owner_flag.load(std::sync::atomic::Ordering::Relaxed),
        "a local-only runtime's stop must not disable the owner's bridge"
    );
}
