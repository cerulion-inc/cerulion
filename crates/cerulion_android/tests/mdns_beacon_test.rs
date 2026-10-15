// SPDX-License-Identifier: AGPL-3.0-only
//! The app gateway's mDNS beacon, browsed by the desk-side resolver.

use std::time::Duration;

use cerulion_android::EmbeddedRuntime;
use cerulion_cli_engine::mdns_discovery;
use serde_json::json;
use tempfile::tempdir;

const ROBOT: &str = "mdns-beacon-test";

#[test]
#[ignore = "requires real multicast/mDNS loopback; run with `-- --ignored`"]
fn app_gateway_beacon_is_browsable_while_running_and_withdrawn_on_stop() {
    let temp = tempdir().unwrap();
    let port = {
        let listener = std::net::TcpListener::bind("0.0.0.0:0").expect("bind ephemeral port");
        listener.local_addr().unwrap().port()
    };
    let mut runtime = EmbeddedRuntime::new(temp.path().to_path_buf(), ROBOT.into()).unwrap();
    runtime
        .start(&json!({"listen": [format!("tcp/0.0.0.0:{port}")]}).to_string())
        .unwrap();
    assert!(runtime.status_handle().lock().unwrap().mdns_advertised);

    let peers = mdns_discovery::browse_rung(Duration::from_secs(3));
    let found = peers
        .iter()
        .find(|peer| peer.robot == ROBOT)
        .unwrap_or_else(|| panic!("beacon for '{ROBOT}' not browsed; saw {peers:?}"));
    assert!(
        found.locator.ends_with(&format!(":{port}")),
        "locator '{}' must carry port {port}",
        found.locator
    );

    runtime.stop().unwrap();
    assert!(!runtime.status_handle().lock().unwrap().mdns_advertised);

    let peers = mdns_discovery::browse_rung(Duration::from_secs(3));
    assert!(
        peers.iter().all(|peer| peer.robot != ROBOT),
        "beacon for '{ROBOT}' still browsed after stop; saw {peers:?}"
    );
}
