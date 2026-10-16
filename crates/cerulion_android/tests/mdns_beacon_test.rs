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
    let witness_port = {
        let listener = std::net::TcpListener::bind("0.0.0.0:0").expect("bind witness port");
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

    let _witness = cerulion_mdns::advertise_gateway("mdns-beacon-witness", witness_port)
        .expect("raise witness beacon");
    runtime.stop().unwrap();
    assert!(!runtime.status_handle().lock().unwrap().mdns_advertised);

    // Absence is only proven over a COMPLETE browse: browse until the
    // witness beacon - started after this robot's stop - is visible, which
    // shows the resolver heard post-stop state, then require the stopped
    // robot's token absent in that same browse.
    // A single browse can be partial (its daemon can disconnect mid-window),
    // and a partial browse that resolved the witness but not the robot would
    // read as a withdrawal that never happened. Require TWO consecutive
    // browses that each see the witness AND not the robot.
    let deadline = std::time::Instant::now() + Duration::from_secs(15);
    let mut complete_absent = 0;
    loop {
        let peers = mdns_discovery::browse_rung(Duration::from_secs(3));
        if peers.iter().any(|peer| peer.robot == "mdns-beacon-witness") {
            assert!(
                peers.iter().all(|peer| peer.robot != ROBOT),
                "beacon for '{ROBOT}' still browsed after stop in a complete browse; saw {peers:?}"
            );
            complete_absent += 1;
            if complete_absent >= 2 {
                break;
            }
            continue;
        }
        complete_absent = 0;
        assert!(
            std::time::Instant::now() < deadline,
            "no complete browse ever saw the witness beacon; absence proves nothing"
        );
    }
}
