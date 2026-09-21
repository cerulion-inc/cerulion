// SPDX-License-Identifier: AGPL-3.0-only
//! The ONE live mDNS loopback acceptance test for the
//! `_cerulion._tcp` robot-discovery rung.
//!
//! Real multicast is required, so this is `#[ignore]`d (the `#[ignore]` is
//! the CI opt-out mechanism — CI has no reliable multicast loopback). Run it
//! on a machine with mDNS/multicast:
//!
//! ```bash
//! cargo test -p cerulion_cli_engine --test mdns_live_test -- --ignored
//! ```
//!
//! It advertises a robot over a real `ServiceDaemon`, browses `_cerulion._tcp`
//! from a SECOND daemon in the same process, and asserts the advertised robot
//! resolves back with the right port + `Mdns` rung.
//!
//! The second arm drives the same round trip through
//! `cerulion-netd`'s beacon — the plane that actually serves a robot's topics.

use std::time::Duration;

use cerulion_cli_engine::discovery_ladder::DiscoveryRung;
use cerulion_cli_engine::mdns_discovery;

/// Probe a free TCP port on loopback. The mDNS SRV record only CARRIES the
/// port number (nothing binds it here), so a momentarily-free ephemeral port
/// is a fine, collision-resistant value for the advertisement.
fn probe_free_port() -> u16 {
    let listener = std::net::TcpListener::bind("127.0.0.1:0").expect("bind ephemeral port");
    let port = listener.local_addr().expect("local_addr").port();
    drop(listener);
    port
}

#[test]
#[ignore = "requires real multicast/mDNS loopback — run on a box with `-- --ignored`"]
fn advertise_then_browse_finds_the_robot() {
    const ROBOT: &str = "test-robot";
    let port = probe_free_port();

    // Advertise. The guard keeps the service live for the test body.
    let guard = mdns_discovery::advertise_gateway(ROBOT, port).expect("advertise_gateway");

    // Browse from a separate daemon. 3s is generous for loopback multicast
    // resolution (advertise → announce → resolve).
    let peers = mdns_discovery::browse_rung(Duration::from_secs(3));

    let found = peers.iter().find(|p| p.robot == ROBOT).unwrap_or_else(|| {
        panic!(
            "expected to resolve robot '{ROBOT}' over mDNS; got {} peer(s): {:?}",
            peers.len(),
            peers
                .iter()
                .map(|p| (p.robot.as_str(), p.locator.as_str()))
                .collect::<Vec<_>>()
        )
    });
    assert!(
        found.locator.ends_with(&format!(":{port}")),
        "resolved locator '{}' must carry the advertised port {port}",
        found.locator
    );
    assert!(
        found.locator.starts_with("tcp/"),
        "resolved locator '{}' must be a zenoh tcp/ locator",
        found.locator
    );
    assert_eq!(found.rung, DiscoveryRung::Mdns);

    // Departure: dropping the guard multicasts the mDNS goodbye + shuts the
    // daemon down. We assert ONLY the positive half above — a re-browse
    // "no longer finds it" negative is deliberately OMITTED because the
    // goodbye-packet visibility window is timing-dependent on loopback
    // (unregister is best-effort by design), which would flake this test.
    drop(guard);
}

/// The NETD-HOSTED half of the same round trip — the path a real robot
/// actually serves on.
///
/// The arm above proves the beacon works when the standalone `graph run-gateway`
/// child raises it. Since that child is only the netd-unreachable
/// FALLBACK: a permissive `graph run` pushes its egress plan into the ONE
/// per-computer `cerulion-netd` and spawns no gateway child, so the robot's
/// beacon is netd's to raise. This drives `cerulion_netd`'s PRODUCTION
/// [`GatewayBeacon::ensure_raised`] — the exact function the embedded egress
/// gateway's boot calls — with a ROBOT-shaped `NetworkConfig` (identity + a
/// `tcp/` listen endpoint, what `CERULION_NETD_LISTEN` produces), then browses it
/// back with the exact `browse_rung` the CLI's `topic list` ladder uses.
///
/// So both ENDS are production code and the LAN is real: this is the arm that
/// would have caught the missing netd beacon — before that fix, netd could not advertise at all (it
/// did not even depend on `mdns-sd`), so `ensure_raised` did not exist and the
/// robot resolved to nothing.
#[test]
#[ignore = "requires real multicast/mDNS loopback — run on a box with `-- --ignored`"]
fn a_netd_hosted_robot_beacon_is_browsable_over_a_real_lan() {
    use cerulion_core::transport::network::NetworkConfig;
    use cerulion_netd::beacon::{BeaconDecision, GatewayBeacon};

    const ROBOT: &str = "netd-robot";
    let port = probe_free_port();

    // The ROBOT shape netd resolves on a real robot: this machine's identity
    // (netd stamps the hostname; pinned here so the browse oracle is a hand
    // value) plus a dialable tcp listen endpoint from CERULION_NETD_LISTEN.
    let cfg = NetworkConfig {
        listen_endpoints: vec![format!("tcp/0.0.0.0:{port}")],
        robot_identity: Some(ROBOT.to_string()),
        ..Default::default()
    };

    let beacon = GatewayBeacon::new();
    assert!(
        beacon.ensure_raised(Some(&cfg)),
        "a robot-shaped netd config must RAISE the beacon"
    );
    assert_eq!(
        beacon.decision(),
        Some(BeaconDecision::Advertise {
            robot: ROBOT.to_string(),
            port,
        })
    );
    assert!(beacon.is_advertising());
    assert_eq!(
        beacon.advertised_fullname().as_deref(),
        Some("netd-robot._cerulion._tcp.local."),
        "the beacon registers under the robot identity, not the hostname of the machine"
    );

    // Browse it back through the PRODUCTION CLI rung — the one `topic list`
    // drives. This is the desk half of "mDNS finds robots".
    let peers = mdns_discovery::browse_rung(Duration::from_secs(3));
    let found = peers.iter().find(|p| p.robot == ROBOT).unwrap_or_else(|| {
        panic!(
            "expected the NETD-hosted beacon for '{ROBOT}' to resolve over mDNS; \
             got {} peer(s): {:?}",
            peers.len(),
            peers
                .iter()
                .map(|p| (p.robot.as_str(), p.locator.as_str()))
                .collect::<Vec<_>>()
        )
    });
    assert!(
        found.locator.starts_with("tcp/") && found.locator.ends_with(&format!(":{port}")),
        "resolved locator '{}' must be a zenoh tcp/ locator carrying the bound port {port}",
        found.locator
    );
    assert_eq!(found.rung, DiscoveryRung::Mdns);

    // Departure: same as the arm above — only the POSITIVE half is asserted (the
    // goodbye-packet window is timing-dependent by design).
    drop(beacon);
}
