// SPDX-License-Identifier: AGPL-3.0-only
//! The live-peer DDS discovery pin — exercises the real
//! `LiveDiscovery` backend (participant + spinner + SPDP/SEDP collect) against
//! a live CycloneDDS peer. `#[ignore]` because CI has no DDS peer / ROS distro;
//! run it on a machine with a CycloneDDS publisher on the target
//! interface (a `ros2 topic pub` talker, or the go2 robot LAN).
//!
//! PRECONDITION-panic pattern (crib `rmw_cerulion::tests::rclpy_xproc_test`):
//! each test panics with the exact bring-up recipe if `DDS_IFACE` is unset,
//! so a bare `cargo test -p cerulion_dds -- --ignored` on a peerless machine fails
//! LOUDLY with instructions rather than hanging or silently passing.
//!
//! RUN (on a machine with a CycloneDDS talker publishing on the interface):
//!   ros2 run demo_nodes_cpp talker &                       # or the robot
//!   DDS_IFACE="REPLACE_WITH_LOCAL_INTERFACE_IP" DDS_DOMAIN=0 \
//!     cargo test -p cerulion_dds --test live_discovery_box_test -- --ignored --nocapture

// LiveDiscovery only exists with the `live` feature (on by default via
// `jazzy` — the 16-byte-GID Iron+ world; a default build CANNOT decode
// Humble-era 24-byte-GID `ros_discovery_info`, so on a pre-Iron robot an empty
// node table is EXPECTED — opt in via --no-default-features --features humble);
// the whole file compiles to nothing under --no-default-features.
#![cfg(feature = "live")]

use std::net::IpAddr;
use std::time::Duration;

use cerulion_dds::{DdsDiscovery, DiscoveryParams, LiveDiscovery};

/// Resolve the `--iface` under test (or PRECONDITION-panic with the recipe).
fn iface_or_precondition_panic() -> IpAddr {
    match std::env::var("DDS_IFACE") {
        Ok(s) => s
            .parse()
            .unwrap_or_else(|e| panic!("DDS_IFACE={s:?} is not a valid IP: {e}")),
        Err(_) => panic!(
            "PRECONDITION: set DDS_IFACE=<robot-LAN-IP> (the local interface a CycloneDDS \
             peer publishes on) to run the live discovery test. See the module docs \
             for the full bring-up recipe (a `ros2 run demo_nodes_cpp talker` on the same LAN)."
        ),
    }
}

fn domain() -> u16 {
    std::env::var("DDS_DOMAIN")
        .ok()
        .and_then(|s| s.parse().ok())
        .unwrap_or(0)
}

/// Discovery runs, builds a participant, spins, collects endpoints for the
/// window, and returns without error. We do NOT assert a specific topic set
/// (that depends on the live peer) — the pin is that the backend LIFECYCLE
/// works end-to-end and returns a well-formed list.
#[test]
#[ignore = "box-only: needs a live CycloneDDS peer on DDS_IFACE"]
fn live_discovery_runs_and_returns_endpoints() {
    let params = DiscoveryParams {
        only_networks: vec![iface_or_precondition_panic()],
        domain_id: domain(),
        window: Duration::from_secs(6),
    };
    let result = LiveDiscovery
        .discover(&params)
        .expect("live discovery should not error against a reachable peer");
    eprintln!(
        "live discovery saw {} foreign endpoint(s) ({} of our own hidden):",
        result.endpoints.len(),
        result.own_endpoints_hidden
    );
    for e in &result.endpoints {
        eprintln!(
            "  {:?} topic={} type={} qos={}",
            e.kind,
            e.dds_topic,
            e.type_name,
            e.qos.summary()
        );
    }
    // Live-P0 fix 2 pin: NONE of the attach participant's OWN endpoints (the
    // discovery node's parameter services under /cerulion_attach/discovery/)
    // may survive the GUID filter into the foreign list.
    for e in &result.endpoints {
        assert!(
            !e.dds_topic.contains("/cerulion_attach/discovery"),
            "our own discovery endpoint leaked past the GUID filter: {}",
            e.dds_topic
        );
    }
    // A live talker publishes at least one endpoint; a peerless machine would
    // return an empty list (still Ok) — assert non-empty ONLY when the operator
    // asked for it via DDS_EXPECT_TOPICS=1 (avoids a false red on a quiet LAN).
    if std::env::var("DDS_EXPECT_TOPICS").as_deref() == Ok("1") {
        assert!(
            !result.endpoints.is_empty(),
            "DDS_EXPECT_TOPICS=1 but discovery saw nothing — is the talker running on this \
             interface/domain?"
        );
    }
}

/// The one-per-process participant slot is RELEASED on drop: two sequential
/// discover() calls (each builds + drops a participant) both succeed.
#[test]
#[ignore = "box-only: needs a live CycloneDDS peer on DDS_IFACE"]
fn sequential_discovery_reuses_the_participant_slot() {
    let params = DiscoveryParams {
        only_networks: vec![iface_or_precondition_panic()],
        domain_id: domain(),
        window: Duration::from_secs(2),
    };
    LiveDiscovery.discover(&params).expect("first discovery");
    LiveDiscovery
        .discover(&params)
        .expect("second discovery (slot released on drop)");
}
