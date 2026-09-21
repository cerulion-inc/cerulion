// SPDX-License-Identifier: AGPL-3.0-only
//! The PRODUCTION announce-watch path over a REAL zenoh session — proof that
//! `GatewayAnnounceWatchPlane` (and cerulion_core's `AnnounceWatch` under it) is not
//! inert (the "no inert shipping" rule).
//!
//! The `catalog_events_e2e_test.rs` suite drives the daemon with a SCRIPTED announce
//! stream, which pins every coalescing / push / no-wedge contract but proves nothing
//! about whether real announce tokens ever reach the watch. This file closes that: it
//! declares REAL `TopicToken`s (the exact primitive `cerulion graph run`'s gateway
//! declares for every produced topic) and asserts the watch reports
//! them — first as `Alive`, then as `Lost` when the token drops.
//!
//! SAME-SESSION by design: liveliness is delivered to subscribers on the declaring
//! session too (the shape `network_ingress_test.rs` uses for the DEMAND space), so one
//! network-configured, scouting-OFF manager gives a hermetic loopback pin with no
//! second process, no port, and no multicast. What that does NOT cover — a token
//! crossing a real link between two machines — is the same zenoh liveliness delivery
//! the DEMAND watch has ridden since the gateway landed and is exercised by the live gateway
//! tests; what could break here and nowhere else is the announce KEY-SPACE wiring
//! (subscribing to the wrong prefix, the history flag, the Put/Delete mapping), which
//! is exactly what this pins.
//!
//! Parallel-safe (per-test SHM roots + scouting-off local sessions, per-test robot
//! names) — NOT `#[serial]`, mirroring `query_plane_iox2_test.rs`.

use std::sync::Arc;
use std::time::{Duration, Instant};

use cerulion_core::testing::iceoryx_test_config;
use cerulion_core::transport::discovery::{AnnounceEvent, AnnounceWatch, TopicToken};
use cerulion_core::transport::network::NetworkConfig;
use cerulion_core::transport::{TransportConfig, TransportManager};

use cerulion_netd::catalog_events::{AnnounceWatchPlane, GatewayAnnounceWatchPlane, WatchError};

/// How long an arm waits for a real liveliness delivery before declaring failure.
const DELIVERY_BUDGET: Duration = Duration::from_secs(5);

fn networked_test_manager(node: &str) -> Arc<TransportManager> {
    TransportManager::init_for_test(
        TransportConfig {
            node_name: node.to_string(),
            network: Some(NetworkConfig::default()),
            ..TransportConfig::default()
        },
        iceoryx_test_config(),
    )
    .expect("init networked test manager")
}

fn local_only_test_manager(node: &str) -> Arc<TransportManager> {
    TransportManager::init_for_test(
        TransportConfig {
            node_name: node.to_string(),
            network: None,
            ..TransportConfig::default()
        },
        iceoryx_test_config(),
    )
    .expect("init local-only test manager")
}

/// Drain the watch until an event satisfying `want` arrives, or the budget expires.
/// Returns every event seen (so a failure message can show what DID arrive).
fn drain_until(
    watch: &AnnounceWatch,
    budget: Duration,
    mut want: impl FnMut(&AnnounceEvent) -> bool,
) -> (bool, Vec<AnnounceEvent>) {
    let deadline = Instant::now() + budget;
    let mut seen = Vec::new();
    while Instant::now() < deadline {
        if let Some(e) = watch.next_event(Duration::from_millis(100)) {
            let hit = want(&e);
            seen.push(e);
            if hit {
                return (true, seen);
            }
        }
    }
    (false, seen)
}

/// THE live pin: a REAL announce token declared the way a robot's gateway declares it
/// reaches the watch as an `Alive`, and DROPPING it reaches the watch as a `Lost`.
/// Both halves matter — the `Lost` arm is the one that makes a dying robot disappear
/// from the sidebar rather than lingering forever.
#[test]
fn a_real_announce_token_reaches_the_watch_alive_then_lost() {
    let manager = networked_test_manager("live_watch");
    let net = manager.network().expect("network configured");
    let session = net.session().expect("session opens");

    let watch = AnnounceWatch::declare(session).expect("the announce watch declares");

    // A robot's gateway announcing a produced topic — the EXACT production primitive.
    let robot = format!("live{}", std::process::id());
    let token = TopicToken::announce(session, &robot, "/utlidar/cloud").expect("announce declares");

    let (alive, seen) = drain_until(&watch, DELIVERY_BUDGET, |e| {
        matches!(
            e,
            AnnounceEvent::Alive { robot: r, topic: Some(t) }
                if r == &robot && t == "/utlidar/cloud"
        )
    });
    assert!(
        alive,
        "a real announce token must reach the watch as Alive; saw {seen:?}"
    );

    // Undeclare (a robot's serve going away) — the watch must be told.
    drop(token);
    let (lost, seen) = drain_until(&watch, DELIVERY_BUDGET, |e| {
        matches!(
            e,
            AnnounceEvent::Lost { robot: r, topic: Some(t) }
                if r == &robot && t == "/utlidar/cloud"
        )
    });
    assert!(
        lost,
        "dropping the token must reach the watch as Lost; saw {seen:?}"
    );
}

/// The BARE identity token (`cerulion_ann/{robot}`, which every gateway declares at
/// boot so a zero-egress robot still shows up) arrives as a topic-less `Alive`. A
/// watch that mis-parsed it would either drop the robot entirely or invent a topic.
#[test]
fn the_bare_identity_token_reaches_the_watch_as_a_topicless_alive() {
    let manager = networked_test_manager("live_identity");
    let net = manager.network().expect("network configured");
    let session = net.session().expect("session opens");
    let watch = AnnounceWatch::declare(session).expect("watch declares");

    let robot = format!("id{}", std::process::id());
    let token = TopicToken::announce_identity(session, &robot).expect("identity declares");

    let (found, seen) = drain_until(
        &watch,
        DELIVERY_BUDGET,
        |e| matches!(e, AnnounceEvent::Alive { robot: r, topic: None } if r == &robot),
    );
    assert!(
        found,
        "the bare identity token must arrive with topic = None; saw {seen:?}"
    );
    drop(token);
}

/// HISTORY replay: a watch declared AFTER a robot is already announcing still learns
/// about it. This is the arm that decides whether a desk that starts late sees a robot
/// that came up first — i.e. whether a consumer needs a separate bootstrap query at all.
#[test]
fn a_watch_declared_after_the_token_still_learns_about_it() {
    let manager = networked_test_manager("live_history");
    let net = manager.network().expect("network configured");
    let session = net.session().expect("session opens");

    // The robot announces FIRST — before anything is watching.
    let robot = format!("hist{}", std::process::id());
    let token = TopicToken::announce(session, &robot, "/odom").expect("announce declares");
    std::thread::sleep(Duration::from_millis(200));

    // Only now does the watch start.
    let watch = AnnounceWatch::declare(session).expect("watch declares");
    let (found, seen) = drain_until(&watch, DELIVERY_BUDGET, |e| {
        matches!(
            e,
            AnnounceEvent::Alive { robot: r, topic: Some(t) } if r == &robot && t == "/odom"
        )
    });
    assert!(
        found,
        "history replay must deliver the already-live token to a late watch; saw {seen:?}"
    );
    drop(token);
}

/// A BURST of real tokens — the `ros2 attach` shape — all reach the watch. The
/// coalescing that turns them into ONE push is pinned in the e2e; what this pins is
/// that none of them is LOST on the way in.
#[test]
fn every_token_of_a_real_burst_reaches_the_watch() {
    const TOPICS: usize = 30;
    let manager = networked_test_manager("live_burst");
    let net = manager.network().expect("network configured");
    let session = net.session().expect("session opens");
    let watch = AnnounceWatch::declare(session).expect("watch declares");

    let robot = format!("burst{}", std::process::id());
    let mut tokens = Vec::new();
    for i in 0..TOPICS {
        tokens.push(
            TopicToken::announce(session, &robot, &format!("/attach/t{i}"))
                .expect("announce declares"),
        );
    }

    // Collect until every expected topic has been seen (or the budget expires).
    let mut outstanding: std::collections::BTreeSet<String> =
        (0..TOPICS).map(|i| format!("/attach/t{i}")).collect();
    let deadline = Instant::now() + DELIVERY_BUDGET;
    while Instant::now() < deadline && !outstanding.is_empty() {
        if let Some(AnnounceEvent::Alive {
            robot: r,
            topic: Some(t),
        }) = watch.next_event(Duration::from_millis(100))
        {
            if r == robot {
                outstanding.remove(&t);
            }
        }
    }
    assert!(
        outstanding.is_empty(),
        "every token of a {TOPICS}-topic burst must reach the watch; missing {outstanding:?}"
    );
    drop(tokens);
}

/// The PRODUCTION plane (not the raw primitive) declares a real watch over a
/// network-configured manager and delivers — the wiring pin that says netd's own seam
/// is the one that works, not just cerulion_core's.
#[test]
fn the_production_plane_declares_a_working_watch() {
    let manager = networked_test_manager("live_plane");
    // The session is LAZY — the plane opens it.
    assert!(
        !manager.network().expect("has network").is_active(),
        "nothing has opened the session yet"
    );

    let plane = GatewayAnnounceWatchPlane::new(Arc::clone(&manager));
    let mut stream = plane
        .watch()
        .expect("the production plane declares a watch");
    assert!(
        manager.network().expect("has network").is_active(),
        "declaring the watch opens the shared session"
    );

    let session = manager
        .network()
        .expect("has network")
        .session()
        .expect("session");
    let robot = format!("plane{}", std::process::id());
    let token = TopicToken::announce(session, &robot, "/tf").expect("announce declares");

    let deadline = Instant::now() + DELIVERY_BUDGET;
    let mut got = false;
    let mut seen = Vec::new();
    while Instant::now() < deadline && !got {
        if let Some(e) = stream.next_event(Duration::from_millis(100)) {
            got = matches!(
                &e,
                AnnounceEvent::Alive { robot: r, topic: Some(t) } if r == &robot && t == "/tf"
            );
            seen.push(e);
        }
    }
    assert!(
        got,
        "the production plane's stream must deliver a real token; saw {seen:?}"
    );
    drop(token);
}

/// A daemon with NO network plane refuses the watch LOUDLY and explicitly — `NoNetwork`,
/// not a silent success that would leave a consumer waiting forever for a push.
#[test]
fn a_local_only_manager_refuses_the_watch_with_no_network() {
    let manager = local_only_test_manager("live_nonet");
    let plane = GatewayAnnounceWatchPlane::new(manager);
    match plane.watch() {
        Err(WatchError::NoNetwork) => {}
        Err(other) => panic!("expected NoNetwork, got {other}"),
        Ok(_) => panic!("a local-only manager must not produce a watch"),
    }
}

/// Anti-tautology for every arm above: a watch on a quiet announce space reports
/// NOTHING. Without this, an implementation that fabricated events (or reported a
/// timeout as an event) would satisfy the positive arms.
#[test]
fn a_quiet_announce_space_yields_no_event() {
    let manager = networked_test_manager("live_quiet");
    let net = manager.network().expect("network configured");
    let session = net.session().expect("session opens");
    let watch = AnnounceWatch::declare(session).expect("watch declares");

    let deadline = Instant::now() + Duration::from_millis(600);
    let mut seen = Vec::new();
    while Instant::now() < deadline {
        if let Some(e) = watch.next_event(Duration::from_millis(100)) {
            seen.push(e);
        }
    }
    assert!(
        seen.is_empty(),
        "a quiet announce space must yield no event, got {seen:?}"
    );
}
