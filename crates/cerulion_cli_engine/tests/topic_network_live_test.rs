// SPDX-License-Identifier: AGPL-3.0-only
//! LIVE discovery tests for `topic list`'s engine
//! half (`topic_cmd::query_remote_topics`).
//!
//! `query_remote_topics` is inherently live-session-only (it opens a zenoh
//! session and queries the liveliness key-spaces), so these tests open REAL
//! zenoh sessions — following `cerulion_core/tests/network_test.rs`
//! conventions (isolated sessions, multicast/gossip scouting disabled via
//! `scouting: false`). They live in THIS separate integration file, NOT in
//! `src/topic_cmd.rs`'s inline test module, so the engine's unit tests stay
//! transport-free (the pure rendering / normalization / error-arm oracles are
//! inline there).
//!
//! `query_remote_topics` now queries BOTH the DEMAND (`declare`) and
//! ANNOUNCE (`announce`) key-spaces, and the CLI-path scouting-ON default is
//! set at the dispatch site — the options struct keeps `scouting: false` so
//! these tests stay hermetic with explicit `--connect` locators.
//!
//! No iceoryx2 involvement — no `--test-threads=1` requirement, no SHM
//! singleton. The loopback test binds a localhost TCP port (pid-derived
//! candidates, bounded retry on collision).

use cerulion_cli_engine::discovery_ladder::{DiscoveredPeer, DiscoveryRung};
use cerulion_cli_engine::topic_cmd::{
    query_remote_topics, query_remote_topics_with_candidates, RemoteTopicsOptions, RobotProvenance,
    RobotRow,
};
use cerulion_core::transport::discovery::TopicToken;
use cerulion_core::transport::network::{NetworkConfig, NetworkManager};
use std::sync::Mutex;
use std::time::{Duration, Instant};

/// The v1 CLI story end-to-end over loopback TCP: a "robot-side" session
/// LISTENS on a localhost locator and advertises two topics via liveliness
/// tokens; the engine's `query_remote_topics` (the `topic list --connect
/// <locator>` path) opens its own one-shot session, connects,
/// and discovers BOTH — asserted against a hand oracle (sorted canonical
/// names), never a self-compare.
///
/// Discovery propagation over a fresh peer link is not instantaneous, so
/// the query is retried (bounded) — each attempt is a full production
/// one-shot (open session → query → drop), exactly what the CLI runs.
/// The `runs` source every arm in this file hands its query surface.
///
/// These arms are ZENOH-ONLY — `robot_mgr` is a bare `NetworkManager` with no
/// `TransportManager` behind it, so there is no iceoryx2 namespace it could
/// gather runs on, and none of them ever GETs the `runs` verb. The
/// source therefore points at a FRESH ISOLATED namespace: a real config that
/// holds nothing, so a serve would answer a settled empty rather than reaching
/// into whatever the process-global namespace happens to contain. Building the
/// struct literally (rather than through a `Default`) is deliberate — a
/// `RunSource` that defaulted to the global namespace is exactly the variant
/// `runs_serve_iox2_test::the_runs_gather_threads_the_gateway_hosts_own_namespace`
/// exists to kill.
fn runs_source_for_a_zenoh_only_surface() -> cerulion_core::transport::network::RunSource {
    cerulion_core::transport::network::RunSource {
        iox_config: std::sync::Arc::new(cerulion_core::testing::iceoryx_test_config()),
        gather_window: cerulion_core::transport::run_registry::RUN_GATHER_WINDOW,
    }
}

#[test]
fn remote_discovery_over_loopback_finds_advertised_topics() {
    // Pid-derived candidate ports: parallel test binaries on one box get
    // distinct candidates; a genuinely occupied port fails the listen-side
    // session open and the next candidate is tried.
    let pid = std::process::id();
    let candidates = [
        17400 + (pid % 500) as u16,
        18000 + (pid % 500) as u16,
        18600 + (pid % 500) as u16,
    ];

    let mut robot_side = None;
    for port in candidates {
        let mgr = NetworkManager::new(NetworkConfig {
            listen_endpoints: vec![format!("tcp/127.0.0.1:{port}")],
            ..NetworkConfig::default()
        });
        if mgr.session().is_ok() {
            robot_side = Some((mgr, port));
            break;
        }
    }
    let (robot_mgr, port) = robot_side.expect(
        "no candidate localhost port could be bound for the listen-side \
         session — all three pid-derived candidates in use?",
    );
    let session = robot_mgr.session().expect("listen-side session (cached)");

    // Advertise two topics (tokens must stay alive through the query).
    let _cloud = TopicToken::declare(session, "/go2/utlidar/cloud").expect("declare cloud token");
    let _imu = TopicToken::declare(session, "/go2/imu/state").expect("declare imu token");

    // Hand oracle: normalize_remote_topics sorts, so the expected order is
    // lexicographic regardless of reply order.
    let expected = vec![
        "/go2/imu/state".to_string(),
        "/go2/utlidar/cloud".to_string(),
    ];

    let opts = RemoteTopicsOptions {
        connect: vec![format!("tcp/127.0.0.1:{port}")],
        listen: vec![],
        scouting: false,
    };
    let mut last = Vec::new();
    for _ in 0..30 {
        let disc = query_remote_topics(&opts).expect("one-shot discovery session must open");
        // Scouting is OFF, so the discovery ladder never runs — no
        // ladder peers on a hermetic, explicit-locator session.
        assert!(
            disc.peers.is_empty(),
            "scouting-off discovery must not run the ladder, got peers: {:?}",
            disc.peers
        );
        last = disc.topics;
        if last == expected {
            return;
        }
        std::thread::sleep(Duration::from_millis(100));
    }
    panic!(
        "remote discovery never converged to the oracle within the retry \
         budget — expected {expected:?}, last observed {last:?}"
    );
}

/// An ANNOUNCE token (egress-presence — declared by a producer /
/// gateway) shows up in `topic list`'s merged output too. A robot-side session
/// LISTENS and `announce`s a produced topic; the engine's `query_remote_topics`
/// (which now queries BOTH the demand AND announce key-spaces) discovers it —
/// hand oracle, never a self-compare.
#[test]
fn remote_discovery_finds_announced_topics() {
    let pid = std::process::id();
    let candidates = [
        19100 + (pid % 400) as u16,
        19600 + (pid % 400) as u16,
        20100 + (pid % 400) as u16,
    ];

    let mut robot_side = None;
    for port in candidates {
        let mgr = NetworkManager::new(NetworkConfig {
            listen_endpoints: vec![format!("tcp/127.0.0.1:{port}")],
            ..NetworkConfig::default()
        });
        if mgr.session().is_ok() {
            robot_side = Some((mgr, port));
            break;
        }
    }
    let (robot_mgr, port) = robot_side.expect(
        "no candidate localhost port could be bound for the announce-side \
         session — all three pid-derived candidates in use?",
    );
    let session = robot_mgr.session().expect("announce-side session (cached)");

    // Advertise a produced topic via an ANNOUNCE token (must outlive the query).
    // Announce keys carry the producing robot as their first chunk —
    // the query side strips it back off for the REMOTE TOPICS list.
    let _cam = TopicToken::announce(session, "go2", "/go2/camera/image").expect("announce token");

    let expected = vec!["/go2/camera/image".to_string()];
    let opts = RemoteTopicsOptions {
        connect: vec![format!("tcp/127.0.0.1:{port}")],
        listen: vec![],
        scouting: false,
    };
    let mut last = Vec::new();
    for _ in 0..30 {
        let disc = query_remote_topics(&opts).expect("one-shot discovery session must open");
        assert!(
            disc.peers.is_empty(),
            "scouting-off discovery must not run the ladder, got peers: {:?}",
            disc.peers
        );
        last = disc.topics;
        if last == expected {
            return;
        }
        std::thread::sleep(Duration::from_millis(100));
    }
    panic!(
        "announce discovery never converged to the oracle — expected {expected:?}, \
         last observed {last:?}"
    );
}

/// The CATALOG verb over loopback — the RICHER data path. A robot-side
/// session LISTENS, announces one topic (presence + robot-identity harvest), and
/// declares the `cerulion_q/{robot}/**` QUERY SURFACE serving a catalog that
/// ALSO contains a CATALOG-ONLY topic (never announced). `query_remote_topics`
/// GETs each discovered robot's catalog and folds its topics in — so the
/// catalog-only topic surfaces ONLY via the catalog GET (the announce listing
/// alone could never show it). Hand oracle, never a self-compare.
///
/// The FALLBACK direction (a robot with NO query surface → catalog GET returns
/// nothing → the announce listing carries the run, no hang/crash) is covered by
/// `remote_discovery_finds_announced_topics` /
/// `remote_discovery_over_loopback_finds_advertised_topics` above — they declare
/// no query surface yet still converge (the catalog GET is best-effort).
#[test]
fn remote_discovery_catalog_get_enriches_with_catalog_only_topics() {
    use cerulion_core::transport::bridge::TopicBridgeManager;
    use cerulion_core::transport::network::{CatalogSource, GetDemandState, SchemaSource};
    use std::collections::{HashMap, HashSet};
    use std::sync::{Arc, Mutex};

    let pid = std::process::id();
    let candidates = [
        20600 + (pid % 300) as u16,
        21000 + (pid % 300) as u16,
        21400 + (pid % 300) as u16,
    ];
    let robot = "cat";
    let mut robot_side = None;
    for port in candidates {
        let mgr = NetworkManager::new(NetworkConfig {
            listen_endpoints: vec![format!("tcp/127.0.0.1:{port}")],
            robot_identity: Some(robot.to_string()),
            ..NetworkConfig::default()
        });
        if mgr.session().is_ok() {
            robot_side = Some((mgr, port));
            break;
        }
    }
    let (robot_mgr, port) = robot_side
        .expect("no candidate localhost port could be bound for the catalog-side session");
    let session = robot_mgr.session().expect("catalog-side session (cached)");

    // Announce ONE topic (presence + robot-identity harvest — drives the catalog
    // GET on the observer). The announce robot chunk MUST match the query
    // surface's robot identity so the observer's explicit catalog selector routes.
    let _ann = TopicToken::announce(session, robot, "/announced").expect("announce token");

    // Build the catalog source: both the announced topic AND a CATALOG-ONLY topic
    // (never announced) are registered on the bridge manager (the catalog's topic
    // source of truth); the catalog-only topic can reach the observer ONLY via the
    // catalog GET.
    let bridge = Arc::new(TopicBridgeManager::new());
    bridge
        .register_topic("/announced")
        .expect("register announced topic");
    bridge
        .register_topic("/catalog_only")
        .expect("register catalog-only topic");
    let hashes = Arc::new(Mutex::new(HashMap::from([(
        "/announced".to_string(),
        0xABCD_u64,
    )])));
    let catalog = CatalogSource {
        hashes,
        boot_topics: Arc::new(HashSet::new()),
        topic_schemas: Arc::new(Mutex::new(HashMap::new())),
        hash_names: Arc::new(Mutex::new(HashMap::new())),
        producer_probe: None,
        liveness: None,
        liveness_clock: None,
    };
    robot_mgr
        .start_query_surface(
            Arc::clone(&bridge),
            Arc::new(GetDemandState::default()),
            catalog,
            SchemaSource::default(),
            runs_source_for_a_zenoh_only_surface(),
        )
        .expect("declare query surface serving the catalog");

    // Oracle (sorted canonical): the announced topic surfaces (via announce AND
    // catalog, deduped), the catalog-only topic surfaces ONLY via the catalog GET.
    let expected = vec!["/announced".to_string(), "/catalog_only".to_string()];
    let opts = RemoteTopicsOptions {
        connect: vec![format!("tcp/127.0.0.1:{port}")],
        listen: vec![],
        scouting: false,
    };
    let mut last = Vec::new();
    for _ in 0..30 {
        let disc = query_remote_topics(&opts).expect("one-shot discovery session must open");
        last = disc.topics;
        // The catalog-only topic is present ONLY because the catalog GET ran +
        // decoded — the announce space never carried it.
        if last == expected {
            return;
        }
        std::thread::sleep(Duration::from_millis(100));
    }
    panic!(
        "catalog enrichment never converged to the oracle — expected {expected:?}, \
         last observed {last:?} (the catalog-only topic proves the catalog GET path)"
    );
}

/// The catalog GET is UNION-with-announce, NOT replace — an announce-only
/// topic SURVIVES the enrichment. The robot announces BOTH `/uni/a` and `/uni/b`
/// but its query-surface catalog lists ONLY `/uni/a`; the final topic list MUST
/// contain BOTH. A union keeps `/uni/b` (announce-only); a "use the catalog
/// exclusively once it decodes" replace would DROP `/uni/b`. This pins the
/// fallback-safety the design promises (announce-only topics are never lost to a
/// leaner catalog). Distinct from the enrichment test above (whose catalog was a
/// SUPERSET of announce, so union and replace pass the same oracle).
#[test]
fn remote_discovery_catalog_union_preserves_announce_only_topics() {
    use cerulion_core::transport::bridge::TopicBridgeManager;
    use cerulion_core::transport::network::{CatalogSource, GetDemandState, SchemaSource};
    use std::collections::{HashMap, HashSet};
    use std::sync::{Arc, Mutex};

    let pid = std::process::id();
    let candidates = [
        21800 + (pid % 300) as u16,
        22200 + (pid % 300) as u16,
        22600 + (pid % 300) as u16,
    ];
    let robot = "union";
    let mut robot_side = None;
    for port in candidates {
        let mgr = NetworkManager::new(NetworkConfig {
            listen_endpoints: vec![format!("tcp/127.0.0.1:{port}")],
            robot_identity: Some(robot.to_string()),
            ..NetworkConfig::default()
        });
        if mgr.session().is_ok() {
            robot_side = Some((mgr, port));
            break;
        }
    }
    let (robot_mgr, port) =
        robot_side.expect("no candidate localhost port could be bound for the union-side session");
    let session = robot_mgr.session().expect("union-side session (cached)");

    // Announce BOTH topics (the presence space carries /uni/a AND /uni/b).
    let _ann_a = TopicToken::announce(session, robot, "/uni/a").expect("announce /uni/a");
    let _ann_b = TopicToken::announce(session, robot, "/uni/b").expect("announce /uni/b");

    // Catalog lists ONLY /uni/a — a STRICT SUBSET of announce (register only /uni/a
    // on the bridge, which is the catalog's topic source). /uni/b is announce-only.
    let bridge = Arc::new(TopicBridgeManager::new());
    bridge.register_topic("/uni/a").expect("register /uni/a");
    let catalog = CatalogSource {
        hashes: Arc::new(Mutex::new(HashMap::new())),
        boot_topics: Arc::new(HashSet::new()),
        topic_schemas: Arc::new(Mutex::new(HashMap::new())),
        hash_names: Arc::new(Mutex::new(HashMap::new())),
        producer_probe: None,
        liveness: None,
        liveness_clock: None,
    };
    robot_mgr
        .start_query_surface(
            Arc::clone(&bridge),
            Arc::new(GetDemandState::default()),
            catalog,
            SchemaSource::default(),
            runs_source_for_a_zenoh_only_surface(),
        )
        .expect("declare query surface (catalog lists only /uni/a)");

    // Oracle: BOTH topics present — /uni/a (announce + catalog) AND /uni/b
    // (announce ONLY). A replace-regression drops /uni/b; the union keeps it.
    let expected = vec!["/uni/a".to_string(), "/uni/b".to_string()];
    let opts = RemoteTopicsOptions {
        connect: vec![format!("tcp/127.0.0.1:{port}")],
        listen: vec![],
        scouting: false,
    };
    let mut last = Vec::new();
    for _ in 0..30 {
        let disc = query_remote_topics(&opts).expect("one-shot discovery session must open");
        last = disc.topics;
        if last == expected {
            return;
        }
        std::thread::sleep(Duration::from_millis(100));
    }
    panic!(
        "catalog union never converged to the oracle — expected {expected:?}, last observed \
         {last:?} (announce-only /uni/b must survive the catalog enrichment — a replace would drop it)"
    );
}

/// The isolated arm is `Ok(empty)`, NOT an error: with no locators and
/// scouting off the session opens fine and simply discovers nothing (the
/// CLI then renders the --connect hint). Mirrors
/// `network_test.rs::test_query_live_topics_empty_network` through the
/// engine seam. Pins the contract that an empty list from
/// `query_remote_topics` always means "session opened, nothing advertised"
/// — failures are loud `Err`s, never a silent empty.
#[test]
fn endpoint_less_discovery_opens_isolated_and_returns_empty() {
    let opts = RemoteTopicsOptions {
        connect: vec![],
        listen: vec![],
        scouting: false,
    };
    let disc = query_remote_topics(&opts)
        .expect("an endpoint-less (isolated) discovery session must open cleanly");
    assert!(
        disc.topics.is_empty(),
        "an isolated scouting-off session can discover no topics, got: {:?}",
        disc.topics
    );
    assert!(
        disc.robots.is_empty(),
        "an isolated scouting-off session can surface no robot rows, got: {:?}",
        disc.robots
    );
    assert!(
        disc.peers.is_empty(),
        "scouting off ⇒ the discovery ladder never runs, got peers: {:?}",
        disc.peers
    );
}

/// An ISOLATED transient resolve (no peer, no locators,
/// scouting off) must report `DiscoveryNotConverged`, NOT `NoProducer`.
///
/// This is the CALL-SITE pin for `classify_transient_miss`. `NoProducer` routes to
/// "topic 'X' not found locally or on any discovered robot", i.e. an assertion about
/// the LAN, and here we read NOBODY's catalog: there is no basis for it. The
/// earlier code returned exactly that, silently, on every failure of this path —
/// including a session that would not open — which is the landing zone for every
/// `cerulion-netd` failure.
///
/// The ANTI-TAUTOLOGY control lives in
/// `resolve_remote_ingress_target_finds_robot_schema_hash_and_walker`: with a REAL
/// reachable robot whose catalog is read and does NOT carry the topic, the same call
/// still resolves to `NoProducer` (that absence claim is correct and must survive).
/// Together they pin both arms of the rule "only claim absence about a catalog you
/// actually read".
#[test]
fn an_isolated_transient_resolve_is_not_converged_never_a_false_absence() {
    use cerulion_cli_engine::topic_cmd::{resolve_remote_ingress_target_with_opts, RemoteResolve};

    // No connect locators + scouting OFF ⇒ the netd query plane is bypassed
    // (`use_netd_query_plane` requires scouting) and the transient session opens
    // isolated, so nothing can answer.
    let resolved = resolve_remote_ingress_target_with_opts("/nobody", &[], &[], false, None);
    match resolved {
        RemoteResolve::DiscoveryNotConverged => {}
        RemoteResolve::NoProducer => panic!(
            "an isolated resolve read NO robot's catalog — reporting NoProducer makes the CLI \
             claim the topic is absent from every discovered robot, which is exactly the \
             false-absence bug"
        ),
        RemoteResolve::Found(_) => panic!("nothing can be found on an isolated session"),
        RemoteResolve::SchemaUnavailable { robot, cause } => {
            panic!("unexpected SchemaUnavailable from an isolated session: {robot} / {cause}")
        }
        // This resolve carries no cancellation source, so an interruption
        // cannot arise. Named explicitly rather than caught by a wildcard.
        RemoteResolve::Cancelled => panic!("no cancellation source was supplied"),
    }
}

/// An ANNOUNCE-presence ROBOTS row over loopback — presence IS the
/// verification (the removed `cerulion_meta` beacon's replacement). A "robot"
/// gateway session with `robot_identity = "e2e"` LISTENS and announces
/// two ABSOLUTE MIRROR topics (`/lf/lowstate` + `/utlidar/cloud` — the
/// flagship `ros2 attach` shape, whose leading segments carry NO robot
/// identity); a scouting-off observer runs the engine's
/// `query_remote_topics`, whose ANNOUNCE gather groups by the keys' EXACT
/// robot chunk and surfaces a single `RobotRow` for `e2e` with
/// `topic_count == 2` — and NO phantom "lf"/"utlidar" rows (a
/// first-segment heuristic would mint those). Asserted against a hand oracle,
/// never a self-compare. After the announcer drops (its liveliness tokens
/// undeclare + the loopback link severs), a re-query shows the row GONE.
///
/// Teardown scope (mirroring the earlier arm): dropping the robot
/// manager BOTH undeclares the announce tokens AND closes the loopback session,
/// so this pins those two effects COLLECTIVELY (no live-session token-leak is
/// distinguishable here — that is pinned by `NetworkManager`'s own `Drop`).
#[test]
fn announce_presence_surfaces_a_robots_row_over_loopback() {
    // Pid-derived candidate ports (distinct from the topic tests' ranges) so
    // parallel test binaries on one box do not collide.
    let pid = std::process::id();
    let candidates = [
        20600 + (pid % 300) as u16,
        21000 + (pid % 300) as u16,
        21400 + (pid % 300) as u16,
    ];

    // Robot side: LISTEN on a localhost locator with the robot identity set,
    // then announce two ABSOLUTE mirror topics (the presence evidence).
    let mut robot_side = None;
    for port in candidates {
        let mgr = NetworkManager::new(NetworkConfig {
            listen_endpoints: vec![format!("tcp/127.0.0.1:{port}")],
            robot_identity: Some("e2e".to_string()),
            ..NetworkConfig::default()
        });
        if mgr.session().is_ok() {
            robot_side = Some((mgr, port));
            break;
        }
    }
    let (robot_mgr, port) = robot_side.expect(
        "no candidate localhost port could be bound for the announce-presence \
         robot session — all three pid-derived candidates in use?",
    );
    robot_mgr
        .announce_egress_topic("/lf/lowstate")
        .expect("announce absolute mirror topic /lf/lowstate");
    robot_mgr
        .announce_egress_topic("/utlidar/cloud")
        .expect("announce absolute mirror topic /utlidar/cloud");

    // Observer side: the engine's own remote-discovery path (scouting off →
    // reaches exactly the named locator, ladder never runs).
    let opts = RemoteTopicsOptions {
        connect: vec![format!("tcp/127.0.0.1:{port}")],
        listen: vec![],
        scouting: false,
    };
    // Hand oracle: ONE announce-presence row for `e2e` (the announce
    // keys' robot chunk), two DISTINCT topics, no locator (no mDNS on the
    // hermetic path), `(announce)` provenance — and the topics list carries
    // the canonical names with the robot chunk STRIPPED.
    let expected_row = RobotRow {
        robot: "e2e".to_string(),
        locator: None,
        provenance: RobotProvenance::Announce,
        topic_count: 2,
    };
    let expected_topics = vec!["/lf/lowstate".to_string(), "/utlidar/cloud".to_string()];
    let mut converged = false;
    for _ in 0..30 {
        let disc = query_remote_topics(&opts).expect("one-shot discovery session must open");
        // scouting off ⇒ the ladder never runs (no mDNS peers).
        assert!(
            disc.peers.is_empty(),
            "scouting-off must not run the ladder"
        );
        // The phantom-row pin holds on EVERY iteration, converged or not: no
        // topic segment may ever mint a robot row.
        for phantom in ["lf", "utlidar"] {
            assert!(
                !disc.robots.iter().any(|r| r.robot == phantom),
                "phantom robot row '{phantom}' minted from a topic segment: {:?}",
                disc.robots
            );
        }
        if disc.robots == vec![expected_row.clone()] && disc.topics == expected_topics {
            converged = true;
            break;
        }
        std::thread::sleep(Duration::from_millis(100));
    }
    assert!(
        converged,
        "the observer never surfaced the announce-presence ROBOTS row (expected {expected_row:?})"
    );

    // Teardown arm: dropping the robot manager undeclares the announce tokens AND
    // severs the link, so a fresh bounded re-query finds the ROBOTS row GONE.
    drop(robot_mgr);
    let mut gone = false;
    for _ in 0..30 {
        let disc = query_remote_topics(&opts).expect("re-query must open");
        if disc.robots.is_empty() {
            gone = true;
            break;
        }
        std::thread::sleep(Duration::from_millis(100));
    }
    assert!(
        gone,
        "the ROBOTS row must vanish once the announcing robot drops \
         (token-undeclare AND session-close, covered collectively)"
    );
}

/// A ZERO-EGRESS gateway (bare identity token only — the ingress-only
/// robot shape) still surfaces a ROBOTS row: a robot session with
/// `robot_identity = "bare"` declares ONLY `announce_gateway_identity()`
/// (no topics); the observer's gather surfaces exactly one zero-topic
/// `(announce)` row and an EMPTY topics list — hand oracle.
#[test]
fn bare_identity_token_alone_surfaces_a_zero_topic_row() {
    let pid = std::process::id();
    let candidates = [
        23200 + (pid % 300) as u16,
        23600 + (pid % 300) as u16,
        24000 + (pid % 300) as u16,
    ];
    let mut robot_side = None;
    for port in candidates {
        let mgr = NetworkManager::new(NetworkConfig {
            listen_endpoints: vec![format!("tcp/127.0.0.1:{port}")],
            robot_identity: Some("bare".to_string()),
            ..NetworkConfig::default()
        });
        if mgr.session().is_ok() {
            robot_side = Some((mgr, port));
            break;
        }
    }
    let (robot_mgr, port) =
        robot_side.expect("bind a loopback listener for the bare-identity test");
    robot_mgr
        .announce_gateway_identity()
        .expect("declare the bare identity token");
    assert!(
        robot_mgr.has_identity_token(),
        "the robot side must report a live identity token after declaring"
    );

    let opts = RemoteTopicsOptions {
        connect: vec![format!("tcp/127.0.0.1:{port}")],
        listen: vec![],
        scouting: false,
    };
    let expected_row = RobotRow {
        robot: "bare".to_string(),
        locator: None,
        provenance: RobotProvenance::Announce,
        topic_count: 0,
    };
    let mut converged = false;
    for _ in 0..30 {
        let disc = query_remote_topics(&opts).expect("one-shot discovery session must open");
        if disc.robots == vec![expected_row.clone()] && disc.topics.is_empty() {
            converged = true;
            break;
        }
        std::thread::sleep(Duration::from_millis(100));
    }
    assert!(
        converged,
        "a zero-egress gateway's bare identity token must surface a zero-topic \
         ROBOTS row (expected {expected_row:?})"
    );
}

/// The black-hole-locator regression pin (STRUCTURAL): a discovery query whose
/// EXPLICIT connect list includes a black-hole locator (RFC 5737 TEST-NET-1
/// `192.0.2.1:7683` — unroutable, will never answer a TCP connect) ALONGSIDE a
/// real loopback announcer finds the REAL robot regardless of network class.
///
/// This is network-INDEPENDENT: the pre-filter probes
/// the dead explicit locator at the 1 s connect bound and DROPS it on EVERY
/// network class — a fast-fail (ENETUNREACH → dropped instantly) AND a
/// SYN-drop that silently swallows the SYN (dropped after the 1 s probe) — so
/// the open only ever connects to the reachable loopback robot. Without the
/// pre-filter, a SYN-drop route lets the dead EXPLICIT locator consume
/// the whole sequential 1 s connect budget and FAIL `zenoh::open`, losing the
/// gather; with it, the locator cannot reach the open at all. Asserts OUTCOME (the robot is
/// found and the run proceeded) + a per-call wall bound, not the mechanism.
#[test]
fn bounded_connect_never_stalls_on_a_black_hole_locator() {
    let pid = std::process::id();
    let candidates = [
        22000 + (pid % 300) as u16,
        22400 + (pid % 300) as u16,
        22800 + (pid % 300) as u16,
    ];
    let mut robot_side = None;
    for port in candidates {
        let mgr = NetworkManager::new(NetworkConfig {
            listen_endpoints: vec![format!("tcp/127.0.0.1:{port}")],
            robot_identity: Some("bh".to_string()),
            ..NetworkConfig::default()
        });
        if mgr.session().is_ok() {
            robot_side = Some((mgr, port));
            break;
        }
    }
    let (robot_mgr, port) = robot_side.expect("bind a loopback listener for the black-hole test");
    robot_mgr
        .announce_egress_topic("/bh/topic")
        .expect("announce the real robot's topic");

    let opts = RemoteTopicsOptions {
        // The dead TEST-NET-1 locator first, then the real loopback listener.
        // The dead one is DROPPED by the pre-filter, so it never reaches the open.
        connect: vec![
            "tcp/192.0.2.1:7683".to_string(),
            format!("tcp/127.0.0.1:{port}"),
        ],
        listen: vec![],
        scouting: false,
    };
    // The pre-filter probes the dead explicit locator at the 1 s bound (a
    // SYN-drop route makes that probe take ~1 s), then the open connects only to
    // the reachable loopback robot — so every one-shot query must complete in
    // WELL under 5 s wall AND succeed (never Err, since the poisoning locator is
    // gone before the open). A reverted policy (fold-the-dead-locator-anyway)
    // fails `zenoh::open` on a SYN-drop route and panics the `.expect` below.
    let mut found = false;
    for _ in 0..10 {
        let call_start = Instant::now();
        let disc = query_remote_topics(&opts).expect(
            "the discovery session must open (the dead explicit locator is dropped by the \
             pre-filter, never folded into the open)",
        );
        let call_elapsed = call_start.elapsed();
        assert!(
            call_elapsed < Duration::from_secs(5),
            "one discovery query took {call_elapsed:?} — the black-hole locator was not \
             dropped before the bounded connect (the black-hole-locator regression)"
        );
        if disc.topics == vec!["/bh/topic".to_string()] {
            found = true;
            break;
        }
        std::thread::sleep(Duration::from_millis(100));
    }
    assert!(
        found,
        "the real robot must be discovered despite a black-hole explicit locator \
         (the pre-filter drops the dead one on every network class)"
    );
}

/// A TCP TARPIT folded as a LADDER
/// candidate must NOT sink the gather. A raw `TcpListener` on loopback ACCEPTS
/// connections (the kernel completes the TCP handshake into the backlog) but
/// never speaks zenoh — the exact `--scan` open-port false positive that
/// PASSES the pre-filter's TCP probe yet HANGS the zenoh handshake, making the
/// bounded open FAIL at the 1 s bound. Injected as a ladder candidate (via the
/// `query_remote_topics_with_candidates` seam) alongside a real loopback robot
/// given as an explicit `--connect`, the discovery must still find the real
/// robot — via the pre-filter (if the tarpit is dropped) OR the open-failure
/// fallback (retry with explicit locators only) — in bounded wall (< 5 s).
/// Asserts on OUTCOME, not mechanism.
#[test]
fn tarpit_ladder_candidate_does_not_sink_the_gather() {
    let pid = std::process::id();
    let candidates = [
        24400 + (pid % 300) as u16,
        24800 + (pid % 300) as u16,
        25200 + (pid % 300) as u16,
    ];

    // The REAL robot: LISTEN + announce a topic (an explicit --connect locator).
    let mut robot_side = None;
    for port in candidates {
        let mgr = NetworkManager::new(NetworkConfig {
            listen_endpoints: vec![format!("tcp/127.0.0.1:{port}")],
            robot_identity: Some("tarpit".to_string()),
            ..NetworkConfig::default()
        });
        if mgr.session().is_ok() {
            robot_side = Some((mgr, port));
            break;
        }
    }
    let (robot_mgr, robot_port) =
        robot_side.expect("bind a loopback listener for the tarpit test's real robot");
    robot_mgr
        .announce_egress_topic("/tarpit/topic")
        .expect("announce the real robot's topic");
    let robot_locator = format!("tcp/127.0.0.1:{robot_port}");
    let expected = vec!["/tarpit/topic".to_string()];

    // The TARPIT: a bound-but-NOT-accepted TcpListener. A connect (probe or
    // zenoh) completes the TCP handshake into the kernel backlog, but nothing
    // ever reads the socket, so zenoh's transport handshake never completes.
    // Held for the whole test so the port stays bound.
    let tarpit = std::net::TcpListener::bind("127.0.0.1:0").expect("bind the tarpit listener");
    let tarpit_port = tarpit.local_addr().expect("tarpit addr").port();
    let tarpit_peer = DiscoveredPeer {
        robot: "tarpit".to_string(),
        locator: format!("tcp/127.0.0.1:{tarpit_port}"),
        rung: DiscoveryRung::Scan,
    };

    let opts = RemoteTopicsOptions {
        connect: vec![robot_locator],
        listen: vec![],
        scouting: false,
    };

    // Warm-up (untimed): confirm the real robot is discoverable via the plain
    // explicit path first (settles the loopback link + liveliness state), so the
    // timed tarpit arm is not also paying first-contact propagation latency.
    let mut warmed = false;
    for _ in 0..30 {
        if query_remote_topics(&opts)
            .map(|d| d.topics == expected)
            .unwrap_or(false)
        {
            warmed = true;
            break;
        }
        std::thread::sleep(Duration::from_millis(100));
    }
    assert!(
        warmed,
        "the real robot must be discoverable before the tarpit arm"
    );

    // Timed tarpit arm: the tarpit is a LADDER candidate. Each poisoned open
    // costs ~1 s before the fallback, so bound the retry loop tightly (a warm
    // link makes the fallback gather find the robot within a call or two) and
    // assert the total stays < 5 s.
    let start = Instant::now();
    let mut found = false;
    while start.elapsed() < Duration::from_secs(3) {
        let disc = query_remote_topics_with_candidates(&opts, vec![tarpit_peer.clone()])
            .expect("the gather must survive a tarpit ladder candidate (pre-filter or fallback)");
        if disc.topics == expected {
            found = true;
            break;
        }
        std::thread::sleep(Duration::from_millis(50));
    }
    let elapsed = start.elapsed();
    assert!(
        found && elapsed < Duration::from_secs(5),
        "a tarpit ladder candidate must not sink the gather — found={found}, wall={elapsed:?}"
    );

    drop(tarpit);
}

/// HEADLINE: a robot serves a CUSTOM type (NEVER in
/// `BUILTIN_MSGS`) over the `schema` verb; a desk with ZERO local knowledge
/// fetches its `.msg` closure, seeds a `FrameWalker` IN MEMORY, and decodes a
/// REAL frame of that type BYTE-CORRECTLY against a HAND oracle. Also pins the
/// nested-custom CLOSURE (a parent type nesting a child comes back with BOTH
/// docs) and the explicit NOT-FOUND path (a type the robot lacks → empty docs +
/// structured error). Real zenoh sessions, no iceoryx2, scouting off.
#[test]
fn schema_verb_serves_custom_type_and_desk_decodes_a_real_frame() {
    use cerulion_cli_engine::topic_cmd::seed_framewalker;
    use cerulion_core::codegen::parse_rosmsg;
    use cerulion_core::transport::bridge::TopicBridgeManager;
    use cerulion_core::transport::discovery::query_robot_schema;
    use cerulion_core::transport::network::{GetDemandState, SchemaSource};
    use cerulion_core::wire::WireHeader;
    use cerulion_core::{SchemaDoc, SchemaEncoding};
    use std::collections::BTreeMap;
    use std::sync::Arc;

    // A CUSTOM package no desk has compiled. `Widget` is FIXED-only (so a frame
    // is hand-buildable) and nests `Gadget` (also custom) — the closure member.
    let widget_text = "float64 x\nfloat64 y\n";
    let gadget_text = "int32 count\n";
    let widget_q = "probe_msgs/Widget";
    let gadget_q = "probe_msgs/Gadget";

    let pid = std::process::id();
    let candidates = [
        23000 + (pid % 300) as u16,
        23400 + (pid % 300) as u16,
        23800 + (pid % 300) as u16,
    ];
    let robot = "sch";
    let mut robot_side = None;
    for port in candidates {
        let mgr = NetworkManager::new(NetworkConfig {
            listen_endpoints: vec![format!("tcp/127.0.0.1:{port}")],
            robot_identity: Some(robot.to_string()),
            ..NetworkConfig::default()
        });
        if mgr.session().is_ok() {
            robot_side = Some((mgr, port));
            break;
        }
    }
    let (robot_mgr, port) =
        robot_side.expect("no candidate localhost port could be bound for the schema-side session");

    // The robot serves TWO custom docs; Widget's deps name Gadget (the closure).
    let schema_docs = [
        SchemaDoc {
            qualified: widget_q.to_string(),
            encoding: SchemaEncoding::Msg,
            text: widget_text.to_string(),
            deps: vec![gadget_q.to_string()],
        },
        SchemaDoc {
            qualified: gadget_q.to_string(),
            encoding: SchemaEncoding::Msg,
            text: gadget_text.to_string(),
            deps: vec![],
        },
    ];
    let schema = SchemaSource {
        docs: Arc::new(Mutex::new(
            schema_docs
                .iter()
                .map(|d| (d.qualified.clone(), d.clone()))
                .collect::<BTreeMap<_, _>>(),
        )),
    };
    // A bridge with one registered topic so the query surface starts (non-empty).
    let bridge = Arc::new(TopicBridgeManager::new());
    bridge.register_topic("/widget").expect("register");
    robot_mgr
        .start_query_surface(
            Arc::clone(&bridge),
            Arc::new(GetDemandState::default()),
            cerulion_core::transport::network::CatalogSource {
                hashes: Arc::new(std::sync::Mutex::new(std::collections::HashMap::new())),
                boot_topics: Arc::new(std::collections::HashSet::new()),
                topic_schemas: Arc::new(std::sync::Mutex::new(std::collections::HashMap::new())),
                hash_names: Arc::new(std::sync::Mutex::new(std::collections::HashMap::new())),
                producer_probe: None,
                liveness: None,
                liveness_clock: None,
            },
            schema,
            runs_source_for_a_zenoh_only_surface(),
        )
        .expect("declare query surface serving the schema");

    // Desk side: a raw session connecting to the robot (scouting off — hermetic).
    let desk = NetworkManager::new(NetworkConfig {
        connect_endpoints: vec![format!("tcp/127.0.0.1:{port}")],
        multicast_scouting: false,
        gossip_scouting: false,
        ..NetworkConfig::default()
    });
    let session = desk.session().expect("desk session");

    // HAND oracle frame: Widget { x: 3.5, y: -7.25 } — a fixed-only frame is the
    // 32-byte header (carrying the recipe-3 hash) + [x_le][y_le]. Compute the hash
    // the SAME way the walker does, so `walk_by_hash` resolves it.
    let widget_schema = parse_rosmsg(widget_text, "Widget", Some("probe_msgs")).expect("parse");
    let widget_hash = widget_schema.schema_hash();
    let (x_oracle, y_oracle) = (3.5f64, -7.25f64);
    let mut frame = vec![0u8; WireHeader::SIZE];
    WireHeader::with_schema(widget_hash).write_to_buf(&mut frame);
    frame.extend_from_slice(&x_oracle.to_le_bytes());
    frame.extend_from_slice(&y_oracle.to_le_bytes());

    // Converge (the connect + queryable route settle asynchronously).
    let mut decoded = None;
    for _ in 0..40 {
        if let Some(reply) = query_robot_schema(
            session,
            robot,
            "probe_msgs/Widget",
            Duration::from_millis(300),
        ) {
            // Closure: BOTH Widget (requested, first) AND Gadget (nested) served.
            let names: Vec<&str> = reply.docs.iter().map(|d| d.qualified.as_str()).collect();
            assert_eq!(
                names,
                vec![widget_q, gadget_q],
                "the schema reply must carry the requested type first, then its custom closure"
            );
            // Seed a FrameWalker from the fetched texts (+ the desk's builtin
            // corpus) and decode the REAL frame BYTE-CORRECTLY vs the hand oracle.
            let walker = seed_framewalker(&reply.docs);
            let fv = walker
                .walk_by_hash(&frame)
                .expect("walk the custom-type frame by hash");
            assert_eq!(fv.schema_name, widget_q);
            let x = match fv.field("x") {
                Some(cerulion_core::codegen::FrameValueKind::F64(v)) => *v,
                other => panic!("field x is not an F64: {other:?}"),
            };
            let y = match fv.field("y") {
                Some(cerulion_core::codegen::FrameValueKind::F64(v)) => *v,
                other => panic!("field y is not an F64: {other:?}"),
            };
            assert_eq!(
                (x, y),
                (x_oracle, y_oracle),
                "decoded values must equal the hand oracle"
            );
            decoded = Some(reply);
            break;
        }
        std::thread::sleep(Duration::from_millis(100));
    }
    assert!(
        decoded.is_some(),
        "the desk never fetched + decoded the custom Widget frame from the robot"
    );

    // Explicit NOT-FOUND: a type the robot does NOT have → answered reply with
    // empty docs + a structured error (never silence).
    let mut saw_not_found = false;
    for _ in 0..20 {
        if let Some(reply) = query_robot_schema(
            session,
            robot,
            "probe_msgs/DoesNotExist",
            Duration::from_millis(300),
        ) {
            assert!(reply.docs.is_empty(), "an unknown type must serve no docs");
            assert!(
                reply.error.is_some(),
                "an unknown type must carry a structured error"
            );
            saw_not_found = true;
            break;
        }
        std::thread::sleep(Duration::from_millis(100));
    }
    assert!(
        saw_not_found,
        "the robot never returned the explicit NOT-FOUND reply"
    );
}

/// The CATALOG carries the qualified schema NAME per topic (the
/// desk's map from a topic to WHICH type to `schema`-fetch). A robot's
/// `CatalogSource.topic_schemas` binding surfaces on the wire as
/// `CatalogEntry.schema_name`. Real zenoh sessions, scouting off.
#[test]
fn catalog_verb_carries_per_topic_schema_name() {
    use cerulion_core::transport::bridge::TopicBridgeManager;
    use cerulion_core::transport::discovery::query_robot_catalog;
    use cerulion_core::transport::network::{CatalogSource, GetDemandState, SchemaSource};
    use std::collections::{HashMap, HashSet};
    use std::sync::{Arc, Mutex};

    let pid = std::process::id();
    let candidates = [
        24200 + (pid % 300) as u16,
        24600 + (pid % 300) as u16,
        25000 + (pid % 300) as u16,
    ];
    let robot = "nm";
    let mut robot_side = None;
    for port in candidates {
        let mgr = NetworkManager::new(NetworkConfig {
            listen_endpoints: vec![format!("tcp/127.0.0.1:{port}")],
            robot_identity: Some(robot.to_string()),
            ..NetworkConfig::default()
        });
        if mgr.session().is_ok() {
            robot_side = Some((mgr, port));
            break;
        }
    }
    let (robot_mgr, port) = robot_side.expect("no candidate localhost port could be bound");
    let session = robot_mgr.session().expect("robot session");
    let _ann = TopicToken::announce(session, robot, "/widget").expect("announce");

    let bridge = Arc::new(TopicBridgeManager::new());
    bridge.register_topic("/widget").expect("register");
    // topic_schemas binds /widget → the qualified custom type name.
    let topic_schemas = Arc::new(Mutex::new(HashMap::from([(
        "/widget".to_string(),
        "probe_msgs/Widget".to_string(),
    )])));
    robot_mgr
        .start_query_surface(
            Arc::clone(&bridge),
            Arc::new(GetDemandState::default()),
            CatalogSource {
                hashes: Arc::new(Mutex::new(HashMap::new())),
                boot_topics: Arc::new(HashSet::new()),
                topic_schemas,
                hash_names: Arc::new(Mutex::new(HashMap::new())),
                producer_probe: None,
                liveness: None,
                liveness_clock: None,
            },
            SchemaSource::default(),
            runs_source_for_a_zenoh_only_surface(),
        )
        .expect("declare query surface");

    let desk = NetworkManager::new(NetworkConfig {
        connect_endpoints: vec![format!("tcp/127.0.0.1:{port}")],
        multicast_scouting: false,
        gossip_scouting: false,
        ..NetworkConfig::default()
    });
    let dsession = desk.session().expect("desk session");
    for _ in 0..40 {
        if let Some(catalog) = query_robot_catalog(dsession, robot, Duration::from_millis(300)) {
            let entry = catalog
                .entries
                .iter()
                .find(|e| e.topic == "/widget")
                .expect("catalog has the widget topic");
            if entry.schema_name.as_deref() == Some("probe_msgs/Widget") {
                return; // the schema name crossed the wire in the catalog.
            }
        }
        std::thread::sleep(Duration::from_millis(100));
    }
    panic!("catalog never carried the per-topic schema_name for /widget");
}

/// Bind a listening robot-side [`NetworkManager`] on a pid-derived localhost port
/// (bounded candidate retry). Returns the manager + the bound port. Shared by the
/// newer live tests (the older schema tests inline the same probe loop).
fn bind_listening_robot(robot: &str, base: u16) -> (NetworkManager, u16) {
    let pid = std::process::id();
    for slot in 0..3u16 {
        let port = base + slot * 400 + (pid % 300) as u16;
        let mgr = NetworkManager::new(NetworkConfig {
            listen_endpoints: vec![format!("tcp/127.0.0.1:{port}")],
            robot_identity: Some(robot.to_string()),
            ..NetworkConfig::default()
        });
        if mgr.session().is_ok() {
            return (mgr, port);
        }
    }
    panic!("no candidate localhost port could be bound for robot '{robot}'");
}

/// A PACKAGE-LESS workspace type (`cerulion schema
/// create <Name>`) is served + fetchable over the `schema` verb's 1-chunk
/// selector (`cerulion_q/{robot}/schema/{Name}`). A desk fetches the bare `Name`
/// and gets its doc back; a bare name the robot does NOT serve returns the explicit
/// NOT-FOUND (the same empty-docs degrade an old robot's 1-chunk-rejecting
/// queryable produces via an empty gather). Real zenoh sessions, scouting off.
#[test]
fn schema_verb_serves_package_less_yaml_type() {
    use cerulion_core::transport::bridge::TopicBridgeManager;
    use cerulion_core::transport::discovery::query_robot_schema;
    use cerulion_core::transport::network::{CatalogSource, GetDemandState, SchemaSource};
    use cerulion_core::{SchemaDoc, SchemaEncoding};
    use std::collections::{BTreeMap, HashMap, HashSet};
    use std::sync::{Arc, Mutex};

    // A package-less workspace YAML type served under its BARE qualified key.
    let mystate_q = "MyState";
    let mystate_text =
        "schemas:\n  MyState:\n    fields:\n      \"float64 x\":\n      \"float64 y\":\n";
    let robot = "bare";
    let (robot_mgr, port) = bind_listening_robot(robot, 26000);

    let schema = SchemaSource {
        docs: Arc::new(Mutex::new(BTreeMap::from([(
            mystate_q.to_string(),
            SchemaDoc {
                qualified: mystate_q.to_string(),
                encoding: SchemaEncoding::Yaml,
                text: mystate_text.to_string(),
                deps: vec![],
            },
        )]))),
    };
    let bridge = Arc::new(TopicBridgeManager::new());
    bridge.register_topic("/state").expect("register");
    robot_mgr
        .start_query_surface(
            Arc::clone(&bridge),
            Arc::new(GetDemandState::default()),
            CatalogSource {
                hashes: Arc::new(Mutex::new(HashMap::new())),
                boot_topics: Arc::new(HashSet::new()),
                topic_schemas: Arc::new(Mutex::new(HashMap::new())),
                hash_names: Arc::new(Mutex::new(HashMap::new())),
                producer_probe: None,
                liveness: None,
                liveness_clock: None,
            },
            schema,
            runs_source_for_a_zenoh_only_surface(),
        )
        .expect("declare query surface serving the bare type");

    let desk = NetworkManager::new(NetworkConfig {
        connect_endpoints: vec![format!("tcp/127.0.0.1:{port}")],
        multicast_scouting: false,
        gossip_scouting: false,
        ..NetworkConfig::default()
    });
    let session = desk.session().expect("desk session");

    // Fetch the BARE name over the 1-chunk `schema/{Name}` selector.
    let mut got = false;
    for _ in 0..40 {
        if let Some(reply) =
            query_robot_schema(session, robot, mystate_q, Duration::from_millis(300))
        {
            assert_eq!(
                reply.docs.len(),
                1,
                "a bare package-less type serves exactly its own doc"
            );
            assert_eq!(reply.docs[0].qualified, mystate_q);
            assert_eq!(reply.docs[0].encoding, SchemaEncoding::Yaml);
            assert_eq!(reply.docs[0].text, mystate_text);
            got = true;
            break;
        }
        std::thread::sleep(Duration::from_millis(100));
    }
    assert!(got, "the desk never fetched the bare package-less type");

    // A bare name the robot does NOT serve → explicit NOT-FOUND (empty docs + error)
    // — the same degrade an old robot's 1-chunk-rejecting queryable yields.
    let mut saw_not_found = false;
    for _ in 0..20 {
        if let Some(reply) = query_robot_schema(session, robot, "Nope", Duration::from_millis(300))
        {
            assert!(reply.docs.is_empty());
            assert!(reply.error.is_some());
            saw_not_found = true;
            break;
        }
        std::thread::sleep(Duration::from_millis(100));
    }
    assert!(
        saw_not_found,
        "a missing bare name never returned NOT-FOUND"
    );
}

/// The fetched NESTED-custom doc is load-bearing AT
/// DECODE — a robot serves `Widget` (a FIELD of type `Gadget`, both custom) + its
/// `Gadget` closure member; a desk fetches the closure, seeds a `FrameWalker`, and
/// decodes a REAL Widget frame whose payload NESTS the `Gadget` field, asserting
/// the NESTED-field value vs a hand oracle. The `gadget.count` assertion KILLS a
/// mutation dropping `deps` from the closure: without the fetched `Gadget` doc the
/// walker cannot resolve `Widget`'s layout and `walk` errors. Real zenoh, off.
#[test]
fn schema_verb_nested_custom_frame_decodes_via_fetched_dependency() {
    use cerulion_cli_engine::topic_cmd::seed_framewalker;
    use cerulion_core::codegen::FrameValueKind;
    use cerulion_core::transport::bridge::TopicBridgeManager;
    use cerulion_core::transport::discovery::query_robot_schema;
    use cerulion_core::transport::network::{CatalogSource, GetDemandState, SchemaSource};
    use cerulion_core::wire::WireHeader;
    use cerulion_core::{SchemaDoc, SchemaEncoding};
    use std::collections::{BTreeMap, HashMap, HashSet};
    use std::sync::{Arc, Mutex};

    // Widget nests Gadget as a FIXED field (Gadget is fixed-only). `scale` (f64)
    // is declared FIRST so it inlines 8-aligned at offset 0 (0..8), then the
    // nested Gadget's `count` (i32) inlines at offset 8 (8..12) — a PACKED 12-byte
    // fixed section (no alignment padding to hand-account for).
    let widget_q = "probe_msgs/Widget";
    let gadget_q = "probe_msgs/Gadget";
    let widget_text = "float64 scale\nprobe_msgs/Gadget gadget\n";
    let gadget_text = "int32 count\n";
    let robot = "nest";
    let (robot_mgr, port) = bind_listening_robot(robot, 26800);

    let schema = SchemaSource {
        docs: Arc::new(Mutex::new(BTreeMap::from([
            (
                widget_q.to_string(),
                SchemaDoc {
                    qualified: widget_q.to_string(),
                    encoding: SchemaEncoding::Msg,
                    text: widget_text.to_string(),
                    deps: vec![gadget_q.to_string()], // the closure dep the DECODE needs
                },
            ),
            (
                gadget_q.to_string(),
                SchemaDoc {
                    qualified: gadget_q.to_string(),
                    encoding: SchemaEncoding::Msg,
                    text: gadget_text.to_string(),
                    deps: vec![],
                },
            ),
        ]))),
    };
    let bridge = Arc::new(TopicBridgeManager::new());
    bridge.register_topic("/nested").expect("register");
    robot_mgr
        .start_query_surface(
            Arc::clone(&bridge),
            Arc::new(GetDemandState::default()),
            CatalogSource {
                hashes: Arc::new(Mutex::new(HashMap::new())),
                boot_topics: Arc::new(HashSet::new()),
                topic_schemas: Arc::new(Mutex::new(HashMap::new())),
                hash_names: Arc::new(Mutex::new(HashMap::new())),
                producer_probe: None,
                liveness: None,
                liveness_clock: None,
            },
            schema,
            runs_source_for_a_zenoh_only_surface(),
        )
        .expect("declare query surface serving the nested closure");

    let desk = NetworkManager::new(NetworkConfig {
        connect_endpoints: vec![format!("tcp/127.0.0.1:{port}")],
        multicast_scouting: false,
        gossip_scouting: false,
        ..NetworkConfig::default()
    });
    let session = desk.session().expect("desk session");

    // HAND oracle frame: Widget { scale: 1.5, gadget: Gadget { count: 42 } }.
    // Header hash is irrelevant — the echo path resolves the name then `walk`s by
    // name (hash 0). Fixed section (declaration order): [scale f64 LE][count i32 LE].
    let (count_oracle, scale_oracle) = (42i32, 1.5f64);
    let mut frame = vec![0u8; WireHeader::SIZE];
    WireHeader::with_schema(0).write_to_buf(&mut frame);
    frame.extend_from_slice(&scale_oracle.to_le_bytes());
    frame.extend_from_slice(&count_oracle.to_le_bytes());

    let mut decoded = false;
    for _ in 0..40 {
        if let Some(reply) =
            query_robot_schema(session, robot, widget_q, Duration::from_millis(300))
        {
            // The closure carries BOTH docs (requested first, then the nested dep).
            let names: Vec<&str> = reply.docs.iter().map(|d| d.qualified.as_str()).collect();
            assert_eq!(names, vec![widget_q, gadget_q]);
            let walker = seed_framewalker(&reply.docs);
            // The mutation-kill: without the fetched Gadget doc, Widget's layout is
            // unresolvable and this `walk` returns Err (expect panics).
            let fv = walker
                .walk(widget_q, &frame)
                .expect("walk the nested Widget frame (needs the fetched Gadget layout)");
            match fv.field("gadget") {
                Some(FrameValueKind::Nested(inner)) => {
                    assert_eq!(inner.schema_name, gadget_q);
                    assert_eq!(
                        inner.field("count"),
                        Some(&FrameValueKind::I32(count_oracle)),
                        "the NESTED custom field decoded via the fetched dependency's layout"
                    );
                }
                other => panic!("field gadget is not a Nested value: {other:?}"),
            }
            match fv.field("scale") {
                Some(FrameValueKind::F64(v)) => assert_eq!(*v, scale_oracle),
                other => panic!("field scale is not an F64: {other:?}"),
            }
            decoded = true;
            break;
        }
        std::thread::sleep(Duration::from_millis(100));
    }
    assert!(decoded, "the desk never decoded the nested Widget frame");
}

/// Drives `fetch_remote_schema` two ways — a
/// SUCCESS (a robot serving the type answers with its docs) and an UNREACHABLE
/// case (a dead connect locator → bounded, exit 0, never a hang or an Err). The
/// garbage-reply warn-once + degrade is covered by the pure `gather_schema_outcome`
/// unit oracles; here we drive the real desk entry point.
///
/// The unreachable arm must not accept a bare `Ok(None)`,
/// which the caller would render as the terminal "schema not found". A dead locator
/// means the harvest found NO robot to ask, so nothing was searched and the correct
/// verdict is `DiscoveryNotConverged`; asserting the CLASSIFICATION (not merely
/// "not an Err") is the stronger pin.
#[test]
fn fetch_remote_schema_success_and_unreachable() {
    use cerulion_cli_engine::topic_cmd::{
        fetch_remote_schema, RemoteSchemaFetch, RemoteTopicsOptions,
    };
    use cerulion_core::transport::bridge::TopicBridgeManager;
    use cerulion_core::transport::discovery::TopicToken;
    use cerulion_core::transport::network::{CatalogSource, GetDemandState, SchemaSource};
    use cerulion_core::{SchemaDoc, SchemaEncoding};
    use std::collections::{BTreeMap, HashMap, HashSet};
    use std::sync::{Arc, Mutex};

    let widget_q = "probe_msgs/Widget";
    let robot = "fetch";
    let (robot_mgr, port) = bind_listening_robot(robot, 27600);
    let session = robot_mgr.session().expect("robot session");
    // Announce the identity so the desk's announce-space harvest discovers the robot.
    let _ann = TopicToken::announce(session, robot, "/fetch").expect("announce");

    let schema = SchemaSource {
        docs: Arc::new(Mutex::new(BTreeMap::from([(
            widget_q.to_string(),
            SchemaDoc {
                qualified: widget_q.to_string(),
                encoding: SchemaEncoding::Msg,
                text: "float64 x\nfloat64 y\n".to_string(),
                deps: vec![],
            },
        )]))),
    };
    let bridge = Arc::new(TopicBridgeManager::new());
    bridge.register_topic("/fetch").expect("register");
    robot_mgr
        .start_query_surface(
            Arc::clone(&bridge),
            Arc::new(GetDemandState::default()),
            CatalogSource {
                hashes: Arc::new(Mutex::new(HashMap::new())),
                boot_topics: Arc::new(HashSet::new()),
                topic_schemas: Arc::new(Mutex::new(HashMap::new())),
                hash_names: Arc::new(Mutex::new(HashMap::new())),
                producer_probe: None,
                liveness: None,
                liveness_clock: None,
            },
            schema,
            runs_source_for_a_zenoh_only_surface(),
        )
        .expect("declare query surface");

    // SUCCESS: a desk fetching Widget over an explicit connect locator (scouting
    // off — hermetic) gets the doc back, provenance = the serving robot.
    let ok_opts = RemoteTopicsOptions {
        connect: vec![format!("tcp/127.0.0.1:{port}")],
        listen: vec![],
        scouting: false,
    };
    let mut fetched = false;
    for _ in 0..40 {
        if let RemoteSchemaFetch::Found(reply) =
            fetch_remote_schema(widget_q, &ok_opts).expect("fetch is best-effort")
        {
            assert_eq!(reply.robot, robot, "provenance names the serving robot");
            assert!(!reply.docs.is_empty());
            assert_eq!(reply.docs[0].qualified, widget_q);
            fetched = true;
            break;
        }
        std::thread::sleep(Duration::from_millis(100));
    }
    assert!(
        fetched,
        "fetch_remote_schema never resolved the served type"
    );

    // NOT-SERVED: the SAME reachable robot, asked for a type it does not have. We
    // really did read a robot, so this IS a genuine absence — `NotServed`, and the
    // caller may re-raise its local "schema not found". This is the anti-tautology
    // control for the unreachable arm below: without it, a "fix" that returned
    // `DiscoveryNotConverged` unconditionally would pass.
    //
    // Wrapped in the SAME bounded retry its sibling above uses. Each call opens a
    // FRESH transient session, so discovery can legitimately miss on any given attempt
    // and answer `DiscoveryNotConverged` — the correct verdict for "we read nobody".
    // A single-shot assert here would be flaky for a reason that is not the contract.
    let mut saw_not_served = false;
    for _ in 0..40 {
        match fetch_remote_schema("probe_msgs/NoSuchType", &ok_opts).expect("fetch is best-effort")
        {
            RemoteSchemaFetch::NotServed => {
                saw_not_served = true;
                break;
            }
            // The robot has not been discovered on THIS attempt — retry.
            RemoteSchemaFetch::DiscoveryNotConverged => {}
            // No cancellation source on this path.
            RemoteSchemaFetch::Cancelled => panic!("no cancellation source was supplied"),
            RemoteSchemaFetch::Found(reply) => panic!(
                "no robot serves 'NoSuchType', yet one answered with docs: {:?}",
                reply.requested
            ),
        }
        std::thread::sleep(Duration::from_millis(100));
    }
    assert!(
        saw_not_served,
        "a robot ANSWERED and lacks the type — that is a real absence (NotServed), which \
         the caller may render as its local 'schema not found'"
    );

    // ANNOUNCED-BUT-SILENT (the MIDDLE case): a robot that ANNOUNCES its
    // identity but serves NO query surface, so its schema GET never comes back inside
    // its window. `query_robot_schemas` DROPS every robot that misses the window, so
    // the gather yields ZERO replies — we read NOTHING — while `robots` is NON-empty.
    //
    // This is the arm the two above structurally cannot reach: the NotServed arm's
    // robot ANSWERS (a docs-less reply is still a reply), and the unreachable arm has
    // zero announces so it short-circuits before the GET. Gating on `robots` alone
    // called this a settled "nobody serves this type" and rendered `cerulion schema
    // info`'s terminal "Schema not found" from a gather that read nothing.
    //
    // Reverting the gate to `robots.is_empty()` only fails here and
    // nowhere else in this file.
    let (silent_mgr, silent_port) = bind_listening_robot("silent", 27980);
    let silent_session = silent_mgr.session().expect("silent robot session");
    let _silent_ann = TopicToken::announce(silent_session, "silent", "/silent").expect("announce");
    let silent_opts = RemoteTopicsOptions {
        connect: vec![format!("tcp/127.0.0.1:{silent_port}")],
        listen: vec![],
        scouting: false,
    };
    let mut saw_not_converged = false;
    for _ in 0..40 {
        match fetch_remote_schema(widget_q, &silent_opts).expect("fetch is best-effort") {
            RemoteSchemaFetch::Cancelled => panic!("no cancellation source was supplied"),
            RemoteSchemaFetch::DiscoveryNotConverged => {
                saw_not_converged = true;
                break;
            }
            // The announce has not propagated yet on this attempt — retry. (Zero
            // announces short-circuits to the SAME verdict, so this cannot mask the
            // contract; what it waits for is the announce that makes `robots`
            // NON-empty, which is the whole point of this arm.)
            RemoteSchemaFetch::NotServed => {}
            RemoteSchemaFetch::Found(reply) => panic!(
                "a robot with NO query surface cannot serve docs, got {:?}",
                reply.requested
            ),
        }
        std::thread::sleep(Duration::from_millis(100));
    }
    assert!(
        saw_not_converged,
        "a robot ANNOUNCED but never served its schema — nothing was READ, so this \
         licenses NO absence claim (it must not render as 'Schema not found')"
    );

    // UNREACHABLE: a dead connect locator (bounded_connect) → no robots harvested →
    // NOTHING was searched, so the correct verdict is `DiscoveryNotConverged`.
    // Never a hang, never an Err. A bare `None` here would be rendered by
    // `cerulion schema info` as a terminal "schema not found" — a claim with
    // no evidence behind it.
    let dead_port = {
        let l = std::net::TcpListener::bind("127.0.0.1:0").expect("probe");
        let p = l.local_addr().unwrap().port();
        drop(l);
        p
    };
    let unreachable = RemoteTopicsOptions {
        connect: vec![format!("tcp/127.0.0.1:{dead_port}")],
        listen: vec![],
        scouting: false,
    };
    let out = fetch_remote_schema(widget_q, &unreachable)
        .expect("unreachable is a classified outcome, not an Err");
    assert!(
        matches!(out, RemoteSchemaFetch::DiscoveryNotConverged),
        "an unreachable robot means we asked NOBODY — that licenses no absence claim \
         (got {})",
        match out {
            RemoteSchemaFetch::Found(_) => "Found",
            RemoteSchemaFetch::NotServed => "NotServed",
            RemoteSchemaFetch::DiscoveryNotConverged => unreachable!(),
            RemoteSchemaFetch::Cancelled => unreachable!("no cancellation source"),
        }
    );
}

/// The `topic echo` decode-SEED path
/// (`resolve_remote_walker_for_topic`) — the RESOLVE branch (a topic whose catalog
/// carries a schema NAME → fetch → seed → the seeded walker decodes a frame of
/// that type, the exact echo decode-vs-hex decision) AND the DEGRADE branch (a
/// topic NOT in any catalog → `None` → the echo loop's hex fallback, exit 0).
/// Driven over the locator-injectable `_with_opts` seam (scouting off — hermetic).
#[test]
fn resolve_remote_walker_for_topic_decodes_named_topic_and_degrades_on_unknown() {
    use cerulion_cli_engine::topic_cmd::resolve_remote_walker_for_topic_with_opts;
    use cerulion_core::codegen::{parse_rosmsg, FrameValueKind};
    use cerulion_core::transport::bridge::TopicBridgeManager;
    use cerulion_core::transport::discovery::TopicToken;
    use cerulion_core::transport::network::{CatalogSource, GetDemandState, SchemaSource};
    use cerulion_core::wire::WireHeader;
    use cerulion_core::{SchemaDoc, SchemaEncoding};
    use std::collections::{BTreeMap, HashMap, HashSet};
    use std::sync::{Arc, Mutex};

    let widget_q = "probe_msgs/Widget";
    let widget_text = "float64 x\nfloat64 y\n";
    let echo_topic = "/echo";
    let robot = "echo";
    let (robot_mgr, port) = bind_listening_robot(robot, 28400);
    let session = robot_mgr.session().expect("robot session");
    // Announce the identity + topic so the desk's announce harvest discovers it.
    let _ann = TopicToken::announce(session, robot, echo_topic).expect("announce");

    let bridge = Arc::new(TopicBridgeManager::new());
    bridge
        .register_topic(echo_topic)
        .expect("register echo topic");
    // The catalog BINDS the echo topic to Widget (the desk learns WHICH type).
    let topic_schemas = Arc::new(Mutex::new(HashMap::from([(
        echo_topic.to_string(),
        widget_q.to_string(),
    )])));
    let schema = SchemaSource {
        docs: Arc::new(Mutex::new(BTreeMap::from([(
            widget_q.to_string(),
            SchemaDoc {
                qualified: widget_q.to_string(),
                encoding: SchemaEncoding::Msg,
                text: widget_text.to_string(),
                deps: vec![],
            },
        )]))),
    };
    robot_mgr
        .start_query_surface(
            Arc::clone(&bridge),
            Arc::new(GetDemandState::default()),
            CatalogSource {
                hashes: Arc::new(Mutex::new(HashMap::new())),
                boot_topics: Arc::new(HashSet::new()),
                topic_schemas,
                hash_names: Arc::new(Mutex::new(HashMap::new())),
                producer_probe: None,
                liveness: None,
                liveness_clock: None,
            },
            schema,
            runs_source_for_a_zenoh_only_surface(),
        )
        .expect("declare query surface");

    let connect = vec![format!("tcp/127.0.0.1:{port}")];
    // HAND oracle Widget frame carrying its REAL hash (so `schema_name_for_hash`
    // resolves it — the exact echo decode-branch seed).
    let widget_hash = parse_rosmsg(widget_text, "Widget", Some("probe_msgs"))
        .unwrap()
        .schema_hash();
    let (x_oracle, y_oracle) = (3.5f64, -7.25f64);
    let mut frame = vec![0u8; WireHeader::SIZE];
    WireHeader::with_schema(widget_hash).write_to_buf(&mut frame);
    frame.extend_from_slice(&x_oracle.to_le_bytes());
    frame.extend_from_slice(&y_oracle.to_le_bytes());

    // RESOLVE branch: the catalogued topic resolves a walker that decodes the frame.
    let mut resolved = false;
    for _ in 0..40 {
        if let Some((got_robot, walker)) =
            resolve_remote_walker_for_topic_with_opts(echo_topic, &connect, &[], false)
        {
            assert_eq!(got_robot, robot);
            // The echo decode-branch: name-for-hash, then walk by name.
            let name = walker
                .schema_name_for_hash(widget_hash)
                .expect("the seeded walker knows the custom hash");
            assert_eq!(name, widget_q);
            let fv = walker.walk(name, &frame).expect("decode the echo frame");
            match fv.field("x") {
                Some(FrameValueKind::F64(v)) => assert_eq!(*v, x_oracle),
                other => panic!("field x not F64: {other:?}"),
            }
            resolved = true;
            break;
        }
        std::thread::sleep(Duration::from_millis(100));
    }
    assert!(
        resolved,
        "the echo decode-seed never resolved the catalogued topic"
    );

    // DEGRADE branch: a topic NOT in any catalog → None (the echo loop falls back
    // to the hash-only hex dump; exit 0). A discovered-robot run whose target topic
    // is absent MUST return None, not hang.
    let degraded =
        resolve_remote_walker_for_topic_with_opts("/not_catalogued", &connect, &[], false);
    assert!(
        degraded.is_none(),
        "an un-catalogued topic yields None (the echo hex-fallback degrade)"
    );
}

/// `resolve_remote_ingress_target_with_opts` is the desk-side
/// resolve `topic echo/info/hz` run for a topic ABSENT locally. It must return
/// the announcing ROBOT, the topic's qualified schema NAME (from the catalog),
/// the wire `schema_hash` `register_ingress_topic` validates against, and a
/// walker that DECODES a real frame of that type. Full pipeline over loopback:
/// announce (robot harvest) + catalog (topic→schema_name) + schema serving (the
/// `.msg` closure). Two refusals: a topic no catalog carries →
/// `NoProducer` (the `topic_not_found_anywhere` floor); a topic a robot DID
/// catalog but whose schema it does NOT serve → `SchemaUnavailable` (the named,
/// actionable error). Real zenoh sessions, scouting off.
#[test]
fn resolve_remote_ingress_target_finds_robot_schema_hash_and_walker() {
    use cerulion_cli_engine::topic_cmd::{resolve_remote_ingress_target_with_opts, RemoteResolve};
    use cerulion_core::codegen::parse_rosmsg;
    use cerulion_core::transport::bridge::TopicBridgeManager;
    use cerulion_core::transport::network::{CatalogSource, GetDemandState, SchemaSource};
    use cerulion_core::wire::WireHeader;
    use cerulion_core::{SchemaDoc, SchemaEncoding};
    use std::collections::{BTreeMap, HashMap, HashSet};
    use std::sync::{Arc, Mutex};

    let widget_text = "float64 x\nfloat64 y\n";
    let widget_q = "absent_probe_msgs/Widget";
    let topic = "/lidar";
    let robot = "rob";

    let pid = std::process::id();
    let candidates = [
        25400 + (pid % 300) as u16,
        25800 + (pid % 300) as u16,
        26200 + (pid % 300) as u16,
    ];
    let mut robot_side = None;
    for port in candidates {
        let mgr = NetworkManager::new(NetworkConfig {
            listen_endpoints: vec![format!("tcp/127.0.0.1:{port}")],
            robot_identity: Some(robot.to_string()),
            ..NetworkConfig::default()
        });
        if mgr.session().is_ok() {
            robot_side = Some((mgr, port));
            break;
        }
    }
    let (robot_mgr, port) = robot_side.expect("no candidate localhost port could be bound");
    let session = robot_mgr.session().expect("robot session");
    // Announce the topic under the robot identity — the desk's robot HARVEST.
    let _ann = TopicToken::announce(session, robot, topic).expect("announce token");

    // A SECOND catalogued topic whose bound type the robot
    // does NOT serve — resolving it must yield SchemaUnavailable (named), never
    // NoProducer.
    let noschema_topic = "/noschema";
    let absent_q = "absent_probe_msgs/Absent";

    let bridge = Arc::new(TopicBridgeManager::new());
    bridge.register_topic(topic).expect("register topic");
    bridge
        .register_topic(noschema_topic)
        .expect("register noschema topic");
    // The catalog binds each topic → its qualified custom type name. `absent_q`
    // is NOT in the SchemaSource below (the robot catalogs it but cannot serve it).
    let topic_schemas = Arc::new(Mutex::new(HashMap::from([
        (topic.to_string(), widget_q.to_string()),
        (noschema_topic.to_string(), absent_q.to_string()),
    ])));
    // The schema source serves that type's `.msg` closure.
    let schema = SchemaSource {
        docs: Arc::new(Mutex::new(BTreeMap::from([(
            widget_q.to_string(),
            SchemaDoc {
                qualified: widget_q.to_string(),
                encoding: SchemaEncoding::Msg,
                text: widget_text.to_string(),
                deps: vec![],
            },
        )]))),
    };
    robot_mgr
        .start_query_surface(
            Arc::clone(&bridge),
            Arc::new(GetDemandState::default()),
            CatalogSource {
                hashes: Arc::new(Mutex::new(HashMap::new())),
                boot_topics: Arc::new(HashSet::new()),
                topic_schemas,
                hash_names: Arc::new(Mutex::new(HashMap::new())),
                producer_probe: None,
                liveness: None,
                liveness_clock: None,
            },
            schema,
            runs_source_for_a_zenoh_only_surface(),
        )
        .expect("declare query surface");

    // Hand oracle: the recipe-3 wire hash the walker + robot both compute; a real
    // Widget frame = 32-byte header (carrying the hash) + [x_le][y_le].
    let widget_hash = parse_rosmsg(widget_text, "Widget", Some("absent_probe_msgs"))
        .expect("parse widget")
        .schema_hash();
    let (x_oracle, y_oracle) = (3.5f64, -7.25f64);
    let mut frame = vec![0u8; WireHeader::SIZE];
    WireHeader::with_schema(widget_hash).write_to_buf(&mut frame);
    frame.extend_from_slice(&x_oracle.to_le_bytes());
    frame.extend_from_slice(&y_oracle.to_le_bytes());

    let connect = vec![format!("tcp/127.0.0.1:{port}")];
    // Converge (connect + queryable route settle asynchronously).
    let mut target = None;
    for _ in 0..40 {
        if let RemoteResolve::Found(t) =
            resolve_remote_ingress_target_with_opts(topic, &connect, &[], false, None)
        {
            target = Some(t);
            break;
        }
        std::thread::sleep(Duration::from_millis(100));
    }
    let target = target.expect("resolve never found the remote ingress target");

    assert_eq!(target.robot, robot, "the announcing robot identity");
    assert_eq!(
        target.schema_name, widget_q,
        "the catalog's qualified schema name"
    );
    assert_eq!(
        target.schema_hash, widget_hash,
        "the wire hash register_ingress_topic validates against"
    );
    // The seeded walker decodes the REAL frame byte-correctly vs the hand oracle.
    let fv = target
        .walker
        .walk_by_hash(&frame)
        .expect("walk widget frame by hash");
    assert_eq!(fv.schema_name, widget_q);
    let x = match fv.field("x") {
        Some(cerulion_core::codegen::FrameValueKind::F64(v)) => *v,
        other => panic!("field x not F64: {other:?}"),
    };
    let y = match fv.field("y") {
        Some(cerulion_core::codegen::FrameValueKind::F64(v)) => *v,
        other => panic!("field y not F64: {other:?}"),
    };
    assert_eq!(
        (x, y),
        (x_oracle, y_oracle),
        "decoded values equal the hand oracle"
    );

    // NOT-FOUND vs SCHEMA-UNAVAILABLE (robust form): in an iteration where a
    // FRESH-session resolve of the KNOWN topic succeeds (proving connectivity +
    // catalog fetch settled), an UNCATALOGUED topic MUST resolve to `NoProducer`
    // while a CATALOGUED-BUT-UNSERVED topic MUST resolve to `SchemaUnavailable`
    // naming the robot — genuinely distinguished, not both collapsed to None.
    let mut proven = false;
    for _ in 0..40 {
        let known = resolve_remote_ingress_target_with_opts(topic, &connect, &[], false, None);
        let missing =
            resolve_remote_ingress_target_with_opts("/missing", &connect, &[], false, None);
        let unserved =
            resolve_remote_ingress_target_with_opts(noschema_topic, &connect, &[], false, None);
        if matches!(known, RemoteResolve::Found(_)) {
            assert!(
                matches!(missing, RemoteResolve::NoProducer),
                "an uncatalogued topic must resolve to NoProducer while a catalogued sibling resolves"
            );
            match unserved {
                RemoteResolve::SchemaUnavailable { robot: r, cause } => {
                    assert_eq!(r, robot, "the unserved-schema error names the robot");
                    assert!(
                        cause.contains(noschema_topic) && cause.contains(absent_q),
                        "the cause names the topic + the unserved type: {cause}"
                    );
                }
                _ => panic!(
                    "a catalogued-but-unserved topic must resolve to SchemaUnavailable, got a \
                     different variant (missing schema serving?)"
                ),
            }
            proven = true;
            break;
        }
        std::thread::sleep(Duration::from_millis(100));
    }
    assert!(
        proven,
        "could not establish a converged fresh session to prove the not-found / \
         schema-unavailable distinction"
    );
}

/// A BUILT-IN type the robot CATALOGS but does NOT serve resolves from
/// the desk's OWN corpus with NO wire schema fetch (the Go2 shape:
/// `topic hz /utlidar/cloud` would fail if the robot cataloged it as
/// `sensor_msgs/PointCloud2` while its serving store held only bridge-config
/// customs). The robot here serves ZERO schemas, so the ONLY way a resolve can
/// succeed is LOCAL resolution — that IS the "no wire fetch" proof. A CUSTOM type
/// the desk lacks AND the robot does not serve still yields `SchemaUnavailable`
/// (the wire fetch stays the fallback, and it correctly fails for a genuinely
/// unknown type). Hand oracles (the built-in's recipe-3 hash via `parse_rosmsg`,
/// and a decoded Vector3 frame), NOT self-compares. Real zenoh, scouting off.
#[test]
fn resolve_built_in_type_uses_local_corpus_without_wire_fetch() {
    use cerulion_cli_engine::topic_cmd::{resolve_remote_ingress_target_with_opts, RemoteResolve};
    use cerulion_core::codegen::parse_rosmsg;
    use cerulion_core::transport::bridge::TopicBridgeManager;
    use cerulion_core::transport::network::{CatalogSource, GetDemandState, SchemaSource};
    use cerulion_core::wire::WireHeader;
    use std::collections::{BTreeMap, HashMap, HashSet};
    use std::sync::{Arc, Mutex};

    // The robot catalogs a built-in on one topic and a desk-unknown CUSTOM type
    // on another — and serves NEITHER schema (empty SchemaSource).
    let builtin_topic = "/cloud";
    let builtin_q = "geometry_msgs/Vector3";
    let custom_topic = "/gadget";
    let custom_q = "custom_probe_msgs/Gadget";
    let robot = "rob";

    let pid = std::process::id();
    let candidates = [
        27400 + (pid % 300) as u16,
        27800 + (pid % 300) as u16,
        28200 + (pid % 300) as u16,
    ];
    let mut robot_side = None;
    for port in candidates {
        let mgr = NetworkManager::new(NetworkConfig {
            listen_endpoints: vec![format!("tcp/127.0.0.1:{port}")],
            robot_identity: Some(robot.to_string()),
            ..NetworkConfig::default()
        });
        if mgr.session().is_ok() {
            robot_side = Some((mgr, port));
            break;
        }
    }
    let (robot_mgr, port) = robot_side.expect("no candidate localhost port could be bound");
    let session = robot_mgr.session().expect("robot session");
    let _ann = TopicToken::announce(session, robot, builtin_topic).expect("announce token");

    let bridge = Arc::new(TopicBridgeManager::new());
    bridge
        .register_topic(builtin_topic)
        .expect("register builtin topic");
    bridge
        .register_topic(custom_topic)
        .expect("register custom topic");
    let topic_schemas = Arc::new(Mutex::new(HashMap::from([
        (builtin_topic.to_string(), builtin_q.to_string()),
        (custom_topic.to_string(), custom_q.to_string()),
    ])));
    // The robot serves NO schemas at all — the built-in must resolve LOCALLY.
    let schema = SchemaSource {
        docs: Arc::new(Mutex::new(BTreeMap::new())),
    };
    robot_mgr
        .start_query_surface(
            Arc::clone(&bridge),
            Arc::new(GetDemandState::default()),
            CatalogSource {
                hashes: Arc::new(Mutex::new(HashMap::new())),
                boot_topics: Arc::new(HashSet::new()),
                topic_schemas,
                hash_names: Arc::new(Mutex::new(HashMap::new())),
                producer_probe: None,
                liveness: None,
                liveness_clock: None,
            },
            schema,
            runs_source_for_a_zenoh_only_surface(),
        )
        .expect("declare query surface");

    // Independent oracle: the recipe-3 hash of geometry_msgs/Vector3 from its bare
    // .msg fields (comments/whitespace do not affect the hash) — the SAME hash the
    // desk's built-in corpus computes and the robot would stamp on the wire.
    let vec3_hash = parse_rosmsg(
        "float64 x\nfloat64 y\nfloat64 z\n",
        "Vector3",
        Some("geometry_msgs"),
    )
    .expect("parse Vector3")
    .schema_hash();
    let (x_o, y_o, z_o) = (1.5f64, -2.25f64, 8.0f64);
    let mut frame = vec![0u8; WireHeader::SIZE];
    WireHeader::with_schema(vec3_hash).write_to_buf(&mut frame);
    frame.extend_from_slice(&x_o.to_le_bytes());
    frame.extend_from_slice(&y_o.to_le_bytes());
    frame.extend_from_slice(&z_o.to_le_bytes());

    let connect = vec![format!("tcp/127.0.0.1:{port}")];
    // Converge (connect + queryable settle asynchronously). `None` schemas_dir =
    // built-ins-only local corpus.
    let mut target = None;
    for _ in 0..40 {
        if let RemoteResolve::Found(t) =
            resolve_remote_ingress_target_with_opts(builtin_topic, &connect, &[], false, None)
        {
            target = Some(t);
            break;
        }
        std::thread::sleep(Duration::from_millis(100));
    }
    let target = target
        .expect("a cataloged built-in must resolve from the local corpus (robot serves none)");

    assert_eq!(target.robot, robot, "the announcing robot identity");
    assert_eq!(
        target.schema_name, builtin_q,
        "the catalog's qualified built-in name"
    );
    assert_eq!(
        target.schema_hash, vec3_hash,
        "the LOCAL built-in resolves to the SAME wire hash the robot stamps"
    );
    // The locally-seeded walker decodes the real Vector3 frame vs the hand oracle.
    let fv = target
        .walker
        .walk_by_hash(&frame)
        .expect("walk Vector3 frame by hash");
    assert_eq!(fv.schema_name, builtin_q);
    let read = |name: &str| match fv.field(name) {
        Some(cerulion_core::codegen::FrameValueKind::F64(v)) => *v,
        other => panic!("field {name} not F64: {other:?}"),
    };
    assert_eq!(
        (read("x"), read("y"), read("z")),
        (x_o, y_o, z_o),
        "the local walker decodes the built-in frame to the hand oracle"
    );

    // Control: a CUSTOM type the desk does NOT have AND the robot does NOT serve
    // stays `SchemaUnavailable` — local resolution cannot fabricate it, and the
    // wire fetch (the fallback) correctly finds nothing.
    let mut control_proven = false;
    for _ in 0..40 {
        let known =
            resolve_remote_ingress_target_with_opts(builtin_topic, &connect, &[], false, None);
        let custom =
            resolve_remote_ingress_target_with_opts(custom_topic, &connect, &[], false, None);
        if matches!(known, RemoteResolve::Found(_)) {
            let custom_label = match &custom {
                RemoteResolve::Found(_) => "Found",
                RemoteResolve::NoProducer => "NoProducer",
                // Unreachable on THIS path (an explicit `--connect` locator
                // bypasses netd entirely — see `use_netd_query_plane` — so the
                // transient resolve never reports a netd discovery state), but the
                // label is exhaustive so a future routing change surfaces here
                // instead of failing to compile with no diagnostic value.
                RemoteResolve::DiscoveryNotConverged => "DiscoveryNotConverged",
                RemoteResolve::SchemaUnavailable { .. } => "SchemaUnavailable",
                RemoteResolve::Cancelled => "Cancelled",
            };
            assert!(
                matches!(custom, RemoteResolve::SchemaUnavailable { .. }),
                "a desk-unknown custom type the robot does not serve must stay SchemaUnavailable \
                 (local resolution helps only types the desk actually has), got {custom_label}"
            );
            control_proven = true;
            break;
        }
        std::thread::sleep(Duration::from_millis(100));
    }
    assert!(
        control_proven,
        "could not converge to prove the custom-type control"
    );
}

/// The DESK half of naming a runtime built-in topic: a catalog row
/// whose name was resolved through the robot's hash→name map (the runtime-topic
/// shape: NO `topic_schemas` entry, the reg-channel hash in `hashes`, and the
/// built-in corpus's bindings in `hash_names`) resolves `Found` on the desk from
/// its OWN corpus, with NO served doc: the robot's `SchemaSource` is EMPTY, so
/// local resolution is the only way this can succeed. The resolved hash is the
/// generated `std_msgs::String::SCHEMA_HASH` (the hash an rmw publisher stamps),
/// and the locally-seeded walker decodes a real String frame to a hand oracle.
///
/// This is the answer to "does a built-in need a served doc?" (NO), pinned
/// on exactly the row shape a runtime topic produces, complementing
/// `resolve_built_in_type_uses_local_corpus_without_wire_fetch` (a
/// `topic_schemas`-named row). CONTROL: a sibling runtime row whose hash NO
/// binding carries is not named and does NOT resolve `Found`. Real zenoh,
/// scouting off, explicit locator.
#[test]
fn a_runtime_builtin_row_named_through_hash_bindings_resolves_found_locally() {
    use cerulion_cli_engine::topic_cmd::{resolve_remote_ingress_target_with_opts, RemoteResolve};
    use cerulion_core::message::ShmMessage as _;
    use cerulion_core::transport::bridge::TopicBridgeManager;
    use cerulion_core::transport::network::{CatalogSource, GetDemandState, SchemaSource};
    use cerulion_core::wire::WireHeader;
    use std::collections::{BTreeMap, HashMap, HashSet};
    use std::sync::{Arc, Mutex};

    let named_topic = "/chatter";
    let unnamed_topic = "/mystery";
    let robot = "rob";
    let string_hash = native_ros2_messages::std_msgs::String::SCHEMA_HASH;
    const MYSTERY_HASH: u64 = 0x1541_0000_DEAD_0002;

    let pid = std::process::id();
    let candidates = [
        28600 + (pid % 300) as u16,
        29000 + (pid % 300) as u16,
        29400 + (pid % 300) as u16,
    ];
    let mut robot_side = None;
    for port in candidates {
        let mgr = NetworkManager::new(NetworkConfig {
            listen_endpoints: vec![format!("tcp/127.0.0.1:{port}")],
            robot_identity: Some(robot.to_string()),
            ..NetworkConfig::default()
        });
        if mgr.session().is_ok() {
            robot_side = Some((mgr, port));
            break;
        }
    }
    let (robot_mgr, port) = robot_side.expect("no candidate localhost port could be bound");
    let session = robot_mgr.session().expect("robot session");
    let _ann = TopicToken::announce(session, robot, named_topic).expect("announce token");

    let bridge = Arc::new(TopicBridgeManager::new());
    bridge.register_topic(named_topic).expect("register named");
    bridge
        .register_topic(unnamed_topic)
        .expect("register unnamed");
    // The runtime-topic row shape: NO topic→name binding; the reg-channel hash per topic;
    // the built-in corpus's hash→name map (the SAME bindings the CLI and netd hand
    // a gateway).
    let hash_names: HashMap<u64, String> = native_ros2_messages::builtin_hash_bindings()
        .into_iter()
        .map(|b| (b.schema_hash, b.qualified))
        .collect();
    assert!(
        !hash_names.contains_key(&MYSTERY_HASH),
        "the control hash must be absent from the corpus map"
    );
    let hashes = Arc::new(Mutex::new(HashMap::from([
        (named_topic.to_string(), string_hash),
        (unnamed_topic.to_string(), MYSTERY_HASH),
    ])));
    robot_mgr
        .start_query_surface(
            Arc::clone(&bridge),
            Arc::new(GetDemandState::default()),
            CatalogSource {
                hashes,
                boot_topics: Arc::new(HashSet::new()),
                topic_schemas: Arc::new(Mutex::new(HashMap::new())),
                hash_names: Arc::new(Mutex::new(hash_names)),
                producer_probe: None,
                liveness: None,
                liveness_clock: None,
            },
            // The robot serves NO schemas — a built-in must resolve LOCALLY.
            SchemaSource {
                docs: Arc::new(Mutex::new(BTreeMap::new())),
            },
            runs_source_for_a_zenoh_only_surface(),
        )
        .expect("declare query surface");

    let connect = vec![format!("tcp/127.0.0.1:{port}")];
    let mut target = None;
    for _ in 0..40 {
        if let RemoteResolve::Found(t) =
            resolve_remote_ingress_target_with_opts(named_topic, &connect, &[], false, None)
        {
            target = Some(t);
            break;
        }
        std::thread::sleep(Duration::from_millis(100));
    }
    let target = target.expect(
        "a runtime built-in row named through the hash bindings must resolve Found from the \
         local corpus (the robot serves no docs)",
    );
    assert_eq!(target.robot, robot);
    assert_eq!(target.schema_name, "std_msgs/String");
    assert_eq!(
        target.schema_hash, string_hash,
        "the LOCAL built-in resolves to the hash an rmw publisher stamps on the wire"
    );
    // The locally-seeded walker decodes a real std_msgs/String frame: a
    // 0-fixed / 1-variable layout — [OffsetEntry{offset, length}][utf-8 bytes].
    let text = b"hello from rmw";
    let mut frame = vec![0u8; WireHeader::SIZE];
    let table_offset = frame.len() as u32;
    frame.extend_from_slice(&8u32.to_le_bytes()); // data offset (after the 8-byte entry)
    frame.extend_from_slice(&(text.len() as u32).to_le_bytes());
    frame.extend_from_slice(text);
    let header = WireHeader {
        schema_hash: string_hash,
        total_size: frame.len() as u32,
        offset_table_offset: table_offset,
        offset_table_count: 1,
        sequence: 0,
        timestamp_ns: 0,
    };
    header.write_to_buf(&mut frame[..WireHeader::SIZE]);
    let fv = target
        .walker
        .walk_by_hash(&frame)
        .expect("walk the String frame by hash");
    assert_eq!(fv.schema_name, "std_msgs/String");
    match fv.field("data") {
        Some(cerulion_core::codegen::FrameValueKind::Str(s)) => {
            assert_eq!(
                *s, "hello from rmw",
                "the local walker decodes the built-in frame"
            )
        }
        other => panic!("field data not Str: {other:?}"),
    }

    // CONTROL: the sibling runtime row under a hash no binding carries is NOT
    // named, so it must not resolve Found (no name ⇒ nothing to resolve locally,
    // nothing to fetch).
    let control =
        resolve_remote_ingress_target_with_opts(unnamed_topic, &connect, &[], false, None);
    assert!(
        !matches!(control, RemoteResolve::Found(_)),
        "a runtime row whose hash no binding names must not resolve Found"
    );
}
