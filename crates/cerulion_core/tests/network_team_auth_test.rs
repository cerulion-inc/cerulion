// SPDX-License-Identifier: AGPL-3.0-only
//! Team sessions end to end on loopback, over `quic/` and over `tls/`: two or
//! three `NetworkManager`s holding test team credentials
//! (`cerulion_core::transport::test_ca`), one listening and the others
//! connecting, each with the generated access policy.
//!
//! What each test pins, on both links:
//! - a robot's frames reach a viewer;
//! - a viewer's frames never reach a robot, and an operator's do;
//! - a viewer's demand liveliness token and its query on `cerulion_q/**`
//!   reach the robot, and the robot's reply comes back;
//! - a machine whose leaf comes from another team's CA never links;
//! - a machine that trusts only an older generation never links to a newer
//!   leaf;
//! - a `tcp/` or `udp/...?rel=1` locator fails the open naming the locator,
//!   and team credentials together with certificate files fail the open;
//! - the config's `Debug` and serialized forms carry no key material.

use std::sync::mpsc;
use std::time::{Duration, Instant};

use cerulion_core::transport::test_ca::TestTeam;
use cerulion_core::{NetworkAuth, NetworkConfig, NetworkManager, NetworkTls, TeamAuth, WireHeader};
use cerulion_pairing::team::TeamRole;
use zenoh::Wait;

const LINKS: [&str; 2] = ["quic", "tls"];

/// A loopback port free on both UDP and TCP a moment ago.
fn free_port() -> u16 {
    loop {
        let udp = std::net::UdpSocket::bind("127.0.0.1:0").unwrap();
        let port = udp.local_addr().unwrap().port();
        if std::net::TcpListener::bind(("127.0.0.1", port)).is_ok() {
            return port;
        }
    }
}

fn manager(auth: TeamAuth, listen: Vec<String>, connect: Vec<String>) -> NetworkManager {
    NetworkManager::new(NetworkConfig {
        listen_endpoints: listen,
        connect_endpoints: connect,
        auth: Some(NetworkAuth::Team(auth)),
        ..NetworkConfig::default()
    })
}

/// A listening `server` and a `client` connecting to it over `link`.
fn pair(link: &str, server: TeamAuth, client: TeamAuth) -> (NetworkManager, NetworkManager) {
    let locator = format!("{link}/127.0.0.1:{}", free_port());
    let s = manager(server, vec![locator.clone()], vec![]);
    s.session().expect("the listener opens");
    let c = manager(client, vec![], vec![locator]);
    (s, c)
}

fn linked(m: &NetworkManager, budget: Duration) -> bool {
    let session = m.session().expect("session opens");
    let deadline = Instant::now() + budget;
    while Instant::now() < deadline {
        if session.info().peers_zid().wait().next().is_some() {
            return true;
        }
        std::thread::sleep(Duration::from_millis(20));
    }
    false
}

fn frame(seq: u64) -> Vec<u8> {
    let mut f = vec![0u8; WireHeader::SIZE + 8];
    f[WireHeader::SIZE..].copy_from_slice(&seq.to_le_bytes());
    f
}

/// Subscribe on `to` and put on `from` until a frame arrives or `budget`
/// passes. Returns the number of frames that arrived.
fn deliver(from: &NetworkManager, to: &NetworkManager, topic: &str, budget: Duration) -> usize {
    let session = to.session().expect("receiver session opens");
    let (tx, rx) = mpsc::channel();
    let _sub = session
        .declare_subscriber(NetworkManager::data_key(topic))
        .callback(move |s| {
            let _ = tx.send(s.payload().len());
        })
        .wait()
        .unwrap();
    let deadline = Instant::now() + budget;
    let mut seq = 0;
    let mut got = 0;
    while Instant::now() < deadline {
        from.publish_to_network(topic, frame(seq))
            .expect("the put itself never fails on a live session");
        seq += 1;
        while let Ok(len) = rx.recv_timeout(Duration::from_millis(50)) {
            assert_eq!(len, WireHeader::SIZE + 8, "the frame arrives verbatim");
            got += 1;
        }
        if got > 0 {
            break;
        }
    }
    got
}

#[test]
fn a_robot_delivers_to_a_viewer_on_both_links() {
    let acme = TestTeam::new("acme", 1);
    for link in LINKS {
        let (robot, viewer) = pair(
            link,
            acme.machine(TeamRole::Robot, 1),
            acme.machine(TeamRole::Viewer, 2),
        );
        assert!(
            linked(&viewer, Duration::from_secs(10)),
            "{link}: viewer links"
        );
        assert!(
            deliver(&robot, &viewer, "/team/telemetry", Duration::from_secs(10)) > 0,
            "{link}: robot frames reach the viewer"
        );
    }
}

#[test]
fn only_a_writer_role_delivers_to_a_robot() {
    let acme = TestTeam::new("acme", 1);
    for link in LINKS {
        let (robot, viewer) = pair(
            link,
            acme.machine(TeamRole::Robot, 1),
            acme.machine(TeamRole::Viewer, 2),
        );
        assert!(
            linked(&viewer, Duration::from_secs(10)),
            "{link}: viewer links"
        );
        assert_eq!(
            deliver(&viewer, &robot, "/team/cmd_vel", Duration::from_secs(3)),
            0,
            "{link}: a viewer's frames never reach the robot"
        );
        let (robot, operator) = pair(
            link,
            acme.machine(TeamRole::Robot, 1),
            acme.machine(TeamRole::Operator, 3),
        );
        assert!(
            linked(&operator, Duration::from_secs(10)),
            "{link}: operator links"
        );
        assert!(
            deliver(&operator, &robot, "/team/cmd_vel", Duration::from_secs(10)) > 0,
            "{link}: an operator's frames reach the robot"
        );
    }
}

#[test]
fn a_viewer_demand_token_and_query_reach_the_robot() {
    let acme = TestTeam::new("acme", 1);
    for link in LINKS {
        let (robot, viewer) = pair(
            link,
            acme.machine(TeamRole::Robot, 1),
            acme.machine(TeamRole::Viewer, 2),
        );
        assert!(
            linked(&viewer, Duration::from_secs(10)),
            "{link}: viewer links"
        );
        let rs = robot.session().unwrap();
        let vs = viewer.session().unwrap();
        let _queryable = rs
            .declare_queryable("cerulion_q/robot1/catalog")
            .callback(|q| {
                let _ = q.reply(q.key_expr().clone(), "catalog").wait();
            })
            .wait()
            .unwrap();
        let _token = vs
            .liveliness()
            .declare_token("cerulion_lv/team/telemetry")
            .wait()
            .unwrap();
        let deadline = Instant::now() + Duration::from_secs(10);
        let (mut saw_token, mut replies) = (false, 0);
        while Instant::now() < deadline && !(saw_token && replies > 0) {
            saw_token |= rs
                .liveliness()
                .get("cerulion_lv/**")
                .timeout(Duration::from_millis(300))
                .wait()
                .unwrap()
                .iter()
                .any(|r| r.result().is_ok());
            replies += vs
                .get("cerulion_q/robot1/catalog")
                .timeout(Duration::from_millis(300))
                .wait()
                .unwrap()
                .iter()
                .filter(|r| r.result().is_ok())
                .count();
        }
        assert!(
            saw_token,
            "{link}: the robot sees the viewer's demand token"
        );
        assert!(replies > 0, "{link}: the viewer's query is answered");
    }
}

#[test]
fn a_machine_of_another_team_never_links() {
    let acme = TestTeam::new("acme", 1);
    let rival = TestTeam::new("rival", 1);
    for link in LINKS {
        let (robot, outsider) = pair(
            link,
            acme.machine(TeamRole::Robot, 1),
            rival.machine(TeamRole::Owner, 2),
        );
        assert!(
            !linked(&outsider, Duration::from_secs(3)),
            "{link}: a leaf from another team's CA never links"
        );
        assert!(!linked(&robot, Duration::from_millis(100)), "{link}");
    }
}

#[test]
fn a_machine_of_an_older_generation_refuses_a_newer_leaf() {
    let g1 = TestTeam::new("acme", 1);
    let g2 = g1.rotated();
    // The g2 robot trusts the retired g1 CA; the g1 machine trusts only g1,
    // so its own side of the handshake refuses the g2 leaf.
    let key = g1.machine_key(2);
    let old = TeamAuth::new(&key, &g1.leaf(TeamRole::Robot, &key), g1.ca_der(), &[])
        .expect("g1 credentials");
    for link in LINKS {
        let (robot, stale) = pair(link, g2.machine(TeamRole::Robot, 1), old.clone());
        assert!(
            !linked(&stale, Duration::from_secs(3)),
            "{link}: a g1-only machine never links to a g2 robot"
        );
        assert_eq!(
            deliver(&robot, &stale, "/team/telemetry", Duration::from_secs(1)),
            0,
            "{link}"
        );
    }
}

#[test]
fn plaintext_locators_fail_the_open_naming_them() {
    let acme = TestTeam::new("acme", 1);
    for locator in ["tcp/127.0.0.1:7999", "udp/127.0.0.1:7999?rel=1;mixed_rel=1"] {
        for (listen, connect) in [
            (vec![locator.to_string()], vec![]),
            (vec![], vec![locator.to_string()]),
        ] {
            let m = manager(acme.machine(TeamRole::Robot, 1), listen, connect);
            let err = m.session().expect_err(locator).to_string();
            assert!(err.contains(locator), "the refusal names {locator}: {err}");
            assert!(err.contains("quic/<host>:<port>"), "{err}");
        }
    }
}

#[test]
fn team_credentials_with_certificate_files_fail_the_open() {
    let acme = TestTeam::new("acme", 1);
    let m = NetworkManager::new(NetworkConfig {
        listen_endpoints: vec![format!("quic/127.0.0.1:{}", free_port())],
        auth: Some(NetworkAuth::Team(acme.machine(TeamRole::Robot, 1))),
        tls: Some(NetworkTls {
            root_ca: Some("/c/ca.pem".into()),
            ..NetworkTls::default()
        }),
        ..NetworkConfig::default()
    });
    let err = m.session().expect_err("both set").to_string();
    assert!(err.contains("remove the graph tls: block"), "{err}");
}

#[test]
fn no_key_material_in_debug_or_serialized_config() {
    let acme = TestTeam::new("acme", 1);
    let key = acme.machine_key(1);
    let auth = acme.machine(TeamRole::Robot, 1);
    let cfg = NetworkConfig {
        listen_endpoints: vec!["quic/127.0.0.1:7999".to_string()],
        auth: Some(NetworkAuth::Team(auth)),
        ..NetworkConfig::default()
    };
    let m = NetworkManager::new(cfg.clone());
    let zcfg = m.zenoh_config_for_test().expect("config builds");
    let seed_hex: String = key.seed().iter().map(|b| format!("{b:02x}")).collect();
    let texts = [
        format!("{cfg:?}"),
        format!("{zcfg:?}"),
        format!("{zcfg}"),
        serde_json::to_string(&zcfg).unwrap(),
    ];
    for text in &texts {
        assert!(!text.contains("PRIVATE KEY"), "{text}");
        assert!(
            !text.contains("LS0tLS1CRUdJTiBQUklWQVRF"),
            "base64 of a PEM key header"
        );
        assert!(!text.contains(&seed_hex));
    }
    // The access policy is in the config, keyed on exact common names.
    assert!(
        texts[3].contains("robot.g1.acme.cerulion.invalid"),
        "{}",
        texts[3]
    );
}

/// The full gateway path over team links: a viewer's ingress gateway demands a
/// robot topic (liveliness token and demand query), the robot's egress gateway
/// enables it, taps the local publisher and forwards, and the viewer's local
/// subscriber receives every frame byte for byte.
#[test]
fn a_viewer_gateway_pulls_a_robot_topic_through_the_demand_path() {
    use std::sync::atomic::Ordering;
    use std::sync::Arc;

    use cerulion_core::clock::{Clock, VirtualClock};
    use cerulion_core::message::ShmMessage;
    use cerulion_core::transport::gateway::{
        GatewayEgressPolicy, GatewayIngressEntry, GatewayPlan, GatewayRuntime,
    };
    use cerulion_core::transport::{TransportConfig, TransportManager};
    use cerulion_core::wire::MaxSliceLen;
    use native_ros2_messages::geometry_msgs::Vector3;

    let acme = TestTeam::new("acme", 1);
    for (n, link) in LINKS.into_iter().enumerate() {
        let topic = format!("/team/{link}/pose_{}", std::process::id());
        let locator = format!("{link}/127.0.0.1:{}", free_port());
        let root_a = cerulion_core::testing::iceoryx_test_config();
        let clock = Arc::new(VirtualClock::new());
        let clock_dyn: Arc<dyn Clock> = clock.clone();
        let producer = TransportManager::init_for_test(
            TransportConfig {
                node_name: format!("team_p_{link}_{n}"),
                clock: clock_dyn,
                ..Default::default()
            },
            root_a.clone(),
        )
        .expect("init producer");
        let robot = TransportManager::init_for_test(
            TransportConfig {
                node_name: format!("team_robot_{link}_{n}"),
                network: Some(NetworkConfig {
                    listen_endpoints: vec![locator.clone()],
                    robot_identity: Some("team-robot".to_string()),
                    auth: Some(NetworkAuth::Team(acme.machine(TeamRole::Robot, 1))),
                    ..NetworkConfig::default()
                }),
                ..Default::default()
            },
            root_a,
        )
        .expect("init robot gateway");
        let mut robot_gw = GatewayRuntime::new(
            robot,
            GatewayPlan {
                egress_policy: GatewayEgressPolicy::AllowAll,
                announce: vec![topic.clone()],
                ingress: vec![],
            },
        )
        .expect("robot gateway boots");
        let mut publisher = producer
            .create_publisher_simple(&topic, MaxSliceLen::const_new(256))
            .expect("producer publisher");

        let viewer = TransportManager::init_for_test(
            TransportConfig {
                node_name: format!("team_viewer_{link}_{n}"),
                network: Some(NetworkConfig {
                    connect_endpoints: vec![locator],
                    robot_identity: Some("team-viewer".to_string()),
                    auth: Some(NetworkAuth::Team(acme.machine(TeamRole::Viewer, 2))),
                    ..NetworkConfig::default()
                }),
                ..Default::default()
            },
            cerulion_core::testing::iceoryx_test_config(),
        )
        .expect("init viewer gateway");
        let sub = viewer.create_subscriber(&topic).expect("viewer subscriber");
        let mut viewer_gw = GatewayRuntime::new(
            viewer,
            GatewayPlan {
                egress_policy: GatewayEgressPolicy::AllowList(vec![]),
                announce: vec![],
                ingress: vec![GatewayIngressEntry {
                    topic: topic.clone(),
                    schema_hash: Vector3::SCHEMA_HASH,
                }],
            },
        )
        .expect("viewer gateway boots");
        let flag = robot_gw
            .manager()
            .bridge_manager()
            .register_topic(&topic)
            .expect("robot bridge flag");

        let deadline = Instant::now() + Duration::from_secs(20);
        while !flag.load(Ordering::Relaxed) {
            assert!(
                Instant::now() < deadline,
                "{link}: the viewer's demand never enabled egress"
            );
            viewer_gw.demand_get_once().expect("demand pass");
            robot_gw.drive_once().expect("robot drive");
            std::thread::sleep(Duration::from_millis(20));
        }
        while robot_gw.active_tap_topics().is_empty() {
            assert!(Instant::now() < deadline, "{link}: tap did not attach");
            robot_gw.drive_once().expect("robot drive");
            std::thread::sleep(Duration::from_millis(10));
        }
        let plan = [
            (10_000_000u64, 1.25f64),
            (20_000_000, 2.5),
            (30_000_000, 3.75),
        ];
        for (ts, x) in plan {
            clock.set(ts);
            let mut proxy = publisher.loan_proxy::<Vector3>().expect("loan");
            proxy.x = x;
            proxy.y = -x;
            proxy.z = 2.0 * x;
        }
        let mut got: Vec<(u64, f64)> = Vec::new();
        while got.len() < plan.len() && Instant::now() < deadline {
            viewer_gw.demand_get_once().expect("demand pass");
            robot_gw.drive_once().expect("robot drive");
            sub.try_receive(|msg| {
                let p = msg.payload();
                let x = f64::from_le_bytes(p[..8].try_into().unwrap());
                got.push((msg.header().timestamp_ns, x));
            })
            .expect("viewer receive");
            std::thread::sleep(Duration::from_millis(20));
        }
        assert_eq!(
            got,
            plan.to_vec(),
            "{link}: every frame arrives through the team link"
        );
    }
}
