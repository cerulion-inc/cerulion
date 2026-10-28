// SPDX-License-Identifier: AGPL-3.0-only
//! Team sessions end to end on loopback, over `quic/` and over `tls/`: two or
//! three `NetworkManager`s holding test team credentials
//! (`cerulion_core::transport::test_ca`), one listening and the others
//! connecting, each with the generated access policy.
//!
//! What each test pins, on both links:
//! - a robot's frames reach a viewer;
//! - a viewer's frames never reach a robot, and an operator's do; a robot
//!   that routes puts to every role still drops a viewer's put on ingress;
//! - a viewer's demand liveliness token and its query on `cerulion_q/**`
//!   reach the robot, and the robot's reply comes back;
//! - a machine whose leaf comes from another team's CA never links, while a
//!   team machine links to the same listener;
//! - a machine of the previous generation fails the rotated robot's data
//!   session, and on its bundle session (with the next CA) gets the bundle
//!   exchange only;
//! - a leaf a retired CA signed with a current-generation robot name fails the
//!   data session, while a data session that anchors the retired CA admits it;
//! - a viewer gateway pulls a robot topic, and a robot gateway that dials an
//!   operator pulls the operator's topic, through the full demand path;
//! - a `tcp/`, `udp/...?rel=1` or `#`-config locator fails the open naming
//!   the locator, and team credentials together with certificate files fail
//!   the open;
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

/// A manager listening on a fresh loopback `link` locator, its session open,
/// and that locator.
fn listen(link: &str, auth: TeamAuth) -> (NetworkManager, String) {
    let locator = format!("{link}/127.0.0.1:{}", free_port());
    let m = manager(auth, vec![locator.clone()], vec![]);
    m.session().expect("the listener opens");
    (m, locator)
}

/// A listening `server` and a `client` connecting to it over `link`.
fn pair(link: &str, server: TeamAuth, client: TeamAuth) -> (NetworkManager, NetworkManager) {
    let (s, locator) = listen(link, server);
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
            deliver(&viewer, &robot, "/team/cmd_vel", Duration::from_secs(5)),
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

/// A robot session with the generated policy plus one extra rule: it
/// declares its subscribers to every role, so a peer of any role routes puts
/// to it, and only the robot's ingress rules decide what arrives.
fn robot_declaring_to_all(link: &str, auth: TeamAuth) -> (zenoh::Session, String) {
    let locator = format!("{link}/127.0.0.1:{}", free_port());
    let mut acl = auth.access_control();
    acl["rules"]
        .as_array_mut()
        .unwrap()
        .push(serde_json::json!({
            "id": "test_declare_to_all",
            "permission": "allow",
            "flows": ["egress"],
            "messages": ["declare_subscriber"],
            "key_exprs": ["cerulion_w2/**"],
        }));
    for policy in acl["policies"].as_array_mut().unwrap() {
        policy["rules"]
            .as_array_mut()
            .unwrap()
            .push("test_declare_to_all".into());
    }
    let m = manager(auth, vec![locator.clone()], vec![]);
    let mut config = m.zenoh_config_for_test().expect("config builds");
    config
        .insert_json5("access_control", &acl.to_string())
        .expect("policy set");
    (zenoh::open(config).wait().expect("robot opens"), locator)
}

/// Frames a raw session holding `auth`'s credentials, with its own policy
/// off, puts to `robot` within `budget`.
fn raw_put_count(robot: &zenoh::Session, auth: TeamAuth, locator: &str, budget: Duration) -> usize {
    let m = manager(auth, vec![], vec![locator.to_string()]);
    let mut config = m.zenoh_config_for_test().expect("config builds");
    config
        .insert_json5("access_control/enabled", "false")
        .expect("policy off");
    let raw = zenoh::open(config).wait().expect("raw session opens");
    let (tx, rx) = mpsc::channel();
    let key = NetworkManager::data_key("/team/cmd_vel");
    let _sub = robot
        .declare_subscriber(key.clone())
        .callback(move |_| {
            let _ = tx.send(());
        })
        .wait()
        .unwrap();
    let deadline = Instant::now() + budget;
    let mut got = 0;
    while Instant::now() < deadline && got == 0 {
        raw.put(key.clone(), frame(0)).wait().unwrap();
        while rx.recv_timeout(Duration::from_millis(50)).is_ok() {
            got += 1;
        }
    }
    got
}

#[test]
fn a_robot_drops_a_viewer_put_on_ingress() {
    let acme = TestTeam::new("acme", 1);
    for link in LINKS {
        let (robot, locator) = robot_declaring_to_all(link, acme.machine(TeamRole::Robot, 1));
        assert!(
            raw_put_count(
                &robot,
                acme.machine(TeamRole::Operator, 3),
                &locator,
                Duration::from_secs(10)
            ) > 0,
            "{link}: an operator's raw put arrives"
        );
        assert_eq!(
            raw_put_count(
                &robot,
                acme.machine(TeamRole::Viewer, 2),
                &locator,
                Duration::from_secs(10)
            ),
            0,
            "{link}: the robot's ingress rules drop a viewer's put it routed to"
        );
    }
}

#[test]
fn a_machine_of_another_team_never_links() {
    let acme = TestTeam::new("acme", 1);
    let rival = TestTeam::new("rival", 1);
    for link in LINKS {
        let (robot, locator) = listen(link, acme.machine(TeamRole::Robot, 1));
        let member = manager(
            acme.machine(TeamRole::Viewer, 2),
            vec![],
            vec![locator.clone()],
        );
        assert!(
            linked(&member, Duration::from_secs(10)),
            "{link}: a team machine links to the listener"
        );
        let outsider = manager(rival.machine(TeamRole::Owner, 2), vec![], vec![locator]);
        assert!(
            !linked(&outsider, Duration::from_secs(3)),
            "{link}: a leaf from another team's CA never links"
        );
        drop(robot);
    }
}

fn bundle_manager(auth: TeamAuth, listen: Vec<String>, connect: Vec<String>) -> NetworkManager {
    NetworkManager::new(NetworkConfig {
        listen_endpoints: listen,
        connect_endpoints: connect,
        auth: Some(NetworkAuth::TeamBundleLane(auth)),
        ..NetworkConfig::default()
    })
}

fn raw_linked(session: &zenoh::Session, budget: Duration) -> bool {
    let deadline = Instant::now() + budget;
    while Instant::now() < deadline {
        if session.info().peers_zid().wait().next().is_some() {
            return true;
        }
        std::thread::sleep(Duration::from_millis(20));
    }
    false
}

#[test]
fn the_previous_generation_gets_the_bundle_session_only() {
    let g1 = TestTeam::new("acme", 1);
    let g2 = g1.rotated();
    let key = g1.machine_key(2);
    let leaf = g1.leaf(TeamRole::Robot, &key);
    let next = g1.next_ca_der();
    let holds_next = TeamAuth::new(&key, &leaf, g1.anchors(Some(&next))).expect("g1 credentials");
    let g1_only = TeamAuth::new(&key, &leaf, g1.anchors(None)).expect("g1 credentials");
    for link in LINKS {
        let robot_auth = g2.machine(TeamRole::Robot, 1);
        let (robot, data_locator) = listen(link, robot_auth.clone());
        let lane_locator = format!("{link}/127.0.0.1:{}", free_port());
        let lane = bundle_manager(robot_auth, vec![lane_locator.clone()], vec![]);
        let ls = lane.session().expect("the bundle listener opens");
        let _bundle = ls
            .declare_queryable("cerulion_auth/bundle")
            .callback(|q| {
                let _ = q.reply(q.key_expr().clone(), "bundle").wait();
            })
            .wait()
            .unwrap();
        let _catalog = ls
            .declare_queryable("cerulion_q/robot1/catalog")
            .callback(|q| {
                let _ = q.reply(q.key_expr().clone(), "catalog").wait();
            })
            .wait()
            .unwrap();

        let old_data = manager(holds_next.clone(), vec![], vec![data_locator]);
        assert!(
            !linked(&old_data, Duration::from_secs(3)),
            "{link}: the robot's data session does not trust the retired g1 CA"
        );
        let err = bundle_manager(g1_only.clone(), vec![], vec![lane_locator.clone()])
            .session()
            .expect_err("no lane anchors")
            .to_string();
        assert!(
            err.contains("needs a retired or next generation CA"),
            "{err}"
        );

        let old = bundle_manager(holds_next.clone(), vec![], vec![lane_locator]);
        assert!(
            linked(&old, Duration::from_secs(10)),
            "{link}: the bundle session links"
        );
        let os = old.session().unwrap();
        let replies = |key: &str| {
            let deadline = Instant::now() + Duration::from_secs(3);
            let mut n = 0;
            while Instant::now() < deadline && n == 0 {
                n += os
                    .get(key)
                    .timeout(Duration::from_millis(300))
                    .wait()
                    .unwrap()
                    .iter()
                    .filter(|r| r.result().is_ok())
                    .count();
            }
            n
        };
        assert!(
            replies("cerulion_auth/bundle") > 0,
            "{link}: the bundle query is answered"
        );
        assert_eq!(
            replies("cerulion_q/robot1/catalog"),
            0,
            "{link}: a catalog query is not, even on the robot's bundle session"
        );
        assert_eq!(
            deliver(&lane, &old, "/team/telemetry", Duration::from_secs(3)),
            0,
            "{link}: no frames cross the bundle session"
        );
        drop(robot);
    }
}

/// A zenoh session holding `auth`'s key with `leaf_der` presented in place
/// of its own leaf and its own policy off, dialing `locator`.
fn forged_dialer(auth: TeamAuth, leaf_der: &[u8], locator: &str) -> zenoh::Session {
    forged_session(auth, leaf_der, vec![], vec![locator.to_string()])
}

/// [`forged_dialer`] with any listen and connect locators.
fn forged_session(
    auth: TeamAuth,
    leaf_der: &[u8],
    listen: Vec<String>,
    connect: Vec<String>,
) -> zenoh::Session {
    let m = manager(auth, listen, connect);
    let mut config = m.zenoh_config_for_test().expect("config builds");
    let leaf = format!("\"{}\"", TestTeam::pem_base64("CERTIFICATE", &[leaf_der]));
    for k in ["listen_certificate_base64", "connect_certificate_base64"] {
        config
            .insert_json5(&format!("transport/link/tls/{k}"), &leaf)
            .unwrap();
    }
    config
        .insert_json5("access_control/enabled", "false")
        .unwrap();
    zenoh::open(config).wait().expect("forged dialer opens")
}

/// Count frames `from` puts that reach a subscriber on `to` within `budget`.
fn raw_deliver(from: &zenoh::Session, to: &zenoh::Session, budget: Duration) -> usize {
    let key = NetworkManager::data_key("/team/cmd_vel");
    let (tx, rx) = mpsc::channel();
    let _sub = to
        .declare_subscriber(key.clone())
        .callback(move |_| {
            let _ = tx.send(());
        })
        .wait()
        .unwrap();
    let deadline = Instant::now() + budget;
    let mut got = 0;
    while Instant::now() < deadline && got == 0 {
        from.put(key.clone(), frame(0)).wait().unwrap();
        while rx.recv_timeout(Duration::from_millis(50)).is_ok() {
            got += 1;
        }
    }
    got
}

#[test]
fn a_retired_ca_cannot_sign_its_way_into_the_data_session() {
    let g1 = TestTeam::new("acme", 1);
    let g2 = g1.rotated();
    // The holder of the retired g1 CA key mints a leaf naming the current
    // generation's robot role, over a key of its own.
    let attacker_key = g2.machine_key(7);
    let forged = g1.forged_leaf(2, TeamRole::Robot, &attacker_key);
    for link in LINKS {
        let robot_auth = g2.machine(TeamRole::Robot, 1);
        let (robot, locator) = listen(link, robot_auth.clone());
        let attacker = forged_dialer(g2.machine(TeamRole::Viewer, 7), &forged, &locator);
        assert!(
            !raw_linked(&attacker, Duration::from_secs(3)),
            "{link}: the data session refuses a leaf a retired CA signed"
        );
        assert_eq!(
            raw_deliver(&attacker, robot.session().unwrap(), Duration::from_secs(2)),
            0,
            "{link}"
        );

        // Control: the same robot session with the retired CA added back to
        // its anchors admits the forged leaf as a robot, and its put arrives.
        let exposed_locator = format!("{link}/127.0.0.1:{}", free_port());
        let m = manager(robot_auth, vec![exposed_locator.clone()], vec![]);
        let mut config = m.zenoh_config_for_test().unwrap();
        let roots = format!(
            "\"{}\"",
            TestTeam::pem_base64(
                "CERTIFICATE",
                &[g2.ca_der(), &g2.next_ca_der(), g1.ca_der()]
            )
        );
        config
            .insert_json5("transport/link/tls/root_ca_certificate_base64", &roots)
            .unwrap();
        let exposed = zenoh::open(config).wait().expect("exposed robot opens");
        let attacker = forged_dialer(g2.machine(TeamRole::Viewer, 7), &forged, &exposed_locator);
        assert!(
            raw_linked(&attacker, Duration::from_secs(10)),
            "{link}: with the retired CA as an anchor the forged leaf links"
        );
        assert!(
            raw_deliver(&attacker, &exposed, Duration::from_secs(10)) > 0,
            "{link}: and its puts reach the robot as a robot's"
        );

        // The other direction: a forged listener that a robot's data session
        // dials is refused by the dialer's anchors, and accepted by a dialer
        // whose anchors include the retired CA.
        let forged_locator = format!("{link}/127.0.0.1:{}", free_port());
        let forged_listener = forged_session(
            g2.machine(TeamRole::Viewer, 7),
            &forged,
            vec![forged_locator.clone()],
            vec![],
        );
        let dialer = manager(
            g2.machine(TeamRole::Robot, 1),
            vec![],
            vec![forged_locator.clone()],
        );
        assert!(
            !linked(&dialer, Duration::from_secs(3)),
            "{link}: a robot's data session refuses a listener a retired CA signed"
        );
        let m = manager(g2.machine(TeamRole::Robot, 1), vec![], vec![forged_locator]);
        let mut config = m.zenoh_config_for_test().unwrap();
        config
            .insert_json5("transport/link/tls/root_ca_certificate_base64", &roots)
            .unwrap();
        let exposed_dialer = zenoh::open(config).wait().expect("exposed dialer opens");
        assert!(
            raw_linked(&exposed_dialer, Duration::from_secs(10)),
            "{link}: a dialer anchoring the retired CA accepts it"
        );
        drop(forged_listener);
    }
}

#[test]
fn plaintext_locators_fail_the_open_naming_them() {
    let acme = TestTeam::new("acme", 1);
    for locator in [
        "tcp/127.0.0.1:7999",
        "udp/127.0.0.1:7999?rel=1;mixed_rel=1",
        "tls/127.0.0.1:7999#enable_mtls=false",
    ] {
        for (listen, connect) in [
            (vec![locator.to_string()], vec![]),
            (vec![], vec![locator.to_string()]),
        ] {
            let m = manager(acme.machine(TeamRole::Robot, 1), listen, connect);
            let err = m.session().expect_err(locator).to_string();
            assert!(err.contains(locator), "the refusal names {locator}: {err}");
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
    use base64::Engine as _;
    let seed = *acme.machine_key(1).seed();
    let auth = acme.machine(TeamRole::Robot, 1);
    let secret = auth.private_key_base64_for_test().to_string();
    let b64 = base64::engine::general_purpose::STANDARD;
    let seed_hex: String = seed.iter().map(|b| format!("{b:02x}")).collect();
    let mut pkcs8 = vec![
        0x30, 0x2e, 0x02, 0x01, 0x00, 0x30, 0x05, 0x06, 0x03, 0x2b, 0x65, 0x70, 0x04, 0x22, 0x04,
        0x20,
    ];
    pkcs8.extend_from_slice(&seed);
    let forms = [
        secret.clone(),
        b64.encode(seed),
        b64.encode(&pkcs8),
        seed_hex,
    ];
    // The PEM key text starts "-----BEGIN PRIVATE KEY-----", whose base64 starts so.
    assert!(secret.starts_with("LS0tLS1CRUdJTiBQUklWQVRF"));
    let cfg = NetworkConfig {
        listen_endpoints: vec!["quic/127.0.0.1:7999".to_string()],
        auth: Some(NetworkAuth::Team(auth)),
        ..NetworkConfig::default()
    };
    let m = NetworkManager::new(cfg.clone());
    let zcfg = m.zenoh_config_for_test().expect("config builds");
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
        for form in &forms {
            assert!(!text.contains(form.as_str()), "key material in {text}");
        }
    }
    // The access policy is in the config, keyed on exact common names.
    assert!(
        texts[3].contains("robot.g1.acme.cerulion.invalid"),
        "{}",
        texts[3]
    );
}

/// One gateway pair over a team link: a producer machine whose egress
/// gateway announces `topic` from a local publisher, and a consumer machine
/// whose ingress gateway demands it. `producer_listens` picks which side
/// listens and which dials. Returns the frames the consumer's local subscriber
/// received, as (timestamp, x) pairs, after the producer published three.
fn gateway_flow(
    link: &str,
    tag: &str,
    producer_auth: TeamAuth,
    consumer_auth: TeamAuth,
    producer_listens: bool,
) -> Vec<(u64, f64)> {
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

    let topic = format!("/team/{link}/{tag}_{}", std::process::id());
    let locator = format!("{link}/127.0.0.1:{}", free_port());
    let (producer_net, consumer_net) = if producer_listens {
        ((vec![locator.clone()], vec![]), (vec![], vec![locator]))
    } else {
        ((vec![], vec![locator.clone()]), (vec![locator], vec![]))
    };
    let root_a = cerulion_core::testing::iceoryx_test_config();
    let clock = Arc::new(VirtualClock::new());
    let clock_dyn: Arc<dyn Clock> = clock.clone();
    let producer = TransportManager::init_for_test(
        TransportConfig {
            node_name: format!("team_p_{link}_{tag}"),
            clock: clock_dyn,
            ..Default::default()
        },
        root_a.clone(),
    )
    .expect("init producer");
    let net = |(listen, connect): (Vec<String>, Vec<String>), robot: &str, auth| NetworkConfig {
        listen_endpoints: listen,
        connect_endpoints: connect,
        robot_identity: Some(robot.to_string()),
        auth: Some(NetworkAuth::Team(auth)),
        ..NetworkConfig::default()
    };
    // The consumer comes up first; a dialing side retries its connect in the
    // background until the listener is up.
    let consumer = TransportManager::init_for_test(
        TransportConfig {
            node_name: format!("team_c_{link}_{tag}"),
            network: Some(net(consumer_net, &format!("{tag}-consumer"), consumer_auth)),
            ..Default::default()
        },
        cerulion_core::testing::iceoryx_test_config(),
    )
    .expect("init consumer gateway");
    let sub = consumer
        .create_subscriber(&topic)
        .expect("consumer subscriber");
    let gateway = TransportManager::init_for_test(
        TransportConfig {
            node_name: format!("team_g_{link}_{tag}"),
            network: Some(net(producer_net, &format!("{tag}-producer"), producer_auth)),
            ..Default::default()
        },
        root_a,
    )
    .expect("init producer gateway");
    let mut producer_gw = GatewayRuntime::new(
        gateway,
        GatewayPlan {
            egress_policy: GatewayEgressPolicy::AllowAll,
            announce: vec![topic.clone()],
            ingress: vec![],
        },
    )
    .expect("producer gateway boots");
    let mut publisher = producer
        .create_publisher_simple(&topic, MaxSliceLen::const_new(256))
        .expect("producer publisher");
    let mut consumer_gw = GatewayRuntime::new(
        consumer,
        GatewayPlan {
            egress_policy: GatewayEgressPolicy::AllowList(vec![]),
            announce: vec![],
            ingress: vec![GatewayIngressEntry {
                topic: topic.clone(),
                schema_hash: Vector3::SCHEMA_HASH,
            }],
        },
    )
    .expect("consumer gateway boots");
    let flag = producer_gw
        .manager()
        .bridge_manager()
        .register_topic(&topic)
        .expect("producer bridge flag");

    let deadline = Instant::now() + Duration::from_secs(20);
    while !flag.load(Ordering::Relaxed) && Instant::now() < deadline {
        consumer_gw.demand_get_once().expect("demand pass");
        producer_gw.drive_once().expect("producer drive");
        std::thread::sleep(Duration::from_millis(20));
    }
    while flag.load(Ordering::Relaxed)
        && producer_gw.active_tap_topics().is_empty()
        && Instant::now() < deadline
    {
        producer_gw.drive_once().expect("producer drive");
        std::thread::sleep(Duration::from_millis(10));
    }
    for (ts, x) in GATEWAY_PLAN {
        clock.set(ts);
        let mut proxy = publisher.loan_proxy::<Vector3>().expect("loan");
        proxy.x = x;
        proxy.y = -x;
        proxy.z = 2.0 * x;
    }
    let mut got = Vec::new();
    while got.len() < GATEWAY_PLAN.len() && Instant::now() < deadline {
        consumer_gw.demand_get_once().expect("demand pass");
        producer_gw.drive_once().expect("producer drive");
        sub.try_receive(|msg| {
            let p = msg.payload();
            let x = f64::from_le_bytes(p[..8].try_into().unwrap());
            got.push((msg.header().timestamp_ns, x));
        })
        .expect("consumer receive");
        std::thread::sleep(Duration::from_millis(20));
    }
    got
}

const GATEWAY_PLAN: [(u64, f64); 3] = [(10_000_000, 1.25), (20_000_000, 2.5), (30_000_000, 3.75)];

/// A viewer's ingress gateway dials a listening robot and pulls a robot topic
/// through the demand path (liveliness token and demand query).
#[test]
fn a_viewer_gateway_pulls_a_robot_topic_through_the_demand_path() {
    let acme = TestTeam::new("acme", 1);
    for link in LINKS {
        let got = gateway_flow(
            link,
            "robot_to_viewer",
            acme.machine(TeamRole::Robot, 1),
            acme.machine(TeamRole::Viewer, 2),
            true,
        );
        assert_eq!(got, GATEWAY_PLAN.to_vec(), "{link}: every frame arrives");
    }
}

/// A robot's ingress gateway dials a listening operator and pulls the
/// operator's topic: the writer rules, and the policy of a dialing robot.
#[test]
fn a_robot_gateway_dials_an_operator_and_pulls_its_topic() {
    let acme = TestTeam::new("acme", 1);
    for link in LINKS {
        let got = gateway_flow(
            link,
            "operator_to_robot",
            acme.machine(TeamRole::Operator, 3),
            acme.machine(TeamRole::Robot, 1),
            true,
        );
        assert_eq!(got, GATEWAY_PLAN.to_vec(), "{link}: every frame arrives");
    }
}
