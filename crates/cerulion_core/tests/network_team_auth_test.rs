// SPDX-License-Identifier: AGPL-3.0-only
//! Team sessions end to end on loopback, over `quic/` and over `tls/`: two or
//! three `NetworkManager`s holding test team credentials
//! (`cerulion_core::transport::test_ca`), one listening and the others
//! connecting, each with the generated access policy.
//!
//! What each test pins, on both links:
//! - a robot's frames reach a viewer;
//! - a robot declares no subscriber to a viewer and a viewer's frames never
//!   reach it, while an operator's do; a robot that declares its subscribers
//!   to every role still drops a viewer's put on ingress;
//! - a viewer's demand liveliness token and its query on `cerulion_q/**`
//!   reach the robot, and the robot's reply comes back;
//! - a machine whose leaf comes from another team's CA never links, while a
//!   team machine links to the same listener;
//! - a machine of the previous generation fails the rotated robot's data
//!   session, and on its bundle session (with the next CA) gets the bundle
//!   exchange only;
//! - a leaf a retired CA signed with a current-generation robot name fails the
//!   data session, while a data session that anchors the retired CA admits it;
//! - a leaf the next generation's CA signed with a current-generation robot
//!   name fails the data session in both directions, and links but has every
//!   put dropped on the cross-generation session, while a genuine
//!   next-generation robot exchanges frames with a current one over the cross
//!   sessions;
//! - a viewer gateway pulls a robot topic, and a robot gateway that dials an
//!   operator pulls the operator's topic, through the full demand path;
//! - a `tcp/`, `udp/...?rel=1` or `#`-config locator fails the open naming
//!   the locator, and team credentials together with certificate files fail
//!   the open;
//! - the config's `Debug` and serialized forms carry no key material.
//!
//! A refused handshake is judged by the handshake itself: the dialing side
//! tries it once inside its open ([`dial_once`]), the open succeeds, and the
//! session holds no peer as the open returns, beside a dial to the same
//! listener that holds one. A dropped put is judged by an in-order sentinel
//! ([`puts_then_query`]): the sender puts once it holds the receiver's
//! declarations, then sends a query the receiver admits on the same session,
//! and only the query arrives, beside a sender whose frames arrive ahead of
//! its query.
//!
//! A session that dials a listener drops before the listener does (locals
//! drop in reverse order of binding), so it closes while its peer is up:
//! closing a `quic/` session whose peer has already closed can wait out
//! zenoh's 10 s default session close timeout.

use std::sync::atomic::AtomicU64;
use std::sync::mpsc;
use std::time::{Duration, Instant};

use cerulion_core::transport::team_auth::access_control;
use cerulion_core::transport::test_ca::TestTeam;
use cerulion_core::{NetworkAuth, NetworkConfig, NetworkManager, NetworkTls, TeamAuth, WireHeader};
use cerulion_pairing::team::TeamRole;
use zenoh::qos::CongestionControl;
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
    let manager = NetworkManager::new(NetworkConfig {
        listen_endpoints: listen,
        connect_endpoints: connect,
        auth: Some(NetworkAuth::Team(auth)),
        ..NetworkConfig::default()
    });
    manager.set_lan_access(cerulion_core::LanAccess::Open);
    manager
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

/// The zenoh config of a data session holding `auth` that dials `locator`.
fn dialer_config(auth: TeamAuth, locator: &str) -> zenoh::Config {
    manager(auth, vec![], vec![locator.to_string()])
        .zenoh_config_for_test()
        .expect("config builds")
}

/// Open `config` with each connect locator tried once inside the open, and
/// report whether the session holds a peer as the open returns.
///
/// `connect/retry` with `period_init_ms: 0` takes zenoh's single-attempt
/// branch: each handshake runs to its end before the open returns, and no
/// background connector dials again. `open/return_conditions/connect_scouted:
/// false` returns without waiting out the scouting delay. Every other key is
/// the session's own, `connect/exit_on_failure` included, so the open itself
/// must succeed whether the peer accepts the handshake or refuses it. With no
/// listen locator and scouting off, a session that holds no peer here never
/// gets one.
fn dial_once(mut config: zenoh::Config, what: &str) -> (zenoh::Session, bool) {
    config
        .insert_json5("connect/retry", "{ period_init_ms: 0 }")
        .expect("connect retry set");
    config
        .insert_json5("open/return_conditions/connect_scouted", "false")
        .expect("return condition set");
    let session = zenoh::open(config).wait().unwrap_or_else(|e| {
        panic!("{what}: the open succeeds whether the peer accepts or refuses: {e}")
    });
    let peer = session.info().peers_zid().wait().next().is_some();
    (session, peer)
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

/// A sending machine: its zenoh session, and its put of one frame on a topic.
trait Sender {
    fn zenoh_session(&self) -> &zenoh::Session;
    fn put_frame(&self, topic: &str, frame: Vec<u8>);
}

impl Sender for NetworkManager {
    fn zenoh_session(&self) -> &zenoh::Session {
        self.session().expect("sender session opens")
    }

    fn put_frame(&self, topic: &str, frame: Vec<u8>) {
        self.publish_to_network(topic, frame)
            .expect("the put itself never fails on a live session");
    }
}

impl Sender for zenoh::Session {
    fn zenoh_session(&self) -> &zenoh::Session {
        self
    }

    /// The put `publish_to_network` makes for a reliable topic.
    fn put_frame(&self, topic: &str, frame: Vec<u8>) {
        self.put(NetworkManager::data_key(topic), frame)
            .congestion_control(CongestionControl::Block)
            .wait()
            .expect("the put itself never fails on a live session");
    }
}

/// The frames [`puts_then_query`] puts before its query.
const FRAMES: u64 = 3;

/// The prefix of the keys [`puts_then_query`] queries: under `cerulion_q/**`,
/// where the generated policy takes a query from every role.
const SENTINEL_PREFIX: &str = "cerulion_q/sentinel";

static NEXT_KEY: AtomicU64 = AtomicU64::new(0);

/// One arrival at the receiving side of [`puts_then_query`].
#[derive(Debug, PartialEq)]
enum Arrival {
    /// A frame on the data key, by its sequence number.
    Frame(u64),
    /// The query sent after the frames.
    Query,
}

/// What [`puts_then_query`] saw.
#[derive(Debug, PartialEq)]
struct Landing {
    /// The sender held a route to the receiver's subscriber when it put.
    routed: bool,
    /// What reached the receiver, in arrival order, up to the query.
    arrived: Vec<Arrival>,
}

impl Landing {
    /// Every frame routed and admitted, in order, ahead of the query.
    fn delivered() -> Landing {
        Landing {
            routed: true,
            arrived: (0..FRAMES)
                .map(Arrival::Frame)
                .chain([Arrival::Query])
                .collect(),
        }
    }

    /// The frames routed to the receiver and dropped on its ingress.
    fn dropped_on_ingress() -> Landing {
        Landing {
            routed: true,
            arrived: vec![Arrival::Query],
        }
    }

    /// No route to the receiver's subscriber, so no frame left the sender.
    fn never_routed() -> Landing {
        Landing {
            routed: false,
            arrived: vec![Arrival::Query],
        }
    }
}

/// Put [`FRAMES`] frames from `from` on a fresh data key `to` subscribes,
/// then send one query on a fresh [`SENTINEL_PREFIX`] key `to` serves, on the
/// same session, and report what reached `to` up to that query.
///
/// `to` declares its subscriber before its queryable, so the two declarations
/// reach `from` in that order on zenoh's one control-priority channel. `from`
/// puts once it sees the queryable, so a subscriber declaration `to` sent is
/// applied by then and `routed` reads the route the frames take. The frames
/// and the query both ride zenoh's data priority, reliable and blocking, so
/// they share one transmit queue and one link (on `quic/`, the data
/// priority's stream), and `to` hands each to its callback in arrival order:
/// it passes every frame through its ingress rules before the query. A
/// landing without frames is a verdict on the frames, not a window that
/// closed early; [`Landing::delivered`] holds that order on every run. Panics
/// when `from` does not see the queryable, or the query does not arrive,
/// within 10 s.
fn puts_then_query(from: &dyn Sender, to: &zenoh::Session) -> Landing {
    let n = NEXT_KEY.fetch_add(1, std::sync::atomic::Ordering::Relaxed);
    let topic = format!("/team/sentinel_{n}");
    let key = NetworkManager::data_key(&topic);
    let query_key = format!("{SENTINEL_PREFIX}/{n}");
    let (tx, rx) = mpsc::channel();
    let frames = tx.clone();
    let _sub = to
        .declare_subscriber(key.clone())
        .callback(move |s| {
            let _ = frames.send(Some(s.payload().to_bytes().into_owned()));
        })
        .wait()
        .unwrap();
    let _queryable = to
        .declare_queryable(query_key.clone())
        .callback(move |_| {
            let _ = tx.send(None);
        })
        .wait()
        .unwrap();
    let session = from.zenoh_session();
    let publisher = session.declare_publisher(key).wait().unwrap();
    let querier = session.declare_querier(query_key).wait().unwrap();
    let seen = querier.matching_listener().wait().unwrap();
    let deadline = Instant::now() + Duration::from_secs(10);
    loop {
        match seen
            .recv_deadline(deadline)
            .expect("matching listener open")
        {
            Some(status) if status.matching() => break,
            Some(_) => {}
            None => panic!("the sender sees the receiver's queryable within 10 s"),
        }
    }
    let routed = publisher.matching_status().wait().unwrap().matching();
    for seq in 0..FRAMES {
        from.put_frame(&topic, frame(seq));
    }
    querier.get().callback(|_| {}).wait().unwrap();
    let deadline = Instant::now() + Duration::from_secs(10);
    let mut arrived = Vec::new();
    loop {
        match rx.recv_timeout(deadline.saturating_duration_since(Instant::now())) {
            Ok(Some(f)) => {
                assert_eq!(f.len(), WireHeader::SIZE + 8, "the frame arrives verbatim");
                let seq = u64::from_le_bytes(f[WireHeader::SIZE..].try_into().unwrap());
                arrived.push(Arrival::Frame(seq));
            }
            Ok(None) => {
                arrived.push(Arrival::Query);
                return Landing { routed, arrived };
            }
            Err(_) => panic!("the query reaches the receiver within 10 s, after {arrived:?}"),
        }
    }
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
            puts_then_query(&viewer, robot.session().unwrap()),
            Landing::never_routed(),
            "{link}: the robot declares no subscriber to a viewer, and a viewer's frames never \
             reach it"
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
        assert_eq!(
            puts_then_query(&operator, robot.session().unwrap()),
            Landing::delivered(),
            "{link}: an operator's frames reach the robot, in order, ahead of its query"
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

/// A session holding `auth`'s credentials with its own policy off, dialing
/// `locator`.
fn raw_dialer(auth: TeamAuth, locator: &str) -> zenoh::Session {
    let mut config = dialer_config(auth, locator);
    config
        .insert_json5("access_control/enabled", "false")
        .expect("policy off");
    zenoh::open(config).wait().expect("raw session opens")
}

#[test]
fn a_robot_drops_a_viewer_put_on_ingress() {
    let acme = TestTeam::new("acme", 1);
    for link in LINKS {
        let (robot, locator) = robot_declaring_to_all(link, acme.machine(TeamRole::Robot, 1));
        let operator = raw_dialer(acme.machine(TeamRole::Operator, 3), &locator);
        assert_eq!(
            puts_then_query(&operator, &robot),
            Landing::delivered(),
            "{link}: an operator's raw puts arrive, in order, ahead of its query"
        );
        let viewer = raw_dialer(acme.machine(TeamRole::Viewer, 2), &locator);
        assert_eq!(
            puts_then_query(&viewer, &robot),
            Landing::dropped_on_ingress(),
            "{link}: the robot's ingress rules drop a viewer's puts it routed to"
        );
    }
}

#[test]
fn a_machine_of_another_team_never_links() {
    let acme = TestTeam::new("acme", 1);
    let rival = TestTeam::new("rival", 1);
    for link in LINKS {
        let (_robot, locator) = listen(link, acme.machine(TeamRole::Robot, 1));
        let (_member, peer) = dial_once(
            dialer_config(acme.machine(TeamRole::Viewer, 2), &locator),
            &format!("{link}: a team machine"),
        );
        assert!(peer, "{link}: a team machine links to the listener");
        let (_outsider, peer) = dial_once(
            dialer_config(rival.machine(TeamRole::Owner, 2), &locator),
            &format!("{link}: a machine of another team"),
        );
        assert!(!peer, "{link}: a leaf from another team's CA never links");
    }
}

fn bundle_manager(auth: TeamAuth, listen: Vec<String>, connect: Vec<String>) -> NetworkManager {
    let manager = NetworkManager::new(NetworkConfig {
        listen_endpoints: listen,
        connect_endpoints: connect,
        auth: Some(NetworkAuth::TeamBundleLane(auth)),
        ..NetworkConfig::default()
    });
    manager.set_lan_access(cerulion_core::LanAccess::Open);
    manager
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
        let (_robot, data_locator) = listen(link, robot_auth.clone());
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

        let (_member, peer) = dial_once(
            dialer_config(g2.machine(TeamRole::Viewer, 2), &data_locator),
            &format!("{link}: a g2 machine"),
        );
        assert!(
            peer,
            "{link}: a g2 machine links to the robot's data session"
        );
        let (_old_data, peer) = dial_once(
            dialer_config(holds_next.clone(), &data_locator),
            &format!("{link}: a g1 machine on the data session"),
        );
        assert!(
            !peer,
            "{link}: the robot's data session does not trust the retired g1 CA"
        );
        let err = bundle_manager(g1_only.clone(), vec![], vec![lane_locator.clone()])
            .session()
            .expect_err("no lane anchors")
            .to_string();
        assert!(
            err.contains("needs a retired CA or the next or previous generation's CA"),
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
    zenoh::open(forged_config(auth, leaf_der, listen, connect))
        .wait()
        .expect("forged dialer opens")
}

/// The zenoh config [`forged_session`] opens.
fn forged_config(
    auth: TeamAuth,
    leaf_der: &[u8],
    listen: Vec<String>,
    connect: Vec<String>,
) -> zenoh::Config {
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
    config
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
        let (_robot, locator) = listen(link, robot_auth.clone());
        let (_member, peer) = dial_once(
            dialer_config(g2.machine(TeamRole::Viewer, 2), &locator),
            &format!("{link}: a g2 machine"),
        );
        assert!(
            peer,
            "{link}: a g2 machine links to the robot's data session"
        );
        let (_attacker, peer) = dial_once(
            forged_config(
                g2.machine(TeamRole::Viewer, 7),
                &forged,
                vec![],
                vec![locator],
            ),
            &format!("{link}: the forged dialer"),
        );
        assert!(
            !peer,
            "{link}: the data session refuses a leaf a retired CA signed"
        );

        // Control: the same robot session with the retired CA added back to
        // its anchors admits the forged leaf as a robot, and its put arrives.
        let exposed_locator = format!("{link}/127.0.0.1:{}", free_port());
        let m = manager(robot_auth, vec![exposed_locator.clone()], vec![]);
        let mut config = m.zenoh_config_for_test().unwrap();
        let roots = format!(
            "\"{}\"",
            TestTeam::pem_base64("CERTIFICATE", &[g2.ca_der(), g1.ca_der()])
        );
        config
            .insert_json5("transport/link/tls/root_ca_certificate_base64", &roots)
            .unwrap();
        let exposed = zenoh::open(config).wait().expect("exposed robot opens");
        let (attacker, peer) = dial_once(
            forged_config(
                g2.machine(TeamRole::Viewer, 7),
                &forged,
                vec![],
                vec![exposed_locator],
            ),
            &format!("{link}: the forged dialer"),
        );
        assert!(
            peer,
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
        let _forged_listener = forged_session(
            g2.machine(TeamRole::Viewer, 7),
            &forged,
            vec![forged_locator.clone()],
            vec![],
        );
        let (_dialer, peer) = dial_once(
            dialer_config(g2.machine(TeamRole::Robot, 1), &forged_locator),
            &format!("{link}: a robot's data dialer"),
        );
        assert!(
            !peer,
            "{link}: a robot's data session refuses a listener a retired CA signed"
        );
        let mut config = dialer_config(g2.machine(TeamRole::Robot, 1), &forged_locator);
        config
            .insert_json5("transport/link/tls/root_ca_certificate_base64", &roots)
            .unwrap();
        let (_exposed_dialer, peer) = dial_once(config, &format!("{link}: the exposed dialer"));
        assert!(peer, "{link}: a dialer anchoring the retired CA accepts it");
    }
}

fn cross_manager(auth: TeamAuth, listen: Vec<String>, connect: Vec<String>) -> NetworkManager {
    let manager = NetworkManager::new(NetworkConfig {
        listen_endpoints: listen,
        connect_endpoints: connect,
        auth: Some(NetworkAuth::TeamCrossGeneration(auth)),
        ..NetworkConfig::default()
    });
    manager.set_lan_access(cerulion_core::LanAccess::Open);
    manager
}

/// `m`'s zenoh config with `policy` (the generated policy the caller passes) plus
/// rules for any peer: subscribers on `cerulion_w2/**` are declared to it, and
/// a [`puts_then_query`] queryable under [`SENTINEL_PREFIX`] is declared to it
/// and takes its query. A peer of any name then routes puts to the session,
/// its ingress rules decide which arrive, and the query that follows them
/// arrives.
fn declaring_to_any(m: &NetworkManager, policy: serde_json::Value) -> zenoh::Config {
    let mut config = m.zenoh_config_for_test().expect("config builds");
    let mut acl = policy;
    let sentinel = format!("{SENTINEL_PREFIX}/**");
    let rules = acl["rules"].as_array_mut().unwrap();
    rules.push(serde_json::json!({
        "id": "test_declare_to_any",
        "permission": "allow",
        "flows": ["egress"],
        "messages": ["declare_subscriber"],
        "key_exprs": ["cerulion_w2/**"],
    }));
    rules.push(serde_json::json!({
        "id": "test_sentinel_to_any",
        "permission": "allow",
        "flows": ["egress"],
        "messages": ["declare_queryable"],
        "key_exprs": [sentinel],
    }));
    rules.push(serde_json::json!({
        "id": "test_sentinel_from_any",
        "permission": "allow",
        "flows": ["ingress"],
        "messages": ["query"],
        "key_exprs": [sentinel],
    }));
    acl["subjects"]
        .as_array_mut()
        .unwrap()
        .push(serde_json::json!({ "id": "test_any" }));
    acl["policies"]
        .as_array_mut()
        .unwrap()
        .push(serde_json::json!({
            "id": "test_policy_any",
            "rules": ["test_declare_to_any", "test_sentinel_to_any", "test_sentinel_from_any"],
            "subjects": ["test_any"],
        }));
    config
        .insert_json5("access_control", &acl.to_string())
        .unwrap();
    config
}

/// The next generation's CA key mints `robot.g2` for a key of its own. A g2
/// robot's data session refuses that leaf in both directions; its
/// cross-generation session, which anchors the next CA, links it and drops
/// its puts, since `robot.g2` is not a next-generation name. A genuine g3
/// robot, rotated and holding g2 as its previous generation, exchanges
/// frames with the g2 robot over the cross sessions.
#[test]
fn a_next_ca_cannot_claim_a_current_generation_name() {
    let g2 = TestTeam::new("acme", 1).rotated();
    let g3 = g2.rotated();
    let attacker_key = g2.machine_key(7);
    let forged = g3.forged_leaf(2, TeamRole::Robot, &attacker_key);
    for link in LINKS {
        let robot = g2.machine(TeamRole::Robot, 1);

        // 1. The data listener refuses the forged leaf at the handshake.
        let (data, data_locator) = listen(link, robot.clone());
        let (_attacker, peer) = dial_once(
            forged_config(
                g2.machine(TeamRole::Viewer, 7),
                &forged,
                vec![],
                vec![data_locator.clone()],
            ),
            &format!("{link}: the forged dialer"),
        );
        assert!(
            !peer,
            "{link}: the data session refuses a leaf the next CA signed"
        );
        // Control: a genuine g2 viewer dialing the same data listener links,
        // and the robot's frames reach it.
        let (_member, peer) = dial_once(
            dialer_config(g2.machine(TeamRole::Viewer, 2), &data_locator),
            &format!("{link}: a g2 machine"),
        );
        assert!(
            peer,
            "{link}: a g2 machine links to the robot's data session"
        );
        let viewer = manager(g2.machine(TeamRole::Viewer, 2), vec![], vec![data_locator]);
        assert!(
            deliver(&data, &viewer, "/team/telemetry", Duration::from_secs(10)) > 0,
            "{link}: control, the data session carries g2 traffic"
        );

        // 2. The cross listener links the forged leaf (its chain is valid)
        // and drops its puts, even with subscribers declared to any peer.
        let cross_locator = format!("{link}/127.0.0.1:{}", free_port());
        let cross_m = cross_manager(robot.clone(), vec![cross_locator.clone()], vec![]);
        let cross = zenoh::open(declaring_to_any(
            &cross_m,
            robot.cross_access_control().unwrap(),
        ))
        .wait()
        .expect("cross session opens");
        // The forged dialer's own data anchors (g2) verify the robot's g2 leaf.
        let attacker = forged_dialer(g2.machine(TeamRole::Viewer, 7), &forged, &cross_locator);
        assert!(
            raw_linked(&attacker, Duration::from_secs(10)),
            "{link}: the cross session completes the handshake with a next-CA chain"
        );
        assert_eq!(
            puts_then_query(&attacker, &cross),
            Landing::dropped_on_ingress(),
            "{link}: and drops every put from a current-generation name"
        );
        // Control: the same forged dialer against a cross session whose
        // policy names generation 2 instead (the policy a data session has)
        // gets its puts through, so the drop above is the name mismatch.
        let wrong_locator = format!("{link}/127.0.0.1:{}", free_port());
        let wrong_m = cross_manager(robot.clone(), vec![wrong_locator.clone()], vec![]);
        let wrong = zenoh::open(declaring_to_any(&wrong_m, access_control(robot.team(), 2)))
            .wait()
            .expect("cross session opens");
        let attacker = forged_dialer(g2.machine(TeamRole::Viewer, 7), &forged, &wrong_locator);
        assert_eq!(
            puts_then_query(&attacker, &wrong),
            Landing::delivered(),
            "{link}: control, a cross session naming g2 takes the forged robot's puts"
        );
        // Control: a genuine g3 robot's raw puts reach the same cross
        // session, and the frame path works both ways over the cross pair.
        let g3_robot = g3.machine_after_rotation(TeamRole::Robot, 4);
        let g3_m = cross_manager(g3_robot.clone(), vec![], vec![cross_locator]);
        let mut g3_config = g3_m.zenoh_config_for_test().unwrap();
        g3_config
            .insert_json5("access_control/enabled", "false")
            .unwrap();
        let g3_raw = zenoh::open(g3_config).wait().unwrap();
        assert_eq!(
            puts_then_query(&g3_raw, &cross),
            Landing::delivered(),
            "{link}: control, a g3 robot's puts reach the g2 cross session"
        );
        let pair_locator = format!("{link}/127.0.0.1:{}", free_port());
        let old_side = cross_manager(robot.clone(), vec![pair_locator.clone()], vec![]);
        old_side.session().unwrap();
        let new_side = cross_manager(g3_robot, vec![], vec![pair_locator]);
        assert!(
            deliver(
                &old_side,
                &new_side,
                "/team/telemetry",
                Duration::from_secs(10)
            ) > 0,
            "{link}: g2 frames reach the rotated g3 robot"
        );
        assert!(
            deliver(
                &new_side,
                &old_side,
                "/team/cmd_vel",
                Duration::from_secs(10)
            ) > 0,
            "{link}: and g3 frames reach the g2 robot"
        );

        // 3. The other direction: a forged listener. The robot's data dialer
        // refuses it; its cross dialer links and gets no frame from it.
        let forged_locator = format!("{link}/127.0.0.1:{}", free_port());
        let forged_listener = forged_session(
            g2.machine(TeamRole::Viewer, 7),
            &forged,
            vec![forged_locator.clone()],
            vec![],
        );
        let (_data_dialer, peer) = dial_once(
            dialer_config(robot.clone(), &forged_locator),
            &format!("{link}: a robot's data dialer"),
        );
        assert!(
            !peer,
            "{link}: a robot's data dialer refuses a listener the next CA signed"
        );
        let forged_dialer_locator = forged_locator.clone();
        let cross_dialer_m = cross_manager(robot.clone(), vec![], vec![forged_locator]);
        let (cross_dialer, peer) = dial_once(
            declaring_to_any(&cross_dialer_m, robot.cross_access_control().unwrap()),
            &format!("{link}: a robot's cross dialer"),
        );
        assert!(peer, "{link}: the cross dialer completes the handshake");
        assert_eq!(
            puts_then_query(&forged_listener, &cross_dialer),
            Landing::dropped_on_ingress(),
            "{link}: and drops every put from the forged listener"
        );
        let wrong_dialer_m = cross_manager(robot.clone(), vec![], vec![forged_dialer_locator]);
        let wrong_dialer = zenoh::open(declaring_to_any(
            &wrong_dialer_m,
            access_control(robot.team(), 2),
        ))
        .wait()
        .expect("cross dialer opens");
        assert_eq!(
            puts_then_query(&forged_listener, &wrong_dialer),
            Landing::delivered(),
            "{link}: control, a cross dialer naming g2 takes the forged listener's puts"
        );
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
    m.set_lan_access(cerulion_core::LanAccess::Open);
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
    m.set_lan_access(cerulion_core::LanAccess::Open);
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
    consumer
        .network()
        .expect("network fixture")
        .set_lan_access(cerulion_core::LanAccess::Open);
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
    gateway
        .network()
        .expect("network fixture")
        .set_lan_access(cerulion_core::LanAccess::Open);
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
