// SPDX-License-Identifier: AGPL-3.0-only
//! `CERULION_UNENCRYPTED_QUIC=on` end to end on loopback, through the
//! production fold and the real QUIC probe, against real zenoh sessions:
//!
//! 1. Off: the fold is exactly today's and the probe is never called.
//! 2. On, the robot listens on both carriers: the fold dials
//!    `udp/...?rel=1`, and the zenoh link that comes up is a `udp/` link
//!    (read from the robot session's link table).
//! 3. On, nothing listens on the robot's UDP port: the probe is refused, the
//!    fold dials `tcp/`, the link is `tcp/`, and the fallback line is logged
//!    exactly once.
//! 4. On, a UDP socket on the robot's port swallows every datagram: the probe
//!    gives up at its bound and the fold falls back to `tcp/`, once.
//! 5. On, an explicit `tcp/` CONNECT locator is used as written and not probed;
//!    an explicit `udp/...?rel=1` one falls back to `tcp/` when UDP is closed.
//!
//! Nothing here touches mDNS: the browse result is handed in, the way the fold's
//! `_at` entry point exists for.
//!
//! Host precondition for arm 3: the loopback interface delivers ICMP port
//! unreachable to a connected UDP socket (Linux does; so does macOS unless its
//! application firewall runs in stealth mode, where the probe reads no answer at
//! its bound instead and the arm fails its "refused at once" assertion).
//!
//! The arms take [`serial`] so one arm's free port cannot be taken by another
//! arm between [`free_port`] and the bind that uses it.

use std::net::{TcpListener, UdpSocket};
use std::time::{Duration, Instant};

use cerulion_core::transport::network::NetworkConfig;
use cerulion_discovery::ladder::{DiscoveredPeer, DiscoveryRung};
use cerulion_discovery::unencrypted_quic::{UdpRelTwin, UnencryptedQuic};
use cerulion_netd::discovery_fold::{
    fold_discovered_peers_at, fold_discovered_peers_unencrypted_quic_at,
};
use cerulion_netd::quic_probe::{probe_quic_locators, QuicProbeOutcome};
use tracing_test::traced_test;
use zenoh::Wait;

/// Serialises the arms of this binary (see the module docs).
fn serial() -> std::sync::MutexGuard<'static, ()> {
    static LOCK: std::sync::Mutex<()> = std::sync::Mutex::new(());
    LOCK.lock().unwrap_or_else(|p| p.into_inner())
}

/// A loopback port free on both TCP and UDP right now.
fn free_port() -> u16 {
    for _ in 0..50 {
        let tcp = TcpListener::bind("127.0.0.1:0").expect("bind tcp");
        let port = tcp.local_addr().expect("addr").port();
        if UdpSocket::bind(("127.0.0.1", port)).is_ok() {
            return port;
        }
    }
    panic!("no port free on both TCP and UDP");
}

fn zenoh_peer(listen: &[String], connect: &[String]) -> zenoh::Session {
    let mut c = zenoh::Config::default();
    let json = |v: &[String]| serde_json::to_string(v).expect("json");
    c.insert_json5("mode", "\"peer\"").expect("mode");
    c.insert_json5("listen/endpoints", &json(listen))
        .expect("listen");
    c.insert_json5("connect/endpoints", &json(connect))
        .expect("connect");
    c.insert_json5("scouting/multicast/enabled", "false")
        .expect("multicast");
    c.insert_json5("scouting/gossip/enabled", "false")
        .expect("gossip");
    zenoh::open(c).wait().expect("zenoh session opens")
}

/// The robot side's link locators (`src -> dst`), once at least one link is up.
fn links_of(session: &zenoh::Session, within: Duration) -> Vec<(String, String)> {
    let deadline = Instant::now() + within;
    loop {
        let links: Vec<(String, String)> = session
            .info()
            .links()
            .wait()
            .map(|l| (l.src().to_string(), l.dst().to_string()))
            .collect();
        if !links.is_empty() || Instant::now() >= deadline {
            return links;
        }
        std::thread::sleep(Duration::from_millis(20));
    }
}

fn desk_cfg(connect: &[String]) -> NetworkConfig {
    cerulion_netd::net::desk_network_config(connect.to_vec(), Vec::new())
}

fn discovered(port: u16) -> (Vec<DiscoveredPeer>, Vec<UdpRelTwin>) {
    let tcp = format!("tcp/127.0.0.1:{port}");
    (
        vec![DiscoveredPeer {
            robot: "loopbot".to_string(),
            locator: tcp.clone(),
            rung: DiscoveryRung::Mdns,
        }],
        vec![UdpRelTwin {
            tcp,
            udp_rel: format!("udp/127.0.0.1:{port}?rel=1"),
        }],
    )
}

fn real_probe(
    timeout: Duration,
) -> impl FnOnce(&[String]) -> std::collections::HashMap<String, QuicProbeOutcome> {
    move |locators| probe_quic_locators(locators, timeout)
}

fn fallback_lines(lines: &[&str], tcp: &str) -> usize {
    lines
        .iter()
        .filter(|l| {
            l.split_whitespace().any(|t| t == "WARN")
                && l.contains("QUIC FALLBACK")
                && l.contains(&format!("tcp={tcp} reason="))
        })
        .count()
}

#[test]
fn off_is_todays_fold_and_never_probes() {
    let _serial = serial();
    let port = free_port();
    let robot = zenoh_peer(&[format!("tcp/127.0.0.1:{port}")], &[]);
    let (browsed, twins) = discovered(port);
    let explicit = vec!["udp/127.0.0.1:1?rel=1".to_string()];
    let (want_cfg, want_fold) =
        fold_discovered_peers_at(desk_cfg(&explicit), browsed.clone(), None);
    let (cfg, fold, outcome) = fold_discovered_peers_unencrypted_quic_at(
        desk_cfg(&explicit),
        browsed,
        &twins,
        None,
        UnencryptedQuic::Off,
        |_| panic!("the opt-in is off: nothing is probed"),
    );
    assert_eq!(format!("{cfg:?}"), format!("{want_cfg:?}"));
    assert_eq!(fold, want_fold);
    assert_eq!(outcome, Default::default());
    assert!(fold.connect.contains(&format!("tcp/127.0.0.1:{port}")));
    drop(robot);
}

#[test]
#[traced_test]
fn on_dials_unencrypted_quic_and_the_link_is_udp() {
    let _serial = serial();
    let port = free_port();
    let robot_listen = cerulion_discovery::unencrypted_quic::with_udp_rel_listeners(&[format!(
        "tcp/127.0.0.1:{port}"
    )]);
    assert_eq!(robot_listen.len(), 2, "{robot_listen:?}");
    let robot = zenoh_peer(&robot_listen, &[]);
    let (browsed, twins) = discovered(port);

    let (cfg, fold, outcome) = fold_discovered_peers_unencrypted_quic_at(
        desk_cfg(&[]),
        browsed,
        &twins,
        None,
        UnencryptedQuic::On,
        real_probe(Duration::from_millis(1000)),
    );
    let quic = format!("udp/127.0.0.1:{port}?rel=1");
    assert_eq!(fold.connect, vec![quic.clone()]);
    assert_eq!(cfg.connect_endpoints, vec![quic.clone()]);
    assert_eq!(outcome.dialled_quic, vec![quic]);
    assert!(outcome.fell_back.is_empty());

    let desk = zenoh_peer(&[], &fold.connect);
    let links = links_of(&robot, Duration::from_secs(10));
    assert!(!links.is_empty(), "no link came up");
    assert!(
        links
            .iter()
            .all(|(src, dst)| src.starts_with("udp/") && dst.starts_with("udp/")),
        "the link must be unencrypted QUIC (udp/): {links:?}"
    );
    logs_assert(
        |lines: &[&str]| match fallback_lines(lines, &format!("tcp/127.0.0.1:{port}")) {
            0 => Ok(()),
            n => Err(format!("{n} fallback lines on a QUIC success")),
        },
    );
    drop(desk);
}

#[test]
#[traced_test]
fn a_closed_udp_port_falls_back_to_tcp_with_one_loud_line() {
    let _serial = serial();
    let port = free_port();
    let tcp = format!("tcp/127.0.0.1:{port}");
    let robot = zenoh_peer(std::slice::from_ref(&tcp), &[]);
    let (browsed, twins) = discovered(port);

    let started = Instant::now();
    let (_cfg, fold, outcome) = fold_discovered_peers_unencrypted_quic_at(
        desk_cfg(&[]),
        browsed,
        &twins,
        None,
        UnencryptedQuic::On,
        real_probe(Duration::from_millis(1000)),
    );
    let took = started.elapsed();
    assert_eq!(fold.connect, vec![tcp.clone()]);
    assert_eq!(outcome.fell_back.len(), 1);
    assert_eq!(outcome.fell_back[0].reason, QuicProbeOutcome::Refused);
    assert!(
        took < Duration::from_millis(900),
        "a closed port answers at once; the fold took {took:?}"
    );

    let desk = zenoh_peer(&[], &fold.connect);
    let links = links_of(&robot, Duration::from_secs(10));
    assert!(
        !links.is_empty() && links.iter().all(|(src, _)| src.starts_with("tcp/")),
        "the fallback link must be tcp/: {links:?}"
    );
    logs_assert(|lines: &[&str]| match fallback_lines(lines, &tcp) {
        1 => Ok(()),
        n => Err(format!("want exactly one fallback line, got {n}")),
    });
    drop(desk);
}

#[test]
#[traced_test]
fn a_udp_port_that_drops_everything_falls_back_at_the_bound() {
    let _serial = serial();
    let port = free_port();
    let tcp = format!("tcp/127.0.0.1:{port}");
    let robot = zenoh_peer(std::slice::from_ref(&tcp), &[]);
    // Bound and never read: every datagram to the port is swallowed, nothing
    // answers, and no ICMP unreachable comes back.
    let _blackhole = UdpSocket::bind(("127.0.0.1", port)).expect("bind the blackhole");
    let (browsed, twins) = discovered(port);

    let bound = Duration::from_millis(400);
    let started = Instant::now();
    let (_cfg, fold, outcome) = fold_discovered_peers_unencrypted_quic_at(
        desk_cfg(&[]),
        browsed,
        &twins,
        None,
        UnencryptedQuic::On,
        real_probe(bound),
    );
    let took = started.elapsed();
    assert_eq!(fold.connect, vec![tcp.clone()]);
    assert_eq!(
        outcome.fell_back[0].reason,
        QuicProbeOutcome::NoAnswer { after: bound }
    );
    assert!(took >= bound, "gave up before the bound: {took:?}");
    assert!(
        took < bound + Duration::from_millis(1500),
        "the bound did not hold: {took:?}"
    );
    logs_assert(|lines: &[&str]| match fallback_lines(lines, &tcp) {
        1 => Ok(()),
        n => Err(format!("want exactly one fallback line, got {n}")),
    });
    drop(robot);
}

#[test]
#[traced_test]
fn explicit_tcp_is_used_as_written_and_explicit_quic_falls_back() {
    let _serial = serial();
    let port = free_port();
    let tcp = format!("tcp/127.0.0.1:{port}");
    let robot = zenoh_peer(std::slice::from_ref(&tcp), &[]);

    let (_cfg, fold, outcome) = fold_discovered_peers_unencrypted_quic_at(
        desk_cfg(std::slice::from_ref(&tcp)),
        Vec::new(),
        &[],
        None,
        UnencryptedQuic::On,
        |_| panic!("an explicit tcp/ locator is never probed"),
    );
    assert_eq!(fold.connect, vec![tcp.clone()]);
    assert_eq!(outcome, Default::default());

    let quic = format!("udp/127.0.0.1:{port}?rel=1");
    let (cfg, fold, outcome) = fold_discovered_peers_unencrypted_quic_at(
        desk_cfg(std::slice::from_ref(&quic)),
        Vec::new(),
        &[],
        None,
        UnencryptedQuic::On,
        real_probe(Duration::from_millis(1000)),
    );
    assert_eq!(fold.connect, vec![tcp.clone()]);
    assert_eq!(cfg.connect_endpoints, vec![tcp.clone()]);
    assert_eq!(outcome.fell_back.len(), 1);
    assert_eq!(outcome.fell_back[0].quic, quic);
    logs_assert(|lines: &[&str]| match fallback_lines(lines, &tcp) {
        1 => Ok(()),
        n => Err(format!("want exactly one fallback line, got {n}")),
    });
    drop(robot);
}
