// SPDX-License-Identifier: AGPL-3.0-only
//! `net.udp`: whether UDP reaches a QUIC endpoint and comes back, probed
//! without a handshake, a certificate or a peer's cooperation.
//!
//! The probe sends one 1200-byte UDP datagram shaped as a QUIC long-header
//! packet with a version no QUIC implementation supports (`0x?a?a?a?a` is
//! reserved for this, RFC 9000 section 15). A QUIC server that receives such a
//! packet of at least 1200 bytes is expected to answer with a Version
//! Negotiation packet (version 0, the probe's connection ids swapped; RFC 9000
//! sections 5.2.2 and 6.1); quinn, the QUIC stack of the `quic/` and
//! `udp/...?rel=1` listeners, does so on every listening endpoint. An answer
//! proves UDP crosses the path in both directions and a QUIC endpoint listens
//! there; a refusal (an ICMP port unreachable) means UDP reached the host and
//! nothing listens on the port; silence within the wait means UDP is blocked
//! on the path or the endpoint does not answer. The probe opens no connection
//! and creates no connection state on either end.
//!
//! The targets are the `quic/` and `udp/...?rel=1` connect locators of the
//! workspace's graphs and every `--udp-target`. With none, the check is
//! skipped: it never probes a host the user did not name. Endpoints are
//! probed at the same time; each resolves its name and probes up to one
//! address per IP family, and the check reports what has finished after
//! [`CHECK_BUDGET`].
//!
//! [`probe`], [`version_negotiation_probe`] and [`parse_version_negotiation`]
//! are public.

use std::net::{SocketAddr, ToSocketAddrs, UdpSocket};
use std::time::{Duration, Instant};

use super::{Check, Context, Outcome, Reading, Status};

pub(super) const UDP: Check = Check {
    id: "net.udp",
    why: "the quic/ and udp/ carriers need UDP to cross the path in both directions; a \
          firewall or NAT that drops UDP leaves a QUIC link unable to connect where tcp/ \
          would.",
    network: true,
    run,
};

/// The reserved version the probe advertises (`0x?a?a?a?a`, RFC 9000 s15).
pub const PROBE_VERSION: u32 = 0x1a2a_3a4a;

/// The probe datagram's size: the smallest a QUIC server must answer.
pub const PROBE_LEN: usize = 1200;

/// How long [`probe`] waits for the answer when the caller gives no wait:
/// the bound the discovery ladder gives an explicitly named endpoint
/// ([`cerulion_discovery::ladder::EXPLICIT_PROBE_TIMEOUT`]).
pub const DEFAULT_WAIT: Duration = cerulion_discovery::ladder::EXPLICIT_PROBE_TIMEOUT;

/// How long the check waits for its endpoints: one name resolution and one
/// probe per IP family, each given [`DEFAULT_WAIT`].
pub const CHECK_BUDGET: Duration = DEFAULT_WAIT.saturating_mul(3);

/// The probe datagram for the connection ids `dcid` and `scid` (each at most
/// 20 bytes): a long-header packet with [`PROBE_VERSION`], padded with zeros
/// to [`PROBE_LEN`].
#[must_use]
pub fn version_negotiation_probe(dcid: &[u8], scid: &[u8]) -> Vec<u8> {
    let mut packet = Vec::with_capacity(PROBE_LEN);
    // Header form 1 (long), fixed bit 1, the rest unused by a server that
    // does not know the version.
    packet.push(0xc0);
    packet.extend_from_slice(&PROBE_VERSION.to_be_bytes());
    packet.push(u8::try_from(dcid.len()).unwrap_or(u8::MAX));
    packet.extend_from_slice(dcid);
    packet.push(u8::try_from(scid.len()).unwrap_or(u8::MAX));
    packet.extend_from_slice(scid);
    packet.resize(PROBE_LEN, 0);
    packet
}

/// The versions a Version Negotiation answer to a probe sent with `dcid` and
/// `scid` lists, or `None` when `answer` is not one: it must be a long header
/// with version 0 whose destination id is the probe's `scid` and whose source
/// id is the probe's `dcid` (RFC 9000 s17.2.1).
#[must_use]
pub fn parse_version_negotiation(answer: &[u8], dcid: &[u8], scid: &[u8]) -> Option<Vec<u32>> {
    let (&first, rest) = answer.split_first()?;
    if first & 0x80 == 0 {
        return None;
    }
    let (version, rest) = rest.split_at_checked(4)?;
    if version != [0, 0, 0, 0] {
        return None;
    }
    let (&dlen, rest) = rest.split_first()?;
    let (answer_dcid, rest) = rest.split_at_checked(usize::from(dlen))?;
    let (&slen, rest) = rest.split_first()?;
    let (answer_scid, rest) = rest.split_at_checked(usize::from(slen))?;
    if answer_dcid != scid
        || answer_scid != dcid
        || rest.is_empty()
        || !rest.len().is_multiple_of(4)
    {
        return None;
    }
    let versions: Vec<u32> = rest
        .as_chunks::<4>()
        .0
        .iter()
        .map(|c| u32::from_be_bytes(*c))
        .collect();
    // RFC 9000 s6.2: a list naming the version the probe sent is discarded.
    (!versions.contains(&PROBE_VERSION)).then_some(versions)
}

/// What one probe found.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum UdpReachability {
    /// A Version Negotiation answer came back.
    Answered {
        /// Time from send to answer.
        rtt: Duration,
        /// The QUIC versions the endpoint lists.
        versions: Vec<u32>,
    },
    /// Nothing that answers the probe arrived within the wait.
    NoAnswer,
    /// The host answered that nothing listens on the port (an ICMP port
    /// unreachable, seen as a refused receive).
    Refused,
    /// The datagram could not be sent (no route, the address family is not
    /// configured).
    SendFailed(String),
}

/// Send one probe to `target` and wait up to `wait` for its answer. Datagrams
/// that are not the answer are skipped until the wait ends.
pub fn probe(target: SocketAddr, wait: Duration) -> UdpReachability {
    let mut ids = [0u8; 16];
    if let Err(e) = getrandom::fill(&mut ids) {
        return UdpReachability::SendFailed(format!("no random connection ids: {e}"));
    }
    let (dcid, scid) = ids.split_at(8);
    let bind: SocketAddr = if target.is_ipv4() {
        (std::net::Ipv4Addr::UNSPECIFIED, 0).into()
    } else {
        (std::net::Ipv6Addr::UNSPECIFIED, 0).into()
    };
    let socket = match UdpSocket::bind(bind).and_then(|s| s.connect(target).map(|()| s)) {
        Ok(s) => s,
        Err(e) => return UdpReachability::SendFailed(e.to_string()),
    };
    let sent_at = Instant::now();
    if let Err(e) = socket.send(&version_negotiation_probe(dcid, scid)) {
        return UdpReachability::SendFailed(e.to_string());
    }
    let mut buf = [0u8; 1500];
    loop {
        let left = wait.saturating_sub(sent_at.elapsed());
        if left.is_zero() || socket.set_read_timeout(Some(left)).is_err() {
            return UdpReachability::NoAnswer;
        }
        match socket.recv(&mut buf) {
            Ok(n) => {
                if let Some(versions) = parse_version_negotiation(&buf[..n], dcid, scid) {
                    return UdpReachability::Answered {
                        rtt: sent_at.elapsed(),
                        versions,
                    };
                }
            }
            Err(e) if e.kind() == std::io::ErrorKind::Interrupted => {}
            // An ICMP port unreachable surfaces as a refused receive on a
            // connected socket.
            Err(e) if e.kind() == std::io::ErrorKind::ConnectionRefused => {
                return UdpReachability::Refused
            }
            Err(_) => return UdpReachability::NoAnswer,
        }
    }
}

/// The `host:port` of a QUIC locator (`quic/` or `udp/...?rel=1`, as
/// [`cerulion_core::transport::carrier::classify_locator`] reads it), or
/// `None` for any other carrier, plain `udp/` included.
pub(super) fn udp_endpoint(locator: &str) -> Option<String> {
    use cerulion_core::transport::carrier::{classify_locator, Carrier};
    let locator = locator.trim();
    match classify_locator(locator).ok()? {
        Carrier::Quic { .. } | Carrier::UnsecureQuic { .. } => {}
        _ => return None,
    }
    let (_, rest) = locator.split_once('/')?;
    let before_config = rest.split_once('#').map_or(rest, |(a, _)| a);
    let address = before_config
        .split_once('?')
        .map_or(before_config, |(a, _)| a);
    (!address.is_empty()).then(|| address.to_string())
}

/// What one endpoint gave.
#[derive(Debug, Clone, PartialEq, Eq)]
pub(super) enum EndpointResult {
    /// The best probe result over the endpoint's addresses.
    Probed(UdpReachability),
    /// The name did not resolve.
    Unresolved(String),
    /// Resolution or the probes had not finished within [`CHECK_BUDGET`].
    Unfinished,
}

/// One probed endpoint.
#[derive(Debug, Clone, PartialEq, Eq)]
pub(super) struct Probed {
    pub(super) endpoint: String,
    pub(super) result: EndpointResult,
}

/// The best of several probes of one endpoint: an answer, else a refusal,
/// else silence, else the first send failure.
pub(super) fn best(results: Vec<UdpReachability>) -> UdpReachability {
    let rank = |r: &UdpReachability| match r {
        UdpReachability::Answered { .. } => 0,
        UdpReachability::Refused => 1,
        UdpReachability::NoAnswer => 2,
        UdpReachability::SendFailed(_) => 3,
    };
    results
        .into_iter()
        .min_by_key(rank)
        .unwrap_or_else(|| UdpReachability::SendFailed("no address to probe".to_string()))
}

pub(super) fn decide(probed: &[Probed]) -> Outcome {
    if probed.is_empty() {
        return Outcome::new(
            Status::Skip,
            "not run: no QUIC endpoint is named (no quic/ or udp/...?rel=1 connect locator in \
             the workspace graphs, and no --udp-target)",
        );
    }
    let mut answered = Vec::new();
    let mut problems = Vec::new();
    let mut fixes: Vec<&'static str> = Vec::new();
    let mut fix = |f: &'static str| {
        if !fixes.contains(&f) {
            fixes.push(f);
        }
    };
    for p in probed {
        let e = &p.endpoint;
        match &p.result {
            EndpointResult::Probed(UdpReachability::Answered { rtt, .. }) => {
                answered.push(format!("{e} ({} ms)", rtt.as_millis()));
            }
            EndpointResult::Probed(UdpReachability::NoAnswer) => {
                problems.push(format!("{e} (no answer)"));
                fix(
                    "allow UDP to and from that port on the firewalls and NAT between the two \
                     machines, or use a tcp/ locator for that link",
                );
            }
            EndpointResult::Probed(UdpReachability::Refused) => {
                problems.push(format!("{e} (refused: nothing listens on that port)"));
                fix(
                    "start the quic/ or udp/...?rel=1 listener on that machine, or correct the \
                     port",
                );
            }
            EndpointResult::Probed(UdpReachability::SendFailed(err)) => {
                problems.push(format!("{e} (send failed: {err})"));
                fix("check that this machine has a route to that address");
            }
            EndpointResult::Unresolved(err) => {
                problems.push(format!("{e} (cannot resolve: {err})"));
                fix("check the host name, or name the endpoint by its IP address");
            }
            EndpointResult::Unfinished => {
                problems.push(format!(
                    "{e} (unfinished within the check's fixed {} ms)",
                    CHECK_BUDGET.as_millis()
                ));
                fix(
                    "check the host name, or name the endpoint by its IP address; a name that \
                     resolves slowly, or two silent address families, use up the fixed budget",
                );
            }
        }
    }
    let count = |n: usize| Reading::Int(u64::try_from(n).unwrap_or(u64::MAX));
    let o = if problems.is_empty() {
        Outcome::new(
            Status::Pass,
            format!(
                "UDP reaches a QUIC endpoint and back: {}",
                answered.join(", ")
            ),
        )
    } else {
        let mut summary = format!("no QUIC answer over UDP from {}", problems.join(", "));
        if !answered.is_empty() {
            summary.push_str(&format!("; answered: {}", answered.join(", ")));
        }
        Outcome::new(Status::Warn, summary).with_fix(fixes.join("; "))
    };
    o.reading("probed", count(probed.len()))
        .reading("answered", count(answered.len()))
}

/// Every endpoint to probe: the workspace graphs' QUIC (`quic/`, `udp/...?rel=1`) connect
/// locators, then `extra`, without repeats, in that order.
pub(super) fn targets(graph_locators: &[String], extra: &[String]) -> Vec<String> {
    let mut out: Vec<String> = Vec::new();
    for endpoint in graph_locators
        .iter()
        .filter_map(|l| udp_endpoint(l))
        .chain(extra.iter().map(|e| e.trim().to_string()))
    {
        if !out.contains(&endpoint) {
            out.push(endpoint);
        }
    }
    out
}

/// The connect locators of every graph under `graphs_dir` that parses.
fn graph_connect_locators(graphs_dir: &std::path::Path) -> Vec<String> {
    let Ok(entries) = std::fs::read_dir(graphs_dir) else {
        return Vec::new();
    };
    let mut paths: Vec<_> = entries
        .filter_map(Result::ok)
        .map(|e| e.path())
        .filter(|p| p.extension().is_some_and(|e| e == "yaml"))
        .collect();
    paths.sort();
    paths
        .iter()
        .filter_map(|p| std::fs::read_to_string(p).ok())
        .filter_map(|text| cerulion_core::graph::parse_graph_raw(&text).ok())
        .filter_map(|config| config.network)
        .flat_map(|network| network.connect)
        .collect()
}

/// Resolve `endpoint` and probe its first address of each IP family.
fn probe_endpoint(endpoint: &str) -> EndpointResult {
    let addrs: Vec<SocketAddr> = match endpoint.to_socket_addrs() {
        Ok(addrs) => addrs.collect(),
        Err(e) => return EndpointResult::Unresolved(e.to_string()),
    };
    let picked: Vec<SocketAddr> = [
        addrs.iter().find(|a| a.is_ipv4()),
        addrs.iter().find(|a| a.is_ipv6()),
    ]
    .into_iter()
    .flatten()
    .copied()
    .collect();
    if picked.is_empty() {
        return EndpointResult::Unresolved("no address".to_string());
    }
    // One family answering settles the endpoint; the other is not probed.
    let mut results = Vec::new();
    for addr in picked {
        let result = probe(addr, DEFAULT_WAIT);
        let answered = matches!(result, UdpReachability::Answered { .. });
        results.push(result);
        if answered {
            break;
        }
    }
    EndpointResult::Probed(best(results))
}

fn run(ctx: &Context) -> Outcome {
    let graph_locators = ctx
        .workspace_root()
        .map(|root| graph_connect_locators(&root.join("graphs")))
        .unwrap_or_default();
    let endpoints = targets(&graph_locators, &ctx.udp_targets);
    let (tx, rx) = std::sync::mpsc::channel();
    for (index, endpoint) in endpoints.iter().enumerate() {
        let tx = tx.clone();
        let endpoint = endpoint.clone();
        // A thread per endpoint, abandoned after `CHECK_BUDGET` as the
        // framework abandons a check: the process exits after the report.
        std::thread::spawn(move || {
            let _ = tx.send((index, probe_endpoint(&endpoint)));
        });
    }
    drop(tx);
    let mut results: Vec<EndpointResult> = vec![EndpointResult::Unfinished; endpoints.len()];
    let deadline = Instant::now() + CHECK_BUDGET;
    while let Ok((index, result)) =
        rx.recv_timeout(deadline.saturating_duration_since(Instant::now()))
    {
        results[index] = result;
    }
    let probed: Vec<Probed> = endpoints
        .into_iter()
        .zip(results)
        .map(|(endpoint, result)| Probed { endpoint, result })
        .collect();
    decide(&probed)
}

#[cfg(test)]
mod tests {
    use super::*;

    const DCID: [u8; 8] = [1, 2, 3, 4, 5, 6, 7, 8];
    const SCID: [u8; 8] = [9, 10, 11, 12, 13, 14, 15, 16];

    #[test]
    fn the_probe_is_a_1200_byte_long_header_with_the_reserved_version() {
        let p = version_negotiation_probe(&DCID, &SCID);
        assert_eq!(p.len(), 1200);
        assert_eq!(
            &p[..23],
            &[
                0xc0, 0x1a, 0x2a, 0x3a, 0x4a, 8, 1, 2, 3, 4, 5, 6, 7, 8, 8, 9, 10, 11, 12, 13, 14,
                15, 16
            ][..]
        );
        assert!(p[23..].iter().all(|b| *b == 0));
        // The reserved pattern: the low nibble of every byte is 0xa.
        assert!(PROBE_VERSION.to_be_bytes().iter().all(|b| b & 0x0f == 0x0a));
    }

    fn answer(dcid: &[u8], scid: &[u8], versions: &[u32]) -> Vec<u8> {
        let mut a = vec![0x80 | 0x2a, 0, 0, 0, 0];
        a.push(u8::try_from(dcid.len()).unwrap());
        a.extend_from_slice(dcid);
        a.push(u8::try_from(scid.len()).unwrap());
        a.extend_from_slice(scid);
        for v in versions {
            a.extend_from_slice(&v.to_be_bytes());
        }
        a
    }

    #[test]
    fn a_version_negotiation_answer_with_swapped_ids_parses() {
        // quinn lists a grease version first, then the versions it supports.
        let a = answer(&SCID, &DCID, &[0x0a1a_2a3a, 1]);
        assert_eq!(
            parse_version_negotiation(&a, &DCID, &SCID),
            Some(vec![0x0a1a_2a3a, 1])
        );
    }

    #[test]
    fn anything_else_is_not_an_answer() {
        // Ids not swapped.
        assert_eq!(
            parse_version_negotiation(&answer(&DCID, &SCID, &[1]), &DCID, &SCID),
            None
        );
        // A non-zero version.
        let mut a = answer(&SCID, &DCID, &[1]);
        a[4] = 1;
        assert_eq!(parse_version_negotiation(&a, &DCID, &SCID), None);
        // A short header.
        let mut a = answer(&SCID, &DCID, &[1]);
        a[0] = 0x40;
        assert_eq!(parse_version_negotiation(&a, &DCID, &SCID), None);
        // No version list, a ragged list, and truncated input.
        assert_eq!(
            parse_version_negotiation(&answer(&SCID, &DCID, &[]), &DCID, &SCID),
            None
        );
        let mut a = answer(&SCID, &DCID, &[1]);
        a.push(0);
        assert_eq!(parse_version_negotiation(&a, &DCID, &SCID), None);
        assert_eq!(parse_version_negotiation(&[0x80, 0, 0], &DCID, &SCID), None);
        assert_eq!(parse_version_negotiation(&[], &DCID, &SCID), None);
    }

    #[test]
    fn a_list_naming_the_probe_version_is_discarded() {
        let a = answer(&SCID, &DCID, &[1, PROBE_VERSION]);
        assert_eq!(parse_version_negotiation(&a, &DCID, &SCID), None);
    }

    #[test]
    fn locators_give_their_endpoint_only_for_the_quic_carriers() {
        assert_eq!(
            udp_endpoint("quic/10.0.0.2:7447").as_deref(),
            Some("10.0.0.2:7447")
        );
        assert_eq!(
            udp_endpoint("udp/[fd00::2]:7447?rel=1;mixed_rel=1").as_deref(),
            Some("[fd00::2]:7447")
        );
        assert_eq!(
            udp_endpoint("quic/10.0.0.2:7447#iface=eth0").as_deref(),
            Some("10.0.0.2:7447")
        );
        assert_eq!(
            udp_endpoint("udp/10.0.0.2:7447"),
            None,
            "plain udp/ is not QUIC"
        );
        assert_eq!(udp_endpoint("tcp/10.0.0.2:7447"), None);
        assert_eq!(udp_endpoint("quic/"), None);
    }

    #[test]
    fn targets_merge_graph_locators_and_flags_without_repeats() {
        let graph = [
            "tcp/10.0.0.1:7447".to_string(),
            "quic/10.0.0.2:7447".to_string(),
            "udp/10.0.0.2:7447?rel=1".to_string(),
            "udp/10.0.0.9:7447".to_string(),
        ];
        assert_eq!(
            targets(
                &graph,
                &["robot.local:7447".to_string(), "10.0.0.2:7447".to_string()]
            ),
            vec!["10.0.0.2:7447".to_string(), "robot.local:7447".to_string()]
        );
    }

    #[test]
    fn no_target_is_a_skip_and_never_a_probe() {
        let o = decide(&[]);
        assert_eq!(o.status, Status::Skip);
        assert!(o.summary.contains("--udp-target"));
    }

    fn probed(endpoint: &str, result: EndpointResult) -> Probed {
        Probed {
            endpoint: endpoint.to_string(),
            result,
        }
    }

    #[test]
    fn every_answer_passes_and_any_silence_warns() {
        let ok = probed(
            "10.0.0.2:7447",
            EndpointResult::Probed(UdpReachability::Answered {
                rtt: Duration::from_millis(31),
                versions: vec![1],
            }),
        );
        let pass = decide(std::slice::from_ref(&ok));
        assert_eq!(pass.status, Status::Pass);
        assert_eq!(
            pass.summary,
            "UDP reaches a QUIC endpoint and back: 10.0.0.2:7447 (31 ms)"
        );
        let warn = decide(&[
            ok,
            probed(
                "10.0.0.3:7447",
                EndpointResult::Probed(UdpReachability::NoAnswer),
            ),
            probed(
                "nowhere:1",
                EndpointResult::Unresolved("lookup failed".to_string()),
            ),
        ]);
        assert_eq!(warn.status, Status::Warn);
        assert_eq!(
            warn.summary,
            "no QUIC answer over UDP from 10.0.0.3:7447 (no answer), nowhere:1 (cannot resolve: \
             lookup failed); answered: 10.0.0.2:7447 (31 ms)"
        );
        assert_eq!(
            warn.fix.as_deref(),
            Some(
                "allow UDP to and from that port on the firewalls and NAT between the two \
                 machines, or use a tcp/ locator for that link; check the host name, or name the \
                 endpoint by its IP address"
            )
        );
        assert_eq!(warn.readings.get("answered"), Some(&Reading::Int(1)));
    }

    #[test]
    fn a_refusal_and_an_unfinished_endpoint_get_their_own_fixes() {
        let o = decide(&[
            probed(
                "10.0.0.4:7447",
                EndpointResult::Probed(UdpReachability::Refused),
            ),
            probed("slow.example:7447", EndpointResult::Unfinished),
        ]);
        assert_eq!(o.status, Status::Warn);
        assert!(o
            .summary
            .contains("10.0.0.4:7447 (refused: nothing listens on that port)"));
        assert!(o
            .summary
            .contains("slow.example:7447 (unfinished within the check's fixed 3000 ms)"));
        assert!(o
            .fix
            .unwrap()
            .starts_with("start the quic/ or udp/...?rel=1 listener"));
    }

    #[test]
    fn the_best_result_over_an_endpoints_addresses_wins() {
        let answered = UdpReachability::Answered {
            rtt: Duration::from_millis(1),
            versions: vec![1],
        };
        assert_eq!(
            best(vec![UdpReachability::NoAnswer, answered.clone()]),
            answered
        );
        assert_eq!(
            best(vec![
                UdpReachability::SendFailed("x".into()),
                UdpReachability::Refused
            ]),
            UdpReachability::Refused
        );
        assert!(matches!(best(vec![]), UdpReachability::SendFailed(_)));
    }

    #[test]
    fn the_check_budget_is_three_waits() {
        assert_eq!(CHECK_BUDGET, Duration::from_millis(3000));
    }

    /// The probe against a local stand-in that answers the way RFC 9000
    /// describes, and against a port with nothing behind it.
    #[test]
    fn the_probe_reads_a_local_version_negotiation_answer() {
        let server = UdpSocket::bind("127.0.0.1:0").unwrap();
        server
            .set_read_timeout(Some(Duration::from_secs(10)))
            .unwrap();
        let addr = server.local_addr().unwrap();
        let handle = std::thread::spawn(move || {
            let mut buf = [0u8; 1500];
            let (n, from) = server.recv_from(&mut buf).unwrap();
            assert_eq!(n, PROBE_LEN);
            let dlen = usize::from(buf[5]);
            let dcid = buf[6..6 + dlen].to_vec();
            let slen = usize::from(buf[6 + dlen]);
            let scid = buf[7 + dlen..7 + dlen + slen].to_vec();
            server.send_to(b"noise", from).unwrap();
            server.send_to(&answer(&scid, &dcid, &[1]), from).unwrap();
        });
        match probe(addr, Duration::from_secs(5)) {
            UdpReachability::Answered { versions, .. } => assert_eq!(versions, vec![1]),
            other => panic!("expected an answer, got {other:?}"),
        }
        handle.join().unwrap();

        let unused = UdpSocket::bind("127.0.0.1:0").unwrap();
        let silent = unused.local_addr().unwrap();
        drop(unused);
        // Loopback answers a closed port with an ICMP port unreachable on
        // most hosts, else nothing comes back: either way, never an answer.
        assert!(matches!(
            probe(silent, Duration::from_millis(300)),
            UdpReachability::Refused | UdpReachability::NoAnswer
        ));
    }
}
