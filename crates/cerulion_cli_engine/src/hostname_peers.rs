// SPDX-License-Identifier: AGPL-3.0-only
//! The HOSTNAME rung of the discovery ladder — the
//! operator escape-hatch for peers that scouting cannot (or should not) find.
//!
//! Some robots aren't discoverable by mDNS: a robot on a different subnet, a
//! CI rig behind a router, or a lab's standing fleet you always want tried
//! first. This rung lets an operator name those peers explicitly and resolves
//! each to a locator via ordinary DNS (or the `<name>.local` mDNS-host
//! convention).
//!
//! # Where names come from
//!
//! Two sources, both optional, combined:
//!
//! - `CERULION_PEERS` env var ([`CERULION_PEERS_ENV`]) — a comma-separated
//!   `host[:port]` list. The scripted / CI / one-off rig hatch: set it inline
//!   for a single invocation.
//! - `~/.cerulion/config.toml` `peers = ["host[:port]", ...]` — the durable
//!   hatch for a lab's standing robots, persisted across invocations.
//!
//! Every entry is parsed by [`parse_peers_spec`]; a bare `<name>` (no dot)
//! that fails DNS is retried as `<name>.local`, so a robot advertising an
//! mDNS host record resolves with no extra configuration.
//!
//! # Testing note
//!
//! [`hostname_rung`] itself does real DNS + reads process env, so it has NO
//! direct CI test — that would be non-hermetic and flaky. Its pieces are all
//! pure and are oracle-tested here: [`parse_peers_spec`] /
//! [`parse_config_peers`] (spec parsing) and the private `select_addr` /
//! `locator_from_ip` helpers (the resolve-result mapping).

use crate::discovery_ladder::{DiscoveredPeer, DiscoveryRung};
use std::net::{IpAddr, SocketAddr, ToSocketAddrs};
use std::str::FromStr;
use std::sync::mpsc;
use std::time::{Duration, Instant};

/// Env var carrying a comma-separated `host[:port]` peer list (the scripted /
/// CI escape hatch). Parsed by [`parse_peers_spec`].
pub const CERULION_PEERS_ENV: &str = "CERULION_PEERS";

/// Parse a comma-separated `host[:port]` spec into `(host, port)` targets.
///
/// PURE. Whitespace around entries is trimmed; empty entries are skipped;
/// `host` is returned WITHOUT surrounding brackets (the caller re-brackets an
/// IPv6 host when building a locator). Forms:
///
/// - `host` / `ipv4` ⇒ `(host, default_port)`.
/// - `host:port` / `ipv4:port` ⇒ split at the last `:` when the tail parses as
///   a `u16` and the head is non-empty.
/// - `[ipv6]` / `[ipv6]:port` ⇒ bracketed host, optional port.
/// - bare `ipv6:port` (e.g. `fe80::1:7683`) ⇒ same last-`:` rule (the tail
///   `7683` parses, so the head `fe80::1` is the host). A bare, PORT-LESS IPv6
///   whose final hextet is all digits is ambiguous and should be bracketed;
///   this is an accepted, documented limitation.
///
/// An entry whose intended port is out of `u16` range (an all-digit tail that
/// does not parse) is skipped with a `warn!` naming it.
pub fn parse_peers_spec(spec: &str, default_port: u16) -> Vec<(String, u16)> {
    spec.split(',')
        .filter_map(|e| parse_entry(e, default_port))
        .collect()
}

/// Parse one already-split spec entry. `None` = skip (empty or malformed);
/// malformed-with-intent (a bad port) also emits a `warn!`.
fn parse_entry(entry: &str, default_port: u16) -> Option<(String, u16)> {
    let entry = entry.trim();
    if entry.is_empty() {
        return None;
    }

    // Bracketed IPv6: `[host]` or `[host]:port`.
    if let Some(after_open) = entry.strip_prefix('[') {
        let close = match after_open.find(']') {
            Some(i) => i,
            None => {
                tracing::warn!(
                    entry = %entry,
                    "skipping peer entry: unterminated '[' in bracketed IPv6 host"
                );
                return None;
            }
        };
        let host = &after_open[..close];
        if host.is_empty() {
            tracing::warn!(entry = %entry, "skipping peer entry: empty bracketed host");
            return None;
        }
        let tail = &after_open[close + 1..]; // "" or ":port"
        if tail.is_empty() {
            return Some((host.to_string(), default_port));
        }
        return match tail.strip_prefix(':') {
            Some(port_str) => match port_str.parse::<u16>() {
                Ok(port) => Some((host.to_string(), port)),
                Err(_) => {
                    tracing::warn!(entry = %entry, port = %port_str, "skipping peer entry: invalid port");
                    None
                }
            },
            None => {
                tracing::warn!(entry = %entry, "skipping peer entry: junk after bracketed host");
                None
            }
        };
    }

    // Bare form. Split at the LAST ':' when the tail is a valid port and the
    // head is non-empty. An all-digit but out-of-range tail is an invalid port
    // (skip + warn); a non-numeric tail means the ':' belongs to the host (a
    // rare unbracketed IPv6), so the whole entry is the host at default port.
    match entry.rsplit_once(':') {
        Some((head, tail)) if !head.is_empty() && !tail.is_empty() => match tail.parse::<u16>() {
            Ok(port) => Some((head.to_string(), port)),
            Err(_) if tail.bytes().all(|b| b.is_ascii_digit()) => {
                tracing::warn!(entry = %entry, port = %tail, "skipping peer entry: invalid port");
                None
            }
            Err(_) => Some((entry.to_string(), default_port)),
        },
        _ => Some((entry.to_string(), default_port)),
    }
}

/// Minimal serde view over `~/.cerulion/config.toml` — only the `peers` array
/// matters; every other key is ignored.
#[derive(serde::Deserialize)]
struct ConfigPeersDoc {
    #[serde(default)]
    peers: Vec<String>,
}

/// Parse the `peers = ["host[:port]", ...]` array out of config-file TOML text.
/// PURE. Returns the RAW entry strings ([`parse_peers_spec`] applies after).
/// Missing key ⇒ empty (a config without `peers` is normal, no warn);
/// malformed TOML (or a `peers` of the wrong shape) ⇒ empty + one `warn!`.
///
/// A key that is a NEAR MISS of `peers` warns. The rule
/// above — "Missing key ⇒ empty … no warn" —
/// is right for a key that is genuinely absent and wrong for
/// `peer = [...]` or `Peers = [...]`, which parse clean, yield NO peers, and
/// take the discovery ladder's hostname rung quietly out of service. A
/// wrong-TYPED `peers` already warned; only the misspellings were silent.
///
/// A WARN and not `deny_unknown_fields`: this file is named for general
/// configuration and exactly one key is read from it today, so refusing a
/// document over a key the parser does not consume would be the wrong trade — see the
/// `near_miss` module (crate-internal) for where each mechanism applies.
pub fn parse_config_peers(toml_text: &str) -> Vec<String> {
    match toml::from_str::<ConfigPeersDoc>(toml_text) {
        Ok(doc) => {
            // Reported only on a document that PARSED. A malformed file, or a
            // `peers` of the wrong shape, already gets the one warn the doc
            // above promises — and on that document the near-miss line adds
            // nothing an operator can act on: their peers are being ignored
            // for a reason they have already been told, and a second line
            // about a different key reads like a second fault.
            warn_on_near_miss_config_keys(toml_text);
            doc.peers
        }
        Err(e) => {
            tracing::warn!(
                error = %e,
                "failed to parse ~/.cerulion/config.toml; ignoring configured peers"
            );
            Vec::new()
        }
    }
}

/// The keys `~/.cerulion/config.toml` is READ for. A near miss of one of these
/// is a misspelling worth reporting; anything else is another tool's business.
const CONFIG_KNOWN_KEYS: &[&str] = &["peers"];

/// Warn once per near-miss top-level key. Silent on a document that will not
/// parse — [`parse_config_peers`] reports that itself, and a second complaint
/// about the same file would be noise.
fn warn_on_near_miss_config_keys(toml_text: &str) {
    let Ok(doc) = toml_text.parse::<toml::Table>() else {
        return;
    };
    for (found, expected) in
        crate::near_miss::near_miss_keys(doc.keys().map(String::as_str), CONFIG_KNOWN_KEYS)
    {
        tracing::warn!(
            found = %found,
            expected = %expected,
            "a top-level key in ~/.cerulion/config.toml looks like a misspelling (see `found` \
             and `expected`) — it is being IGNORED, so anything configured under it never takes \
             effect. Rename the key (or, if it really belongs to another tool, ignore this)."
        );
    }
}

/// The hostname rung: gather peer names from `CERULION_PEERS` and
/// `~/.cerulion/config.toml`, parse them, and resolve each CONCURRENTLY to a
/// locator (DNS blocks, so per-entry threads are bounded by `budget`). Each
/// resolved peer is stamped [`DiscoveryRung::Hostname`]. INFALLIBLE — an unset
/// env, absent config, and unresolvable names all just contribute nothing.
///
/// NOT CI-tested directly (real DNS + process env); its pure pieces are.
pub fn hostname_rung(budget: Duration) -> Vec<DiscoveredPeer> {
    let default_port = crate::graph_cmd::GATEWAY_WELL_KNOWN_PORT;
    let mut targets: Vec<(String, u16)> = Vec::new();

    // (a) env override.
    if let Ok(spec) = std::env::var(CERULION_PEERS_ENV) {
        targets.extend(parse_peers_spec(&spec, default_port));
    }

    // (b) ~/.cerulion/config.toml `peers = [...]`; a missing file is silent.
    if let Some(home) = dirs::home_dir() {
        let cfg = home.join(".cerulion").join("config.toml");
        match std::fs::read_to_string(&cfg) {
            Ok(text) => {
                for entry in parse_config_peers(&text) {
                    targets.extend(parse_peers_spec(&entry, default_port));
                }
            }
            Err(e) if e.kind() == std::io::ErrorKind::NotFound => {}
            Err(e) => {
                tracing::warn!(error = %e, path = %cfg.display(), "failed to read ~/.cerulion/config.toml");
            }
        }
    }

    // Dedup identical (host, port) targets (a name in both env and config)
    // while preserving first-seen order.
    let mut seen = std::collections::HashSet::new();
    targets.retain(|t| seen.insert(t.clone()));
    if targets.is_empty() {
        return Vec::new();
    }

    let jobs: Vec<_> = targets
        .into_iter()
        .map(|(host, port)| move || resolve_target(host, port))
        .collect();
    run_bounded(jobs, budget)
}

/// Resolve one `(host, port)` to a [`DiscoveredPeer`]. An IP literal skips DNS.
/// A bare label (no dot) that fails DNS is retried as `<host>.local` (the mDNS
/// host convention). The returned `robot` is always the ORIGINAL host label
/// (never the `.local` form). Unresolvable ⇒ a debug breadcrumb + `None`.
fn resolve_target(host: String, port: u16) -> Option<DiscoveredPeer> {
    if let Ok(ip) = IpAddr::from_str(&host) {
        return Some(DiscoveredPeer {
            robot: host,
            locator: locator_from_ip(ip, port),
            rung: DiscoveryRung::Hostname,
        });
    }

    let addrs = match resolve_dns(&host, port) {
        Some(a) => a,
        None if !host.contains('.') => {
            let mdns = format!("{host}.local");
            match resolve_dns(&mdns, port) {
                Some(a) => a,
                None => {
                    tracing::debug!(host = %host, "peer hostname did not resolve (also tried .local)");
                    return None;
                }
            }
        }
        None => {
            tracing::debug!(host = %host, "peer hostname did not resolve");
            return None;
        }
    };

    let addr = select_addr(&addrs)?;
    Some(DiscoveredPeer {
        robot: host,
        locator: locator_from_ip(addr.ip(), port),
        rung: DiscoveryRung::Hostname,
    })
}

/// Resolve `host:port` via the OS resolver, returning the addresses (or `None`
/// on failure / empty). Uses the `(&str, u16)` `ToSocketAddrs` impl so IPv6
/// hosts need no bracketing here.
fn resolve_dns(host: &str, port: u16) -> Option<Vec<SocketAddr>> {
    match (host, port).to_socket_addrs() {
        Ok(it) => {
            let v: Vec<SocketAddr> = it.collect();
            if v.is_empty() {
                None
            } else {
                Some(v)
            }
        }
        Err(_) => None,
    }
}

/// Pick the address to use from a resolver result: prefer the first IPv4 (best
/// LAN reachability), else the first IPv6, else the first address of any kind.
/// Pure — oracle-tested with hand-built addresses.
fn select_addr(addrs: &[SocketAddr]) -> Option<SocketAddr> {
    addrs
        .iter()
        .find(|a| a.is_ipv4())
        .copied()
        .or_else(|| addrs.iter().find(|a| a.is_ipv6()).copied())
        .or_else(|| addrs.first().copied())
}

/// Build a zenoh TCP locator from an IP + port, bracketing IPv6. Pure —
/// oracle-tested.
fn locator_from_ip(ip: IpAddr, port: u16) -> String {
    match ip {
        IpAddr::V4(v4) => format!("tcp/{v4}:{port}"),
        IpAddr::V6(v6) => format!("tcp/[{v6}]:{port}"),
    }
}

/// Run `jobs` concurrently (one detached thread each) and collect every result
/// reporting back within `budget`. A `None` result contributes nothing; a job
/// still running at the deadline is abandoned so one hung DNS lookup cannot
/// exceed the budget. Detached threads (not `std::thread::scope`) are what make
/// the bound real — a scope would join every straggler.
fn run_bounded<T, F>(jobs: Vec<F>, budget: Duration) -> Vec<T>
where
    T: Send + 'static,
    F: FnOnce() -> Option<T> + Send + 'static,
{
    let n = jobs.len();
    let (tx, rx) = mpsc::channel();
    for job in jobs {
        let tx = tx.clone();
        std::thread::spawn(move || {
            let _ = tx.send(job());
        });
    }
    drop(tx);
    let deadline = Instant::now() + budget;
    let mut out = Vec::new();
    for _ in 0..n {
        let remaining = deadline.saturating_duration_since(Instant::now());
        match rx.recv_timeout(remaining) {
            Ok(Some(v)) => out.push(v),
            Ok(None) => {}
            Err(_) => break,
        }
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::net::{Ipv4Addr, Ipv6Addr, SocketAddrV4, SocketAddrV6};

    const PORT: u16 = 7683;

    fn owned(pairs: &[(&str, u16)]) -> Vec<(String, u16)> {
        pairs.iter().map(|(h, p)| (h.to_string(), *p)).collect()
    }

    #[test]
    fn parse_bare_host_uses_default_port() {
        assert_eq!(parse_peers_spec("go2", PORT), owned(&[("go2", PORT)]));
    }

    /// A misspelled `peers` key is REPORTED, and the document
    /// still parses (the never-bricks posture — this is a warn, not a deny).
    ///
    /// The oracle matches the LEVEL TOKEN as well as the message: this is the
    /// one place a level regression is invisible, since a `debug!` would leave
    /// the peer list empty exactly as the silence did.
    #[tracing_test::traced_test]
    #[test]
    fn a_misspelled_peers_key_warns_and_the_document_still_parses() {
        // The likeliest misspelling. Without the warn: parses clean, yields NO
        // peers, says nothing.
        let peers = parse_config_peers("peer = [\"go2\", \"orin\"]\n");
        assert!(peers.is_empty(), "a misspelled key still yields no peers");
        assert!(
            logs_contain("looks like a misspelling"),
            "a near-miss key must be reported"
        );
        // The structured fields carry which key and which spelling — the
        // message deliberately does not interpolate them.
        assert!(logs_contain("found=peer"), "the offending key is named");
        assert!(logs_contain("expected=peers"), "and the intended one");
        assert!(logs_contain("WARN"), "at WARN, not below it");
    }

    /// A document that will not DESERIALIZE gets exactly ONE
    /// warn — the parse one — never a near-miss line on top.
    ///
    /// The doc for this function promises one warn for "malformed TOML (or a
    /// `peers` of the wrong shape)", and a wrongly-typed `peers` beside a
    /// near-miss key produced two: a line saying the peers were ignored, and
    /// a second about a different key, which reads like a second fault when
    /// the operator has already been told the actionable thing.
    #[tracing_test::traced_test]
    #[test]
    fn a_wrongly_shaped_peers_gets_one_warn_not_two() {
        // Syntactically VALID TOML, so the near-miss walk would happily parse
        // it — the type failure is downstream, which is what made the double
        // line reachable.
        let peers = parse_config_peers("peers = 1\npeer = [\"go2\"]\n");
        assert!(peers.is_empty(), "a wrongly-typed peers yields none");
        assert!(
            logs_contain("failed to parse ~/.cerulion/config.toml"),
            "the parse failure is the ONE warn this document earns"
        );
        assert!(
            !logs_contain("looks like a misspelling"),
            "and the near-miss line must not ride on top of it"
        );

        // ANTI-TAUTOLOGY: the same near-miss key in a document that DOES
        // deserialize is still reported — the gate suppresses a redundant
        // line, not the feature.
        let peers = parse_config_peers("peers = [\"go2\"]\npeer = [\"orin\"]\n");
        assert_eq!(peers, vec!["go2".to_string()]);
        assert!(
            logs_contain("looks like a misspelling"),
            "a parseable document still reports its near-miss key"
        );
    }

    /// ANTI-TAUTOLOGY, and the reason this is a warn rather than a deny: the
    /// CORRECT spelling is silent, and so is a genuinely FOREIGN key — this
    /// file is named for general configuration and we read one key from it.
    #[tracing_test::traced_test]
    #[test]
    fn a_correct_key_and_a_foreign_key_are_both_silent() {
        let peers = parse_config_peers("peers = [\"go2\"]\ntheme = \"dark\"\n");
        assert_eq!(peers, vec!["go2".to_string()], "the correct key is read");
        assert!(
            !logs_contain("looks like a misspelling"),
            "neither `peers` nor `theme` may be reported as a misspelling"
        );
    }

    #[test]
    fn parse_host_with_explicit_port() {
        assert_eq!(parse_peers_spec("go2:7684", PORT), owned(&[("go2", 7684)]));
    }

    #[test]
    fn parse_multi_entry_mixed_ports() {
        assert_eq!(
            parse_peers_spec("go2, rover:1", PORT),
            owned(&[("go2", PORT), ("rover", 1)])
        );
    }

    #[test]
    fn parse_bracketed_ipv6_with_port_strips_brackets() {
        assert_eq!(
            parse_peers_spec("[::1]:7683", PORT),
            owned(&[("::1", 7683)])
        );
    }

    #[test]
    fn parse_bracketed_ipv6_without_port_uses_default() {
        assert_eq!(parse_peers_spec("[::1]", PORT), owned(&[("::1", PORT)]));
    }

    #[test]
    fn parse_bare_ipv6_with_port_splits_at_last_colon() {
        // fe80::1:7683 → host fe80::1, port 7683 (the tail parses as u16).
        assert_eq!(
            parse_peers_spec("fe80::1:7683", PORT),
            owned(&[("fe80::1", 7683)])
        );
    }

    #[test]
    fn parse_ipv4_literal_no_port() {
        assert_eq!(
            parse_peers_spec("10.0.0.5", PORT),
            owned(&[("10.0.0.5", PORT)])
        );
    }

    #[test]
    fn parse_trailing_comma_skips_empty() {
        assert_eq!(parse_peers_spec("go2,", PORT), owned(&[("go2", PORT)]));
    }

    #[test]
    fn parse_whitespace_is_trimmed() {
        assert_eq!(
            parse_peers_spec("  go2 , rover ", PORT),
            owned(&[("go2", PORT), ("rover", PORT)])
        );
    }

    #[test]
    fn parse_out_of_range_port_is_skipped() {
        assert_eq!(parse_peers_spec("bad:99999", PORT), Vec::new());
    }

    #[test]
    fn parse_empty_spec_is_empty() {
        assert_eq!(parse_peers_spec("", PORT), Vec::new());
    }

    #[test]
    fn parse_bad_port_entry_skipped_others_kept() {
        // The invalid entry drops out; the valid siblings survive.
        assert_eq!(
            parse_peers_spec("go2, bad:99999, rover:8", PORT),
            owned(&[("go2", PORT), ("rover", 8)])
        );
    }

    #[test]
    fn config_peers_reads_array() {
        assert_eq!(
            parse_config_peers("peers = [\"go2\", \"rover:7\"]"),
            vec!["go2".to_string(), "rover:7".to_string()]
        );
    }

    #[test]
    fn config_peers_missing_key_is_empty() {
        assert_eq!(
            parse_config_peers("other = 1\nname = \"lab\""),
            Vec::<String>::new()
        );
    }

    #[test]
    fn config_peers_malformed_toml_is_empty() {
        assert_eq!(parse_config_peers("peers = ["), Vec::<String>::new());
    }

    #[test]
    fn select_addr_prefers_ipv4() {
        let v4 = SocketAddr::V4(SocketAddrV4::new(Ipv4Addr::new(10, 0, 0, 5), PORT));
        let v6 = SocketAddr::V6(SocketAddrV6::new(Ipv6Addr::LOCALHOST, PORT, 0, 0));
        // v6 listed first, v4 still wins.
        assert_eq!(select_addr(&[v6, v4]), Some(v4));
    }

    #[test]
    fn select_addr_falls_back_to_ipv6() {
        let v6 = SocketAddr::V6(SocketAddrV6::new(Ipv6Addr::LOCALHOST, PORT, 0, 0));
        assert_eq!(select_addr(&[v6]), Some(v6));
    }

    #[test]
    fn select_addr_empty_is_none() {
        assert_eq!(select_addr(&[]), None);
    }

    #[test]
    fn locator_from_ipv4_is_unbracketed() {
        assert_eq!(
            locator_from_ip(IpAddr::V4(Ipv4Addr::new(1, 2, 3, 4)), PORT),
            "tcp/1.2.3.4:7683"
        );
    }

    #[test]
    fn locator_from_ipv6_is_bracketed() {
        assert_eq!(
            locator_from_ip(IpAddr::V6(Ipv6Addr::LOCALHOST), PORT),
            "tcp/[::1]:7683"
        );
    }
}
