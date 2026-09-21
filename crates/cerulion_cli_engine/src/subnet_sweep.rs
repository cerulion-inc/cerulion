// SPDX-License-Identifier: AGPL-3.0-only
//! Rung 4: the OPT-IN unicast subnet-sweep discovery rung (`--scan`).
//!
//! When mDNS, the cache, the hostname convention, and zenoh scouting all come
//! up empty — a network that blocks BOTH multicast AND mDNS reflection — the
//! last LAN rung is a plain unicast sweep of the local IPv4 `/24`(s) on the
//! well-known gateway port: try to `connect()` to `<host>:<port>` for every
//! address on the wire, and whatever answers is a candidate gateway.
//!
//! # Why opt-in — and STRUCTURALLY so
//!
//! A horizontal `connect()` sweep across a `/24` is exactly the traffic
//! signature corporate/enterprise IDS flags as port-scan reconnaissance. So the
//! sweep is opt-in ONLY (`cerulion topic list --scan`) and it must be
//! impossible to trigger by accident: the rung is wired into the ladder solely
//! inside [`crate::discovery_ladder::discover_peers`]'s `if scan { … }` block,
//! and the only producer of that `scan` boolean is the `--scan` CLI flag — no
//! env var, no config key reaches it. On a default run the sweep never fires.
//!
//! # How it works (two phases, all bounded)
//!
//! 1. **Enumerate** the local non-loopback IPv4 interfaces and, for each, the
//!    host addresses of its subnet. Only PRIVATE / CGNAT interfaces are swept
//!    (RFC 1918 `10/8` `172.16/12` `192.168/16`, plus CGNAT/Tailscale
//!    `100.64/10`); a PUBLIC-IP interface is skipped with a warn — horizontally
//!    scanning third-party public hosts is exactly the recon the opt-in gate
//!    exists to prevent. A subnet WIDER than `/24` is clamped to the host's own
//!    `/24` (`MIN_SWEEP_PREFIX`) so a `/16` (65k hosts) is never swept; a
//!    NARROWER subnet (`/25`+) is respected verbatim (never probe outside your
//!    own broadcast domain). Network, broadcast, and EVERY local interface
//!    address are excluded — including a sibling interface sharing the swept
//!    `/24`, so its own gateway port is never self-probed.
//! 2. **Sweep** (`tcp_connect_sweep`): a bounded pool of worker threads
//!    (`SWEEP_CONCURRENCY`) `connect_timeout`s each candidate; whatever answers
//!    on the gateway port becomes a [`DiscoveryRung::Scan`] CANDIDATE (robot =
//!    the host IP string). The sweep opens NO zenoh session of its own —
//!    each open-port survivor's locator is folded into the caller's ONE query
//!    session, and PRESENCE in the announce gather (not a per-survivor beacon
//!    probe) is what confirms it live. An unrelated open port that is not a
//!    Cerulion gateway simply never surfaces an announce token, so it creates no
//!    ROBOTS row.
//!
//! Every phase is bounded by the caller's `budget`; a hung host never delays
//! the others. Like the other rungs, [`sweep_rung`] is INFALLIBLE — an interface
//! error, an empty LAN, or unreachable hosts all yield an empty `Vec`, never an
//! error.
//!
//! # Testing note
//!
//! [`sweep_rung`] itself does real network I/O (interface enumeration + live
//! `connect`s), so it has no direct CI test — that would be non-hermetic and
//! flaky. Its pieces are pure or loopback-hermetic and ARE oracle-tested below:
//! `subnet_prefix_len` / `sweep_targets` / `sweep_candidate_targets` /
//! `subnets_from_interfaces` / `is_sweepable_private` / `subnet_cidr` /
//! `candidates_from_open` (pure), and `tcp_connect_sweep` (against a live
//! loopback listener + a closed port).

use std::collections::HashSet;
use std::net::{Ipv4Addr, SocketAddr, SocketAddrV4};
use std::sync::mpsc;
use std::thread;
use std::time::{Duration, Instant};

use crate::discovery_ladder::{DiscoveredPeer, DiscoveryRung};

/// The narrowest subnet the sweep will WIDEN a mask to — a `/24` (256 addrs).
/// A subnet wider than this (a shorter prefix, e.g. a `/16`) is clamped DOWN to
/// the host's own `/24`, capping any one sweep at ≤254 unicast probes. A subnet
/// already `/24` or narrower is swept verbatim.
const MIN_SWEEP_PREFIX: u32 = 24;

/// Per-candidate TCP `connect` timeout in the sweep's phase 2. On a LAN a closed
/// port refuses instantly; this bounds only the firewalled/dropped-SYN case.
const SWEEP_PROBE_TIMEOUT: Duration = Duration::from_millis(300);

/// Wall-clock cap on phase 2 (the whole connect sweep). Sized so a `/24` at
/// [`SWEEP_CONCURRENCY`] workers completes several `SWEEP_PROBE_TIMEOUT` batches
/// within [`crate::discovery_ladder::SWEEP_RUNG_BUDGET`] (the sweep is
/// now connect-only — no separate verify phase to leave room for).
const SWEEP_CONNECT_BUDGET: Duration = Duration::from_millis(1800);

/// How many candidate `connect`s run concurrently in phase 2. Bounds the thread
/// count (a `/24` is ≤254 targets) while keeping the sweep wall well under the
/// connect budget.
const SWEEP_CONCURRENCY: usize = 64;

/// The opt-in subnet-sweep rung. Enumerates the local `/24`(s), sweeps the
/// well-known gateway port, and returns each open-port survivor as a
/// [`DiscoveryRung::Scan`] CANDIDATE (robot = the host IP string). The
/// sweep opens NO zenoh session — the caller folds each survivor's locator into
/// its ONE query session and the announce gather confirms which are real
/// gateways. INFALLIBLE — any failure (no interface, empty LAN, unreachable
/// hosts) yields an empty `Vec`.
///
/// Reached ONLY via [`crate::discovery_ladder::discover_peers`]`(scan = true)`,
/// itself reachable only through the `--scan` CLI flag (see the module docs).
pub fn sweep_rung(budget: Duration) -> Vec<DiscoveredPeer> {
    let deadline = Instant::now() + budget;
    let port = sweep_port();

    // First phase: enumerate the sweepable (private/CGNAT) subnets AND every local
    // interface IPv4 for self-exclusion.
    let (subnets, all_local_ips) = local_ipv4_subnets();
    if subnets.is_empty() {
        tracing::warn!(
            "--scan: no sweepable private IPv4 interface found — nothing to sweep; \
             reach a peer directly with `--connect tcp/<host>:{port}`"
        );
        return Vec::new();
    }

    // Build the candidate set across every local subnet, dropping EVERY local
    // interface address — including a sibling on the same /24 that is NOT the
    // deduped `subnets` representative (a self-probe would beacon-answer our OWN
    // gateway and surface the local machine as a phantom Scan peer).
    let targets = sweep_candidate_targets(&subnets, &all_local_ips, port);
    if targets.is_empty() {
        tracing::debug!("--scan: every candidate resolved to a local address — nothing to sweep");
        return Vec::new();
    }

    // Opt-in audit line: name the EXACT swept CIDRs so an IDS-flagged sweep can be
    // correlated to a log line.
    let cidrs: Vec<String> = subnets
        .iter()
        .map(|(ip, nm)| subnet_cidr(*ip, *nm))
        .collect();
    tracing::info!(
        subnets = subnets.len(),
        subnet_cidrs = ?cidrs,
        candidates = targets.len(),
        port,
        "--scan: sweeping the local /24(s) for gateways (opt-in; \
         reads as a port scan to some IDS)"
    );

    // Second phase: the SYN sweep, bounded to a fraction of the rung budget.
    let connect_budget = SWEEP_CONNECT_BUDGET.min(remaining_until(deadline));
    let open = tcp_connect_sweep(targets, SWEEP_PROBE_TIMEOUT, connect_budget);
    if open.is_empty() {
        tracing::debug!("--scan: no host answered on the gateway port");
        return Vec::new();
    }

    // Each open-port survivor becomes a Scan CANDIDATE directly (the
    // pure `candidates_from_open`). Its locator is folded into the caller's ONE
    // query session; PRESENCE in the gather (an announce token arriving —
    // mDNS would have found a reachable-by-mDNS gateway already) confirms it
    // live, so an unrelated open port that is not a Cerulion gateway simply
    // surfaces no announce token and creates no ROBOTS row.
    candidates_from_open(open)
}

/// Map open-port sweep survivors to
/// [`DiscoveryRung::Scan`] CANDIDATES — robot = the host IP string (an
/// anonymous but reachable address), locator = `tcp/{ip}:{port}` (the exact
/// zenoh locator format the query session's connect list expects). Pure —
/// oracle-tested below (the sweep rung itself does live network I/O and is
/// untestable in CI, so its output mapping is pinned here).
fn candidates_from_open(open: Vec<SocketAddrV4>) -> Vec<DiscoveredPeer> {
    open.into_iter()
        .map(|addr| DiscoveredPeer {
            robot: addr.ip().to_string(),
            locator: format!("tcp/{}:{}", addr.ip(), addr.port()),
            rung: DiscoveryRung::Scan,
        })
        .collect()
}

/// The gateway port the sweep probes — the same well-known base port the
/// permissive gateway binds ([`crate::graph_cmd::resolve_gateway_base_port`],
/// honoring the `CERULION_GATEWAY_PORT` override). A malformed override warns and
/// falls back to the well-known port rather than skipping the sweep.
fn sweep_port() -> u16 {
    match crate::graph_cmd::resolve_gateway_base_port() {
        Ok(p) => p,
        Err(e) => {
            tracing::warn!(
                error = %e,
                "--scan: invalid CERULION_GATEWAY_PORT — sweeping the well-known port {}",
                crate::graph_cmd::GATEWAY_WELL_KNOWN_PORT
            );
            crate::graph_cmd::GATEWAY_WELL_KNOWN_PORT
        }
    }
}

/// Enumerate the local non-loopback IPv4 picture the sweep needs: the deduped,
/// private-only `(ip, netmask)` subnets to sweep, AND every local interface IPv4
/// seen (pre-dedup, pre-filter) so self-exclusion catches a sibling interface on
/// a swept `/24` that is not the deduped subnet representative. INFALLIBLE — an
/// interface-enumeration error warns and yields two empty `Vec`s.
fn local_ipv4_subnets() -> (Vec<(Ipv4Addr, Ipv4Addr)>, Vec<Ipv4Addr>) {
    let ifaces = match if_addrs::get_if_addrs() {
        Ok(v) => v,
        Err(e) => {
            tracing::warn!(
                error = %e,
                "--scan: cannot enumerate local network interfaces — skipping the sweep"
            );
            return (Vec::new(), Vec::new());
        }
    };
    let v4: Vec<(Ipv4Addr, Ipv4Addr)> = ifaces
        .into_iter()
        .filter_map(|iface| match iface.addr {
            if_addrs::IfAddr::V4(v4) => Some((v4.ip, v4.netmask)),
            if_addrs::IfAddr::V6(_) => None,
        })
        .collect();
    // Self-exclusion is built from ALL enumerated V4 addresses (a public or
    // loopback address is harmless here — it is never in a swept subnet — but
    // a private sibling on a swept /24 MUST be excluded).
    let all_ips: Vec<Ipv4Addr> = v4.iter().map(|(ip, _)| *ip).collect();
    (subnets_from_interfaces(v4), all_ips)
}

/// Build the deduped candidate target set across every swept subnet, excluding
/// EVERY local interface IPv4 in `all_local_ips` — not merely the deduped subnet
/// representatives — so a sibling interface sharing a swept `/24` is never
/// probed. Self-probing our own gateway port would beacon-answer our OWN gateway
/// and surface the local machine as a phantom [`DiscoveryRung::Scan`] peer. Pure
/// — oracle-tested.
fn sweep_candidate_targets(
    subnets: &[(Ipv4Addr, Ipv4Addr)],
    all_local_ips: &[Ipv4Addr],
    port: u16,
) -> Vec<SocketAddrV4> {
    let local_ips: HashSet<Ipv4Addr> = all_local_ips.iter().copied().collect();
    let mut targets: Vec<SocketAddrV4> = Vec::new();
    for (ip, netmask) in subnets {
        targets.extend(sweep_targets(*ip, *netmask, port));
    }
    targets.retain(|t| !local_ips.contains(t.ip()));
    targets.sort();
    targets.dedup();
    targets
}

/// Filter raw interface `(ip, netmask)` pairs down to the subnets worth
/// sweeping: drop loopback / link-local (APIPA `169.254/16`) / unspecified /
/// broadcast addresses (no robot lives there), and DEDUPE so two of our IPs on
/// one subnet produce a single sweep. Keeps the first `(ip, netmask)` seen for
/// each distinct (network, clamped-prefix). Pure — oracle-tested.
fn subnets_from_interfaces(ifaces: Vec<(Ipv4Addr, Ipv4Addr)>) -> Vec<(Ipv4Addr, Ipv4Addr)> {
    let mut out = Vec::new();
    let mut seen: HashSet<(u32, u32)> = HashSet::new();
    for (ip, netmask) in ifaces {
        if ip.is_loopback() || ip.is_link_local() || ip.is_unspecified() || ip.is_broadcast() {
            continue;
        }
        // Defense-in-depth: never horizontally scan a PUBLIC /24 (worse recon
        // optics than sweeping your own LAN, against the opt-in-safety rationale).
        if !is_sweepable_private(ip) {
            tracing::warn!(
                ip = %ip,
                "--scan: skipping public-IP interface — the sweep probes only \
                 private/CGNAT LANs (recon safety); reach a peer directly with \
                 `--connect tcp/<host>:<port>`"
            );
            continue;
        }
        let prefix = clamped_prefix(netmask);
        let network = u32::from(ip) & mask_for_prefix(prefix);
        if seen.insert((network, prefix)) {
            out.push((ip, netmask));
        }
    }
    out
}

/// Is `ip` in a range the sweep is willing to probe? Only PRIVATE / CGNAT
/// space — RFC 1918 (`10/8`, `172.16/12`, `192.168/16`) and CGNAT `100.64/10`
/// (which Tailscale's `100.x` mesh also lives in). A PUBLIC IPv4 returns
/// `false`: a horizontal port scan of third-party public hosts is strictly worse
/// recon than sweeping your own LAN and defeats the whole opt-in-safety
/// rationale. Loopback / link-local / unspecified / broadcast are already
/// excluded upstream; this adds the public-IP skip. Pure — oracle-tested.
fn is_sweepable_private(ip: Ipv4Addr) -> bool {
    // `Ipv4Addr::is_private` covers 10/8, 172.16/12, 192.168/16. CGNAT 100.64/10
    // is NOT `is_private`, so add it explicitly.
    ip.is_private() || is_cgnat(ip)
}

/// The CGNAT / shared-address block `100.64.0.0/10` (RFC 6598) —
/// `100.64.0.0`..=`100.127.255.255`. Tailscale hands nodes addresses here. Pure.
fn is_cgnat(ip: Ipv4Addr) -> bool {
    let o = ip.octets();
    o[0] == 100 && (64..=127).contains(&o[1])
}

/// The `network/prefix` CIDR string for a swept `(ip, netmask)` — the ACTUAL
/// clamped range the sweep probes (a `/8` renders as the host's own `/24`), for
/// audit correlation. Pure — oracle-tested.
fn subnet_cidr(ip: Ipv4Addr, netmask: Ipv4Addr) -> String {
    let prefix = clamped_prefix(netmask);
    let network = Ipv4Addr::from(u32::from(ip) & mask_for_prefix(prefix));
    format!("{network}/{prefix}")
}

/// The prefix length of a netmask: the number of leading `1` bits. A
/// non-contiguous mask (illegal, but defensive) counts only its leading run,
/// yielding a wider subnet that the clamp then narrows to `/24`. Pure.
fn subnet_prefix_len(netmask: Ipv4Addr) -> u32 {
    u32::from(netmask).leading_ones()
}

/// The effective sweep prefix for a mask: never wider than `/24`
/// ([`MIN_SWEEP_PREFIX`]), never beyond `/32`.
fn clamped_prefix(netmask: Ipv4Addr) -> u32 {
    subnet_prefix_len(netmask).clamp(MIN_SWEEP_PREFIX, 32)
}

/// The 32-bit mask for a prefix length in `[24, 32]` (the clamped range).
fn mask_for_prefix(prefix: u32) -> u32 {
    // prefix ∈ [24, 32] ⇒ shift ∈ [0, 8], always in range.
    u32::MAX << (32 - prefix)
}

/// The candidate sweep targets for one local `(ip, netmask)`: every host address
/// in the subnet EXCLUDING the network address, the broadcast address, and the
/// host's own address. The mask is clamped to `/24` first (a wider subnet sweeps
/// only the host's own `/24`; a `/25`+ subnet stays verbatim), so the result is
/// always ≤254 addresses. A `/31` or `/32` has no sweepable hosts ⇒ empty. Pure
/// — oracle-tested.
fn sweep_targets(local: Ipv4Addr, netmask: Ipv4Addr, port: u16) -> Vec<SocketAddrV4> {
    let prefix = clamped_prefix(netmask);
    // A /31 (RFC 3021 point-to-point, 2 addrs both endpoints) or /32 (single
    // host) has no interior host range to sweep.
    if prefix >= 31 {
        return Vec::new();
    }
    let mask = mask_for_prefix(prefix);
    let local_u = u32::from(local);
    let network = local_u & mask;
    let broadcast = network | !mask;

    // Hosts are strictly between the network and broadcast addresses.
    ((network + 1)..broadcast)
        .filter(|&host| host != local_u)
        .map(|host| SocketAddrV4::new(Ipv4Addr::from(host), port))
        .collect()
}

/// The second phase — the unicast connect sweep. `connect_timeout`s every target on a
/// bounded pool of [`SWEEP_CONCURRENCY`] worker threads and returns the addresses
/// that ACCEPTED a connection (their gateway port is open). Bounded by `budget`:
/// a worker stops pulling new targets past the deadline, and per-probe timeouts
/// are clamped to the remaining window, so a wall of firewalled hosts cannot
/// overrun. Targets are round-robin-partitioned across workers so a contiguous
/// dropped-SYN range does not pile onto one worker.
fn tcp_connect_sweep(
    mut targets: Vec<SocketAddrV4>,
    probe_timeout: Duration,
    budget: Duration,
) -> Vec<SocketAddrV4> {
    targets.sort();
    targets.dedup();
    if targets.is_empty() {
        return Vec::new();
    }
    let deadline = Instant::now() + budget;
    let workers = SWEEP_CONCURRENCY.min(targets.len());

    // Round-robin partition so a contiguous firewalled block spreads across
    // workers instead of stalling one.
    let mut chunks: Vec<Vec<SocketAddrV4>> = (0..workers).map(|_| Vec::new()).collect();
    for (i, t) in targets.into_iter().enumerate() {
        chunks[i % workers].push(t);
    }

    let (tx, rx) = mpsc::channel::<SocketAddrV4>();
    for chunk in chunks {
        let chunk_len = chunk.len();
        let tx = tx.clone();
        // Best-effort worker: a spawn failure (resource exhaustion) just means
        // that chunk is not probed — the sweep degrades, never fails.
        let spawn_result = thread::Builder::new()
            .name("subnet-sweep".to_string())
            .spawn(move || {
                for target in chunk {
                    let remaining = remaining_until(deadline);
                    if remaining.is_zero() {
                        break;
                    }
                    // The shared bounded TCP probe (also used by the `topic list`
                    // ladder pre-filter) — one "is this port open?" primitive.
                    let per = probe_timeout.min(remaining);
                    if crate::discovery_ladder::tcp_port_open(SocketAddr::V4(target), per) {
                        // A send failure only means the collector already hit its
                        // deadline and hung up — safe to ignore.
                        let _ = tx.send(target);
                    }
                }
            });
        if let Err(e) = spawn_result {
            // Don't drop the chunk silently — a lost worker means unswept
            // candidates, so some gateways may be missed.
            tracing::warn!(
                error = %e,
                dropped_targets = chunk_len,
                "--scan: failed to spawn a sweep worker — some gateways may be missed"
            );
        }
    }
    drop(tx);

    let mut open = Vec::new();
    loop {
        let remaining = remaining_until(deadline);
        if remaining.is_zero() {
            break;
        }
        // Ends on the deadline OR when every worker's sender has dropped.
        match rx.recv_timeout(remaining) {
            Ok(addr) => open.push(addr),
            Err(_) => break,
        }
    }
    open.sort();
    open.dedup();
    open
}

/// Time left until `deadline`, saturating to zero once past it.
fn remaining_until(deadline: Instant) -> Duration {
    deadline.saturating_duration_since(Instant::now())
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::net::TcpListener;

    fn sock(a: &str, port: u16) -> SocketAddrV4 {
        SocketAddrV4::new(a.parse().unwrap(), port)
    }

    #[test]
    fn subnet_prefix_len_counts_leading_ones() {
        assert_eq!(subnet_prefix_len(Ipv4Addr::new(255, 255, 255, 0)), 24);
        assert_eq!(subnet_prefix_len(Ipv4Addr::new(255, 255, 0, 0)), 16);
        assert_eq!(subnet_prefix_len(Ipv4Addr::new(255, 255, 255, 128)), 25);
        assert_eq!(subnet_prefix_len(Ipv4Addr::new(255, 255, 255, 192)), 26);
        assert_eq!(subnet_prefix_len(Ipv4Addr::new(255, 255, 255, 255)), 32);
        assert_eq!(subnet_prefix_len(Ipv4Addr::new(0, 0, 0, 0)), 0);
    }

    #[test]
    fn clamped_prefix_never_wider_than_slash24() {
        // A /8 mask widens the sweep too far → clamped to /24.
        assert_eq!(clamped_prefix(Ipv4Addr::new(255, 0, 0, 0)), 24);
        // A /24 stays /24; a /26 stays /26 (narrower is respected).
        assert_eq!(clamped_prefix(Ipv4Addr::new(255, 255, 255, 0)), 24);
        assert_eq!(clamped_prefix(Ipv4Addr::new(255, 255, 255, 192)), 26);
        assert_eq!(clamped_prefix(Ipv4Addr::new(255, 255, 255, 255)), 32);
    }

    #[test]
    fn sweep_targets_slash24_excludes_self_network_broadcast() {
        let got = sweep_targets(
            Ipv4Addr::new(192, 168, 1, 50),
            Ipv4Addr::new(255, 255, 255, 0),
            7683,
        );
        // 256 addrs − network(.0) − broadcast(.255) − self(.50) = 253.
        assert_eq!(got.len(), 253);
        assert_eq!(got[0], sock("192.168.1.1", 7683), "first host is .1");
        assert_eq!(*got.last().unwrap(), sock("192.168.1.254", 7683));
        assert!(
            !got.contains(&sock("192.168.1.0", 7683)),
            "network excluded"
        );
        assert!(
            !got.contains(&sock("192.168.1.255", 7683)),
            "broadcast excluded"
        );
        assert!(!got.contains(&sock("192.168.1.50", 7683)), "self excluded");
        assert!(got.contains(&sock("192.168.1.49", 7683)));
        assert!(got.contains(&sock("192.168.1.51", 7683)));
        // Every target carries the requested port and shares the /24.
        assert!(got
            .iter()
            .all(|t| t.port() == 7683 && t.ip().octets()[..3] == [192, 168, 1]));
    }

    #[test]
    fn sweep_targets_clamps_wide_subnet_to_hosts_slash24() {
        // A /8 mask must NOT produce 16M targets — it sweeps only the host's /24.
        let got = sweep_targets(
            Ipv4Addr::new(10, 1, 2, 5),
            Ipv4Addr::new(255, 0, 0, 0),
            7683,
        );
        assert_eq!(got.len(), 253, "clamped to the host's own /24 minus self");
        assert!(
            got.iter().all(|t| t.ip().octets()[..3] == [10, 1, 2]),
            "every candidate is inside the host's /24 (10.1.2.x)"
        );
        assert!(!got.contains(&sock("10.1.2.5", 7683)), "self excluded");
    }

    #[test]
    fn sweep_targets_respects_narrow_subnet() {
        // A /26 (mask .192): network .64, broadcast .127 → hosts .65..=.126.
        let got = sweep_targets(
            Ipv4Addr::new(192, 168, 1, 66),
            Ipv4Addr::new(255, 255, 255, 192),
            7683,
        );
        // 64 addrs − network − broadcast − self = 61.
        assert_eq!(got.len(), 61);
        assert_eq!(got[0], sock("192.168.1.65", 7683));
        assert_eq!(*got.last().unwrap(), sock("192.168.1.126", 7683));
        assert!(
            !got.contains(&sock("192.168.1.64", 7683)),
            "network excluded"
        );
        assert!(
            !got.contains(&sock("192.168.1.127", 7683)),
            "broadcast excluded"
        );
        assert!(!got.contains(&sock("192.168.1.66", 7683)), "self excluded");
        // Nothing outside the /26 window.
        assert!(!got.contains(&sock("192.168.1.63", 7683)));
        assert!(!got.contains(&sock("192.168.1.128", 7683)));
    }

    #[test]
    fn sweep_targets_slash31_and_slash32_have_no_hosts() {
        assert!(sweep_targets(
            Ipv4Addr::new(10, 0, 0, 2),
            Ipv4Addr::new(255, 255, 255, 254), // /31
            7683,
        )
        .is_empty());
        assert!(sweep_targets(
            Ipv4Addr::new(10, 0, 0, 2),
            Ipv4Addr::new(255, 255, 255, 255), // /32
            7683,
        )
        .is_empty());
    }

    #[test]
    fn subnets_from_interfaces_filters_and_dedupes() {
        let ifaces = vec![
            (Ipv4Addr::new(127, 0, 0, 1), Ipv4Addr::new(255, 0, 0, 0)), // loopback → drop
            (Ipv4Addr::new(169, 254, 3, 4), Ipv4Addr::new(255, 255, 0, 0)), // link-local → drop
            (Ipv4Addr::new(0, 0, 0, 0), Ipv4Addr::new(0, 0, 0, 0)),     // unspecified → drop
            (
                Ipv4Addr::new(192, 168, 1, 10),
                Ipv4Addr::new(255, 255, 255, 0),
            ), // keep (first of this /24)
            (
                Ipv4Addr::new(192, 168, 1, 11),
                Ipv4Addr::new(255, 255, 255, 0),
            ), // same /24 → dedupe
            (Ipv4Addr::new(10, 5, 6, 7), Ipv4Addr::new(255, 255, 255, 0)), // keep (distinct /24)
        ];
        assert_eq!(
            subnets_from_interfaces(ifaces),
            vec![
                (
                    Ipv4Addr::new(192, 168, 1, 10),
                    Ipv4Addr::new(255, 255, 255, 0)
                ),
                (Ipv4Addr::new(10, 5, 6, 7), Ipv4Addr::new(255, 255, 255, 0)),
            ],
            "loopback/link-local/unspecified dropped; the first IP of each /24 kept"
        );
    }

    #[test]
    fn tcp_connect_sweep_empty_targets_is_empty() {
        assert!(tcp_connect_sweep(
            Vec::new(),
            Duration::from_millis(50),
            Duration::from_millis(200)
        )
        .is_empty());
    }

    /// HERMETIC (loopback): a live listener's port is ALWAYS reported open; a
    /// just-freed (closed) port is not. No external network.
    ///
    /// The OPEN half is a deterministic pin. The CLOSED half tolerates the rare
    /// flake where a concurrent process on a busy CI runner grabs the just-freed
    /// ephemeral port between our `drop` and the probe — a guaranteed-closed AND
    /// guaranteed-unreusable loopback port is impossible (holding it bound would
    /// make it listen ⇒ open), so tolerate-reuse is the least-flaky hermetic
    /// option. The open-half pin is never weakened.
    #[test]
    fn tcp_connect_sweep_finds_open_ignores_closed() {
        let listener = TcpListener::bind("127.0.0.1:0").expect("bind loopback listener");
        let open_addr = match listener.local_addr().unwrap() {
            SocketAddr::V4(a) => a,
            SocketAddr::V6(_) => unreachable!("bound 127.0.0.1"),
        };
        // A (usually) closed port: bind an ephemeral, capture it, then drop the
        // listener so nothing is listening there for the test window.
        let closed_addr = {
            let l = TcpListener::bind("127.0.0.1:0").expect("bind ephemeral");
            let a = match l.local_addr().unwrap() {
                SocketAddr::V4(a) => a,
                SocketAddr::V6(_) => unreachable!("bound 127.0.0.1"),
            };
            drop(l);
            a
        };

        let got = tcp_connect_sweep(
            vec![open_addr, closed_addr],
            Duration::from_millis(200),
            Duration::from_secs(2),
        );
        // (a) DETERMINISTIC: the live, listening loopback port is always found.
        assert!(
            got.contains(&open_addr),
            "a bound+listening loopback port must be reported open: got {got:?}"
        );
        if got.contains(&closed_addr) {
            // (b) A concurrent process re-bound the freed port between drop and
            // probe (the tolerated rare race). Assert only that no PHANTOM address
            // beyond the two we probed appears — the result is still trustworthy.
            assert!(
                got.iter().all(|a| *a == open_addr || *a == closed_addr),
                "no phantom addresses beyond the two probed: got {got:?}"
            );
        } else {
            // (b) The common case: the freed port refused (loopback RSTs instantly).
            assert_eq!(
                got,
                vec![open_addr],
                "only the live listener's port is open (loopback refuses the closed one)"
            );
        }
    }

    #[test]
    fn sweep_candidate_targets_excludes_every_local_ip_on_shared_subnet() {
        // Dual-homed host: eth0=.10 and eth1=.11 BOTH on 192.168.1.0/24. The
        // deduped subnet list carries only ONE representative (.10), but
        // self-exclusion must catch the sibling .11 too.
        let subnets = vec![(
            Ipv4Addr::new(192, 168, 1, 10),
            Ipv4Addr::new(255, 255, 255, 0),
        )];
        let all_local_ips = vec![
            Ipv4Addr::new(192, 168, 1, 10),
            Ipv4Addr::new(192, 168, 1, 11),
        ];
        let got = sweep_candidate_targets(&subnets, &all_local_ips, 7683);
        assert!(
            !got.contains(&sock("192.168.1.10", 7683)),
            "own subnet IP excluded"
        );
        assert!(
            !got.contains(&sock("192.168.1.11", 7683)),
            "sibling interface IP on the same /24 excluded (not in the deduped subnets)"
        );
        // A real neighbour IS probed.
        assert!(got.contains(&sock("192.168.1.20", 7683)));
        // /24 host range (254) minus BOTH local IPs (.10, .11) = 252.
        assert_eq!(got.len(), 252, "254 hosts minus the two local addresses");
    }

    #[test]
    fn is_sweepable_private_accepts_private_cgnat_rejects_public() {
        // RFC 1918.
        assert!(is_sweepable_private(Ipv4Addr::new(10, 0, 0, 1)));
        assert!(is_sweepable_private(Ipv4Addr::new(10, 255, 255, 254)));
        assert!(is_sweepable_private(Ipv4Addr::new(172, 16, 0, 1)));
        assert!(is_sweepable_private(Ipv4Addr::new(172, 31, 255, 254)));
        assert!(is_sweepable_private(Ipv4Addr::new(192, 168, 1, 1)));
        // CGNAT / Tailscale 100.64/10.
        assert!(is_sweepable_private(Ipv4Addr::new(100, 64, 0, 1)));
        assert!(is_sweepable_private(Ipv4Addr::new(100, 127, 255, 254)));
        // Public → NOT swept.
        assert!(!is_sweepable_private(Ipv4Addr::new(8, 8, 8, 8)));
        assert!(!is_sweepable_private(Ipv4Addr::new(1, 1, 1, 1)));
        assert!(!is_sweepable_private(Ipv4Addr::new(203, 0, 113, 5)));
        // Just OUTSIDE the private/CGNAT boundaries — public.
        assert!(!is_sweepable_private(Ipv4Addr::new(172, 15, 255, 254)));
        assert!(!is_sweepable_private(Ipv4Addr::new(172, 32, 0, 1)));
        assert!(!is_sweepable_private(Ipv4Addr::new(100, 63, 255, 255)));
        assert!(!is_sweepable_private(Ipv4Addr::new(100, 128, 0, 1)));
    }

    #[test]
    fn subnets_from_interfaces_drops_public_interface() {
        let ifaces = vec![
            // A public-IP interface (TEST-NET-3, documentation range) — must NOT
            // be swept (recon safety).
            (
                Ipv4Addr::new(203, 0, 113, 5),
                Ipv4Addr::new(255, 255, 255, 0),
            ),
            // A private interface — kept.
            (
                Ipv4Addr::new(192, 168, 9, 4),
                Ipv4Addr::new(255, 255, 255, 0),
            ),
        ];
        assert_eq!(
            subnets_from_interfaces(ifaces),
            vec![(
                Ipv4Addr::new(192, 168, 9, 4),
                Ipv4Addr::new(255, 255, 255, 0)
            )],
            "public-IP interface dropped; the private /24 kept"
        );
    }

    #[test]
    fn subnet_cidr_renders_clamped_network() {
        // /24 verbatim.
        assert_eq!(
            subnet_cidr(
                Ipv4Addr::new(192, 168, 1, 50),
                Ipv4Addr::new(255, 255, 255, 0)
            ),
            "192.168.1.0/24"
        );
        // /8 clamped to the host's own /24.
        assert_eq!(
            subnet_cidr(Ipv4Addr::new(10, 1, 2, 5), Ipv4Addr::new(255, 0, 0, 0)),
            "10.1.2.0/24"
        );
        // /26 respected verbatim (network .64).
        assert_eq!(
            subnet_cidr(
                Ipv4Addr::new(192, 168, 1, 66),
                Ipv4Addr::new(255, 255, 255, 192)
            ),
            "192.168.1.64/26"
        );
    }

    /// The pure survivor→candidate mapping:
    /// robot = the host IP string, locator = EXACTLY `tcp/{ip}:{port}` (the
    /// zenoh locator format the query session's connect list expects), rung =
    /// Scan. Hand oracle; empty in → empty out.
    #[test]
    fn candidates_from_open_maps_ip_and_locator_format() {
        let got = candidates_from_open(vec![sock("10.0.0.42", 7683), sock("192.168.1.9", 7690)]);
        assert_eq!(
            got,
            vec![
                DiscoveredPeer {
                    robot: "10.0.0.42".to_string(),
                    locator: "tcp/10.0.0.42:7683".to_string(),
                    rung: DiscoveryRung::Scan,
                },
                DiscoveredPeer {
                    robot: "192.168.1.9".to_string(),
                    locator: "tcp/192.168.1.9:7690".to_string(),
                    rung: DiscoveryRung::Scan,
                },
            ]
        );
        assert!(candidates_from_open(Vec::new()).is_empty());
    }
}
