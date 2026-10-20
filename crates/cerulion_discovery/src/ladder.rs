// SPDX-License-Identifier: AGPL-3.0-only
//! The SHARED half of the robot-gateway discovery ladder —
//! the candidate types, the locator parser + dedupe key, the bounded TCP
//! reachability pre-filter, and the pure connect-set planner.
//!
//! # Model
//!
//! mDNS (and the other rungs) find robots; zenoh connects and does everything
//! after. A rung's only job is to surface a `(robot, locator)` so the caller can
//! hand the locator to ONE session — liveliness/announce discovery, tapping, and
//! every richer interaction ride the connected session, not the ladder. A rung
//! NEVER opens its own zenoh session; PRESENCE in the caller's gather is what
//! confirms a robot live (never a per-candidate probe).
//!
//! # Split
//!
//! The RUNGS themselves (mDNS browse, hostname convention, subnet sweep), the
//! parallel `gather_rungs` engine, its budgets, and the `discover_peers` wiring
//! stay in `cerulion_cli_engine::discovery_ladder`, which re-exports everything
//! here so its call sites are unchanged. What lives in THIS crate is only what
//! `cerulion-netd` also needs to fold cached peers into its own session —
//! `cerulion_cli_engine` depends on `cerulion_netd`, so netd cannot import from
//! the CLI engine (that edge is cyclic).

use std::collections::HashSet;
use std::net::{IpAddr, SocketAddr, TcpStream, ToSocketAddrs};
use std::str::FromStr;
use std::sync::mpsc;
use std::thread;
use std::time::{Duration, Instant};

/// A robot gateway found by one of the ladder's discovery rungs.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct DiscoveredPeer {
    /// Robot identity — the resolved hostname or `CERULION_ROBOT_IDENTITY`
    /// override, NOT the graph prefix.
    pub robot: String,
    /// zenoh locator, e.g. `"tcp/192.168.1.7:7683"`.
    pub locator: String,
    /// Which rung found it.
    pub rung: DiscoveryRung,
}

/// The discovery rung that surfaced a [`DiscoveredPeer`]. Ordered by the
/// ladder's preference (see `cerulion_cli_engine::discovery_ladder::discover_peers`):
/// mDNS is the primary rung, so an mDNS-found peer wins a dedupe tie over a
/// cache- or hostname-found twin.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum DiscoveryRung {
    /// The cached-peer file (`~/.cerulion/peers.json`).
    Cache,
    /// The hostname-convention rung (well-known robot names).
    Hostname,
    /// The mDNS browse rung (pure-Rust `mdns-sd`) — the PRIMARY rung.
    Mdns,
    /// The OPT-IN unicast subnet-sweep rung (`--scan`; rung 4). It never
    /// appears on a default run — the sweep is structurally gated behind the
    /// flag.
    Scan,
}

impl std::fmt::Display for DiscoveryRung {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.write_str(match self {
            DiscoveryRung::Cache => "cache",
            DiscoveryRung::Hostname => "hostname",
            DiscoveryRung::Mdns => "mdns",
            DiscoveryRung::Scan => "scan",
        })
    }
}

/// Deduplicate peers by their locator's normalized address key, keeping the
/// FIRST occurrence (so an earlier rung in the merge order wins the tie).
///
/// The key (built by the private `resolved_addr_key`) is PURE (no DNS) — an
/// IP-literal host keys on the parse-normalized `"{ip}|{port}"`, a non-literal
/// host on the case-folded `"{host}|{port}"`, and an unparseable shape on the
/// literal locator string (so distinct unresolvable locators stay distinct, but
/// exact duplicates still collapse).
pub fn dedupe_by_resolved_addr(peers: Vec<DiscoveredPeer>) -> Vec<DiscoveredPeer> {
    let mut seen: HashSet<String> = HashSet::new();
    let mut out = Vec::with_capacity(peers.len());
    for peer in peers {
        let key = resolved_addr_key(&peer.locator);
        if seen.insert(key) {
            out.push(peer);
        }
    }
    out
}

/// The dedupe key for one locator — PURE, never blocks, never resolves DNS.
///
/// An IP-literal host (v4, or bracketed v6) keys on the parse-normalized
/// `"{ip}|{port}"` so equivalent spellings of one address collapse
/// (`[::1]` == `[0:0:0:0:0:0:0:1]`). A non-literal host keys on the case-folded
/// `"{host}|{port}"` with NO resolution — distinct hostnames stay distinct,
/// same-host-different-case collapses. A shape the parser cannot split keys on
/// the literal locator string.
///
/// Every PRODUCTION rung emits IP-literal locators (mDNS maps resolved
/// addresses; the hostname rung resolves BEFORE minting the locator; the cache
/// stores what the rungs minted), so literal-keying IS resolved-keying in
/// practice — DNS in the dedupe would add nothing but a blocking, timeout-less
/// getaddrinfo call on the calling thread, run AFTER the ladder's join deadline.
fn resolved_addr_key(locator: &str) -> String {
    let (host, port) = match parse_locator_host_port(locator) {
        Some(hp) => hp,
        // Shape the parser can't split ⇒ key on the literal.
        None => return locator.to_string(),
    };
    // Numeric host (v4 or bracketed v6): normalize via `IpAddr` so equivalent
    // spellings of one address share a key. No resolver involvement.
    if let Ok(ip) = IpAddr::from_str(host) {
        return format!("{ip}|{port}");
    }
    // Non-literal host: key on the case-folded host, purely. mDNS/DNS names are
    // case-insensitive, so `ROBOT.local` and `robot.local` are one peer.
    format!("{}|{port}", host.to_ascii_lowercase())
}

/// Parse a zenoh locator (`<proto>/<host>:<port>`) into `(host, port)`. The host
/// may be a bracketed IPv6 literal (`[::1]`). Returns `None` if the shape does
/// not match — the caller then keys on the literal locator string.
fn parse_locator_host_port(locator: &str) -> Option<(&str, u16)> {
    // Strip the protocol chunk: "tcp/127.0.0.1:7683" -> "127.0.0.1:7683".
    let after_proto = locator.split_once('/')?.1;
    // Bracketed IPv6: "[::1]:7683" -> ("::1", 7683).
    if let Some(rest) = after_proto.strip_prefix('[') {
        let (host, port_part) = rest.split_once(']')?;
        let port = port_part.strip_prefix(':')?.parse().ok()?;
        return Some((host, port));
    }
    // IPv4 / hostname: split on the LAST ':' (neither has an interior colon;
    // zenoh emits IPv6 bracketed, handled above).
    let (host, port_part) = after_proto.rsplit_once(':')?;
    let port = port_part.parse().ok()?;
    Some((host, port))
}

/// Is `locator` SYNTACTICALLY a zenoh locator (`<proto>/<host>:<port>`)?
/// True iff `parse_locator_host_port` can split it. The caller uses this to
/// distinguish a user's TYPO in an EXPLICIT `--connect` locator (a hard,
/// actionable error before any probing) from a syntactically-VALID-but-
/// unreachable locator (a warn-and-skip — a valid host can legitimately be down
/// or unresolvable on this network). Ladder candidates are NOT gated by this —
/// they are discovery output, not user input, so an unparseable one is just
/// dropped. Pure.
pub fn locator_is_syntactically_valid(locator: &str) -> bool {
    parse_locator_host_port(locator).is_some()
}

// ─── The discovery-connect TCP PRE-FILTER ─────────────────────────────────────
//
// The `topic list` discovery query hard-bounds its zenoh connect at 1 s
// (`NetworkConfig.bounded_connect`), but that bound is FATAL: zenoh 1.8's
// `start_peer` propagates the global-connect-timeout Elapsed error UNCONDITIONALLY
// (`connect_peers(..).await?` — `exit_on_failure` is NOT consulted on that arm),
// AND the endpoints are tried SEQUENTIALLY, so ONE endpoint that hangs the
// connect (a SYN-drop, or a tarpit that accepts TCP but never speaks zenoh —
// exactly a `--scan` open-port false positive) can consume the whole 1 s budget
// and make `zenoh::open` FAIL, losing every OTHER reachable robot in the gather.
//
// The principle: anything that cannot TCP-connect within the session's own 1 s
// connect bound can NEVER contribute to the gather — so dropping it loses
// nothing. So we PRE-FILTER every connect endpoint by a bounded parallel TCP
// probe and fold only the survivors: LADDER candidates at
// [`LADDER_PROBE_TIMEOUT`], and EXPLICIT `--connect` locators at
// [`EXPLICIT_PROBE_TIMEOUT`] (= the connect bound, so we drop exactly what the
// session could not have used — with a LOUD warn, since it was the user's word).
// A candidate that PASSES the probe can still poison the open (a tarpit), so the
// caller retries once with the reachable-explicit set.
//
// A note for the netd consumer: netd's session is NOT `bounded_connect`,
// so the FATAL-open hazard above does not apply to it. The pre-filter still
// earns its place there for a different reason — a stale cache entry would
// otherwise hand zenoh a dead address to retry in the background for the whole
// life of a long-running daemon. See `cerulion_netd::discovery_fold`.

/// The per-probe timeout for a LADDER candidate (mDNS / cache / hostname / scan
/// — a discovered address, treated as a hint). Short (interactive) — a same-LAN
/// gateway accepts a TCP connection well within it.
pub const LADDER_PROBE_TIMEOUT: Duration = Duration::from_millis(300);

/// The per-probe timeout for an EXPLICIT `--connect` locator — deliberately
/// EQUAL to the discovery session's bounded-connect global timeout (1 s). An
/// explicit locator that does not answer a TCP connect within this window could
/// not have contributed to the gather within the connect bound anyway, so the
/// pre-filter drops exactly what the session could not use (with a loud warn) —
/// rather than folding it in and letting one hanging locator sink the whole
/// gather (the SYN-drop failure mode).
pub const EXPLICIT_PROBE_TIMEOUT: Duration = Duration::from_millis(1000);

/// Raw primitive: does `addr` accept a TCP connection within `timeout`?
/// `connect_timeout` panics on a zero duration, so floor at 1 ms. Shared by the
/// subnet sweep's port scan (`cerulion_cli_engine::subnet_sweep`) and the
/// ladder-candidate pre-filter, so the "is a port open?" probe lives in exactly
/// one place.
pub fn tcp_port_open(addr: SocketAddr, timeout: Duration) -> bool {
    TcpStream::connect_timeout(&addr, timeout.max(Duration::from_millis(1))).is_ok()
}

/// Parse + resolve a zenoh locator (`tcp/host:port`) and probe its FIRST
/// resolved address. Unparseable or unresolvable ⇒ `false` (unreachable — it
/// could never be connected anyway). The blocking DNS resolve is bounded only
/// by this running inside a per-candidate thread under the batch deadline.
fn probe_locator(locator: &str, timeout: Duration) -> bool {
    let Some((host, port)) = parse_locator_host_port(locator) else {
        return false;
    };
    match (host, port).to_socket_addrs() {
        Ok(mut addrs) => addrs.next().is_some_and(|a| tcp_port_open(a, timeout)),
        Err(_) => false,
    }
}

/// Probe every locator in `locators` CONCURRENTLY (one detached thread each,
/// per-probe `timeout`) and return the set that accepted a TCP connection.
/// Detached (not joined) so one hung DNS resolve can't exceed the bound; the
/// collector stops at `timeout + a small slack`. An empty input is an empty set
/// (no probing, and — load-bearing for the netd fold — no wall cost at all).
/// INFALLIBLE — a thread-spawn failure just leaves that locator out of the
/// reachable set (treated as unreachable, the safe default).
pub fn probe_reachable_locators(locators: &[String], timeout: Duration) -> HashSet<String> {
    if locators.is_empty() {
        return HashSet::new();
    }
    let (tx, rx) = mpsc::channel::<String>();
    for locator in locators {
        let locator = locator.clone();
        let tx = tx.clone();
        let _ = thread::Builder::new()
            .name("discovery-probe".to_string())
            .spawn(move || {
                if probe_locator(&locator, timeout) {
                    // A send failure only means the collector already hung up.
                    let _ = tx.send(locator);
                }
            });
    }
    drop(tx);
    // All probes run in parallel bounded by `timeout`; give the collector a
    // little slack past it so a probe finishing right at the edge still counts.
    let deadline = Instant::now() + timeout + Duration::from_millis(200);
    let mut reachable = HashSet::new();
    loop {
        let remaining = deadline.saturating_duration_since(Instant::now());
        if remaining.is_zero() {
            break;
        }
        match rx.recv_timeout(remaining) {
            Ok(locator) => {
                reachable.insert(locator);
            }
            // Every sender dropped (all probes done) or the window elapsed.
            Err(_) => break,
        }
    }
    reachable
}

/// The discovery-session connect set derived by [`plan_connect_set`] — the pure
/// mapping from `(explicit --connect locators, ladder candidates, reachability
/// probe result)` to what to fold and what to log.
#[derive(Debug, Clone, PartialEq, Eq, Default)]
pub struct ConnectPlan {
    /// The full connect set to fold into the session (reachable explicit
    /// locators first, then reachable ladder survivors — deduped).
    pub connect: Vec<String>,
    /// The ladder-candidate locators actually folded (a survivor that duplicates
    /// an explicit locator is NOT counted here). Drives the open-failure
    /// fallback — non-empty ⇒ a retry with the reachable-explicit set only is
    /// worthwhile (a folded ladder candidate might be the tarpit that poisoned
    /// the open).
    pub folded_ladder: Vec<String>,
    /// Explicit `--connect` locators that did NOT probe reachable within the
    /// connect bound — EXCLUDED from `connect` (they could not have contributed
    /// to the gather within the session's own connect bound, so folding them in
    /// would only risk one hanging locator sinking the whole gather) and each
    /// earns a LOUD WARN.
    ///
    /// A caller for whom an operator-supplied locator must survive a failed
    /// probe (`cerulion-netd`, whose session is long-lived and unbounded, so
    /// zenoh keeps retrying a currently-down robot in the background) passes a
    /// `reachable` set that already contains its explicit locators — see
    /// `cerulion_netd::discovery_fold::trust_explicit_by_fiat`. This field is
    /// then necessarily empty, which is the correct report: nothing was dropped.
    pub explicit_unreachable: Vec<String>,
    /// Ladder candidates dropped as unreachable — each earns a DEBUG log
    /// (locator + rung).
    pub dropped_ladder: Vec<(String, DiscoveryRung)>,
}

/// Pure: plan the discovery connect set.
///
/// EVERYTHING is folded ONLY when reachable — a locator that could not
/// TCP-connect within the session's connect bound could never have contributed
/// to the gather, so dropping it loses nothing:
///
/// - Explicit `--connect` `locators` that probed reachable are folded; an
///   unreachable one is EXCLUDED from `connect` and recorded in
///   `explicit_unreachable` (the LOUD-warn source) — the policy change from the
///   old "never dropped" (one hanging explicit locator must not sink the gather).
/// - Ladder `candidates` are folded ONLY when their locator probed reachable
///   (`reachable`); the rest land in `dropped_ladder` for a DEBUG log. A survivor
///   whose locator already came in as an explicit locator is not folded twice
///   (and does not count toward `folded_ladder`).
///
/// `reachable` is the UNION of the two probe batches (explicit @1 s, ladder
/// @300 ms), so an explicit locator's reachability reflects the more generous
/// budget. Input order is preserved (explicit first, then ladder). Pure —
/// oracle-tested.
pub fn plan_connect_set(
    explicit: &[String],
    candidates: &[DiscoveredPeer],
    reachable: &HashSet<String>,
) -> ConnectPlan {
    let mut plan = ConnectPlan::default();
    for loc in explicit {
        if reachable.contains(loc) {
            if !plan.connect.contains(loc) {
                plan.connect.push(loc.clone());
            }
        } else {
            plan.explicit_unreachable.push(loc.clone());
        }
    }
    for peer in candidates {
        if reachable.contains(&peer.locator) {
            if !plan.connect.contains(&peer.locator) {
                plan.connect.push(peer.locator.clone());
                plan.folded_ladder.push(peer.locator.clone());
            }
        } else {
            plan.dropped_ladder.push((peer.locator.clone(), peer.rung));
        }
    }
    plan
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Build a `DiscoveredPeer` tersely for the plan oracles.
    fn peer(robot: &str, locator: &str, rung: DiscoveryRung) -> DiscoveredPeer {
        DiscoveredPeer {
            robot: robot.to_string(),
            locator: locator.to_string(),
            rung,
        }
    }

    fn reachable_set(locators: &[&str]) -> HashSet<String> {
        locators.iter().map(|s| s.to_string()).collect()
    }

    /// The `plan_connect_set` survivor logic:
    /// a REACHABLE ladder candidate folds (into `connect` + `folded_ladder`); an
    /// UNREACHABLE one is dropped (into `dropped_ladder` with its rung, NOT
    /// folded); order is explicit-then-ladder; a survivor duplicating an explicit
    /// locator is not folded twice. Hand oracle, never a self-compare.
    #[test]
    fn plan_connect_set_folds_reachable_ladder_and_drops_the_rest() {
        let candidates = vec![
            peer("go2", "tcp/10.0.0.5:7683", DiscoveryRung::Mdns), // reachable → fold
            peer("dead", "tcp/10.0.0.9:7683", DiscoveryRung::Cache), // unreachable → drop
            peer("scan-hit", "tcp/10.0.0.7:7683", DiscoveryRung::Scan), // reachable → fold
        ];
        let reachable = reachable_set(&["tcp/10.0.0.5:7683", "tcp/10.0.0.7:7683"]);
        let plan = plan_connect_set(&[], &candidates, &reachable);
        assert_eq!(
            plan,
            ConnectPlan {
                connect: vec![
                    "tcp/10.0.0.5:7683".to_string(),
                    "tcp/10.0.0.7:7683".to_string()
                ],
                folded_ladder: vec![
                    "tcp/10.0.0.5:7683".to_string(),
                    "tcp/10.0.0.7:7683".to_string()
                ],
                explicit_unreachable: vec![],
                dropped_ladder: vec![("tcp/10.0.0.9:7683".to_string(), DiscoveryRung::Cache)],
            }
        );
    }

    /// A policy-change pin: an UNREACHABLE
    /// explicit `--connect` locator is now DROPPED (EXCLUDED from `connect`) and
    /// recorded in `explicit_unreachable` (the loud-warn source) — one hanging
    /// explicit locator must not sink the whole gather (the SYN-drop
    /// failure mode). A REACHABLE explicit locator still folds. A ladder survivor
    /// duplicating a reachable explicit locator is folded once (not counted as
    /// ladder). Hand oracle, never a self-compare.
    #[test]
    fn plan_connect_set_drops_unreachable_explicit_with_warn_record() {
        let explicit = vec![
            "tcp/1.2.3.4:7683".to_string(), // unreachable explicit → DROP + record
            "tcp/10.0.0.5:7683".to_string(), // reachable explicit → fold, no record
        ];
        let candidates = vec![
            // A ladder candidate for a locator ALSO given explicitly (reachable)
            // → not folded twice, not counted as ladder.
            peer("dup", "tcp/10.0.0.5:7683", DiscoveryRung::Mdns),
            // A distinct reachable ladder candidate → folded as ladder.
            peer("extra", "tcp/10.0.0.6:7683", DiscoveryRung::Mdns),
        ];
        // Note: 1.2.3.4 is NOT reachable → dropped.
        let reachable = reachable_set(&["tcp/10.0.0.5:7683", "tcp/10.0.0.6:7683"]);
        let plan = plan_connect_set(&explicit, &candidates, &reachable);
        assert_eq!(
            plan,
            ConnectPlan {
                // ONLY the reachable explicit, then the distinct ladder survivor —
                // the unreachable explicit locator is EXCLUDED from connect.
                connect: vec![
                    "tcp/10.0.0.5:7683".to_string(),
                    "tcp/10.0.0.6:7683".to_string(),
                ],
                // only the distinct ladder survivor counts as folded-ladder.
                folded_ladder: vec!["tcp/10.0.0.6:7683".to_string()],
                // the unreachable explicit locator is recorded (the warn source).
                explicit_unreachable: vec!["tcp/1.2.3.4:7683".to_string()],
                dropped_ladder: vec![],
            }
        );
    }

    /// Empty inputs are an empty plan (no probing, no fold).
    #[test]
    fn plan_connect_set_empty_is_empty() {
        assert_eq!(
            plan_connect_set(&[], &[], &HashSet::new()),
            ConnectPlan::default()
        );
    }

    /// An EMPTY locator list costs no wall time and no threads — the
    /// netd fold runs this on every daemon boot, and a desk that has never seen
    /// a robot must pay nothing at all for the feature.
    #[test]
    fn probing_an_empty_set_is_free() {
        let start = Instant::now();
        assert!(probe_reachable_locators(&[], LADDER_PROBE_TIMEOUT).is_empty());
        assert!(
            start.elapsed() < Duration::from_millis(50),
            "an empty probe set must return immediately, not sleep out the \
             collector deadline (took {:?})",
            start.elapsed()
        );
    }

    #[test]
    fn rung_display_strings() {
        assert_eq!(DiscoveryRung::Cache.to_string(), "cache");
        assert_eq!(DiscoveryRung::Hostname.to_string(), "hostname");
        assert_eq!(DiscoveryRung::Mdns.to_string(), "mdns");
        assert_eq!(DiscoveryRung::Scan.to_string(), "scan");
    }

    #[test]
    fn probe_timeout_consts_are_sane() {
        assert_eq!(LADDER_PROBE_TIMEOUT, Duration::from_millis(300));
        assert_eq!(EXPLICIT_PROBE_TIMEOUT, Duration::from_millis(1000));
        assert!(
            LADDER_PROBE_TIMEOUT <= EXPLICIT_PROBE_TIMEOUT,
            "a discovered address is a HINT and gets the shorter budget; an \
             operator-supplied locator gets the full connect bound"
        );
    }

    #[test]
    fn parse_locator_ipv4_hostname_and_bracketed_ipv6() {
        assert_eq!(
            parse_locator_host_port("tcp/127.0.0.1:7683"),
            Some(("127.0.0.1", 7683))
        );
        assert_eq!(
            parse_locator_host_port("tcp/robot.local:7683"),
            Some(("robot.local", 7683))
        );
        assert_eq!(
            parse_locator_host_port("tcp/[::1]:7683"),
            Some(("::1", 7683))
        );
        assert_eq!(
            parse_locator_host_port("tcp/[fe80::1]:9000"),
            Some(("fe80::1", 9000))
        );
        // Malformed shapes → None (the caller keys on the literal).
        assert_eq!(parse_locator_host_port("garbage"), None);
        assert_eq!(parse_locator_host_port("tcp/no-port"), None);
        assert_eq!(parse_locator_host_port("tcp/host:not-a-port"), None);
    }

    #[test]
    fn resolved_key_numeric_host_never_does_dns() {
        // Numeric hosts (v4 + bracketed v6) key on ip|port with no resolver.
        assert_eq!(resolved_addr_key("tcp/127.0.0.1:7683"), "127.0.0.1|7683");
        assert_eq!(resolved_addr_key("tcp/[::1]:7683"), "::1|7683");
        // Equivalent v6 spellings normalize to ONE key (no resolver).
        assert_eq!(
            resolved_addr_key("tcp/[0:0:0:0:0:0:0:1]:7683"),
            resolved_addr_key("tcp/[::1]:7683")
        );
        // A shape the parser rejects keys on the literal.
        assert_eq!(resolved_addr_key("garbage"), "garbage");
    }

    #[test]
    fn resolved_key_hostname_is_case_folded_and_dns_free() {
        // A non-literal host keys on the case-folded (host, port) — no resolver.
        assert_eq!(
            resolved_addr_key("tcp/Robot.Local:7683"),
            "robot.local|7683"
        );
        // Different case → SAME key; a genuinely different host → different key.
        assert_eq!(
            resolved_addr_key("tcp/ROBOT.LOCAL:7683"),
            resolved_addr_key("tcp/robot.local:7683")
        );
        assert_ne!(
            resolved_addr_key("tcp/robot-a.local:7683"),
            resolved_addr_key("tcp/robot-b.local:7683")
        );
    }
}
