// SPDX-License-Identifier: AGPL-3.0-only
//! FIRST-CONTACT AUTOMAGIC: fold the peers netd can find at boot (one
//! `_cerulion._tcp` mDNS browse plus the verified peer cache) into netd's zenoh
//! session, so a desk reaches a robot on its LAN with NO env var and with zenoh
//! multicast scouting off.
//!
//! # Where peers come from
//!
//! netd's session runs with multicast scouting OFF by default
//! ([`crate::net::desk_network_config`]); it is enabled only by
//! [`cerulion_core::transport::network::MULTICAST_SCOUTING_ENV`]`=on`. A session
//! with no scouting reaches exactly its connect endpoints, so this fold is how
//! netd learns them. At daemon start, before `TransportManager::init`,
//! [`fold_discovered_peers`] builds the connect set from, in order:
//!
//! 1. [`crate::net::CONNECT_ENV`], explicit, first and VERBATIM (see below;
//!    the one exception is `CERULION_UNENCRYPTED_QUIC=on`, under which an
//!    explicit `udp/<host>:<port>?rel=1` that does not answer the QUIC probe is
//!    dialled as `tcp/<host>:<port>`, see [`crate::quic_first`]).
//! 2. One bounded `_cerulion._tcp` browse ([`MDNS_FOLD_BUDGET`], one query,
//!    [`cerulion_mdns::browse_rung`]): every gateway that answers, one locator
//!    per IPv4 address in its answer (IPv6 only when it carries no IPv4).
//! 3. TTL-fresh `~/.cerulion/peers.json` entries.
//!
//! Rungs 2 and 3 are deduped by resolved address (mDNS first) and TCP-probed in
//! one parallel round; only survivors are folded. The browse's `ServiceDaemon`
//! is shut down when the budget ends, so after boot netd sends no mDNS queries.
//!
//! # Two policies, and why they differ
//!
//! An OPERATOR-supplied locator is folded WHETHER OR NOT it probes reachable —
//! [`trust_explicit_by_fiat`] marks it reachable before the shared planner runs.
//! This is the opposite of `topic list`'s policy: that is a one-shot gather under
//! a FATAL 1 s `bounded_connect` bound, where an endpoint that cannot answer in
//! time can only sink the whole gather. netd's session is long-lived and NOT
//! `bounded_connect`, so zenoh keeps a configured endpoint on a background retry
//! connector, a robot that is merely booting comes up on its own a few seconds
//! later. Dropping it here would turn "wait for the robot" into "never connect".
//!
//! A DISCOVERED locator (mDNS or cache) gets the probe, at
//! [`cerulion_discovery::ladder::LADDER_PROBE_TIMEOUT`]. A cached address may be
//! stale, and an mDNS answer carries the addresses of the interface it was sent
//! on, which may not be routable from here, so an unprobed row would buy a
//! background connector retrying a dead address for the whole life of the
//! daemon.
//!
//! COST: the browse holds the boot for [`MDNS_FOLD_BUDGET`] (800 ms). The probes
//! then run in ONE parallel round bounded by `LADDER_PROBE_TIMEOUT` (300 ms)
//! plus the collector's 200 ms slack (`probe_reachable_locators`); a reachable
//! LAN peer answers in milliseconds. `cerulion_discovery::peer_cache::MAX_CACHED_PEERS`
//! bounds the cache row count on the read side.
//!
//! # Cadence: ONCE, at daemon start
//!
//! `NetworkConfig` is immutable at `TransportManager::init` and the session is
//! SHARED (Principle #8), so re-folding into a live session would need runtime
//! mutation of an open zenoh session. netd is spawned on the first demand and
//! SELF-EXITS after its idle grace, so a restart re-runs the fold.
//!
//! RESIDUAL: a netd held alive by a standing demand (a running vizd) keeps its
//! boot-time connect set. A robot that boots, or changes address, after that
//! netd started is not dialled until netd restarts, unless it is listed in
//! [`crate::net::CONNECT_ENV`] (an explicit locator is retried in the
//! background) or multicast scouting was turned on. The folded set is reported by `status`
//! ([`crate::protocol::StatusResponse::connect_endpoints`]) so an operator can
//! see what this daemon is CONFIGURED TO DIAL.
//!
//! That is the limit of the observable: the set is fixed BEFORE
//! `TransportManager::init` and netd's zenoh session is LAZY (a netd nobody has
//! demanded from has opened no session at all), so a fiat-trusted DEAD locator
//! reads identically to a live connected one. It answers "why is my desk talking
//! to that address?", never "is that robot up?".

use cerulion_discovery::ladder::{
    dedupe_by_resolved_addr, plan_connect_set, probe_reachable_locators, DiscoveredPeer,
    DiscoveryRung, LADDER_PROBE_TIMEOUT,
};
use cerulion_discovery::peer_cache::{cache_rung, default_cache_path, CACHE_RUNG_BUDGET};
use std::collections::HashSet;
use std::path::Path;
use std::time::Duration;

/// How long the boot-time `_cerulion._tcp` browse listens for answers.
///
/// `mdns-sd` 0.20 sends a browse's first PTR query when the browse starts and
/// repeats it 1 s later (then 2 s, 4 s, ...). 800 ms ends the browse, and shuts
/// its daemon down, before that repeat, so the boot browse sends one PTR query
/// per interface and address family, plus a resolve query for any answer that
/// arrives without its SRV or address records.
pub const MDNS_FOLD_BUDGET: Duration = Duration::from_millis(800);

// The one-query property above holds only below mdns-sd's 1 s repeat.
const _: () = assert!(MDNS_FOLD_BUDGET.as_millis() < 1000);

/// What one fold decided — the observable record behind the `info!` breadcrumbs
/// and `status`'s `connect_endpoints` (Principle #3).
#[derive(Debug, Clone, PartialEq, Eq, Default)]
pub struct PeerFold {
    /// The full connect set for the session: explicit locators first (verbatim),
    /// then the discovered (mDNS, then cache) survivors, deduped.
    pub connect: Vec<String>,
    /// Discovered locators actually folded (excludes any that duplicate an
    /// explicit one). Non-empty ⇒ this daemon reaches a robot it was never told
    /// about.
    pub folded_discovered: Vec<String>,
    /// Discovered locators dropped because they did not answer a TCP connect
    /// inside the probe budget, with the rung that produced them
    /// ([`DiscoveryRung::Mdns`] or [`DiscoveryRung::Cache`]).
    pub dropped_discovered: Vec<(String, DiscoveryRung)>,
}

/// PURE. Mark every EXPLICIT locator reachable, whatever the probe said, and
/// return the reachability set to plan with.
///
/// This is the one policy difference between netd and `topic list`, and it is
/// deliberate — see the module docs. `topic list` runs a one-shot gather under a
/// FATAL 1 s connect bound, so an unreachable explicit locator can only sink it
/// and is dropped. netd's session is long-lived and unbounded, so zenoh retries a
/// configured endpoint on a background connector: dropping one because the robot
/// happened to be down at daemon start would mean never connecting to it at all,
/// and would break the operator's existing `CERULION_NETD_CONNECT` workaround.
///
/// Feeding the result to the SHARED
/// [`cerulion_discovery::ladder::plan_connect_set`] keeps every other rule
/// identical (order, dedupe, per-candidate accounting) — the fiat is expressed as
/// an INPUT to that planner rather than as a second copy of it, and the planner's
/// `explicit_unreachable` is then necessarily empty, which is the accurate report:
/// netd dropped nothing.
pub fn trust_explicit_by_fiat(explicit: &[String], probed: HashSet<String>) -> HashSet<String> {
    let mut trusted = probed;
    for loc in explicit {
        trusted.insert(loc.clone());
    }
    trusted
}

/// PURE. Assemble the fold from the explicit locators, the discovered candidates
/// and the reachability verdict.
///
/// Kept separate from every syscall, the browse, the file read, the TCP probes
/// and the session config all live in [`fold_discovered_peers`], so the
/// DECISION can be oracle-tested (and mutated) without a test ever touching the
/// network or the developer's real `~/.cerulion/peers.json`.
pub fn decide_fold(
    explicit: &[String],
    discovered: &[DiscoveredPeer],
    reachable: &HashSet<String>,
) -> PeerFold {
    let trusted = trust_explicit_by_fiat(explicit, reachable.clone());
    let plan = plan_connect_set(explicit, discovered, &trusted);
    // NOT a `debug_assert!`: this invariant is about the OPERATOR's own locator
    // being silently discarded, and a `debug_assert!` compiles out of the only
    // build that runs on a desk. `PeerFold` carries no field for it either, so a
    // regression here would drop `CERULION_NETD_CONNECT` with no log, no counter
    // and no `status` trace — exactly the outcome `trust_explicit_by_fiat` exists
    // to prevent, unobservable in release. Loud in EVERY build instead.
    if !plan.explicit_unreachable.is_empty() {
        tracing::error!(
            dropped = ?plan.explicit_unreachable,
            "cerulion-netd: BUG — explicit {} locators are trusted by fiat and must never \
             be dropped by the planner, but these were. They will NOT be dialled; set them \
             again or report this.",
            crate::net::CONNECT_ENV
        );
    }
    PeerFold {
        connect: plan.connect,
        folded_discovered: plan.folded_ladder,
        dropped_discovered: plan.dropped_ladder,
    }
}

/// Run the fold against the peer cache at `path` with no mDNS candidates: load
/// the TTL-fresh entries, TCP-probe their locators in ONE bounded parallel round,
/// and assemble the connect set via [`decide_fold`]. Equivalent to
/// [`fold_from_candidates_at`] with an empty `browsed` list.
pub fn fold_from_cache_at(explicit: &[String], path: &Path) -> PeerFold {
    fold_from_candidates_at(explicit, Vec::new(), Some(path))
}

/// Run the fold over `browsed` (mDNS finds) plus the peer cache at `cache`
/// (`None`: no cache rung):
/// the candidates are concatenated mDNS-first, deduped by resolved address,
/// TCP-probed in ONE bounded parallel round, and assembled via [`decide_fold`].
/// Path- and candidate-parameterized so tests drive the real function without
/// multicast I/O or the developer's own `~/.cerulion/peers.json`.
///
/// INFALLIBLE: a missing / corrupt / version-mismatched cache yields no cached
/// candidates (loud inside the reader); with no candidates at all the fold is
/// exactly the explicit locators.
pub fn fold_from_candidates_at(
    explicit: &[String],
    browsed: Vec<DiscoveredPeer>,
    cache: Option<&Path>,
) -> PeerFold {
    let mut candidates = browsed;
    if let Some(path) = cache {
        candidates.extend(cache_rung(CACHE_RUNG_BUDGET, path));
    }
    let candidates = dedupe_by_resolved_addr(candidates);
    // An empty candidate list costs nothing: `probe_reachable_locators` returns
    // immediately without spawning a thread.
    let locators: Vec<String> = candidates.iter().map(|p| p.locator.clone()).collect();
    let reachable = probe_reachable_locators(&locators, LADDER_PROBE_TIMEOUT);
    decide_fold(explicit, &candidates, &reachable)
}

/// PRODUCTION ENTRY: browse `_cerulion._tcp` once for [`MDNS_FOLD_BUDGET`], fold
/// the answers and the verified peer cache into `cfg`'s connect endpoints, and
/// report what happened.
///
/// `cfg.connect_endpoints` on entry are the EXPLICIT
/// [`crate::net::CONNECT_ENV`] locators; on return they are the full fold
/// (explicit verbatim and first, then reachable discovered survivors). Called
/// once from `main` BEFORE `TransportManager::init`, since the config is
/// immutable after it. With no home directory the cache rung is skipped and the
/// mDNS answers are still folded.
pub fn fold_discovered_peers(
    cfg: cerulion_core::transport::network::NetworkConfig,
) -> (cerulion_core::transport::network::NetworkConfig, PeerFold) {
    let report = cerulion_mdns::browse_rung_report(MDNS_FOLD_BUDGET);
    let udp_rel = report.udp_rel;
    let mut browsed = report.peers;
    // This machine's own gateway processes answer under its identity on its own
    // addresses.
    if let Some(own) = cfg.robot_identity.as_deref() {
        browsed = cerulion_discovery::ladder::drop_this_hosts_gateways(browsed, own);
    }
    tracing::debug!(
        answers = browsed.len(),
        budget_ms = MDNS_FOLD_BUDGET.as_millis() as u64,
        "cerulion-netd: boot-time `_cerulion._tcp` browse finished"
    );
    let cache = default_cache_path();
    if cache.is_none() {
        tracing::debug!(
            "cerulion-netd: no home directory; cannot locate the peer cache, folding \
             mDNS answers only"
        );
    }
    let switch = cerulion_discovery::unencrypted_quic::unencrypted_quic_from_env();
    let (cfg, fold, _) = fold_discovered_peers_unencrypted_quic_at(
        cfg,
        browsed,
        &udp_rel,
        cache.as_deref(),
        switch,
        |locators| {
            crate::quic_probe::probe_quic_locators(
                locators,
                crate::quic_probe::quic_probe_timeout_from_env(),
            )
        },
    );
    (cfg, fold)
}

/// [`fold_discovered_peers_at`], then the `CERULION_UNENCRYPTED_QUIC` opt-in
/// ([`crate::quic_first::apply_unencrypted_quic`]) over its result: `udp_rel`
/// lists the browsed peers that advertised an unencrypted QUIC port, `switch` is
/// the opt-in, and `probe` checks a set of `udp/` locators. With
/// [`UnencryptedQuic::Off`] the config and fold are exactly
/// [`fold_discovered_peers_at`]'s and `probe` is never called.
///
/// [`UnencryptedQuic::Off`]: cerulion_discovery::unencrypted_quic::UnencryptedQuic::Off
pub fn fold_discovered_peers_unencrypted_quic_at(
    cfg: cerulion_core::transport::network::NetworkConfig,
    browsed: Vec<DiscoveredPeer>,
    udp_rel: &[cerulion_discovery::unencrypted_quic::UdpRelTwin],
    cache: Option<&Path>,
    switch: cerulion_discovery::unencrypted_quic::UnencryptedQuic,
    probe: impl FnOnce(
        &[String],
    ) -> std::collections::HashMap<String, crate::quic_probe::QuicProbeOutcome>,
) -> (
    cerulion_core::transport::network::NetworkConfig,
    PeerFold,
    crate::quic_first::QuicFirstOutcome,
) {
    let explicit = cfg.connect_endpoints.clone();
    let (mut cfg, mut fold) = fold_discovered_peers_at(cfg, browsed, cache);
    let outcome = crate::quic_first::apply_unencrypted_quic(
        &mut cfg, &mut fold, &explicit, udp_rel, switch, probe,
    );
    (cfg, fold, outcome)
}

/// The cache-only fold: [`fold_discovered_peers_at`] with no mDNS candidates.
pub fn fold_cached_peers_at(
    cfg: cerulion_core::transport::network::NetworkConfig,
    path: &Path,
) -> (cerulion_core::transport::network::NetworkConfig, PeerFold) {
    fold_discovered_peers_at(cfg, Vec::new(), Some(path))
}

/// The REAL fold body, parameterized on the mDNS candidates and the cache path
/// (`None`: no cache rung) so tests drive the production function without
/// multicast I/O and against a temp cache. The only thing [`fold_discovered_peers`] adds is the browse and
/// resolving [`default_cache_path`].
pub fn fold_discovered_peers_at(
    mut cfg: cerulion_core::transport::network::NetworkConfig,
    browsed: Vec<DiscoveredPeer>,
    cache: Option<&Path>,
) -> (cerulion_core::transport::network::NetworkConfig, PeerFold) {
    let explicit = cfg.connect_endpoints.clone();
    let fold = fold_from_candidates_at(&explicit, browsed, cache);
    let path = cache.map_or_else(
        || "<no home directory>".to_string(),
        |p| p.display().to_string(),
    );

    for (locator, rung) in &fold.dropped_discovered {
        tracing::debug!(
            locator = %locator,
            rung = %rung,
            timeout_ms = LADDER_PROBE_TIMEOUT.as_millis() as u64,
            "cerulion-netd: a discovered peer did not answer a TCP connect within the probe \
             budget — not dialled this run (it will be retried when netd next starts)"
        );
    }
    // The two non-empty arms fire ONCE PER BOOT at `info`, the shipped release
    // ceiling, so an operator can tell "nothing was found" from "something was
    // found and none of it answered".
    match (
        fold.folded_discovered.is_empty(),
        fold.dropped_discovered.is_empty(),
    ) {
        // Nothing found at all, the ordinary cold-desk case, quiet.
        (true, true) => tracing::debug!(
            path = %path,
            "cerulion-netd: no peer answered the `_cerulion._tcp` browse and none is cached, \
             the session dials only {} locators",
            crate::net::CONNECT_ENV
        ),
        (true, false) => tracing::info!(
            dropped = fold.dropped_discovered.len(),
            path = %path,
            "every discovered peer failed the reachability probe, none dialled (was the \
             robot asleep at daemon start?). netd folds peers ONCE at boot, so restart \
             netd once the robot is up, or set {}",
            crate::net::CONNECT_ENV
        ),
        (false, _) => tracing::info!(
            folded = ?fold.folded_discovered,
            dropped = fold.dropped_discovered.len(),
            path = %path,
            "folded verified discovered peers (mDNS + cache) into the netd session, first contact \
             works with no {} set",
            crate::net::CONNECT_ENV
        ),
    }

    cfg.connect_endpoints = fold.connect.clone();
    (cfg, fold)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn peer(robot: &str, locator: &str) -> DiscoveredPeer {
        DiscoveredPeer {
            robot: robot.to_string(),
            locator: locator.to_string(),
            rung: DiscoveryRung::Cache,
        }
    }

    fn set(locators: &[&str]) -> HashSet<String> {
        locators.iter().map(|s| s.to_string()).collect()
    }

    fn strs(v: &[&str]) -> Vec<String> {
        v.iter().map(|s| s.to_string()).collect()
    }

    /// THE HEADLINE (the acceptance criterion, at the decision layer): a desk
    /// with NO explicit locators and a robot in its cache dials that robot.
    /// Hand oracle.
    #[test]
    fn a_cached_peer_is_dialled_with_no_env_set() {
        let cached = vec![peer("go2", "tcp/10.0.0.5:7683")];
        let reachable = set(&["tcp/10.0.0.5:7683"]);
        assert_eq!(
            decide_fold(&[], &cached, &reachable),
            PeerFold {
                connect: strs(&["tcp/10.0.0.5:7683"]),
                folded_discovered: strs(&["tcp/10.0.0.5:7683"]),
                dropped_discovered: vec![],
            }
        );
    }

    /// The explicit env locators come FIRST and VERBATIM, with the cached
    /// survivors appended — a UNION, not a replacement, so an operator who set
    /// `CERULION_NETD_CONNECT` for one robot still reaches the others. Hand
    /// oracle.
    #[test]
    fn explicit_locators_come_first_and_cached_survivors_are_appended() {
        let explicit = strs(&["tcp/1.1.1.1:7683"]);
        let cached = vec![
            peer("go2", "tcp/10.0.0.5:7683"),
            peer("rover", "tcp/10.0.0.6:7683"),
        ];
        let reachable = set(&["tcp/10.0.0.5:7683", "tcp/10.0.0.6:7683"]);
        assert_eq!(
            decide_fold(&explicit, &cached, &reachable),
            PeerFold {
                connect: strs(&["tcp/1.1.1.1:7683", "tcp/10.0.0.5:7683", "tcp/10.0.0.6:7683"]),
                folded_discovered: strs(&["tcp/10.0.0.5:7683", "tcp/10.0.0.6:7683"]),
                dropped_discovered: vec![],
            }
        );
    }

    /// THE POLICY PIN (netd's deliberate divergence from `topic list`): an
    /// UNREACHABLE explicit locator is still folded — netd's session is
    /// long-lived and unbounded, so zenoh retries it on a background connector
    /// and a robot that is merely booting comes up on its own. Dropping it would
    /// mean never connecting, and would break the operator's existing
    /// `CERULION_NETD_CONNECT` workaround.
    ///
    /// The same probe verdict is applied STRICTLY to the cached candidate in the
    /// same body — the two policies are asserted against each other, so a
    /// mutation that unifies them (either way) fails here.
    #[test]
    fn an_unreachable_explicit_locator_is_still_dialled_while_a_cached_one_is_not() {
        let explicit = strs(&["tcp/1.2.3.4:7683"]);
        let cached = vec![peer("gone", "tcp/10.0.0.9:7683")];
        // NOTHING probed reachable.
        let reachable = HashSet::new();
        assert_eq!(
            decide_fold(&explicit, &cached, &reachable),
            PeerFold {
                // the operator's word survives; the stale cache row does not.
                connect: strs(&["tcp/1.2.3.4:7683"]),
                folded_discovered: vec![],
                dropped_discovered: vec![("tcp/10.0.0.9:7683".to_string(), DiscoveryRung::Cache)],
            }
        );
    }

    /// A cached row that names the SAME locator the operator gave is folded
    /// ONCE and is not double-counted as a cached find (the operator already
    /// told us about that robot).
    #[test]
    fn a_cached_locator_duplicating_an_explicit_one_is_folded_once() {
        let explicit = strs(&["tcp/10.0.0.5:7683"]);
        let cached = vec![
            peer("go2", "tcp/10.0.0.5:7683"),
            peer("rover", "tcp/10.0.0.6:7683"),
        ];
        let reachable = set(&["tcp/10.0.0.5:7683", "tcp/10.0.0.6:7683"]);
        assert_eq!(
            decide_fold(&explicit, &cached, &reachable),
            PeerFold {
                connect: strs(&["tcp/10.0.0.5:7683", "tcp/10.0.0.6:7683"]),
                folded_discovered: strs(&["tcp/10.0.0.6:7683"]),
                dropped_discovered: vec![],
            }
        );
    }

    /// The DEGRADE floor: no cache rows at all ⇒ exactly the explicit set,
    /// nothing added, nothing dropped — byte-identical to earlier behaviour
    /// on a desk that has never seen a robot.
    #[test]
    fn an_empty_cache_degrades_to_exactly_the_explicit_locators() {
        let explicit = strs(&["tcp/1.1.1.1:7683"]);
        assert_eq!(
            decide_fold(&explicit, &[], &HashSet::new()),
            PeerFold {
                connect: strs(&["tcp/1.1.1.1:7683"]),
                folded_discovered: vec![],
                dropped_discovered: vec![],
            }
        );
        // And with no env either, the whole fold is empty.
        assert_eq!(decide_fold(&[], &[], &HashSet::new()), PeerFold::default());
    }

    /// `trust_explicit_by_fiat` PRESERVES the probe verdict it is handed and only
    /// ADDS the explicit locators — it must not, for instance, replace the set.
    /// Hand oracle on the set itself (the arm above pins the consequence; this
    /// pins the mechanism).
    #[test]
    fn the_fiat_adds_explicit_locators_without_discarding_probe_results() {
        let trusted = trust_explicit_by_fiat(
            &strs(&["tcp/1.2.3.4:7683", "tcp/5.6.7.8:7683"]),
            set(&["tcp/10.0.0.5:7683"]),
        );
        assert_eq!(
            trusted,
            set(&["tcp/10.0.0.5:7683", "tcp/1.2.3.4:7683", "tcp/5.6.7.8:7683"])
        );
        // No explicit locators ⇒ the probe verdict is returned unchanged.
        assert_eq!(
            trust_explicit_by_fiat(&[], set(&["tcp/10.0.0.5:7683"])),
            set(&["tcp/10.0.0.5:7683"])
        );
    }
}
