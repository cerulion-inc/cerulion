// SPDX-License-Identifier: AGPL-3.0-only
//! FIRST-CONTACT AUTOMAGIC — fold the verified peer cache into netd's
//! zenoh session so a desk reaches a robot it has seen before with NO env var.
//!
//! # The gap this closes
//!
//! `topic list` is automagic: it runs a discovery LADDER (mDNS + the
//! `~/.cerulion/peers.json` cache + a hostname convention) and dials what it
//! finds. But `cerulion-netd` — the daemon EVERY other surface rides (`topic
//! hz`/`echo`/`info`, `schema info`, vizd/Studio attaches) — opened its one
//! session with SCOUTING ONLY, plus the [`crate::net::CONNECT_ENV`] escape
//! hatch. So on any network where scouting is degraded (Tailscale,
//! multicast-filtered wifi — the common case) the desk's own cache KNEW the
//! robot's verified locator while the daemon serving Studio never dialled it,
//! and first contact worked only after an operator hand-exported
//! `CERULION_NETD_CONNECT` — which a netd respawned by another consumer then
//! silently lost.
//!
//! It is worse than a discovery-latency gap. Per the data-plane A/B on a live
//! Go2, a scouting-DISCOVERED session's query plane converges but its DATA plane
//! is dead (mirror registers, zero frames for 14+ minutes), while the
//! bounded-connect session flows at 500 Hz. So dialling the cached locator is
//! not merely faster — it is currently the only mode that provably carries data.
//!
//! # What the fold does
//!
//! At daemon start, before `TransportManager::init`, [`fold_cached_peers`] adds
//! connect endpoints from the SAME sources the ladder already trusts, in the
//! same order:
//!
//! 1. [`crate::net::CONNECT_ENV`] — explicit, first, and VERBATIM (see below).
//! 2. TTL-fresh `~/.cerulion/peers.json` entries, TCP-probed, survivors only.
//!
//! No new user surface, no new env var: a desk with an empty cache folds nothing
//! and is byte-identical to earlier behaviour.
//!
//! # Two policies, and why they differ
//!
//! An OPERATOR-supplied locator is folded WHETHER OR NOT it probes reachable —
//! [`trust_explicit_by_fiat`] marks it reachable before the shared planner runs.
//! This is the opposite of `topic list`'s policy, deliberately: that is a
//! one-shot gather under a FATAL 1 s `bounded_connect` bound, where an endpoint
//! that cannot answer in time can only sink the whole gather. netd's session is
//! long-lived and NOT `bounded_connect`, so zenoh keeps a configured endpoint on
//! a background retry connector — a robot that is merely booting comes up on its
//! own a few seconds later. Dropping it here would turn "wait for the robot" into
//! "never connect", and would silently break the operator's existing
//! `CERULION_NETD_CONNECT` workaround.
//!
//! A CACHED locator gets the probe, at
//! [`cerulion_discovery::ladder::LADDER_PROBE_TIMEOUT`]. A cached address is a
//! HINT, not an operator's word: the robot may be off, moved, or re-addressed,
//! and every stale row would otherwise buy a background connector retrying a dead
//! address for the whole life of the daemon.
//!
//! COST: the probes run in ONE parallel round, so the added boot
//! wall is bounded by the ROUND, not by the row count — but the round's worst
//! case is `LADDER_PROBE_TIMEOUT` (300 ms) plus the collector's own 200 ms slack
//! (`probe_reachable_locators`), i.e. **~500 ms**, not the 300 ms the timeout
//! constant suggests. It is ZERO when the cache is empty (the probe returns
//! before spawning anything), and in practice a reachable LAN peer answers in
//! single-digit milliseconds — the 500 ms is what an all-dead cache costs.
//! `cerulion_discovery::peer_cache::MAX_CACHED_PEERS` bounds the row count on the
//! READ side too, so a hand-edited or foreign `peers.json` cannot turn one boot
//! into thousands of resolver threads.
//!
//! # Cadence: ONCE, at daemon start
//!
//! `NetworkConfig` is immutable at `TransportManager::init` and the session is
//! SHARED (Principle #8), so re-folding into a live session would need runtime
//! mutation of an open zenoh session — new machinery, new failure surface, for a
//! case the daemon lifecycle already covers. netd is spawned on the first demand
//! and SELF-EXITS after its idle grace, so a restart IS the refresh: a desk that
//! stops using the network re-folds on its next use.
//!
//! RESIDUAL: a netd held alive for hours by a standing demand (a running
//! vizd) keeps its boot-time connect set, so a robot that changes address
//! mid-session is not picked up until netd restarts. That is exactly today's
//! `CERULION_NETD_CONNECT` behaviour, scouting still covers a same-LAN
//! re-address, and the folded set is reported by `status`
//! ([`crate::protocol::StatusResponse::connect_endpoints`]) so an operator can
//! see what this daemon is CONFIGURED TO DIAL rather than having to infer it.
//!
//! That phrasing is deliberate and is the limit of the observable: the
//! set is fixed BEFORE `TransportManager::init` and netd's zenoh session is
//! LAZY (a netd nobody has demanded from has opened no session at all), so a
//! fiat-trusted DEAD locator reads identically to a live connected one. It
//! answers "why is my desk talking to that address?", never "is that robot up?"
//! — reporting zenoh's live peer set would be a different (and larger) feature.

use cerulion_discovery::ladder::{
    plan_connect_set, probe_reachable_locators, DiscoveredPeer, DiscoveryRung, LADDER_PROBE_TIMEOUT,
};
use cerulion_discovery::peer_cache::{cache_rung, default_cache_path, CACHE_RUNG_BUDGET};
use std::collections::HashSet;
use std::path::Path;

/// What one fold decided — the observable record behind the `info!` breadcrumbs
/// and `status`'s `connect_endpoints` (Principle #3).
#[derive(Debug, Clone, PartialEq, Eq, Default)]
pub struct PeerFold {
    /// The full connect set for the session: explicit locators first (verbatim),
    /// then the cached survivors, deduped.
    pub connect: Vec<String>,
    /// Cached locators actually folded (excludes any that duplicate an explicit
    /// one). Non-empty ⇒ this daemon reaches a robot it was never told about.
    pub folded_cached: Vec<String>,
    /// Cached locators dropped because they did not answer a TCP connect inside
    /// the probe budget, with the rung that produced them (always
    /// [`DiscoveryRung::Cache`] here — carried so the debug line reads the same
    /// as the ladder's).
    pub dropped_cached: Vec<(String, DiscoveryRung)>,
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

/// PURE. Assemble the fold from the explicit locators, the cached candidates and
/// the reachability verdict.
///
/// Kept separate from every syscall — the file read, the TCP probes and the
/// session config all live in [`fold_cached_peers`] — so the DECISION can be
/// oracle-tested (and mutated) without a test ever touching the network or the
/// developer's real `~/.cerulion/peers.json`.
pub fn decide_fold(
    explicit: &[String],
    cached: &[DiscoveredPeer],
    reachable: &HashSet<String>,
) -> PeerFold {
    let trusted = trust_explicit_by_fiat(explicit, reachable.clone());
    let plan = plan_connect_set(explicit, cached, &trusted);
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
        folded_cached: plan.folded_ladder,
        dropped_cached: plan.dropped_ladder,
    }
}

/// Run the fold against the peer cache at `path`: load the TTL-fresh entries,
/// TCP-probe their locators in ONE bounded parallel round, and assemble the
/// connect set via [`decide_fold`]. Path-parameterized so tests drive the REAL
/// function against a temp file — the production wiring resolves
/// [`default_cache_path`] in [`fold_cached_peers`].
///
/// INFALLIBLE: a missing / corrupt / version-mismatched cache yields no
/// candidates (loud inside the reader) and the fold degrades to exactly the
/// explicit locators — i.e. earlier behaviour, never an error, never a stall.
pub fn fold_from_cache_at(explicit: &[String], path: &Path) -> PeerFold {
    let cached = cache_rung(CACHE_RUNG_BUDGET, path);
    // An empty candidate list costs nothing: `probe_reachable_locators` returns
    // immediately without spawning a thread, so a desk that has never seen a
    // robot pays no wall time for this feature at all.
    let locators: Vec<String> = cached.iter().map(|p| p.locator.clone()).collect();
    let reachable = probe_reachable_locators(&locators, LADDER_PROBE_TIMEOUT);
    decide_fold(explicit, &cached, &reachable)
}

/// PRODUCTION ENTRY: fold the verified peer cache into `cfg`'s connect endpoints
/// and report what happened.
///
/// `cfg.connect_endpoints` on entry are the EXPLICIT
/// [`crate::net::CONNECT_ENV`] locators; on return they are the full fold
/// (explicit verbatim and first, then reachable cached survivors). Called once
/// from `main` BEFORE `TransportManager::init`, since the config is immutable
/// after it.
///
/// Degrades quietly and completely: no home directory ⇒ a `debug!` and the
/// config is returned untouched.
pub fn fold_cached_peers(
    cfg: cerulion_core::transport::network::NetworkConfig,
) -> (cerulion_core::transport::network::NetworkConfig, PeerFold) {
    let Some(path) = default_cache_path() else {
        tracing::debug!(
            "cerulion-netd: no home directory; cannot locate the peer cache — first contact \
             relies on scouting or {}",
            crate::net::CONNECT_ENV
        );
        let fold = PeerFold {
            connect: cfg.connect_endpoints.clone(),
            ..PeerFold::default()
        };
        return (cfg, fold);
    };
    fold_cached_peers_at(cfg, &path)
}

/// The REAL fold body, path-parameterized so tests drive the production function
/// against a temp cache instead of the developer's own `~/.cerulion/peers.json`
/// (the same reason `peer_cache::record_confirmed` takes a path). The only thing
/// [`fold_cached_peers`] adds is resolving [`default_cache_path`] and owning the
/// no-home degrade.
pub fn fold_cached_peers_at(
    mut cfg: cerulion_core::transport::network::NetworkConfig,
    path: &Path,
) -> (cerulion_core::transport::network::NetworkConfig, PeerFold) {
    let explicit = cfg.connect_endpoints.clone();
    let fold = fold_from_cache_at(&explicit, path);

    for (locator, rung) in &fold.dropped_cached {
        tracing::debug!(
            locator = %locator,
            rung = %rung,
            timeout_ms = LADDER_PROBE_TIMEOUT.as_millis() as u64,
            "cerulion-netd: a cached peer did not answer a TCP connect within the probe \
             budget — not dialled this run (it will be retried when netd next starts)"
        );
    }
    // The FAILURE narrative has to survive the shipped default level (`info`).
    // The failure this feature newly creates is "a cached row exists but the
    // robot was asleep at daemon boot", and per the once-at-boot cadence it then
    // persists for the life of a vizd-held daemon — so an operator who sees no
    // connection needs to know the difference between "nothing was cached" and
    // "something was cached and none of it answered". These fire ONCE PER BOOT,
    // so the flood-suppression argument does not apply.
    match (
        fold.folded_cached.is_empty(),
        fold.dropped_cached.is_empty(),
    ) {
        // Nothing cached at all — the ordinary cold-desk case, quiet.
        (true, true) => tracing::debug!(
            path = %path.display(),
            "cerulion-netd: no cached peers to fold — first contact relies on scouting or {}",
            crate::net::CONNECT_ENV
        ),
        // Candidates EXISTED and none survived the probe. Loud: this is the
        // "why can't my desk see the robot?" case, and it is indistinguishable
        // from an empty cache in the resulting connect set.
        (true, false) => tracing::info!(
            dropped = fold.dropped_cached.len(),
            path = %path.display(),
            "every cached peer failed the reachability probe — none dialled (was the \
             robot asleep at daemon start?). netd folds the cache ONCE at boot, so restart \
             netd once the robot is up, or set {}",
            crate::net::CONNECT_ENV
        ),
        // LOUD: the line that explains why a desk with no env var can suddenly
        // see a robot, and (with `status`) why it is talking to a particular
        // address. Carries `dropped` too — folding 1 of 9 is a different
        // situation from folding 1 of 1.
        (false, _) => tracing::info!(
            folded = ?fold.folded_cached,
            dropped = fold.dropped_cached.len(),
            path = %path.display(),
            "folded verified cached peers into the netd session — first contact \
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
                folded_cached: strs(&["tcp/10.0.0.5:7683"]),
                dropped_cached: vec![],
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
                folded_cached: strs(&["tcp/10.0.0.5:7683", "tcp/10.0.0.6:7683"]),
                dropped_cached: vec![],
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
                folded_cached: vec![],
                dropped_cached: vec![("tcp/10.0.0.9:7683".to_string(), DiscoveryRung::Cache)],
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
                folded_cached: strs(&["tcp/10.0.0.6:7683"]),
                dropped_cached: vec![],
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
                folded_cached: vec![],
                dropped_cached: vec![],
            }
        );
        // And with no env either, the whole fold is empty (scouting only).
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
