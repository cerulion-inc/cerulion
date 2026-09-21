// SPDX-License-Identifier: AGPL-3.0-only
//! The robot-gateway discovery LADDER.
//!
//! `topic list` (and, in time, every networked CLI verb) needs to FIND the
//! robots on the LAN before it can talk to them. The ladder answers "which
//! gateways exist?" by running several independent discovery RUNGS — mDNS
//! (the primary rung), a cached-peer file, and a hostname
//! convention — and merging what they find.
//!
//! # Model
//!
//! mDNS finds robots; zenoh connects and does everything after. A rung's only
//! job is to surface a `(robot, locator)` so the caller can hand the locator to
//! ONE query session — liveliness/announce discovery, tapping, and every richer
//! interaction ride the connected session, not the ladder. A rung
//! NEVER opens its own zenoh session — it produces a candidate locator; the
//! caller's single query session connects to everything, and PRESENCE in the
//! gather — an announce token arriving, or an mDNS browse answer — is what
//! confirms a robot live (never a per-candidate probe).
//!
//! The rungs run in PARALLEL under a single [`LADDER_TOTAL_CEILING`] and the
//! ladder is GATHER-ALL, never short-circuit: multiple robots on one network is
//! the norm, so every rung's results are merged (a robot found by two rungs is
//! deduped by its normalized `(ip|port)` address key — pure, no DNS — with
//! first-rung-order winning the tie). A
//! rung that overruns the ceiling has its results DROPPED (its thread is
//! detached, never blocked on); a rung that panics is contained. Every rung fn
//! is INFALLIBLE by contract — it warns internally and returns an empty vec on
//! failure — so the ladder never has to reason about per-rung error types.
//!
//! # Injectability
//!
//! [`gather_rungs`] is the injectable core: it takes a list of named rung
//! closures and the ceiling, so the parallelism / ceiling / panic-isolation /
//! dedupe / determinism contracts are unit-testable with hermetic in-memory
//! rungs (no mDNS, no multicast, no sessions). [`discover_peers`] is the real
//! wiring that feeds it the three production rungs in preference order.
//!
//! # Where the shared half lives
//!
//! The candidate TYPES ([`DiscoveredPeer`] / [`DiscoveryRung`]), the locator
//! parser + dedupe key, the bounded TCP reachability pre-filter
//! ([`probe_reachable_locators`]) and the pure connect-set planner
//! ([`plan_connect_set`]) moved to [`cerulion_discovery::ladder`] and are
//! RE-EXPORTED here, so every call site in this crate is unchanged.
//!
//! They moved because `cerulion-netd` needs the SAME primitives to fold
//! TTL-fresh cached peers into its own session at boot, and
//! `cerulion_cli_engine` already depends on `cerulion_netd` — so netd importing
//! from here would be a cyclic package edge. What stays in this file is exactly
//! what netd does NOT need: the RUNGS (mDNS / hostname / subnet sweep), the
//! parallel gather engine, and their budgets.

pub use cerulion_discovery::ladder::{
    dedupe_by_resolved_addr, locator_is_syntactically_valid, plan_connect_set,
    probe_reachable_locators, ConnectPlan, DiscoveredPeer, DiscoveryRung, EXPLICIT_PROBE_TIMEOUT,
    LADDER_PROBE_TIMEOUT,
};
// `pub(crate)`, NOT `pub`: this was `pub(crate)` before the move to
// `cerulion_discovery` and its only consumer is the subnet sweep
// (`subnet_sweep.rs`). Re-exporting it publicly would widen this crate's API
// surface for nobody.
pub(crate) use cerulion_discovery::ladder::tcp_port_open;
// The cached-peer rung's budget lives with the rung it belongs to (the peer
// cache reader moved to `cerulion_discovery`); re-exported so
// `discover_peers` below — and the budget oracle — read unchanged.
pub use cerulion_discovery::peer_cache::CACHE_RUNG_BUDGET;

use std::sync::mpsc;
use std::thread;
use std::time::{Duration, Instant};

/// The total wall-clock ceiling for one ladder run. Every rung is joined
/// against a shared absolute deadline computed from this; a rung still running
/// past it has its results dropped.
pub const LADDER_TOTAL_CEILING: Duration = Duration::from_millis(1500);

/// Per-rung budget handed to the mDNS browse rung — the PRIMARY rung gets the
/// largest slice (multicast browse needs the most time to hear responders).
pub const MDNS_RUNG_BUDGET: Duration = Duration::from_millis(1000);

/// Per-rung budget handed to the hostname-convention rung (a fast resolve).
pub const HOSTNAME_RUNG_BUDGET: Duration = Duration::from_millis(300);

/// Per-rung budget handed to the OPT-IN `--scan` subnet-sweep rung (rung 4).
/// Deliberately LARGER than the default rungs' budgets — a unicast /24 connect
/// sweep across up to ~254 hosts needs the most time. It EXCEEDS
/// [`LADDER_TOTAL_CEILING`] by design, so a scan run
/// switches to the wider [`SCAN_LADDER_CEILING`] (see [`ladder_ceiling`]). Only
/// ever consumed when `--scan` is passed (see [`discover_peers`]).
pub const SWEEP_RUNG_BUDGET: Duration = Duration::from_millis(2500);

/// The wider wall-clock ceiling used for a `--scan` ladder run, sized to
/// accommodate [`SWEEP_RUNG_BUDGET`] (which exceeds the default
/// [`LADDER_TOTAL_CEILING`]). A default (non-scan) run keeps the snappy
/// [`LADDER_TOTAL_CEILING`].
pub const SCAN_LADDER_CEILING: Duration = Duration::from_millis(3000);

/// The ladder's wall-clock ceiling for a run: the snappy [`LADDER_TOTAL_CEILING`]
/// by default, or the wider [`SCAN_LADDER_CEILING`] when the opt-in `--scan`
/// sweep (rung 4) is included (it is inherently slower — a /24 connect sweep of
/// up to ~254 hosts). Pure — oracle-tested.
pub fn ladder_ceiling(scan: bool) -> Duration {
    if scan {
        SCAN_LADDER_CEILING
    } else {
        LADDER_TOTAL_CEILING
    }
}

/// A discovery rung: a name (for breadcrumbs) plus a boxed, `Send` closure that
/// runs the rung and returns whatever peers it found. The closure is INFALLIBLE
/// by contract (warns internally, returns empty on failure).
pub type NamedRung = (
    &'static str,
    Box<dyn FnOnce() -> Vec<DiscoveredPeer> + Send>,
);

/// Internal tagged result funnelled back from each rung thread.
enum RungMsg {
    /// The rung at this index finished and returned these peers.
    Done(usize, Vec<DiscoveredPeer>),
    /// The rung at this index panicked (its payload is discarded).
    Panicked(usize),
}

/// Run every rung in parallel under a shared absolute deadline and merge the
/// results (rung-list order, then deduped by resolved address, then sorted for
/// deterministic presentation).
///
/// Each rung runs on its own thread. Results are joined against ONE deadline
/// (`ceiling` from call start): a rung whose result has not arrived by the
/// deadline is DROPPED with a loud `tracing::warn!` naming it and the
/// ceiling, and its thread is detached (never blocked on). A rung that PANICS
/// is caught, warned, and skipped — the other rungs are unaffected.
pub fn gather_rungs(rungs: Vec<NamedRung>, ceiling: Duration) -> Vec<DiscoveredPeer> {
    let deadline = Instant::now() + ceiling;
    let n = rungs.len();
    let names: Vec<&'static str> = rungs.iter().map(|(name, _)| *name).collect();
    let (tx, rx) = mpsc::channel::<RungMsg>();

    let mut spawned = vec![false; n];
    for (idx, (name, rung)) in rungs.into_iter().enumerate() {
        let tx = tx.clone();
        match thread::Builder::new()
            .name(format!("discovery-rung-{name}"))
            .spawn(move || {
                // The rung is not necessarily unwind-safe, but we discard it on
                // panic — asserting unwind-safety is sound here.
                match std::panic::catch_unwind(std::panic::AssertUnwindSafe(rung)) {
                    Ok(peers) => {
                        let _ = tx.send(RungMsg::Done(idx, peers));
                    }
                    Err(_) => {
                        let _ = tx.send(RungMsg::Panicked(idx));
                    }
                }
            }) {
            Ok(_handle) => spawned[idx] = true,
            Err(e) => {
                // Thread spawn failure (resource exhaustion) is rare; report it
                // and leave the slot empty (it is never awaited below).
                tracing::warn!(
                    rung = %name,
                    error = %e,
                    "failed to spawn a discovery rung thread — skipped"
                );
            }
        }
    }
    // Drop our own sender so the channel disconnects once every spawned rung has
    // reported, letting the receive loop end early on a clean sweep.
    drop(tx);

    // Slots preserve rung-list order for the merge (dedupe preference).
    let mut slots: Vec<Option<Vec<DiscoveredPeer>>> = (0..n).map(|_| None).collect();
    let expected = spawned.iter().filter(|s| **s).count();
    let mut received = 0usize;
    while received < expected {
        let remaining = deadline.saturating_duration_since(Instant::now());
        if remaining.is_zero() {
            break;
        }
        match rx.recv_timeout(remaining) {
            Ok(RungMsg::Done(idx, peers)) => {
                slots[idx] = Some(peers);
                received += 1;
            }
            Ok(RungMsg::Panicked(idx)) => {
                tracing::warn!(
                    rung = %names[idx],
                    "a discovery rung panicked — skipped (other rungs unaffected)"
                );
                slots[idx] = Some(Vec::new());
                received += 1;
            }
            // Window elapsed, or every sender is gone — either way the bounded
            // gather ends with whatever arrived in time.
            Err(_) => break,
        }
    }

    let mut merged: Vec<DiscoveredPeer> = Vec::new();
    for (idx, slot) in slots.into_iter().enumerate() {
        match slot {
            Some(peers) => merged.extend(peers),
            // Spawned but never reported ⇒ it overran the ceiling. (An unspawned
            // slot was already warned above.)
            None if spawned[idx] => {
                tracing::warn!(
                    rung = %names[idx],
                    ceiling_ms = ceiling.as_millis() as u64,
                    "a discovery rung exceeded the ladder ceiling — its results were \
                     dropped (thread detached, never blocked on)"
                );
            }
            None => {}
        }
    }

    let mut deduped = dedupe_by_resolved_addr(merged);
    deduped.sort_by(|a, b| {
        a.robot
            .cmp(&b.robot)
            .then_with(|| a.locator.cmp(&b.locator))
    });
    deduped
}

/// Run the real discovery ladder: the three production rungs in preference
/// order (mDNS FIRST so it wins dedupe ties), gathered under
/// [`ladder_ceiling`]. The ladder no longer opens any zenoh session
/// (the rungs are pure candidate producers) and no longer persists the cache
/// here — the cache write-back moved POST-gather to
/// [`crate::topic_cmd::query_remote_topics_with_scan`], where only robots
/// CONFIRMED live by the gather (announce presence OR an mDNS browse answer)
/// are recorded.
///
/// `scan` (rung 4, `--scan`): the OPT-IN unicast subnet-sweep rung is appended
/// ONLY when this is `true`. That is the STRUCTURAL guarantee the opt-in needs —
/// the sweep is unreachable on a default run because the sole producer of a
/// `true` here is the `--scan` CLI flag (there is no env var, no config key that
/// enables it). A horizontal SYN sweep reads as port-scan recon to corporate
/// IDS, so it must never fire unopted. A scan run also widens the gather ceiling
/// (see [`ladder_ceiling`]) so the slower sweep is not dropped.
pub fn discover_peers(scan: bool) -> Vec<DiscoveredPeer> {
    let mut rungs: Vec<NamedRung> = vec![
        (
            "mdns",
            Box::new(|| crate::mdns_discovery::browse_rung(MDNS_RUNG_BUDGET)),
        ),
        (
            "cache",
            // The rung itself is path-parameterized (testable against a temp
            // file); the production wiring resolves the default path HERE and
            // owns the no-home warn.
            Box::new(|| match crate::peer_cache::default_cache_path() {
                Some(path) => crate::peer_cache::cache_rung(CACHE_RUNG_BUDGET, &path),
                None => {
                    tracing::warn!(
                        "no home directory; cannot locate the peer cache — skipping the cache rung"
                    );
                    Vec::new()
                }
            }),
        ),
        (
            "hostname",
            Box::new(|| crate::hostname_peers::hostname_rung(HOSTNAME_RUNG_BUDGET)),
        ),
    ];
    if scan {
        // Rung 4, opt-in ONLY. Appended here and NOWHERE else — this `if scan`
        // is the structural gate. A scan-tagged peer is LAST in the rung list,
        // so it loses a dedupe tie to an mDNS/cache/hostname twin for the same
        // resolved `(ip, port)` (an already-verified rung wins). The explicit
        // `NamedRung` binding drives the `Box<dyn ...>` unsizing coercion, exactly
        // like the `Vec<NamedRung>` annotation above.
        let scan_rung: NamedRung = (
            "scan",
            Box::new(|| crate::subnet_sweep::sweep_rung(SWEEP_RUNG_BUDGET)),
        );
        rungs.push(scan_rung);
    }
    let result = gather_rungs(rungs, ladder_ceiling(scan));

    if result.is_empty() {
        // Loud (but info — an empty LAN is not an error): name the escapes.
        tracing::info!(
            "discovery ladder found no robots — pass `--connect tcp/<host>:7683` to reach a \
             known gateway directly; `cerulion rendezvous` is the future answer for \
             isolated networks with no LAN discovery"
        );
    } else {
        tracing::info!(peers = result.len(), "discovery ladder gathered");
    }
    result
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn budget_consts_are_sane() {
        assert_eq!(LADDER_TOTAL_CEILING, Duration::from_millis(1500));
        assert_eq!(MDNS_RUNG_BUDGET, Duration::from_millis(1000));
        assert_eq!(CACHE_RUNG_BUDGET, Duration::from_millis(300));
        assert_eq!(HOSTNAME_RUNG_BUDGET, Duration::from_millis(300));
        // Every DEFAULT per-rung budget fits inside the total ceiling. (The
        // opt-in scan rung deliberately exceeds it — it rides the wider
        // SCAN_LADDER_CEILING, pinned separately below.)
        for b in [MDNS_RUNG_BUDGET, CACHE_RUNG_BUDGET, HOSTNAME_RUNG_BUDGET] {
            assert!(
                b <= LADDER_TOTAL_CEILING,
                "rung budget {b:?} exceeds the ceiling"
            );
        }
    }

    /// Rung 4: the scan budgets are internally consistent — the sweep
    /// rung's budget must fit inside the WIDER scan ceiling (else the ladder
    /// would drop the sweep's own results), and the scan ceiling must exceed
    /// the default ceiling (the whole point of widening it).
    #[test]
    fn scan_budget_consts_are_sane() {
        assert_eq!(SWEEP_RUNG_BUDGET, Duration::from_millis(2500));
        assert_eq!(SCAN_LADDER_CEILING, Duration::from_millis(3000));
        assert!(
            SWEEP_RUNG_BUDGET <= SCAN_LADDER_CEILING,
            "the sweep rung budget must fit inside the scan ceiling or its own \
             results get dropped"
        );
        assert!(
            SCAN_LADDER_CEILING > LADDER_TOTAL_CEILING,
            "the scan ceiling must be wider than the default (the sweep is slower)"
        );
        // The default rungs cannot fit the sweep — that mismatch is exactly why
        // a scan run switches ceilings.
        assert!(SWEEP_RUNG_BUDGET > LADDER_TOTAL_CEILING);
    }

    /// `ladder_ceiling` picks the snappy default for a normal run and
    /// the wider one for a `--scan` run. Hand oracle.
    #[test]
    fn ladder_ceiling_widens_only_for_scan() {
        assert_eq!(ladder_ceiling(false), LADDER_TOTAL_CEILING);
        assert_eq!(ladder_ceiling(true), SCAN_LADDER_CEILING);
    }

    /// STRUCTURAL pin: the subnet sweep must be UNREACHABLE without the
    /// `--scan` flag. `discover_peers` takes an explicit `scan: bool` (Rust has
    /// no default args, so every caller passes it), and the `sweep_rung` push is
    /// gated behind an `if scan` INSIDE the function body — the sole `sweep_rung`
    /// reference in production must sit after that gate. A refactor that wires
    /// the sweep unconditionally (or reads it from an env/config) fails here.
    #[test]
    fn subnet_sweep_is_structurally_gated_behind_the_scan_flag() {
        let src = include_str!("discovery_ladder.rs");
        // Isolate the production region (exclude this test module, which names
        // the tokens in its own assertions).
        let prod = &src[..src
            .find("#[cfg(test)]")
            .expect("the test module marker must exist")];

        assert!(
            prod.contains("pub fn discover_peers(scan: bool)"),
            "discover_peers must take an explicit `scan: bool` — no default arg, \
             so the sweep cannot be reached without threading the flag"
        );
        // The sweep rung is wired in exactly one place, and it is gated.
        assert_eq!(
            prod.matches("subnet_sweep::sweep_rung").count(),
            1,
            "the sweep rung must be wired in exactly one place (the gated push)"
        );
        let gate = prod
            .find("if scan {")
            .expect("the `if scan {` gate must exist in discover_peers");
        let wired = prod
            .find("subnet_sweep::sweep_rung")
            .expect("the sweep rung must be wired");
        assert!(
            gate < wired,
            "the sole `sweep_rung` wiring must sit AFTER the `if scan {{` gate \
             (structurally unreachable on a default run)"
        );
    }
}
