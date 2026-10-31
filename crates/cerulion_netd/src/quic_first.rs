// SPDX-License-Identifier: AGPL-3.0-only
//! `CERULION_UNENCRYPTED_QUIC=on`: netd listens on zenoh's unencrypted QUIC
//! carrier beside TCP, and dials it first, falling back to TCP loudly.
//!
//! Off (the default) changes nothing. On, applied once at daemon start inside
//! [`crate::discovery_fold::fold_discovered_peers`], before the lazy session is
//! configured:
//!
//! 1. **Listen.** Every `tcp/<host>:<port>` in [`crate::net::LISTEN_ENV`] gains
//!    `udp/<host>:<port>?rel=1` (UDP and TCP port numbers are separate spaces, so
//!    the same number serves both). The beacon then advertises the QUIC port as
//!    the `udp_rel` TXT key ([`crate::beacon`]).
//! 2. **Dial.** A candidate is a discovered peer that advertised `udp_rel` and
//!    survived the TCP probe, or an explicit [`crate::net::CONNECT_ENV`] locator
//!    spelled `udp/<host>:<port>?rel=1`. Each candidate's QUIC locator is probed
//!    ([`crate::quic_probe`]) in one parallel round. Answered: the QUIC locator
//!    goes into the connect list. Not answered: the `tcp/` locator for the same
//!    address does, and ONE `warn!` names the peer's QUIC locator, the reason and
//!    the TCP locator. The fold puts one locator per peer address into zenoh's connect
//!    list, so zenoh does not run a QUIC and a TCP connector for the same peer
//!    side by side (unless [`crate::net::CONNECT_ENV`] lists both itself).
//!
//! An explicit `tcp/` locator is never probed or rewritten. Cached peers
//! (`~/.cerulion/peers.json`) carry no TXT record and stay `tcp/`.
//!
//! The QUIC this selects has encryption and peer authentication switched off,
//! like the `tcp/` carrier it replaces: use it on a network that already
//! provides both (a WireGuard overlay), or on a LAN where `tcp/` was acceptable.

use std::collections::{HashMap, HashSet};

use cerulion_core::transport::network::NetworkConfig;
use cerulion_discovery::unencrypted_quic::{
    tcp_twin_of_udp_rel, with_udp_rel_listeners, UdpRelTwin, UnencryptedQuic, UNENCRYPTED_QUIC_ENV,
};

use crate::discovery_fold::PeerFold;
use crate::quic_probe::QuicProbeOutcome;

/// One peer the fold dialled over TCP although QUIC was wanted.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct QuicFallback {
    /// The unencrypted QUIC locator that was probed.
    pub quic: String,
    /// The `tcp/` locator dialled instead.
    pub tcp: String,
    /// Why the QUIC locator was not used.
    pub reason: QuicProbeOutcome,
}

/// What [`apply_unencrypted_quic`] did, for the log lines and the tests.
#[derive(Debug, Clone, PartialEq, Eq, Default)]
pub struct QuicFirstOutcome {
    /// `udp/...?rel=1` listeners added beside the `tcp/` ones.
    pub listen_added: Vec<String>,
    /// QUIC locators that answered and are in the connect list.
    pub dialled_quic: Vec<String>,
    /// Candidates that fell back to TCP.
    pub fell_back: Vec<QuicFallback>,
}

/// A connect-list entry the opt-in may dial over QUIC.
struct Candidate {
    quic: String,
    tcp: String,
}

/// Apply the opt-in to the folded config. `explicit` is the
/// [`crate::net::CONNECT_ENV`] list, `twins` the discovered peers that
/// advertised a QUIC port, and `probe` the QUIC probe over a set of `udp/`
/// locators (production: [`crate::quic_probe::probe_quic_locators`]).
///
/// With [`UnencryptedQuic::Off`] it returns at once and `cfg` / `fold` are not
/// touched.
pub fn apply_unencrypted_quic(
    cfg: &mut NetworkConfig,
    fold: &mut PeerFold,
    explicit: &[String],
    twins: &[UdpRelTwin],
    switch: UnencryptedQuic,
    probe: impl FnOnce(&[String]) -> HashMap<String, QuicProbeOutcome>,
) -> QuicFirstOutcome {
    if switch == UnencryptedQuic::Off {
        return QuicFirstOutcome::default();
    }
    let mut outcome = QuicFirstOutcome::default();

    let listen = with_udp_rel_listeners(&cfg.listen_endpoints);
    outcome.listen_added = listen
        .iter()
        .filter(|l| !cfg.listen_endpoints.contains(l))
        .cloned()
        .collect();
    cfg.listen_endpoints = listen;
    if !outcome.listen_added.is_empty() {
        tracing::info!(
            added = ?outcome.listen_added,
            "cerulion-netd: {UNENCRYPTED_QUIC_ENV}=on, listening on unencrypted QUIC beside \
             tcp/ (no encryption and no peer authentication, the same as tcp/)"
        );
    }

    let explicit: HashSet<&str> = explicit.iter().map(String::as_str).collect();
    let twin_of: HashMap<&str, &str> = twins
        .iter()
        .map(|t| (t.tcp.as_str(), t.udp_rel.as_str()))
        .collect();
    let candidates: HashMap<String, Candidate> = fold
        .connect
        .iter()
        .filter_map(|locator| {
            let candidate = if explicit.contains(locator.as_str()) {
                tcp_twin_of_udp_rel(locator).map(|tcp| Candidate {
                    quic: locator.clone(),
                    tcp,
                })
            } else {
                twin_of.get(locator.as_str()).map(|quic| Candidate {
                    quic: (*quic).to_string(),
                    tcp: locator.clone(),
                })
            };
            candidate.map(|c| (locator.clone(), c))
        })
        .collect();
    if candidates.is_empty() {
        return outcome;
    }

    let mut to_probe: Vec<String> = candidates.values().map(|c| c.quic.clone()).collect();
    to_probe.sort();
    to_probe.dedup();
    let probed = probe(&to_probe);

    let mut seen: HashSet<String> = HashSet::new();
    // One verdict per QUIC locator: an explicit `udp/` entry and a discovered
    // `tcp/` entry for the same address share one probe, one log line, one row.
    let mut decided: HashSet<String> = HashSet::new();
    let mut rewrite: HashMap<String, String> = HashMap::new();
    let mut connect = Vec::with_capacity(fold.connect.len());
    for locator in &fold.connect {
        let chosen = match candidates.get(locator) {
            None => locator.clone(),
            Some(c) => {
                let reason = probed.get(&c.quic).cloned().unwrap_or_else(|| {
                    QuicProbeOutcome::Failed("the probe returned no result".to_string())
                });
                let first = decided.insert(c.quic.clone());
                if reason.answered() {
                    if first {
                        outcome.dialled_quic.push(c.quic.clone());
                    }
                    c.quic.clone()
                } else if !first {
                    c.tcp.clone()
                } else {
                    tracing::warn!(
                        quic = %c.quic,
                        tcp = %c.tcp,
                        reason = %reason,
                        "QUIC FALLBACK: a cerulion-netd peer did not answer on its \
                         unencrypted QUIC locator, so it is dialled over TCP instead. \
                         {UNENCRYPTED_QUIC_ENV}=on asked for QUIC first; this peer stays on TCP \
                         until netd next starts."
                    );
                    outcome.fell_back.push(QuicFallback {
                        quic: c.quic.clone(),
                        tcp: c.tcp.clone(),
                        reason,
                    });
                    c.tcp.clone()
                }
            }
        };
        if chosen != *locator {
            rewrite.insert(locator.clone(), chosen.clone());
        }
        if seen.insert(chosen.clone()) {
            connect.push(chosen);
        }
    }
    if !outcome.dialled_quic.is_empty() {
        tracing::info!(
            quic = ?outcome.dialled_quic,
            "cerulion-netd: dialling these peers over unencrypted QUIC ({UNENCRYPTED_QUIC_ENV}=on)"
        );
    }
    for locator in &mut fold.folded_discovered {
        if let Some(new) = rewrite.get(locator) {
            *locator = new.clone();
        }
    }
    fold.connect.clone_from(&connect);
    cfg.connect_endpoints = connect;
    outcome
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::cell::RefCell;

    fn strs(v: &[&str]) -> Vec<String> {
        v.iter().map(|s| s.to_string()).collect()
    }

    fn cfg(connect: &[&str], listen: &[&str]) -> NetworkConfig {
        crate::net::desk_network_config(strs(connect), strs(listen))
    }

    fn fold_of(connect: &[&str]) -> PeerFold {
        PeerFold {
            connect: strs(connect),
            ..PeerFold::default()
        }
    }

    fn twin(tcp: &str, udp: &str) -> UdpRelTwin {
        UdpRelTwin {
            tcp: tcp.to_string(),
            udp_rel: udp.to_string(),
        }
    }

    #[test]
    fn off_touches_nothing_and_never_probes() {
        let mut c = cfg(&["udp/10.0.0.1:7683?rel=1"], &["tcp/0.0.0.0:7683"]);
        let before = format!("{c:?}");
        let mut f = fold_of(&["udp/10.0.0.1:7683?rel=1", "tcp/10.0.0.5:7683"]);
        let f_before = f.clone();
        let out = apply_unencrypted_quic(
            &mut c,
            &mut f,
            &strs(&["udp/10.0.0.1:7683?rel=1"]),
            &[twin("tcp/10.0.0.5:7683", "udp/10.0.0.5:7683?rel=1")],
            UnencryptedQuic::Off,
            |_| panic!("off never probes"),
        );
        assert_eq!(out, QuicFirstOutcome::default());
        assert_eq!(format!("{c:?}"), before);
        assert_eq!(f, f_before);
    }

    #[test]
    fn on_adds_listen_twins_and_rewrites_only_candidates() {
        let mut c = cfg(&[], &["tcp/0.0.0.0:7683"]);
        let mut f = PeerFold {
            connect: strs(&["tcp/1.1.1.1:7683", "tcp/10.0.0.5:7683", "tcp/10.0.0.6:7683"]),
            folded_discovered: strs(&["tcp/10.0.0.5:7683", "tcp/10.0.0.6:7683"]),
            dropped_discovered: Vec::new(),
        };
        let probed = RefCell::new(Vec::new());
        let out = apply_unencrypted_quic(
            &mut c,
            &mut f,
            &strs(&["tcp/1.1.1.1:7683"]),
            &[
                twin("tcp/10.0.0.5:7683", "udp/10.0.0.5:7683?rel=1"),
                // An explicit tcp/ twin is never a candidate.
                twin("tcp/1.1.1.1:7683", "udp/1.1.1.1:7683?rel=1"),
            ],
            UnencryptedQuic::On,
            |locs| {
                probed.borrow_mut().extend_from_slice(locs);
                locs.iter()
                    .map(|l| (l.clone(), QuicProbeOutcome::Answered))
                    .collect()
            },
        );
        assert_eq!(*probed.borrow(), strs(&["udp/10.0.0.5:7683?rel=1"]));
        assert_eq!(
            c.listen_endpoints,
            strs(&["tcp/0.0.0.0:7683", "udp/0.0.0.0:7683?rel=1"])
        );
        assert_eq!(out.listen_added, strs(&["udp/0.0.0.0:7683?rel=1"]));
        let want = strs(&[
            "tcp/1.1.1.1:7683",
            "udp/10.0.0.5:7683?rel=1",
            "tcp/10.0.0.6:7683",
        ]);
        assert_eq!(f.connect, want);
        assert_eq!(c.connect_endpoints, want);
        assert_eq!(
            f.folded_discovered,
            strs(&["udp/10.0.0.5:7683?rel=1", "tcp/10.0.0.6:7683"])
        );
        assert_eq!(out.dialled_quic, strs(&["udp/10.0.0.5:7683?rel=1"]));
        assert!(out.fell_back.is_empty());
    }

    #[test]
    fn an_unanswered_candidate_falls_back_to_its_tcp_locator() {
        let mut c = cfg(&["udp/10.0.0.1:7683?rel=1"], &[]);
        let mut f = fold_of(&["udp/10.0.0.1:7683?rel=1", "tcp/10.0.0.5:7683"]);
        let out = apply_unencrypted_quic(
            &mut c,
            &mut f,
            &strs(&["udp/10.0.0.1:7683?rel=1"]),
            &[twin("tcp/10.0.0.5:7683", "udp/10.0.0.5:7690?rel=1")],
            UnencryptedQuic::On,
            |locs| {
                locs.iter()
                    .map(|l| {
                        (
                            l.clone(),
                            QuicProbeOutcome::NoAnswer {
                                after: std::time::Duration::from_millis(1000),
                            },
                        )
                    })
                    .collect()
            },
        );
        assert_eq!(f.connect, strs(&["tcp/10.0.0.1:7683", "tcp/10.0.0.5:7683"]));
        assert_eq!(c.connect_endpoints, f.connect);
        assert_eq!(out.fell_back.len(), 2);
        assert!(out.dialled_quic.is_empty());
        // No listen locator to twin: none added.
        assert!(out.listen_added.is_empty());
    }

    #[test]
    fn an_explicit_and_a_discovered_entry_for_one_address_get_one_verdict() {
        let mut c = cfg(&["udp/10.0.0.5:7683?rel=1"], &[]);
        let mut f = fold_of(&["udp/10.0.0.5:7683?rel=1", "tcp/10.0.0.5:7683"]);
        let out = apply_unencrypted_quic(
            &mut c,
            &mut f,
            &strs(&["udp/10.0.0.5:7683?rel=1"]),
            &[twin("tcp/10.0.0.5:7683", "udp/10.0.0.5:7683?rel=1")],
            UnencryptedQuic::On,
            |locs| {
                assert_eq!(locs.len(), 1, "one probe per QUIC locator: {locs:?}");
                locs.iter()
                    .map(|l| (l.clone(), QuicProbeOutcome::Refused))
                    .collect()
            },
        );
        assert_eq!(f.connect, strs(&["tcp/10.0.0.5:7683"]));
        assert_eq!(out.fell_back.len(), 1, "{out:?}");
    }

    #[test]
    fn a_fallback_that_duplicates_an_existing_entry_is_listed_once() {
        let mut c = cfg(&[], &[]);
        let mut f = fold_of(&["udp/10.0.0.1:7683?rel=1", "tcp/10.0.0.1:7683"]);
        apply_unencrypted_quic(
            &mut c,
            &mut f,
            &strs(&["udp/10.0.0.1:7683?rel=1", "tcp/10.0.0.1:7683"]),
            &[],
            UnencryptedQuic::On,
            |locs| {
                locs.iter()
                    .map(|l| (l.clone(), QuicProbeOutcome::Refused))
                    .collect()
            },
        );
        assert_eq!(f.connect, strs(&["tcp/10.0.0.1:7683"]));
    }
}
