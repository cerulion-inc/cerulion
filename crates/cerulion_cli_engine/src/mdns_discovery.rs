// SPDX-License-Identifier: AGPL-3.0-only
//! The mDNS (`_cerulion._tcp`) robot-discovery rung.
//!
//! Decision: mDNS is the primary robot-discovery rung on all
//! networks — mDNS finds robots; zenoh connects and does everything after.
//! The gateway ALWAYS advertises `_cerulion._tcp` (SRV carries the ACTUAL
//! bound zenoh listen port, TXT carries `robot=<identity>` — the resolved
//! hostname or `CERULION_ROBOT_IDENTITY` override, NOT the graph prefix); a
//! discovering
//! CLI browses the same service type and hands each resolved instance to
//! the [`crate::discovery_ladder`] as an `Mdns`-rung [`DiscoveredPeer`].
//!
//! Both halves are ADDITIVE overlays: an advertise/browse failure is a loud
//! `tracing::warn!` and the caller keeps serving / falls back to the other
//! rungs — a discovery beacon NEVER crashes or fails a run.
//!
//! # The ADVERTISE half moved to `cerulion_mdns`
//!
//! This module used to own BOTH halves, and its advertise had exactly ONE
//! production caller — `graph_cmd::graph_run_gateway`, the standalone `graph
//! run-gateway` child. then folded the permissive gateway plane into
//! the per-computer `cerulion-netd`, and that path spawns NO `run-gateway` child,
//! so a robot serving topics through netd advertised nothing (measured on the Go2:
//! 86 topics served, `dns-sd -B _cerulion._tcp local.` empty from a desk on the
//! same /24). netd must advertise too — and `cerulion_cli_engine` depends on
//! `cerulion_netd`, so netd cannot import the beacon from here (cyclic package
//! edge). The advertise therefore lives in the leaf crate
//! [`cerulion_mdns`], once, and both processes link it; the BROWSE half stays
//! here (netd never browses, and these functions return
//! `cerulion_discovery::DiscoveredPeer` and normalize eids through the CLI's own
//! [`crate::connect_cmd::normalize_eid`] seam).
//!
//! [`advertise_gateway`] below is a thin adapter that keeps the CLI's
//! `CliError` surface byte-unchanged for `graph run-gateway`.

use std::net::IpAddr;
use std::time::{Duration, Instant};

use mdns_sd::{ServiceDaemon, ServiceEvent};

use crate::discovery_ladder::{DiscoveredPeer, DiscoveryRung};
use crate::error::CliError;

// ONE definition of the service type, in the crate the ADVERTISER
// lives in — so the beacon and this browser cannot drift on the one string that
// has to match. Re-exported because `pair_cmd` / the ladder read it from here,
// as is `MdnsAdvertiseGuard` (the adapter below returns it, and
// `graph_cmd::graph_run_gateway` names the type to hold the guard).
//
// The RAW advertise is a PRIVATE import: the adapter is this crate's whole
// public advertise surface, and `pub use`-ing the un-adapted function beside it
// offers callers a second door past the `CliError` mapping the adapter exists
// for. `srv_port_from_listen_endpoints` is NOT re-exported at all — it had no
// consumer through this module (`graph_cmd` calls `cerulion_mdns` directly), and
// a `pub use` escapes `dead_code = deny`, so an unused re-export is invisible
// dead surface rather than a build failure.
pub use cerulion_mdns::{MdnsAdvertiseGuard, CERULION_SERVICE_TYPE};

use cerulion_mdns::{advertise_gateway as advertise_gateway_raw, is_valid_eid};

/// Advertise this robot's gateway over mDNS (`_cerulion._tcp`) — the CLI-side
/// adapter over [`cerulion_mdns::advertise_gateway`], mapping the beacon's typed
/// error onto this crate's `CliError` so `graph run-gateway`'s call site (which
/// demotes it to a loud warn) is byte-unchanged.
///
/// `port` is the ACTUAL bound zenoh listen port (the SRV record); `robot` is the
/// resolved robot identity — the hostname or `CERULION_ROBOT_IDENTITY` override,
/// NOT the graph prefix (the TXT `robot=` value).
pub fn advertise_gateway(robot: &str, port: u16) -> Result<MdnsAdvertiseGuard, CliError> {
    advertise_gateway_raw(robot, port).map_err(|e| CliError::Validation(e.to_string()))
}

/// Browse `_cerulion._tcp` for up to `budget`, returning every resolved
/// robot as an `Mdns`-rung [`DiscoveredPeer`]. Best-effort: a daemon-start
/// or browse failure is a loud `tracing::warn!` and an EMPTY vec (discovery
/// continues on the other rungs) — NEVER an `Err`, never a panic. The daemon
/// is shut down before returning.
pub fn browse_rung(budget: Duration) -> Vec<DiscoveredPeer> {
    let daemon = match ServiceDaemon::new() {
        Ok(d) => d,
        Err(e) => {
            tracing::warn!(
                error = %e,
                "mDNS: cannot start the ServiceDaemon to browse `_cerulion._tcp` — robot discovery continues on the other rungs (cache, hostname)"
            );
            return Vec::new();
        }
    };
    let receiver = match daemon.browse(CERULION_SERVICE_TYPE) {
        Ok(rx) => rx,
        Err(e) => {
            tracing::warn!(
                error = %e,
                "mDNS: cannot browse `_cerulion._tcp` — robot discovery continues on the other rungs (cache, hostname)"
            );
            let _ = daemon.shutdown();
            return Vec::new();
        }
    };

    let mut peers: Vec<DiscoveredPeer> = Vec::new();
    let deadline = Instant::now() + budget;
    loop {
        let now = Instant::now();
        if now >= deadline {
            break;
        }
        let remaining = deadline - now;
        // `recv_timeout` returns Err on BOTH a clean window-timeout and a
        // sender disconnect. Split them by the receiver's own disconnect state
        // — a method call, avoiding any need to name flume's transitive-only
        // `RecvTimeoutError` type (flume is an mdns-sd dependency, not a direct
        // one). A plain timeout is the normal window end (break quietly); a
        // disconnect means the ServiceDaemon's background thread died mid-browse
        // (abnormal — we own the daemon and have NOT shut it down yet), which
        // today silently looks like "no robots". Warn loudly, name that results
        // may be PARTIAL + the escape, then break.
        match receiver.recv_timeout(remaining) {
            Ok(ServiceEvent::ServiceResolved(resolved)) => {
                let addrs: Vec<IpAddr> =
                    resolved.addresses.iter().map(|a| a.to_ip_addr()).collect();
                let robot_txt = resolved.get_property_val_str("robot");
                peers.extend(peer_from_resolved(
                    robot_txt,
                    &resolved.fullname,
                    &addrs,
                    resolved.port,
                ));
            }
            Ok(_) => continue,
            Err(_) => {
                if receiver.is_disconnected() {
                    tracing::warn!(
                        "mDNS: the `_cerulion._tcp` browse daemon died mid-browse — robot discovery results may be PARTIAL; if an expected robot is missing, reach it directly with `--connect tcp/<host>:7683`"
                    );
                }
                break;
            }
        }
    }
    let _ = daemon.shutdown();
    peers
}

/// Resolve a robot NAME to its iroh endpoint id (hex) by browsing
/// `_cerulion._tcp` for up to `budget` and reading the `eid=` TXT record of the
/// record whose `robot=` matches `name`.
///
/// This is the desk-side reader of the `eid=` TXT the gateway ALREADY advertises
/// (see `gateway_txt_properties`) — the discovery half of `cerulion pair
/// <robot>` (and, later, `cerulion connect <robot>`). Best-effort: a daemon-start
/// / browse failure, or no matching record with a valid eid, yields `None` (the
/// caller falls back to `~/.cerulion/robots.toml`) — NEVER an `Err`, never a
/// panic. Returns as soon as the matching robot's valid eid is seen (no need to
/// wait out the whole budget). The daemon is shut down before returning.
pub fn resolve_robot_eid(name: &str, budget: Duration) -> Option<String> {
    let daemon = match ServiceDaemon::new() {
        Ok(d) => d,
        Err(e) => {
            tracing::warn!(
                error = %e, robot = %name,
                "mDNS: cannot start the ServiceDaemon to resolve the robot eid — falling back to ~/.cerulion/robots.toml"
            );
            return None;
        }
    };
    let receiver = match daemon.browse(CERULION_SERVICE_TYPE) {
        Ok(rx) => rx,
        Err(e) => {
            tracing::warn!(
                error = %e, robot = %name,
                "mDNS: cannot browse `_cerulion._tcp` to resolve the robot eid — falling back to ~/.cerulion/robots.toml"
            );
            let _ = daemon.shutdown();
            return None;
        }
    };

    let deadline = Instant::now() + budget;
    let mut found = None;
    loop {
        let now = Instant::now();
        if now >= deadline {
            break;
        }
        let remaining = deadline - now;
        match receiver.recv_timeout(remaining) {
            Ok(ServiceEvent::ServiceResolved(resolved)) => {
                let robot = resolved
                    .get_property_val_str("robot")
                    .map(str::to_string)
                    .unwrap_or_else(|| instance_from_fullname(&resolved.fullname));
                let eid = resolved.get_property_val_str("eid").map(str::to_string);
                if let Some(e) = pick_eid_for_name(&[(robot, eid)], name) {
                    found = Some(e);
                    break;
                }
            }
            Ok(_) => continue,
            Err(_) => break,
        }
    }
    let _ = daemon.shutdown();
    found
}

/// PURE record→eid pick: given resolved `(robot, eid)` records, return the eid of
/// the FIRST record whose `robot` equals `name` AND whose `eid` — normalized via
/// the SHARED [`crate::connect_cmd::normalize_eid`] seam (trim + strip a `0x` prefix +
/// lowercase) — is FORMAT-valid (64 lowercase hex, per `is_valid_eid`). Reusing
/// `normalize_eid` (rather than a local trim+lowercase) makes the mDNS `eid=` TXT
/// rung agree BYTE-FOR-BYTE with the positional / `--eid` / `robots.toml` rungs:
/// an uppercase-advertised OR `0x`-prefixed eid still resolves (returned in
/// canonical lowercase) rather than silently falling through to `robots.toml`. A
/// matching robot with a missing / malformed eid yields `None`. Oracle-tested.
pub fn pick_eid_for_name(records: &[(String, Option<String>)], name: &str) -> Option<String> {
    records.iter().find_map(|(robot, eid)| {
        if robot != name {
            return None;
        }
        let normalized = crate::connect_cmd::normalize_eid(eid.as_ref()?);
        is_valid_eid(&normalized).then_some(normalized)
    })
}

/// Recover the instance name from a resolved fullname by stripping the
/// `.<service-type>` suffix (`myrobot._cerulion._tcp.local.` → `myrobot`).
/// The TXT-`robot` fallback when a peer advertised no `robot=` property.
fn instance_from_fullname(fullname: &str) -> String {
    let suffix = format!(".{CERULION_SERVICE_TYPE}");
    fullname
        .strip_suffix(&suffix)
        .unwrap_or(fullname)
        .to_string()
}

/// Map ONE resolved mDNS record → zero or more [`DiscoveredPeer`]s. The
/// record→peer seam `browse_rung` delegates to (PURE — hand-oracle tested):
///
/// - `robot` = the TXT `robot=` value, or the fullname's instance name when
///   absent.
/// - Address selection PREFERS IPv4: if the record carries any IPv4 address,
///   one peer is emitted per IPv4 address (`tcp/<ip>:<port>`); only when the
///   record is IPv6-ONLY are the IPv6 addresses used (bracket form
///   `tcp/[<ip>]:<port>`). No addresses ⇒ no peers.
fn peer_from_resolved(
    robot_txt: Option<&str>,
    fullname: &str,
    addrs: &[IpAddr],
    port: u16,
) -> Vec<DiscoveredPeer> {
    let robot = robot_txt
        .map(|s| s.to_string())
        .unwrap_or_else(|| instance_from_fullname(fullname));
    let v4: Vec<&IpAddr> = addrs.iter().filter(|a| a.is_ipv4()).collect();
    let chosen: Vec<&IpAddr> = if v4.is_empty() {
        addrs.iter().filter(|a| a.is_ipv6()).collect()
    } else {
        v4
    };
    chosen
        .into_iter()
        .map(|ip| {
            let locator = match ip {
                IpAddr::V4(v4) => format!("tcp/{v4}:{port}"),
                IpAddr::V6(v6) => format!("tcp/[{v6}]:{port}"),
            };
            DiscoveredPeer {
                robot: robot.clone(),
                locator,
                rung: DiscoveryRung::Mdns,
            }
        })
        .collect()
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::net::{Ipv4Addr, Ipv6Addr};

    #[test]
    fn pick_eid_for_name_selects_the_matching_valid_eid() {
        let eid = "00112233445566778899aabbccddeeff00112233445566778899aabbccddeeff";
        let recs = |v: &[(&str, Option<&str>)]| -> Vec<(String, Option<String>)> {
            v.iter()
                .map(|(r, e)| (r.to_string(), e.map(str::to_string)))
                .collect()
        };
        // Matching robot + valid eid → the eid.
        assert_eq!(
            pick_eid_for_name(&recs(&[("go2", Some(eid))]), "go2"),
            Some(eid.to_string())
        );
        // Picks the RIGHT robot among several.
        assert_eq!(
            pick_eid_for_name(
                &recs(&[("spot", Some("aa")), ("go2", Some(eid)), ("orin", None)]),
                "go2"
            ),
            Some(eid.to_string())
        );
        // Matching robot but NO eid advertised → None (fall back to robots.toml).
        assert_eq!(pick_eid_for_name(&recs(&[("go2", None)]), "go2"), None);
        // Matching robot but MALFORMED eid (too short) → None.
        assert_eq!(
            pick_eid_for_name(&recs(&[("go2", Some("dead"))]), "go2"),
            None
        );
        // An UPPERCASE-advertised eid is case-folded to canonical lowercase (so all
        // rungs agree on case), NOT silently dropped.
        assert_eq!(
            pick_eid_for_name(&recs(&[("go2", Some(&eid.to_uppercase()))]), "go2"),
            Some(eid.to_string()),
            "an uppercase eid= TXT resolves, normalized to lowercase"
        );
        // No matching robot → None.
        assert_eq!(
            pick_eid_for_name(&recs(&[("spot", Some(eid))]), "go2"),
            None
        );
        // Empty → None.
        assert_eq!(pick_eid_for_name(&[], "go2"), None);
    }

    /// `pick_eid_for_name`
    /// normalizes through the SHARED `connect_cmd::normalize_eid` seam, so a `0x`
    /// prefix on an advertised `eid=` TXT is stripped EXACTLY as the positional /
    /// `--eid` / `robots.toml` rungs strip it — bare hex and `0x`-prefixed hex
    /// resolve to the SAME canonical lowercase eid. Hand oracle (never a
    /// self-compare): the bare canonical form is the fixed expected value.
    #[test]
    fn pick_eid_for_name_strips_0x_prefix_like_normalize_eid() {
        let bare = "00112233445566778899aabbccddeeff00112233445566778899aabbccddeeff";
        let recs = |v: &[(&str, Option<&str>)]| -> Vec<(String, Option<String>)> {
            v.iter()
                .map(|(r, e)| (r.to_string(), e.map(str::to_string)))
                .collect()
        };
        // Bare 64-hex → itself (baseline, no prefix).
        assert_eq!(
            pick_eid_for_name(&recs(&[("go2", Some(bare))]), "go2"),
            Some(bare.to_string())
        );
        // `0x`-prefixed lowercase → the SAME bare canonical form (prefix stripped).
        let prefixed = format!("0x{bare}");
        assert_eq!(
            pick_eid_for_name(&recs(&[("go2", Some(&prefixed))]), "go2"),
            Some(bare.to_string()),
            "a 0x-prefixed eid resolves to the bare canonical eid"
        );
        // `0x`-prefixed MIXED case + surrounding whitespace → bare lowercase (the
        // full normalize_eid contract: trim, strip 0x, lowercase).
        let messy = format!("  0x{}  ", bare.to_uppercase());
        assert_eq!(
            pick_eid_for_name(&recs(&[("go2", Some(&messy))]), "go2"),
            Some(bare.to_string()),
            "trim + strip 0x + lowercase all apply, matching normalize_eid"
        );
        // Anti-tautology: an `0X`-prefixed eid is NOT stripped (mirroring
        // normalize_eid EXACTLY — it strips only a lowercase `0x`), so the 66-char
        // string fails is_valid_eid → None. Consistent with the connect path.
        let upper_prefix = format!("0X{bare}");
        assert_eq!(
            pick_eid_for_name(&recs(&[("go2", Some(&upper_prefix))]), "go2"),
            None,
            "0X (uppercase prefix) is NOT stripped — identical to normalize_eid"
        );
    }

    #[test]
    fn instance_from_fullname_strips_service_suffix() {
        assert_eq!(
            instance_from_fullname("myrobot._cerulion._tcp.local."),
            "myrobot"
        );
        // A fullname WITHOUT the suffix is returned verbatim (defensive).
        assert_eq!(instance_from_fullname("bare-name"), "bare-name");
    }

    #[test]
    fn peer_from_resolved_uses_txt_robot_value() {
        let addrs = [IpAddr::V4(Ipv4Addr::new(192, 168, 1, 7))];
        let peers = peer_from_resolved(
            Some("robotA"),
            "ignored._cerulion._tcp.local.",
            &addrs,
            7683,
        );
        assert_eq!(peers.len(), 1);
        assert_eq!(peers[0].robot, "robotA");
        assert_eq!(peers[0].locator, "tcp/192.168.1.7:7683");
        assert_eq!(peers[0].rung, DiscoveryRung::Mdns);
    }

    #[test]
    fn peer_from_resolved_falls_back_to_instance_name_when_no_txt() {
        let addrs = [IpAddr::V4(Ipv4Addr::new(10, 0, 0, 1))];
        let peers = peer_from_resolved(None, "myrobot._cerulion._tcp.local.", &addrs, 7683);
        assert_eq!(peers.len(), 1);
        assert_eq!(peers[0].robot, "myrobot");
        assert_eq!(peers[0].locator, "tcp/10.0.0.1:7683");
    }

    #[test]
    fn peer_from_resolved_prefers_ipv4_over_ipv6() {
        // A dual-stack record: v4 wins, the v6 sibling is dropped → ONE peer.
        let addrs = [
            IpAddr::V6(Ipv6Addr::new(0xfe80, 0, 0, 0, 0, 0, 0, 1)),
            IpAddr::V4(Ipv4Addr::new(192, 168, 1, 7)),
        ];
        let peers = peer_from_resolved(Some("dual"), "dual._cerulion._tcp.local.", &addrs, 7683);
        assert_eq!(peers.len(), 1, "v4 preference collapses the v6 sibling");
        assert_eq!(peers[0].locator, "tcp/192.168.1.7:7683");
    }

    #[test]
    fn peer_from_resolved_uses_ipv6_bracket_form_when_v6_only() {
        let addrs = [IpAddr::V6(Ipv6Addr::new(0xfe80, 0, 0, 0, 0, 0, 0, 1))];
        let peers = peer_from_resolved(Some("v6"), "v6._cerulion._tcp.local.", &addrs, 7683);
        assert_eq!(peers.len(), 1);
        assert_eq!(peers[0].locator, "tcp/[fe80::1]:7683");
        assert_eq!(peers[0].rung, DiscoveryRung::Mdns);
    }

    #[test]
    fn peer_from_resolved_emits_one_peer_per_v4_address() {
        let addrs = [
            IpAddr::V4(Ipv4Addr::new(192, 168, 1, 7)),
            IpAddr::V4(Ipv4Addr::new(192, 168, 1, 8)),
        ];
        let peers = peer_from_resolved(Some("multi"), "multi._cerulion._tcp.local.", &addrs, 7683);
        // Input Vec order is preserved by the map → deterministic oracle.
        assert_eq!(peers.len(), 2);
        assert_eq!(peers[0].locator, "tcp/192.168.1.7:7683");
        assert_eq!(peers[1].locator, "tcp/192.168.1.8:7683");
        assert!(peers.iter().all(|p| p.robot == "multi"));
    }

    #[test]
    fn peer_from_resolved_empty_addrs_yields_no_peers() {
        let peers = peer_from_resolved(Some("x"), "x._cerulion._tcp.local.", &[], 7683);
        assert!(peers.is_empty());
    }
}
