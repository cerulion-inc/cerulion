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
//! the [`crate::discovery_ladder`] as an `Mdns`-rung
//! [`cerulion_discovery::ladder::DiscoveredPeer`].
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
//! [`cerulion_mdns`], once, and both processes link it. The bounded browse
//! ([`browse_rung`]) moved there too, because netd's boot-time peer fold browses
//! as well; it is re-exported here for the ladder. The `eid=` reader
//! ([`resolve_robot_eid`]) stays here: it normalizes eids through the CLI's own
//! [`crate::connect_cmd::normalize_eid`] seam.
//!
//! [`advertise_gateway`] below is a thin adapter that keeps the CLI's
//! `CliError` surface byte-unchanged for `graph run-gateway`.

use std::time::{Duration, Instant};

use mdns_sd::{ServiceDaemon, ServiceEvent};

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
// The bounded browse lives in `cerulion_mdns` so `cerulion-netd`'s boot fold
// runs the same record-to-locator mapping as the ladder.
pub use cerulion_mdns::browse_rung;
use cerulion_mdns::instance_from_fullname;

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

#[cfg(test)]
mod tests {
    use super::*;

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
}
