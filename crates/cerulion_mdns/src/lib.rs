// SPDX-License-Identifier: AGPL-3.0-only
//! The `_cerulion._tcp` mDNS gateway beacon: the advertise half.
//!
//! mDNS is the primary robot-discovery rung on every network: mDNS finds robots,
//! zenoh connects and does everything after. A serving gateway always advertises
//! `_cerulion._tcp`. The SRV record carries the zenoh listen port that was actually
//! bound, and the TXT record carries `robot=<identity>` (the resolved hostname or
//! the `CERULION_ROBOT_IDENTITY` override, not the graph prefix) plus the
//! remote-access endpoint facts when the remote-access daemon has published them.
//! A discovering CLI browses the same service type and hands each resolved instance
//! to its discovery ladder.
//!
//! # Who uses it
//!
//! `cerulion-netd` and the fallback gateway process of `cerulion graph run` both
//! advertise through [`advertise_gateway`], which returns a guard that withdraws the
//! service on drop. This is an internal building block, published because both
//! depend on it; two copies of the TXT vocabulary would drift, so it lives here
//! once.
//!
//! # What is not here
//!
//! The browse half (`browse_rung`, `resolve_robot_eid`, the record-to-peer mapping)
//! stays in `cerulion_cli_engine::mdns_discovery`: netd never browses, and those
//! functions return `cerulion_discovery::DiscoveredPeer`. [`CERULION_SERVICE_TYPE`]
//! is defined once, here, and imported by the browser, so the one string that must
//! match on both sides cannot drift.
//!
//! # Failure posture
//!
//! An advertise failure is a loud warn at the caller, and the caller keeps serving:
//! a discovery beacon never crashes or fails a run. This crate returns a typed
//! [`MdnsError`] and logs nothing at the failure sites for that reason: the CLI maps
//! it onto a `CliError` it demotes to a warn, and netd warns directly.
//!
//! Design notes for contributors live in `docs/internals/network-daemons.md` in the
//! repository (the egress plane and mDNS beacon section).

// Maintainer notes (plain comments, not rendered). Why this is a crate: an earlier
// change put the advertise in the standalone `cerulion graph run-gateway` child.
// The permissive gateway plane then moved into the one per-computer
// `cerulion-netd`, so on a robot serving topics through netd there was no
// `run-gateway` child at all and the beacon went silent: a robot actively serving
// 86 topics on the same subnet as the desk answered NOTHING to
// `dns-sd -B _cerulion._tcp local.` The `run-gateway` path survives as the fallback
// when netd is unreachable, so both processes must advertise, and
// `cerulion_cli_engine` already depends on `cerulion_netd`, so the beacon cannot
// live in the CLI engine without a cyclic package edge. The browse side also
// normalizes endpoint ids through the CLI's own `connect_cmd::normalize_eid` seam.

// The project logging rule: library code never prints — it logs through
// `tracing`. Scoped `not(test)` so unit tests keep printing diagnostics, and
// applied at the crate root rather than in `[workspace.lints]` because that
// table cannot distinguish a lib target from a test binary. Pinned by
// `cerulion_cli_engine/tests/library_print_ban_test.rs`.
#![cfg_attr(not(test), deny(clippy::print_stdout, clippy::print_stderr))]

use std::path::PathBuf;

use mdns_sd::{ServiceDaemon, ServiceInfo};
use serde::Deserialize;

/// The mDNS service type Cerulion robots advertise + browse. The trailing
/// `.local.` is required by mDNS (`ServiceInfo`/`browse` both reject a
/// non-`.local.` domain).
pub const CERULION_SERVICE_TYPE: &str = "_cerulion._tcp.local.";

/// The product token stamped into the beacon's `agent=` TXT key (identifies the
/// advertising software; gateway-self-known).
const AGENT_TOKEN: &str = "cerulion";

/// Why an advertise could not be registered. Both callers demote this to a loud
/// warn — the beacon is an additive overlay, never load-bearing — so it carries
/// the robot + the underlying failure rather than any remediation the caller
/// would have to re-word for its own verb.
#[derive(Debug, thiserror::Error)]
pub enum MdnsError {
    /// Existing endpoint facts could not be read during a refresh.
    #[error("mDNS: cannot read robot endpoint facts at {}: {source}", path.display())]
    FactsRead {
        /// The shared public facts path.
        path: PathBuf,
        /// The filesystem failure.
        source: std::io::Error,
    },
    /// The `ServiceInfo` could not be built (an mdns-sd validation refusal — a
    /// hostile instance label, an over-length TXT value that survived the
    /// per-key validation, …).
    #[error("mDNS: cannot build the `_cerulion._tcp` ServiceInfo for robot '{robot}': {source}")]
    BuildServiceInfo {
        /// The robot identity the beacon was being built for.
        robot: String,
        /// The mdns-sd refusal.
        source: mdns_sd::Error,
    },
    /// The mDNS `ServiceDaemon` could not be started (no multicast socket, a
    /// sandbox denying `SO_REUSEPORT`, …).
    #[error("mDNS: cannot start the ServiceDaemon to advertise `_cerulion._tcp`: {source}")]
    DaemonStart {
        /// The mdns-sd refusal.
        source: mdns_sd::Error,
    },
    /// The service could not be registered on a daemon that DID start.
    #[error(
        "mDNS: cannot register the `_cerulion._tcp` advertisement for robot '{robot}': \
         {source}"
    )]
    Register {
        /// The robot identity the beacon was being registered for.
        robot: String,
        /// The mdns-sd refusal.
        source: mdns_sd::Error,
    },
}

// ── Beacon-facts — the remoted→gateway public seam ─────────
//
// `cerulion_remoted` owns the ONE iroh endpoint and publishes a small PUBLIC
// JSON facts file (`<state-root>/remoted/beacon_facts.json`) carrying the three
// facts only IT can know — the endpoint's public key (`eid`), its bound UDP port
// (`iroh_port`), and whether the robot is still claimable. An advertiser reads it
// at advertise time to enrich the mDNS TXT record. This crate CANNOT link
// `cerulion_remoted` (it pulls iroh — ~390 crates), so the state-root convention
// below MIRRORS remoted's (`config.rs`); keep the on-disk shape in lockstep (both
// sides pin it with an identical literal-JSON oracle). A missing / malformed /
// partial file NEVER fails or delays the beacon — the facts-derived keys are
// simply omitted.

/// The state-root override env var (mirrors remoted's `--state-root` clap env).
#[cfg(test)]
const STATE_ROOT_ENV: &str = "CERULION_STATE_ROOT";
/// remoted's namespaced subdir under the state root (mirrors `config.rs`).
const REMOTED_STATE_SUBDIR: &str = "remoted";
/// The public beacon-facts filename (mirrors `config.rs::BEACON_FACTS_FILENAME`).
const BEACON_FACTS_FILENAME: &str = "beacon_facts.json";

/// The beacon-facts an advertising gateway reads from remoted's public facts
/// file — the keys only remoted can know. Each is INDEPENDENTLY optional so a
/// partial / forward-compat file contributes only the keys it carries.
#[derive(Debug, Clone, Default, PartialEq, Eq)]
pub struct BeaconTxtFacts {
    /// The endpoint's hex public key (the `eid=` TXT value).
    pub eid: Option<String>,
    /// The endpoint's bound UDP port (the `iroh_port=` TXT value).
    pub iroh_port: Option<u16>,
    /// The claimable flag string (`"0"`/`"1"`, the `claimable=` TXT value).
    pub claimable: Option<String>,
}

/// The serde view of remoted's public beacon-facts file. Every field is
/// `#[serde(default)]`-optional so a partial file (missing keys) parses (the
/// present keys survive). The `version` frame remoted writes is IGNORED (serde
/// drops unknown fields) — the facts are additive, so an unknown version is
/// never a hard failure.
#[derive(Debug, Deserialize)]
struct BeaconFactsOnDisk {
    #[serde(default)]
    eid: Option<String>,
    #[serde(default)]
    iroh_port: Option<u16>,
    #[serde(default)]
    claimable: Option<String>,
}

/// The path remoted publishes its beacon facts to:
/// `<state-root>/remoted/beacon_facts.json`. Automatic startup and this reader
/// share the per-user root resolver; an explicit deployment override wins.
fn beacon_facts_path() -> Option<PathBuf> {
    Some(
        cerulion_discovery::robot_state::resolve()?
            .join(REMOTED_STATE_SUBDIR)
            .join(BEACON_FACTS_FILENAME),
    )
}

/// Load remoted's PUBLIC beacon facts, if present. An ABSENT file → `None`
/// (silent — the common case: no remoted running, or no facts yet). A
/// present-but-UNREADABLE file (permission denied / I/O error) → `None` + a
/// `warn!`: the file EXISTS but can't be read is an operator-fixable
/// misconfiguration, not the benign no-file case, and mirrors remoted's loud
/// write-side failure. A malformed file's own `debug!` fires in
/// [`parse_beacon_facts`]. Either way the enrichment is additive and NEVER fails
/// or delays the beacon (once per advertise, so a `warn!` cannot flood).
fn load_beacon_facts() -> Option<BeaconTxtFacts> {
    match read_beacon_facts() {
        Ok(facts) => facts,
        Err(error) => {
            tracing::warn!(error = %error, "robot endpoint facts unavailable; LAN advertisement omits their TXT fields");
            None
        }
    }
}

// Refresh callers receive I/O failures so their bounded loop can stop and log
// once. The initial advertisement reports the error while keeping LAN available.
fn read_beacon_facts() -> Result<Option<BeaconTxtFacts>, MdnsError> {
    let Some(path) = beacon_facts_path() else {
        return Ok(None);
    };
    match std::fs::read_to_string(&path) {
        Ok(text) => Ok(parse_beacon_facts(&text)),
        Err(error) if error.kind() == std::io::ErrorKind::NotFound => Ok(None),
        Err(source) => Err(MdnsError::FactsRead { path, source }),
    }
}

/// Parse remoted's beacon-facts JSON. PURE. Malformed JSON → `None` + a
/// `debug!` breadcrumb (the beacon still builds); a partial document yields a
/// [`BeaconTxtFacts`] carrying only the keys it has.
fn parse_beacon_facts(json_text: &str) -> Option<BeaconTxtFacts> {
    match serde_json::from_str::<BeaconFactsOnDisk>(json_text) {
        Ok(od) => Some(BeaconTxtFacts {
            eid: od.eid,
            iroh_port: od.iroh_port,
            claimable: od.claimable,
        }),
        Err(e) => {
            tracing::debug!(
                error = %e,
                "beacon facts JSON malformed; mDNS TXT enrichment (eid/iroh_port/claimable) omitted"
            );
            None
        }
    }
}

/// Assemble the gateway's mDNS TXT properties. PURE — oracle-tested.
///
/// ALWAYS present (gateway-self-known): `robot` (the UNSANITIZED identity),
/// `arch` / `os` ([`std::env::consts`]), `ver` (this build's version), `agent`
/// ([`AGENT_TOKEN`]). Present ONLY when remoted published them:
/// `eid` / `iroh_port` / `claimable` — each contributed independently, so a
/// partial facts file yields only its present keys.
///
/// Each facts-derived value is FORMAT-VALIDATED before it is pushed, and a
/// present-but-malformed value is DROPPED (that key omitted) — never advertised
/// verbatim: `eid` must be exactly 64 lowercase-hex chars (remoted writes
/// `hex::encode`), `claimable` must be exactly `"0"`/`"1"`, `iroh_port` must be
/// nonzero. This is a HARD invariant, not cosmetics: `mdns-sd`'s `ServiceInfo::new`
/// REJECTS a TXT property whose `key.len()+value.len()+1 > 255`, so an
/// over-length `eid`/`claimable` from a corrupt / tampered / buggy-writer facts
/// file (the file is PUBLIC + unauthenticated) would otherwise fail the ENTIRE
/// `_cerulion._tcp` registration and drop even the base `robot=`/SRV presence.
/// Validating to the canonical short forms bounds every value well under 255, so
/// a bad facts file degrades to OMITTED enrichment keys, never a dead beacon.
///
/// **The `ver` key is THIS crate's version**, which is the workspace version, so
/// it is the same string the earlier CLI-side beacon stamped.
fn gateway_txt_properties(robot: &str, facts: Option<&BeaconTxtFacts>) -> Vec<(String, String)> {
    let mut props: Vec<(String, String)> = vec![
        ("robot".to_string(), robot.to_string()),
        ("arch".to_string(), std::env::consts::ARCH.to_string()),
        ("os".to_string(), std::env::consts::OS.to_string()),
        ("ver".to_string(), env!("CARGO_PKG_VERSION").to_string()),
        ("agent".to_string(), AGENT_TOKEN.to_string()),
    ];
    if let Some(f) = facts {
        match f.eid.as_deref() {
            Some(eid) if is_valid_eid(eid) => props.push(("eid".to_string(), eid.to_string())),
            Some(eid) if !eid.is_empty() => tracing::debug!(
                len = eid.len(),
                "beacon facts `eid` is not 64 lowercase-hex chars; omitting the eid TXT key (facts file corrupt/tampered) — the beacon still advertises"
            ),
            _ => {}
        }
        if let Some(port) = f.iroh_port.filter(|&p| p != 0) {
            props.push(("iroh_port".to_string(), port.to_string()));
        }
        match f.claimable.as_deref() {
            Some(c) if c == "0" || c == "1" => props.push(("claimable".to_string(), c.to_string())),
            Some(c) if !c.is_empty() => tracing::debug!(
                value = %c,
                "beacon facts `claimable` is not \"0\"/\"1\"; omitting the claimable TXT key (facts file corrupt/tampered) — the beacon still advertises"
            ),
            _ => {}
        }
    }
    props
}

/// Whether a facts-file `eid` is a canonical `cerulion_link::EndpointId` hex:
/// EXACTLY 64 lowercase-hex chars (what remoted's `hex::encode(endpoint.id())`
/// writes). Anything else (wrong length — including a hostile over-length string
/// — non-hex, or uppercase) is dropped from the TXT record. Bounding the length
/// here is what keeps a corrupt facts file from failing `ServiceInfo::new`'s
/// 255-byte cap.
///
/// This is FORMAT-only, not an ed25519 canonicality check (that
/// `EndpointId::from_bytes` decode). That is DELIBERATE and sufficient here:
///   1. this crate cannot link `cerulion_link`/iroh (it would pull ~390 crates
///      into the plain build), so `EndpointId::from_bytes` is unreachable
///      reader-side;
///   2. remoted WRITES a real `endpoint.id()`, which is always a canonical key,
///      so a non-canonical eid can only appear via a corrupt / tampered facts
///      file (the acknowledged public/unauthenticated surface); and
///   3. such a tampered non-canonical eid is HARMLESS — it advertises, then the
///      dialer's `cerulion_link::parse_endpoint_id` rejects it (or dials a key no
///      peer holds, which TLS-fails), never a wrong-peer connection.
pub fn is_valid_eid(eid: &str) -> bool {
    eid.len() == 64
        && eid
            .bytes()
            .all(|b| b.is_ascii_digit() || (b'a'..=b'f').contains(&b))
}

/// Sanitize a robot identity into an mDNS instance / hostname label: `/` and
/// `.` (the two chars that break single-label DNS names — a `.` would split
/// the fullname, a `/` is illegal) become `-`.
pub fn sanitize_instance(robot: &str) -> String {
    robot.replace(['/', '.'], "-")
}

/// The SRV port a gateway advertises, derived from its zenoh LISTEN endpoints.
///
/// Returns the port of the FIRST `tcp/` endpoint (the substring after its LAST
/// `:`, so the IPv6 bracket form `tcp/[::]:7683` and the `tcp/0.0.0.0:7683` /
/// `tcp/host:7683` forms all parse), or `None` when no endpoint is `tcp/` or the
/// port is unparseable. PURE — oracle-tested.
///
/// `None` is a REFUSAL TO ADVERTISE, not a default: the beacon's whole payload is
/// "dial me at this port", so a gateway with no dialable tcp port has nothing to
/// say and both callers skip the beacon LOUDLY rather than publishing a port
/// nothing is bound to. On the netd-hosted plane this is also the desk/robot
/// discriminator — a desk netd sets no `CERULION_NETD_LISTEN`, so it has no
/// listen endpoint and never advertises itself as a robot.
pub fn srv_port_from_listen_endpoints(listen_endpoints: &[String]) -> Option<u16> {
    let ep = listen_endpoints.iter().find(|e| e.starts_with("tcp/"))?;
    ep.rsplit(':').next()?.parse::<u16>().ok()
}

/// Build the advertise-side [`ServiceInfo`] for this robot's gateway:
/// service type [`CERULION_SERVICE_TYPE`], instance name = the sanitized
/// robot identity, hostname = `<sanitized-robot>.local.`, the SRV `port` (the
/// bound ZENOH port — unchanged; iroh's UDP port is a TXT key, not the SRV
/// port), and the TXT properties from [`gateway_txt_properties`]. Reads
/// remoted's PUBLIC beacon facts for the `eid`/`iroh_port`/`claimable`
/// TXT enrichment; an absent/malformed file simply omits THOSE keys. The
/// gateway-self-stamped `robot`/`arch`/`os`/`ver`/`agent` keys are ALWAYS
/// present (so the no-facts TXT is five keys, not the `robot=`-only form), so
/// this is backward-COMPATIBLE — an older browser reads `robot=` and
/// ignores the extra keys — NOT byte-identical to the old beacon. Address
/// auto-detection is enabled so the daemon fills the A/AAAA records — the `""`
/// ip is intentional.
#[cfg(test)]
fn build_gateway_service_info(robot: &str, port: u16) -> Result<ServiceInfo, MdnsError> {
    let facts = load_beacon_facts();
    build_gateway_service_info_with_facts(robot, port, facts.as_ref())
}

/// The PURE inner builder — takes the already-loaded beacon facts (so the TXT
/// assembly is oracle-testable without touching the filesystem / env).
fn build_gateway_service_info_with_facts(
    robot: &str,
    port: u16,
    facts: Option<&BeaconTxtFacts>,
) -> Result<ServiceInfo, MdnsError> {
    let instance = sanitize_instance(robot);
    let host_name = format!("{instance}.local.");
    let properties = gateway_txt_properties(robot, facts);
    let prop_refs: Vec<(&str, &str)> = properties
        .iter()
        .map(|(k, v)| (k.as_str(), v.as_str()))
        .collect();
    let info = ServiceInfo::new(
        CERULION_SERVICE_TYPE,
        &instance,
        &host_name,
        "",
        port,
        &prop_refs[..],
    )
    .map_err(|e| MdnsError::BuildServiceInfo {
        robot: robot.to_string(),
        source: e,
    })?
    .enable_addr_auto();
    Ok(info)
}

/// Advertise this robot's gateway over mDNS (`_cerulion._tcp`), returning a
/// guard that keeps the service live for the gateway's whole lifetime.
/// Dropping the guard unregisters + shuts the daemon down, so the robot
/// disappears from mDNS when the gateway exits.
///
/// `port` is the ACTUAL bound zenoh listen port (the SRV record); `robot`
/// is the resolved robot identity — the hostname or `CERULION_ROBOT_IDENTITY`
/// override, NOT the graph prefix (the TXT `robot=` value). A daemon-start /
/// registration failure is a loud `Err` the caller demotes to a warn — the
/// advertisement is an additive overlay, never load-bearing.
pub fn advertise_gateway(robot: &str, port: u16) -> Result<MdnsAdvertiseGuard, MdnsError> {
    // Build the (pure) ServiceInfo BEFORE starting the daemon: `ServiceDaemon`
    // has no `Drop` shutdown, so any early return between `new()` and the
    // guard construction would leak its background
    // thread. Info-first leaves `register` as the only fallible step after
    // the daemon exists, and that path shuts it down explicitly below.
    let facts = load_beacon_facts();
    let info = build_gateway_service_info_with_facts(robot, port, facts.as_ref())?;
    let fullname = info.get_fullname().to_string();
    let daemon = ServiceDaemon::new().map_err(|e| MdnsError::DaemonStart { source: e })?;
    if let Err(e) = daemon.register(info) {
        // Shut the just-started daemon down so a register failure leaks no
        // background thread (the guard — the normal shutdown owner — is never
        // constructed on this path).
        let _ = daemon.shutdown();
        return Err(MdnsError::Register {
            robot: robot.to_string(),
            source: e,
        });
    }
    tracing::info!(
        robot = %robot,
        port,
        fullname = %fullname,
        "advertising the gateway over mDNS (_cerulion._tcp)"
    );
    Ok(MdnsAdvertiseGuard {
        daemon,
        fullname,
        robot: robot.into(),
        port,
        facts,
    })
}

/// Live mDNS advertisement handle. Held for the gateway's lifetime; its
/// [`Drop`] unregisters the service + shuts the daemon down, so the robot
/// disappears from `_cerulion._tcp` the moment the gateway tears down.
pub struct MdnsAdvertiseGuard {
    daemon: ServiceDaemon,
    /// The registered service fullname (`<instance>._cerulion._tcp.local.`),
    /// the key `unregister` takes.
    fullname: String,
    robot: String,
    port: u16,
    facts: Option<BeaconTxtFacts>,
}

impl MdnsAdvertiseGuard {
    /// Update the existing registration after this robot's server writes its facts.
    /// Partial, stale-identity, and unchanged facts do not replace the record.
    pub fn refresh_robot_facts(&mut self, expected_eid: &str) -> Result<bool, MdnsError> {
        let Some(facts) = read_beacon_facts()? else {
            return Ok(false);
        };
        if !should_refresh_facts(self.facts.as_ref(), &facts, expected_eid) {
            return Ok(false);
        }
        let info = build_gateway_service_info_with_facts(&self.robot, self.port, Some(&facts))?;
        self.daemon
            .register(info)
            .map_err(|source| MdnsError::Register {
                robot: self.robot.clone(),
                source,
            })?;
        self.facts = Some(facts);
        Ok(true)
    }

    /// The registered service fullname (`<instance>._cerulion._tcp.local.`) —
    /// the Principle-#3 observable a caller reports or a test asserts on.
    pub fn fullname(&self) -> &str {
        &self.fullname
    }
}

fn should_refresh_facts(
    current: Option<&BeaconTxtFacts>,
    incoming: &BeaconTxtFacts,
    expected_eid: &str,
) -> bool {
    is_valid_eid(expected_eid)
        && incoming.eid.as_deref() == Some(expected_eid)
        && incoming.iroh_port.is_some_and(|port| port != 0)
        && incoming.claimable.as_deref() == Some("0")
        && current != Some(incoming)
}

impl Drop for MdnsAdvertiseGuard {
    fn drop(&mut self) {
        // Both steps are best-effort: a teardown-race failure (daemon already
        // gone) is a debug breadcrumb, never a warn — the robot is departing
        // anyway. `unregister` multicasts the mDNS goodbye packet; `shutdown`
        // stops the background thread.
        if let Err(e) = self.daemon.unregister(&self.fullname) {
            tracing::debug!(
                error = %e,
                fullname = %self.fullname,
                "mDNS unregister failed on gateway teardown (best-effort)"
            );
        }
        if let Err(e) = self.daemon.shutdown() {
            tracing::debug!(
                error = %e,
                "mDNS daemon shutdown failed on gateway teardown (best-effort)"
            );
        }
    }
}

#[cfg(test)]
mod tests {
    #[test]
    fn late_robot_facts_refresh_only_the_expected_complete_changed_identity() {
        let mut facts = full_facts();
        facts.claimable = Some("0".into());
        let eid = facts.eid.clone().unwrap();
        assert!(should_refresh_facts(None, &facts, &eid));
        assert!(!should_refresh_facts(Some(&facts), &facts, &eid));
        assert!(!should_refresh_facts(None, &facts, &"44".repeat(32)));
        for change in 0..3 {
            let mut incomplete = facts.clone();
            match change {
                0 => incomplete.iroh_port = Some(0),
                1 => incomplete.eid = None,
                _ => incomplete.claimable = Some("1".into()),
            }
            assert!(!should_refresh_facts(None, &incomplete, &eid));
        }
        let mut restarted = facts.clone();
        restarted.iroh_port = Some(55555);
        assert!(should_refresh_facts(Some(&facts), &restarted, &eid));
        let info = build_gateway_service_info_with_facts("robot", 7447, Some(&restarted)).unwrap();
        assert_eq!(info.get_port(), 7447);
        assert_eq!(info.get_property_val_str("iroh_port"), Some("55555"));
        assert_eq!(info.get_property_val_str("claimable"), Some("0"));
    }

    use super::*;

    #[test]
    fn sanitize_instance_replaces_slash_and_dot() {
        // Hand oracle: BOTH offending chars → '-'; a clean label is untouched.
        assert_eq!(
            sanitize_instance("orin/perception.node"),
            "orin-perception-node"
        );
        assert_eq!(sanitize_instance("plainrobot"), "plainrobot");
        assert_eq!(sanitize_instance("a.b/c"), "a-b-c");
    }

    /// The SRV-port derivation, moved here from the CLI's
    /// `graph_cmd::parse_listen_port` so the netd-hosted plane and the
    /// `run-gateway` fallback derive the advertised port from ONE rule.
    #[test]
    fn srv_port_reads_the_first_tcp_endpoint() {
        // IPv6 bracket form: the port is the substring after the LAST colon.
        assert_eq!(
            srv_port_from_listen_endpoints(&["tcp/[::]:7683".to_string()]),
            Some(7683)
        );
        // IPv4 wildcard + explicit host forms.
        assert_eq!(
            srv_port_from_listen_endpoints(&["tcp/0.0.0.0:7683".to_string()]),
            Some(7683)
        );
        assert_eq!(
            srv_port_from_listen_endpoints(&["tcp/192.168.1.5:7001".to_string()]),
            Some(7001)
        );
        // The FIRST tcp/ endpoint wins, even when a non-tcp one precedes it.
        assert_eq!(
            srv_port_from_listen_endpoints(&[
                "udp/0.0.0.0:1".to_string(),
                "tcp/0.0.0.0:7683".to_string(),
                "tcp/0.0.0.0:9999".to_string(),
            ]),
            Some(7683)
        );
        // NO advertise: empty, no tcp/ endpoint, an unparseable port, a missing
        // port, and a port past u16 all yield None (the caller skips loudly).
        assert_eq!(srv_port_from_listen_endpoints(&[]), None);
        assert_eq!(
            srv_port_from_listen_endpoints(&["udp/0.0.0.0:7683".to_string()]),
            None
        );
        assert_eq!(
            srv_port_from_listen_endpoints(&["tcp/0.0.0.0:notaport".to_string()]),
            None
        );
        assert_eq!(
            srv_port_from_listen_endpoints(&["tcp/0.0.0.0".to_string()]),
            None
        );
        assert_eq!(
            srv_port_from_listen_endpoints(&["tcp/0.0.0.0:70000".to_string()]),
            None
        );
    }

    #[test]
    fn build_gateway_service_info_pins_fullname_port_and_txt() {
        // Oracle pins on the advertise-side ServiceInfo — sanitized instance
        // in the fullname, the SRV port, and the UNSANITIZED robot in TXT. The
        // BACK-COMPAT pin: the fullname, the SRV port, and the `robot=` value
        // are byte-unchanged from the older beacon (the arch/os/ver/agent
        // + facts keys are purely ADDITIVE — an older browser reads
        // `robot=` and ignores the rest), so this is backward-compatible, NOT a
        // byte-identical TXT record.
        let info = build_gateway_service_info("orin/cam.1", 7683).expect("builds");
        assert_eq!(info.get_fullname(), "orin-cam-1._cerulion._tcp.local.");
        assert_eq!(info.get_port(), 7683);
        assert_eq!(info.get_property_val_str("robot"), Some("orin/cam.1"));
        assert_eq!(info.get_type(), CERULION_SERVICE_TYPE);
    }

    // ── TXT enrichment ──────────────────────────────────────

    /// Look up a TXT key in a `gateway_txt_properties` result.
    fn txt<'a>(props: &'a [(String, String)], key: &str) -> Option<&'a str> {
        props
            .iter()
            .find(|(k, _)| k == key)
            .map(|(_, v)| v.as_str())
    }

    /// A full, VALID facts document (the shape remoted writes).
    fn full_facts() -> BeaconTxtFacts {
        BeaconTxtFacts {
            eid: Some(
                "00112233445566778899aabbccddeeff00112233445566778899aabbccddeeff".to_string(),
            ),
            iroh_port: Some(41234),
            claimable: Some("1".to_string()),
        }
    }

    /// Every gateway-self-known key must be present with its exact value.
    fn assert_self_keys(props: &[(String, String)], robot: &str) {
        assert_eq!(txt(props, "robot"), Some(robot));
        assert_eq!(txt(props, "arch"), Some(std::env::consts::ARCH));
        assert_eq!(txt(props, "os"), Some(std::env::consts::OS));
        assert_eq!(txt(props, "ver"), Some(env!("CARGO_PKG_VERSION")));
        // The LITERAL, not just the constant: `agent=` is WIRE SURFACE that a
        // browsing desk reads, so changing `AGENT_TOKEN` must fail a test rather
        // than silently re-brand every beacon on the LAN. Comparing only against
        // the constant is a self-compare and cannot see that.
        assert_eq!(txt(props, "agent"), Some(AGENT_TOKEN));
        assert_eq!(
            txt(props, "agent"),
            Some("cerulion"),
            "the `agent=` TXT value is wire surface — changing AGENT_TOKEN is a \
             deliberate wire change, not a rename"
        );
    }

    #[test]
    fn gateway_txt_properties_full_facts_stamps_every_key() {
        let facts = full_facts();
        let props = gateway_txt_properties("go2", Some(&facts));
        assert_self_keys(&props, "go2");
        assert_eq!(txt(&props, "eid"), facts.eid.as_deref());
        assert_eq!(txt(&props, "iroh_port"), Some("41234"));
        assert_eq!(txt(&props, "claimable"), Some("1"));
        assert_eq!(props.len(), 8, "5 self keys + 3 facts keys");
    }

    #[test]
    fn gateway_txt_properties_no_facts_omits_facts_keys_but_stamps_self_keys() {
        let props = gateway_txt_properties("go2", None);
        assert_self_keys(&props, "go2");
        assert_eq!(txt(&props, "eid"), None);
        assert_eq!(txt(&props, "iroh_port"), None);
        assert_eq!(txt(&props, "claimable"), None);
        assert_eq!(props.len(), 5, "exactly the 5 gateway-self-known keys");
    }

    #[test]
    fn gateway_txt_properties_partial_facts_emits_present_keys_only() {
        // Only `iroh_port` present → only that facts key appears.
        let facts = BeaconTxtFacts {
            eid: None,
            iroh_port: Some(7),
            claimable: None,
        };
        let props = gateway_txt_properties("go2", Some(&facts));
        assert_self_keys(&props, "go2");
        assert_eq!(txt(&props, "iroh_port"), Some("7"));
        assert_eq!(txt(&props, "eid"), None);
        assert_eq!(txt(&props, "claimable"), None);
        assert_eq!(props.len(), 6);
    }

    #[test]
    fn gateway_txt_properties_empty_string_eid_is_treated_as_absent() {
        let facts = BeaconTxtFacts {
            eid: Some(String::new()),
            iroh_port: None,
            claimable: Some(String::new()),
        };
        let props = gateway_txt_properties("go2", Some(&facts));
        assert_eq!(txt(&props, "eid"), None, "an empty eid is not advertised");
        assert_eq!(
            txt(&props, "claimable"),
            None,
            "an empty claimable is not advertised"
        );
        assert_eq!(props.len(), 5);
    }

    #[test]
    fn gateway_txt_properties_zero_iroh_port_is_treated_as_absent() {
        let facts = BeaconTxtFacts {
            eid: None,
            iroh_port: Some(0),
            claimable: None,
        };
        let props = gateway_txt_properties("go2", Some(&facts));
        assert_eq!(
            txt(&props, "iroh_port"),
            None,
            "a zero iroh_port is not dialable — omitted"
        );
        assert_eq!(props.len(), 5);
    }

    #[test]
    fn is_valid_eid_accepts_only_64_lowercase_hex() {
        let ok = "00112233445566778899aabbccddeeff00112233445566778899aabbccddeeff";
        assert!(is_valid_eid(ok));
        assert!(!is_valid_eid(""), "empty");
        assert!(!is_valid_eid(&ok[..63]), "too short");
        assert!(!is_valid_eid(&format!("{ok}0")), "too long");
        assert!(!is_valid_eid(&ok.to_uppercase()), "uppercase hex");
        assert!(
            !is_valid_eid(&format!("{}g", &ok[..63])),
            "non-hex character"
        );
    }

    #[test]
    fn gateway_txt_properties_drops_malformed_eid_and_claimable() {
        // A HOSTILE facts file: an over-length eid (would blow the 255-byte TXT
        // cap and fail the WHOLE registration) and a garbage claimable.
        let facts = BeaconTxtFacts {
            eid: Some("a".repeat(4096)),
            iroh_port: Some(41234),
            claimable: Some("yes-please".to_string()),
        };
        let props = gateway_txt_properties("go2", Some(&facts));
        assert_self_keys(&props, "go2");
        assert_eq!(txt(&props, "eid"), None, "an over-length eid is DROPPED");
        assert_eq!(
            txt(&props, "claimable"),
            None,
            "a non-0/1 claimable is DROPPED"
        );
        assert_eq!(
            txt(&props, "iroh_port"),
            Some("41234"),
            "the VALID sibling key still rides"
        );
        assert_eq!(props.len(), 6);

        // The NEAR-MISS shapes, restored from the earlier CLI-side test the
        // crate move dropped. These are the ones a lenient validator lets
        // through, and they are the ones that matter: a CONTROL CHARACTER in a
        // TXT value is what a truncated / line-buffered / hand-edited facts file
        // produces, and an UPPERCASE eid is what a writer using `hex::encode`'s
        // uppercase sibling produces. Both must be DROPPED, not advertised
        // verbatim — `"1\n"` is not `"1"`, and the beacon must never emit a raw
        // newline into a TXT record a browser will parse.
        let near_miss = BeaconTxtFacts {
            eid: Some("AB".repeat(32)), // 64 chars, hex, but UPPERCASE
            iroh_port: None,
            claimable: Some("1\n".to_string()), // "0"/"1" plus a control char
        };
        let props = gateway_txt_properties("go2", Some(&near_miss));
        assert_self_keys(&props, "go2");
        assert_eq!(
            txt(&props, "eid"),
            None,
            "an UPPERCASE (non-canonical) eid is DROPPED"
        );
        assert_eq!(
            txt(&props, "claimable"),
            None,
            "a claimable carrying a control character is DROPPED — it is not \"1\""
        );
        assert_eq!(props.len(), 5, "only the 5 self-known keys survive");
    }

    #[test]
    fn build_service_info_with_hostile_facts_still_builds_the_beacon() {
        // The behavioral half of the drop rule: a hostile facts file must not
        // fail ServiceInfo::new (which would kill the whole beacon).
        let facts = BeaconTxtFacts {
            eid: Some("z".repeat(1000)),
            iroh_port: Some(41234),
            claimable: Some("maybe".to_string()),
        };
        let info = build_gateway_service_info_with_facts("go2", 7683, Some(&facts))
            .expect("a hostile facts file must NOT fail the registration");
        assert_eq!(info.get_property_val_str("robot"), Some("go2"));
        assert_eq!(info.get_property_val_str("eid"), None);
        assert_eq!(info.get_property_val_str("claimable"), None);
        assert_eq!(info.get_property_val_str("iroh_port"), Some("41234"));
    }

    #[test]
    fn parse_beacon_facts_full_valid_document() {
        // The EXACT literal remoted writes (version frame + the three facts).
        let json = r#"{
            "version": 1,
            "eid": "00112233445566778899aabbccddeeff00112233445566778899aabbccddeeff",
            "iroh_port": 41234,
            "claimable": "1"
        }"#;
        let facts = parse_beacon_facts(json).expect("parses");
        assert_eq!(facts, full_facts());
    }

    #[test]
    fn parse_beacon_facts_malformed_is_none() {
        assert_eq!(parse_beacon_facts("not json"), None);
        assert_eq!(parse_beacon_facts(""), None);
        // A well-formed JSON of the WRONG shape (iroh_port as a string) is also
        // None — the whole document fails, the beacon still builds.
        assert_eq!(parse_beacon_facts(r#"{"iroh_port":"41234"}"#), None);
    }

    #[test]
    fn parse_beacon_facts_partial_document_keeps_present_keys() {
        let facts = parse_beacon_facts(r#"{"version":1,"claimable":"0"}"#).expect("parses");
        assert_eq!(
            facts,
            BeaconTxtFacts {
                eid: None,
                iroh_port: None,
                claimable: Some("0".to_string()),
            }
        );
    }

    #[test]
    fn build_service_info_with_full_facts_carries_every_txt_key() {
        let facts = full_facts();
        let info =
            build_gateway_service_info_with_facts("go2", 7683, Some(&facts)).expect("builds");
        assert_eq!(info.get_fullname(), "go2._cerulion._tcp.local.");
        assert_eq!(info.get_port(), 7683);
        assert_eq!(info.get_property_val_str("robot"), Some("go2"));
        assert_eq!(info.get_property_val_str("eid"), facts.eid.as_deref());
        assert_eq!(info.get_property_val_str("iroh_port"), Some("41234"));
        assert_eq!(info.get_property_val_str("claimable"), Some("1"));
        assert_eq!(info.get_property_val_str("agent"), Some(AGENT_TOKEN));
    }

    /// The crate-wide env mutex. `set_var`/`remove_var` race on the process
    /// environ block whatever the KEY is, so every env-mutating test in this
    /// crate takes this one lock (a per-test
    /// `#[serial]` is an INDEPENDENT mutex and does not serialize against a
    /// sibling module's env writes).
    fn env_lock() -> std::sync::MutexGuard<'static, ()> {
        static LOCK: std::sync::Mutex<()> = std::sync::Mutex::new(());
        LOCK.lock().unwrap_or_else(|e| e.into_inner())
    }

    /// RAII: restore `CERULION_STATE_ROOT` to its pre-test value on drop.
    struct StateRootGuard(Option<std::ffi::OsString>, Option<std::ffi::OsString>);
    impl StateRootGuard {
        fn set(value: &std::path::Path) -> Self {
            let prev = std::env::var_os(STATE_ROOT_ENV);
            std::env::set_var(STATE_ROOT_ENV, value);
            StateRootGuard(prev, std::env::var_os("CERULION_HOME"))
        }
    }
    impl StateRootGuard {
        fn config_only(value: &std::path::Path) -> Self {
            let guard = Self(
                std::env::var_os(STATE_ROOT_ENV),
                std::env::var_os("CERULION_HOME"),
            );
            std::env::remove_var(STATE_ROOT_ENV);
            std::env::set_var("CERULION_HOME", value);
            guard
        }
    }
    impl Drop for StateRootGuard {
        fn drop(&mut self) {
            match &self.0 {
                Some(v) => std::env::set_var(STATE_ROOT_ENV, v),
                None => std::env::remove_var(STATE_ROOT_ENV),
            }
            match &self.1 {
                Some(v) => std::env::set_var("CERULION_HOME", v),
                None => std::env::remove_var("CERULION_HOME"),
            }
        }
    }

    #[test]
    fn load_beacon_facts_reads_the_state_root_path_and_enriches_the_service_info() {
        let _env_lk = env_lock();
        // The I/O wiring pin: with CERULION_STATE_ROOT pointing at a tempdir
        // holding `remoted/beacon_facts.json`, the reader resolves the right
        // path and `build_gateway_service_info` enriches the TXT record.
        let dir = tempfile::tempdir().unwrap();
        let remoted = dir.path().join("remoted");
        std::fs::create_dir_all(&remoted).unwrap();
        std::fs::write(
            remoted.join("beacon_facts.json"),
            br#"{"version":1,"eid":"11223344556677889900aabbccddeeff11223344556677889900aabbccddeeff","iroh_port":50505,"claimable":"0"}"#,
        )
        .unwrap();

        let _guard = StateRootGuard::set(dir.path());

        // The reader resolves the path + parses the file.
        let facts = load_beacon_facts().expect("facts loaded from the state root");
        assert_eq!(facts.iroh_port, Some(50505));
        assert_eq!(facts.claimable.as_deref(), Some("0"));

        // And the production entry point enriches the ServiceInfo end-to-end.
        let info = build_gateway_service_info("go2", 7683).expect("builds");
        assert_eq!(info.get_property_val_str("robot"), Some("go2"));
        assert_eq!(
            info.get_property_val_str("eid"),
            Some("11223344556677889900aabbccddeeff11223344556677889900aabbccddeeff")
        );
        assert_eq!(info.get_property_val_str("iroh_port"), Some("50505"));
        assert_eq!(info.get_property_val_str("claimable"), Some("0"));
    }

    #[test]
    fn refresh_read_errors_are_returned_for_one_shot_reporting() {
        let _env_lk = env_lock();
        let dir = tempfile::tempdir().unwrap();
        let _guard = StateRootGuard::set(dir.path());
        let path = dir.path().join("remoted/beacon_facts.json");
        std::fs::create_dir_all(&path).unwrap();
        assert!(
            matches!(read_beacon_facts(), Err(MdnsError::FactsRead { path: failed, .. }) if failed == path)
        );
    }

    #[test]
    fn automatic_state_root_uses_the_login_config_home() {
        let _env_lk = env_lock();
        let dir = tempfile::tempdir().unwrap();
        let _guard = StateRootGuard::config_only(dir.path());
        assert_eq!(
            beacon_facts_path(),
            Some(dir.path().join("robot-state/remoted/beacon_facts.json"))
        );
    }

    #[test]
    fn load_beacon_facts_absent_file_is_none() {
        let _env_lk = env_lock();
        // A state root with NO facts file → None (the common no-remoted case).
        let dir = tempfile::tempdir().unwrap();
        let _guard = StateRootGuard::set(dir.path());
        assert_eq!(load_beacon_facts(), None);
    }
}
