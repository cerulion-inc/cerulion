// SPDX-License-Identifier: AGPL-3.0-only
//! The Go2 DDS participant wrapper — a ONE-per-process `ros2-client`/`rustdds`
//! `DomainParticipant`, plus the `only_networks` config knob that is
//! REQUIRED for CycloneDDS interop on a multi-homed host.
//!
//! # One participant per process (Principle #8 analog)
//!
//! Cerulion runs ONE iceoryx2 node per process; the DDS side mirrors that with
//! ONE `DomainParticipant`. A second [`Go2Participant::new`] while one is live
//! returns [`DdsError::ParticipantAlreadyExists`] — never a silent second
//! participant (which would duplicate discovery traffic and double the SPDP/SEDP
//! locator bloat the crate docs warn about). The guard is a process-global
//! `AtomicBool` released on `Drop`, so a process may build -> drop -> rebuild
//! (at most one LIVE at a time, not one-ever).
//!
//! # `only_networks` is REQUIRED on a multi-homed host
//!
//! CycloneDDS (ALL versions) DROPS fragmented builtin
//! (SPDP/SEDP) discovery data ("DATAFRAG ... fragmented builtin data not yet
//! supported"), and there is no receiver-side fix. rustdds fragments a builtin
//! sample once it exceeds ~1.4 KB — which a many-interface host (docker/veth/
//! VPN) reaches because rustdds advertises EVERY local address as a unicast
//! locator. Restricting to the robot-LAN interface via
//! `DomainParticipantBuilder::with_only_networks` shrinks the discovery payload
//! back under the fragmentation threshold. So [`ParticipantConfig::only_networks`]
//! is the load-bearing knob: leave it unset ONLY on a single-interface host.
//! The constraint is restated, with the operator remedy, in
//! `graphs/go2.bridge.yaml`'s header ("WHICH UNIT IS THIS FOR").

use std::net::IpAddr;
use std::sync::atomic::{AtomicBool, Ordering};
use std::time::Duration as StdDuration;

use ros2_client::ros2::{policy, Duration as DdsDuration, QosPolicies, QosPolicyBuilder};
use ros2_client::{Context, Node, NodeName, NodeOptions};
use thiserror::Error;

/// The ROS 2 node namespace all Go2-bridge nodes are created under.
pub const NODE_NAMESPACE: &str = "/cerulion_go2";

/// The environment variable consulted (after the config value) for the
/// `only_networks` restriction — a comma/space-separated list of local IPv4/
/// IPv6 addresses, e.g. `GO2_IFACE=192.168.123.5`.
pub const GO2_IFACE_ENV: &str = "GO2_IFACE";

/// The Go2's default DDS domain id (`ROS_DOMAIN_ID`).
pub const GO2_DEFAULT_DOMAIN: u16 = 0;

/// The default SPDP participant lease duration Cerulion Go2 participants
/// advertise. Short by design: if a bridge process dies WITHOUT
/// sending a DDS dispose (SIGKILL, power-cut, panic — anything no signal handler
/// can catch), the robot's writers age our orphaned readers out roughly this
/// long after our last SPDP announce, instead of the stock rustdds ~50 s
/// (`5 * SPDP_PUBLISH_PERIOD`). For that 50 s window the robot's writers keep
/// serving ghost readers. 10 s keeps a comfortable margin
/// above the FLOOR: the rustdds discovery event loop is single-threaded and the
/// announce period derives as `lease / 5` (2 s here), so a much shorter lease
/// would risk FALSE lease-timeout evictions of a healthy-but-busy peer.
pub const DEFAULT_LEASE_DURATION: StdDuration = StdDuration::from_secs(10);

// ---------------------------------------------------------------------------
// Errors
// ---------------------------------------------------------------------------

/// A DDS participant / node construction failure.
///
/// The underlying `ros2-client`/`rustdds` error types differ across versions
/// and are not `Clone`/`PartialEq`, so their causes are captured as `Debug`
/// text in a `cause` field carried by Display — NOT as a `#[source]` chain
/// (a field literally named `source` is an implicit `#[source]` to thiserror,
/// which demands `std::error::Error` on it; `String` isn't one). The wrapper
/// never needs to match on a cause, only surface it loudly; keeping strings
/// preserves `Clone + PartialEq + Eq` for oracle tests.
#[derive(Debug, Clone, PartialEq, Eq, Error)]
pub enum DdsError {
    /// A `Go2Participant` already exists in this process (Principle #8 analog).
    #[error("a Go2 DDS participant already exists in this process (Principle #8: ONE participant per process) — drop the existing Go2Participant before creating another")]
    ParticipantAlreadyExists,
    /// `DomainParticipantBuilder::build` failed (this is EVERY
    /// path — the empty-`only_networks` case also builds via the builder so
    /// the lease can be set; `only_networks` is empty in that case).
    #[error("failed to build DDS DomainParticipant (domain {domain_id}, only_networks {only_networks:?}): {cause}")]
    ParticipantBuild {
        domain_id: u16,
        only_networks: Vec<IpAddr>,
        cause: String,
    },
    /// `Context::from_domain_participant` failed.
    #[error("failed to create DDS Context (domain {domain_id}): {cause}")]
    ContextBuild { domain_id: u16, cause: String },
    /// An invalid ROS node name.
    #[error("invalid DDS node name {name:?}: {cause}")]
    NodeName { name: String, cause: String },
    /// `Context::new_node` failed.
    #[error("failed to create DDS node {name:?}: {cause}")]
    NodeCreate { name: String, cause: String },
}

// ---------------------------------------------------------------------------
// Config
// ---------------------------------------------------------------------------

/// Configuration for a [`Go2Participant`]. [`Default`] is domain
/// [`GO2_DEFAULT_DOMAIN`] (0, the Go2 default) with no interface restriction
/// (the loud-warn path — see [`Self::resolve_only_networks`]) and the
/// [`DEFAULT_LEASE_DURATION`] (10 s) SPDP lease.
#[derive(Debug, Clone)]
pub struct ParticipantConfig {
    /// The DDS domain id — must match the peer's `ROS_DOMAIN_ID`
    /// ([`GO2_DEFAULT_DOMAIN`] = 0 for the Go2).
    pub domain_id: u16,
    /// Restrict rustdds to these local interface IPs (multicast joins AND the
    /// unicast locators advertised in SPDP/SEDP). REQUIRED on a multi-homed
    /// host — see the module docs. Empty => fall back to the [`GO2_IFACE_ENV`]
    /// env var, else advertise every interface (loud warn).
    pub only_networks: Vec<IpAddr>,
    /// The SPDP participant lease duration advertised to the robot's DDS
    /// writers. Defaults to [`DEFAULT_LEASE_DURATION`] (10 s) — a
    /// SHORT lease so a SIGKILLed/crashed bridge's orphaned readers age out on
    /// the robot in seconds, not the stock ~50 s. See [`DEFAULT_LEASE_DURATION`]
    /// for the floor rationale.
    pub lease_duration: StdDuration,
}

impl Default for ParticipantConfig {
    fn default() -> Self {
        Self {
            domain_id: GO2_DEFAULT_DOMAIN,
            only_networks: Vec::new(),
            lease_duration: DEFAULT_LEASE_DURATION,
        }
    }
}

impl ParticipantConfig {
    /// A config on the given domain with an explicit interface restriction and
    /// the [`DEFAULT_LEASE_DURATION`] (10 s) SPDP lease. Override the lease with
    /// [`Self::with_lease_duration`].
    pub fn new(domain_id: u16, only_networks: Vec<IpAddr>) -> Self {
        Self {
            domain_id,
            only_networks,
            lease_duration: DEFAULT_LEASE_DURATION,
        }
    }

    /// Override the SPDP participant lease duration (builder-through). Threads to
    /// [`ros2_client::rustdds::DomainParticipantBuilder::participant_lease_duration`]
    /// at [`Go2Participant::new`]. Keep it at least a few seconds — see
    /// [`DEFAULT_LEASE_DURATION`].
    pub fn with_lease_duration(mut self, lease_duration: StdDuration) -> Self {
        self.lease_duration = lease_duration;
        self
    }

    /// Resolve the effective `only_networks` with precedence
    /// **config value → `GO2_IFACE` env → none**. Emits a loud `warn!` when the
    /// result is empty (a multi-homed host WILL silently fail discovery — see
    /// the module docs), and an `info!` naming the source when it is non-empty.
    pub fn resolve_only_networks(&self) -> Vec<IpAddr> {
        if !self.only_networks.is_empty() {
            tracing::info!(only_networks = ?self.only_networks, "cerulion_go2_dds: only_networks from config");
            return self.only_networks.clone();
        }
        if let Ok(raw) = std::env::var(GO2_IFACE_ENV) {
            let parsed = parse_iface_list(&raw);
            if !parsed.is_empty() {
                tracing::info!(only_networks = ?parsed, env = %GO2_IFACE_ENV, "cerulion_go2_dds: only_networks from env");
                return parsed;
            }
        }
        tracing::warn!(
            env = %GO2_IFACE_ENV,
            "cerulion_go2_dds: no only_networks set (config empty and GO2_IFACE unset/unparseable) — \
             rustdds will advertise EVERY local interface as a DDS locator. On a multi-homed host \
             (docker/veth/VPN) that bloats SPDP/SEDP past ~1.4 KB, CycloneDDS DROPS the fragmented builtin \
             data (DATAFRAG), and discovery SILENTLY fails. Set ParticipantConfig.only_networks or \
             GO2_IFACE=<robot-LAN-IP> (see the header of graphs/go2.bridge.yaml)."
        );
        Vec::new()
    }
}

/// Parse a comma/whitespace-separated list of IP addresses (the `GO2_IFACE`
/// env format). Unparseable tokens are skipped with a `warn!`; the result may
/// be empty. Pure (no env access) so it is directly oracle-testable.
pub fn parse_iface_list(raw: &str) -> Vec<IpAddr> {
    raw.split([',', ' ', '\t'])
        .map(str::trim)
        .filter(|tok| !tok.is_empty())
        .filter_map(|tok| match tok.parse::<IpAddr>() {
            Ok(ip) => Some(ip),
            Err(_) => {
                tracing::warn!(token = %tok, "cerulion_go2_dds: ignoring unparseable GO2_IFACE entry");
                None
            }
        })
        .collect()
}

// ---------------------------------------------------------------------------
// One-per-process guard
// ---------------------------------------------------------------------------

/// Process-global "a participant is live" flag. `AtomicBool` (not `OnceLock`/
/// `Once`) DELIBERATELY, mirroring `cerulion_viz`'s viz-statics guard: a `Once` is
/// unresettable, but this slot must be RELEASABLE — on `Drop` (build -> drop ->
/// rebuild) and via the test seam ([`reset_participant_slot_for_test`]) for a
/// panic-leaked claim.
static PARTICIPANT_CLAIMED: AtomicBool = AtomicBool::new(false);

/// RAII claim on the single-participant slot; releases on `Drop`.
#[derive(Debug)]
struct ParticipantClaim;

fn claim_participant_slot() -> Result<ParticipantClaim, DdsError> {
    if PARTICIPANT_CLAIMED.swap(true, Ordering::SeqCst) {
        Err(DdsError::ParticipantAlreadyExists)
    } else {
        Ok(ParticipantClaim)
    }
}

impl Drop for ParticipantClaim {
    fn drop(&mut self) {
        PARTICIPANT_CLAIMED.store(false, Ordering::SeqCst);
    }
}

/// Test-only force-release of the process-global participant slot. Mirrors
/// `cerulion_viz::tf::rearm_viz_statics`: `Drop` is the PRIMARY release,
/// but a panicked constructor (or a downstream integration test that stranded a
/// claim) can leave the slot set; this clears it. `#[doc(hidden)]` + the
/// `_for_test` name keep it from reading as production API — do not call it on
/// a live path.
#[doc(hidden)]
pub fn reset_participant_slot_for_test() {
    PARTICIPANT_CLAIMED.store(false, Ordering::SeqCst);
}

// ---------------------------------------------------------------------------
// Participant wrapper
// ---------------------------------------------------------------------------

/// A single-per-process Go2 DDS participant. Wraps the `ros2-client` [`Context`]
/// (the `rustdds` `DomainParticipant`) and hands out [`Node`]s for the DDS
/// ingress/egress node crates.
pub struct Go2Participant {
    context: Context,
    domain_id: u16,
    // Released on Drop -> frees the process slot. Held only for its Drop; the
    // leading underscore exempts it from the dead-code "never read" lint.
    _claim: ParticipantClaim,
}

impl Go2Participant {
    /// Create the process participant. Errors with
    /// [`DdsError::ParticipantAlreadyExists`] if one is already live. Wires the
    /// `only_networks` restriction via `DomainParticipantBuilder`
    /// when the resolved list is non-empty; otherwise builds
    /// a plain domain context.
    pub fn new(config: &ParticipantConfig) -> Result<Self, DdsError> {
        // Claim FIRST so two concurrent constructors cannot both build. On any
        // error `?`-return below, `claim` drops and frees the slot (RAII).
        let claim = claim_participant_slot()?;
        let only = config.resolve_only_networks();

        // Build via `DomainParticipantBuilder` in BOTH the restricted and
        // unrestricted cases (rather than taking the `Context::with_options`
        // shortcut for the empty case) so the SPDP participant lease is set on
        // EVERY path — stock `ContextOptions` has no lease knob. The empty case
        // is exactly what `Context::with_options` does internally
        // (`DomainParticipantBuilder::new(domain).build()` -> `from_domain_participant`),
        // just with `.participant_lease_duration(..)` added.
        let mut builder = ros2_client::rustdds::DomainParticipantBuilder::new(config.domain_id)
            .participant_lease_duration(config.lease_duration);
        if !only.is_empty() {
            builder = builder.with_only_networks(only.iter().copied());
        }
        let participant = builder.build().map_err(|e| DdsError::ParticipantBuild {
            domain_id: config.domain_id,
            only_networks: only.clone(),
            cause: format!("{e:?}"),
        })?;
        let context =
            Context::from_domain_participant(participant).map_err(|e| DdsError::ContextBuild {
                domain_id: config.domain_id,
                cause: format!("{e:?}"),
            })?;

        tracing::info!(
            domain = config.domain_id,
            only_networks = ?only,
            lease_secs = config.lease_duration.as_secs_f64(),
            "cerulion_go2_dds: DDS participant created"
        );
        Ok(Self {
            context,
            domain_id: config.domain_id,
            _claim: claim,
        })
    }

    /// The DDS domain id this participant runs on.
    pub fn domain_id(&self) -> u16 {
        self.domain_id
    }

    /// Borrow the underlying `ros2-client` [`Context`] for advanced use (e.g.
    /// discovery-status streams). Node creation should go through
    /// [`Go2Participant::create_node`].
    pub fn context(&self) -> &Context {
        &self.context
    }

    /// Create a ROS 2 [`Node`] named `<node_name>` under [`NODE_NAMESPACE`],
    /// with rosout logging disabled. rosout-off keeps the
    /// bridge off the `/rosout` topic; the ROS `rt/` DDS-topic prefix is applied
    /// by ros2-client when the node creates topics.
    pub fn create_node(&self, node_name: &str) -> Result<Node, DdsError> {
        let name = NodeName::new(NODE_NAMESPACE, node_name).map_err(|e| DdsError::NodeName {
            name: node_name.to_string(),
            cause: format!("{e:?}"),
        })?;
        self.context
            .new_node(name, NodeOptions::new().enable_rosout(false))
            .map_err(|e| DdsError::NodeCreate {
                name: node_name.to_string(),
                cause: format!("{e:?}"),
            })
    }
}

// ---------------------------------------------------------------------------
// QoS helpers (needed by every pub/sub in the node crates)
// ---------------------------------------------------------------------------

/// ROS 2 default-ish QoS: RELIABLE + VOLATILE, KeepLast(10). Matches
/// ros2-client's own talker/listener example. Use for
/// commands + state topics.
pub fn reliable_volatile_qos() -> QosPolicies {
    QosPolicyBuilder::new()
        .history(policy::History::KeepLast { depth: 10 })
        .reliability(policy::Reliability::Reliable {
            max_blocking_time: DdsDuration::from_millis(100),
        })
        .durability(policy::Durability::Volatile)
        .build()
}

/// BEST_EFFORT + VOLATILE, KeepLast(10) — for lidar-style high-rate publishers
/// (the Go2 point cloud). A best-effort SUB still matches a reliable PUB, but a
/// reliable SUB will NOT match a best-effort PUB (QoS compatibility).
pub fn best_effort_qos() -> QosPolicies {
    QosPolicyBuilder::new()
        .history(policy::History::KeepLast { depth: 10 })
        .reliability(policy::Reliability::BestEffort)
        .durability(policy::Durability::Volatile)
        .build()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn parse_iface_list_parses_and_skips_garbage() {
        // Comma + space separated, mixed v4/v6, trims, skips empties + garbage.
        let ips = parse_iface_list("192.168.123.5, 10.0.0.1  ::1 ,,not-an-ip");
        assert_eq!(
            ips,
            vec![
                "192.168.123.5".parse::<IpAddr>().unwrap(),
                "10.0.0.1".parse().unwrap(),
                "::1".parse().unwrap(),
            ]
        );
        assert!(parse_iface_list("").is_empty());
        assert!(parse_iface_list("  , ,\t").is_empty());
    }

    #[test]
    fn config_precedence_prefers_explicit_over_env() {
        // A non-empty config value is used verbatim (no env consulted).
        let cfg = ParticipantConfig::new(0, vec!["10.1.2.3".parse().unwrap()]);
        assert_eq!(
            cfg.resolve_only_networks(),
            vec!["10.1.2.3".parse::<IpAddr>().unwrap()]
        );
        // An empty config with no env resolves to empty (the warn path).
        // (We do NOT set GO2_IFACE here to keep the test env-free/parallel-safe;
        // the env branch is exercised behaviorally by the live-DDS e2e tests.)
        let empty = ParticipantConfig::default();
        assert!(empty.only_networks.is_empty());
    }

    /// The SPDP lease knob defaults to 10 s and is overridable. This is
    /// the CI-safe pin (pure value oracle — no real participant, which needs
    /// a multicast-capable interface); the value actually reaching the wire is pinned by the fork's
    /// SPDP serialize-seam test and the `#[ignore]`d two-process ageout test. `Go2Participant::new`
    /// threads `config.lease_duration` into
    /// `DomainParticipantBuilder::participant_lease_duration`.
    #[test]
    fn lease_duration_defaults_to_ten_seconds_and_is_overridable() {
        // The module const is the 10 s SHORT lease (not the stock ~50 s).
        assert_eq!(DEFAULT_LEASE_DURATION, StdDuration::from_secs(10));
        // Every construction path carries that default...
        assert_eq!(
            ParticipantConfig::default().lease_duration,
            DEFAULT_LEASE_DURATION
        );
        assert_eq!(
            ParticipantConfig::new(0, vec![]).lease_duration,
            DEFAULT_LEASE_DURATION
        );
        // ...and the builder-through override replaces it verbatim.
        let custom = StdDuration::from_secs(6);
        let cfg = ParticipantConfig::new(0, vec![]).with_lease_duration(custom);
        assert_eq!(cfg.lease_duration, custom);
        // Overriding does not disturb the other fields.
        assert_eq!(cfg.domain_id, 0);
        assert!(cfg.only_networks.is_empty());
    }

    /// The one-per-process guard lifecycle, folded into ONE test body so it is
    /// the SOLE toucher of the process-global `PARTICIPANT_CLAIMED` — hence
    /// parallel-safe WITHOUT `#[serial]` (no other test claims the slot; the
    /// real-participant construction that would claim it needs a live DDS
    /// peer). Exercises: claim -> second claim rejected -> Drop releases ->
    /// re-claim -> reset seam clears a leaked claim.
    #[test]
    fn single_participant_slot_guard_lifecycle() {
        reset_participant_slot_for_test(); // clean start regardless of prior state
        let first = claim_participant_slot().expect("first claim");
        assert_eq!(
            claim_participant_slot().unwrap_err(),
            DdsError::ParticipantAlreadyExists
        );
        drop(first);
        // Slot freed on Drop -> a fresh claim now succeeds.
        let second = claim_participant_slot().expect("claim after drop");
        // The test seam force-releases even a still-held claim (models a
        // panic-leaked slot). After it, a new claim succeeds while `second`
        // still exists — proving the seam actually clears the flag.
        reset_participant_slot_for_test();
        let third = claim_participant_slot().expect("claim after reset seam");
        drop(second);
        drop(third);
        // Leave the slot clean for any other test/process.
        reset_participant_slot_for_test();
    }
}
