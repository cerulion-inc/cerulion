// SPDX-License-Identifier: AGPL-3.0-only
//! DDS discovery for Cerulion: the crate behind `cerulion ros2 attach`, and the only
//! main-workspace crate that depends on the DDS stack (`ros2-client` and `rustdds`).
//!
//! `cerulion ros2 attach` finds the topics of a ROS 2 stack that is already running
//! and bridges them into a Cerulion graph; this crate is the part that looks at the
//! DDS network. A user reaches it through that verb, not by depending on it.
//!
//! It provides two things:
//!
//! 1. **The DDS-free discovery vocabulary**: [`DiscoveredEndpoint`],
//!    [`DiscoveredQos`], [`DiscoveryParams`], [`DiscoveryResult`], the
//!    [`DdsDiscovery`] trait, and [`DdsError`]. These carry no `rustdds` types, so
//!    the `cerulion ros2 attach` command logic (in `cerulion_cli_engine::ros_cmd`) is
//!    pure over them and fully testable without a live DDS peer.
//!
//! 2. **The live backend**: [`LiveDiscovery`], a [`DdsDiscovery`] impl that builds a
//!    one-per-process participant ([`participant`]) with the `with_only_networks`
//!    multi-homed-discovery fix, then over a bounded window ([`discovery`]) harvests
//!    the `ros_discovery_info` node table and the participant vendors from the SPDP
//!    and SEDP event stream and, at window end, snapshots the participant's internal
//!    discovery database for the endpoints (each carrying its SEDP `USER_DATA`, where
//!    the REP-2011 RIHS01 type hash rides). Upstream rustdds 0.14.2 provides both the
//!    snapshot accessors and `USER_DATA` parsing; the published `cerulion-rustdds`
//!    fork is retained only for its participant lease duration knob.
//!
//! The live backend is behind the `live` feature. `jazzy` (the default) selects it
//! with the 16-byte GID of ROS 2 Iron and newer; `humble` selects it with the
//! 24-byte GID of older distributions.
//!
//! The dependency edge runs from `cerulion_cli_engine` to `cerulion_dds`, never the
//! reverse, so this crate stays a leaf: the trait and error live here (not in the
//! engine) to keep that edge acyclic, and the engine maps [`DdsError`] into its own
//! `CliError`.
//!
//! Design notes for contributors live in `docs/internals/network-daemons.md` in the
//! repository (the DDS wire rung section).

// P12 (the project logging rule): library code never prints — it logs through
// `tracing`. Scoped `not(test)` so unit tests keep printing diagnostics, and
// applied at the crate root rather than in `[workspace.lints]` because that
// table cannot distinguish a lib target from a test binary. Pinned by
// `cerulion_cli_engine/tests/library_print_ban_test.rs`.
#![cfg_attr(not(test), deny(clippy::print_stdout, clippy::print_stderr))]

use std::collections::BTreeMap;
use std::net::IpAddr;
use std::time::Duration;

use thiserror::Error;

// The live ros2-client/rustdds backend is behind the `live` feature so a
// `default-features = false` dependant (cerulion_cli_engine) gets ONLY the
// DDS-free vocabulary below — no mio/net2/rustdds in its build.
#[cfg(feature = "live")]
pub mod discovery;
#[cfg(feature = "live")]
pub mod participant;

// The wire rung's PURE half (USER_DATA parse, endpoint→node
// join, vendor→mapping decision, service-response → `.msg` closure mapping).
// ALWAYS compiled — no `ros2-client`/`rustdds` types — so the humble/no-live
// builds stay green and the helpers are oracle-testable with hand-built byte
// vectors. The live wire rung (`wire_acquirer`) is a thin DDS adapter over it.
pub mod wire;

// The live `~/get_type_description` service rung. Behind
// `live` (it speaks ros2-client services); constructed by the binary and
// prepended to the acquisition ladder. Designed against the REP-2011 spec + the
// ros2-client 0.10 source and validated against a live ROS 2 Jazzy peer. The pure
// decision engine it delegates to lives in `wire` (fully oracle-tested).
#[cfg(feature = "live")]
pub mod wire_acquirer;

#[cfg(feature = "live")]
pub use discovery::LiveDiscovery;
#[cfg(feature = "live")]
pub use wire_acquirer::WireServiceAcquirer;

// ─────────────────────────── Discovery vocabulary ──────────────────────────

/// Offered/requested DDS reliability, summarized for the DDS-free pure half.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum QosReliability {
    Reliable,
    BestEffort,
    /// The endpoint advertised no reliability policy.
    Unknown,
}

impl QosReliability {
    /// The lowercase wire word used in the discovery report.
    pub fn as_str(self) -> &'static str {
        match self {
            Self::Reliable => "reliable",
            Self::BestEffort => "best_effort",
            Self::Unknown => "unknown",
        }
    }
}

/// Summarized DDS durability.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum QosDurability {
    Volatile,
    TransientLocal,
    Transient,
    Persistent,
    /// The endpoint advertised no durability policy.
    Unknown,
}

impl QosDurability {
    /// The lowercase wire word used in the discovery report.
    pub fn as_str(self) -> &'static str {
        match self {
            Self::Volatile => "volatile",
            Self::TransientLocal => "transient_local",
            Self::Transient => "transient",
            Self::Persistent => "persistent",
            Self::Unknown => "unknown",
        }
    }
}

/// A summarized QoS profile for one discovered endpoint.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct DiscoveredQos {
    pub reliability: QosReliability,
    pub durability: QosDurability,
}

impl DiscoveredQos {
    /// `<reliability>/<durability>` — the report's QoS cell.
    pub fn summary(&self) -> String {
        format!("{}/{}", self.reliability.as_str(), self.durability.as_str())
    }
}

/// Whether a discovered endpoint is a DDS Writer (publisher) or Reader
/// (subscriber). Writers drive the bridge (their offered QoS is what a bridge
/// subscriber must match).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum EndpointKind {
    Writer,
    Reader,
}

/// One endpoint seen during DDS discovery — the DDS-free unit the backend
/// hands the pure half. `dds_topic`/`type_name` are the RAW discovery strings
/// (e.g. `rt/utlidar/cloud`, `sensor_msgs::msg::dds_::PointCloud2_`);
/// normalization happens in the engine.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct DiscoveredEndpoint {
    pub dds_topic: String,
    pub type_name: String,
    pub qos: DiscoveredQos,
    pub kind: EndpointKind,
    /// The wire rung: the RIHS01 type hash harvested from this
    /// endpoint's SEDP `USER_DATA` (`typehash=RIHS01_<64hex>;`), or `None` when
    /// the publisher advertised none — a pre-Iron distro or a non-ROS DDS peer
    /// (the Go2's raw CycloneDDS). This is the REQUIRED input to
    /// `~/get_type_description` (the server looks up the type BY HASH; the type
    /// name is ignored server-side), so its absence is what makes the wire rung
    /// skip. DDS-free — a plain `String`. Since a later revision the `USER_DATA` comes from the
    /// window-end DiscoveryDB snapshot (provided by rustdds 0.14.2), parsed at
    /// the DDS boundary via [`wire::parse_type_hash_from_user_data`].
    pub type_hash: Option<String>,
    /// This endpoint's own 16-byte DDS GUID (the canonical
    /// RTPS GUID — 12-byte participant prefix + 4-byte entity id). Used to map
    /// a writer to its owning ROS node via `ros_discovery_info`'s
    /// `writer_gid_seq`, and (via its first 12 bytes) to the participant's
    /// vendor id for the service correlation-mapping pick. `None` if the
    /// backend could not surface it. DDS-free — `[u8; 16]`.
    pub writer_guid: Option<[u8; 16]>,
}

/// Parameters handed to the discovery backend.
#[derive(Debug, Clone)]
pub struct DiscoveryParams {
    /// Local interface IPs to restrict rustdds to (`with_only_networks`).
    /// REQUIRED on a multi-homed host — the `--iface` value lands here.
    pub only_networks: Vec<IpAddr>,
    /// DDS domain id (`--domain`, default 0).
    pub domain_id: u16,
    /// How long to collect discovery events before returning.
    pub window: Duration,
}

/// One ROS 2 node seen in `ros_discovery_info`
/// (`rmw_dds_common::ParticipantEntitiesInfo` → `NodeEntitiesInfo`), reduced to
/// the DDS-free fields the wire rung needs to map a discovered publisher
/// endpoint to its owning node's `~/get_type_description` service.
///
/// The join key is `participant_prefix` — the 12-byte GUID prefix of the DDS
/// participant that hosts this node, matched against a discovered endpoint's
/// `writer_guid[..12]`. (Per-node DataWriter GUIDs would be exacter, but
/// `ros2-client` 0.10's `NodeEntitiesInfo` keeps `writer_gid_seq` PRIVATE with
/// no getter, so the finest granularity the public API exposes is the
/// participant. This matches the ROS graph regardless: a node's endpoints all
/// carry its participant's prefix, and any node in the participant that holds
/// the type can answer its `get_type_description` — exactly the retry set r727
/// prescribes.) No `rustdds` types — the prefix is raw `[u8; 12]`, names are
/// `String`.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct DiscoveredNode {
    /// The node's namespace (`/` or `/robot`).
    pub namespace: String,
    /// The node's base name (`talker`).
    pub name: String,
    /// The 12-byte GUID prefix of the DDS participant hosting this node — the
    /// join key against an endpoint's `writer_guid[..12]`.
    pub participant_prefix: [u8; 12],
}

impl DiscoveredNode {
    /// The node's fully-qualified name (`/talker`, `/robot/talker`) — the
    /// namespace under which its `~/get_type_description` service is served.
    /// Mirrors ROS 2 name joining: a root namespace (`/`) yields `/<name>`; a
    /// nested namespace yields `<namespace>/<name>`.
    pub fn fully_qualified_name(&self) -> String {
        wire::join_node_fqn(&self.namespace, &self.name)
    }
}

/// One remote DDS participant's vendor identity, keyed by its
/// 12-byte GUID prefix (shared by all endpoints it owns). Used to pick the ROS 2
/// service request/reply correlation mapping — Fast DDS ⇒ Enhanced, CycloneDDS
/// ⇒ Cyclone (see [`wire::mapping_order_for_vendor`]). DDS-free.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct DiscoveredParticipant {
    /// The participant's 12-byte GUID prefix (the first 12 bytes of every one
    /// of its endpoints' 16-byte GUIDs).
    pub guid_prefix: [u8; 12],
    /// The RTPS vendor id (`[0x01, 0x0F]` eProsima Fast DDS, `[0x01, 0x10]`
    /// Eclipse Cyclone DDS, `[0x01, 0x12]` RustDDS, …).
    pub vendor_id: [u8; 2],
}

/// The outcome of one discovery run (live P0 fix 2: the attach participant's
/// OWN endpoints — its parameter-service readers/writers etc. — are filtered
/// out STRUCTURALLY by GUID prefix, and counted here so the report can say so
/// instead of silently dropping them).
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct DiscoveryResult {
    /// Every FOREIGN endpoint seen during the window.
    pub endpoints: Vec<DiscoveredEndpoint>,
    /// Endpoints belonging to the attach participant itself, hidden from
    /// `endpoints` (matched by GUID prefix — never a name heuristic).
    pub own_endpoints_hidden: usize,
    /// The ROS graph's node table harvested from
    /// `ros_discovery_info` (empty on a non-ROS DDS network, or when nothing
    /// published it during the window). CONSUMED by the wire rung: this whole
    /// `DiscoveryResult` is threaded into [`SchemaAcquirer::acquire`], and the
    /// wire rung joins an endpoint's `writer_guid` to its owning node here to
    /// locate the node's `~/get_type_description` service — it runs NO second
    /// discovery window of its own.
    pub nodes: Vec<DiscoveredNode>,
    /// Discovered remote participants' vendor ids, for the
    /// service correlation-mapping pick. Empty when SPDP surfaced none. Also
    /// CONSUMED by the wire rung via the threaded `DiscoveryResult` (see
    /// [`SchemaAcquirer::acquire`]).
    pub participants: Vec<DiscoveredParticipant>,
}

impl DiscoveryResult {
    /// An empty discovery result (no endpoints, nodes, or participants) — the
    /// convenience constructor the acquirer seam's tests pass when a rung does
    /// NOT consult discovery (every rung but the wire rung ignores it).
    pub fn empty() -> Self {
        Self {
            endpoints: Vec::new(),
            own_endpoints_hidden: 0,
            nodes: Vec::new(),
            participants: Vec::new(),
        }
    }
}

/// The discovery backend seam. The `cerulion ros2 attach` command takes
/// `&dyn DdsDiscovery` so tests inject a hand-built endpoint list and
/// production injects [`LiveDiscovery`].
pub trait DdsDiscovery {
    /// Run DDS discovery for `params.window` and return every foreign
    /// endpoint seen (+ the own-endpoint hidden count). LOUD `Err` on
    /// participant/discovery failure — never a silently-empty list (an empty
    /// `Ok` means "discovery ran and saw nothing foreign").
    fn discover(&self, params: &DiscoveryParams) -> Result<DiscoveryResult, DdsError>;
}

/// A DDS discovery / participant failure. The underlying `ros2-client`/`rustdds`
/// error types differ across versions and are not `Clone`/`PartialEq`, so their
/// causes are captured as `Debug` text in a `cause` field (NOT a `#[source]`
/// chain) — mirroring `cerulion_go2_dds::participant::DdsError`.
#[derive(Debug, Clone, PartialEq, Eq, Error)]
pub enum DdsError {
    /// A discovery participant already exists in this process (Principle #8
    /// analog — ONE participant per process).
    #[error("a Cerulion DDS discovery participant already exists in this process (ONE participant per process) — drop the existing one before starting another")]
    ParticipantAlreadyExists,
    /// Building the `rustdds` `DomainParticipant` failed. Every
    /// participant is built via `DomainParticipantBuilder` (threading the SPDP
    /// `.participant_lease_duration(..)`), so this can arise on either the
    /// restricted (`with_only_networks`) or the unrestricted path.
    #[error("failed to build DDS DomainParticipant (domain {domain_id}, only_networks {only_networks:?}): {cause}")]
    ParticipantBuild {
        domain_id: u16,
        only_networks: Vec<IpAddr>,
        cause: String,
    },
    /// Building the ros2-client `Context` failed.
    #[error("failed to create DDS Context (domain {domain_id}): {cause}")]
    ContextBuild { domain_id: u16, cause: String },
    /// An invalid ROS node name.
    #[error("invalid DDS node name {name:?}: {cause}")]
    NodeName { name: String, cause: String },
    /// Creating the ROS node failed.
    #[error("failed to create DDS node {name:?}: {cause}")]
    NodeCreate { name: String, cause: String },
    /// Starting the node spinner (needed for discovery status events) failed.
    #[error("failed to start the DDS node spinner: {cause}")]
    Spinner { cause: String },
}

// ─────────────────────── Schema acquisition seam ──────────────────
//
// The second DDS-free seam this crate exports, mirroring [`DdsDiscovery`]: the
// vocabulary + trait live HERE (the leaf) so `cerulion ros2 attach`'s ladder
// driver — in `cerulion_cli_engine::ros_cmd` — stays pure over them and fully
// oracle-testable with a canned acquirer, while the live wire rung
// (`~/get_type_description`) implements the trait in THIS crate behind the
// `live` feature. No `rustdds`/`ros2-client` types appear
// below; the acquired `.msg` text is carried as verbatim strings.

/// Which acquisition-ladder rung produced a schema. Surfaced in
/// the attach report so the provenance of every materialized `.msg` is loud,
/// never inferred silently.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum AcquisitionRung {
    /// Rung 2 — the wire-native `~/get_type_description` service call, on the
    /// existing `ros2-client` stack (this crate's `live` feature).
    WireService,
    /// The LOCAL ament harvest — scan the host's own `$AMENT_PREFIX_PATH` ROS
    /// install (the `rosidl_interfaces` ament index + `share/<pkg>/msg/*.msg`)
    /// for the type's verbatim `.msg` and its full nested closure. Covers a
    /// robot when attach runs ON it, and any dev machine with a ROS 2 install —
    /// filesystem-only, no ssh, no network (LOCAL half; the ladder design's
    /// item 4 "local $AMENT_PREFIX_PATH scan").
    LocalAment,
    /// Rung 3 — scavenge a running `foxglove_bridge` / `rosbridge`.
    BridgeScavenge,
    /// Rung 4 — robot-local SSH / `ament_index` harvest.
    SshHarvest,
    /// Rung 5 — public lookup (`packages.ros.org` / GitHub), consent-gated
    /// LAST.
    PublicLookup,
}

impl AcquisitionRung {
    /// A short, stable, PAREN-FREE human label for the report line and
    /// structured logs. Paren-free by contract:
    /// every call site embeds the label inside a parenthesized context
    /// (`(pkg/Type, via <label>)`), so a label carrying its own parens would
    /// render nested parens on the highest-traffic acquisition lines.
    pub fn label(self) -> &'static str {
        match self {
            Self::WireService => "wire service via get_type_description",
            Self::LocalAment => "local ROS install via ament index",
            Self::BridgeScavenge => "bridge scavenge",
            Self::SshHarvest => "ssh harvest",
            Self::PublicLookup => "public lookup",
        }
    }
}

/// One `.msg` file in an acquired schema's nested closure: the verbatim
/// original text plus the package/type it defines. `package` + `type_name`
/// form the ament-mirror store path `schemas/<package>/msg/<type_name>.msg`
/// the engine materializes it to (inside the consent gate).
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct AcquiredMsg {
    /// The package the `.msg` belongs to (`sensor_msgs`).
    pub package: String,
    /// The bare type name (`PointCloud2`) — the `.msg` file stem.
    pub type_name: String,
    /// Verbatim `.msg` file contents (comments preserved), byte-identical to
    /// the robot's own definition where copied raw.
    pub msg_text: String,
}

impl AcquiredMsg {
    /// The qualified `package/type_name` this file defines (`sensor_msgs/Imu`).
    pub fn qualified_name(&self) -> String {
        format!("{}/{}", self.package, self.type_name)
    }
}

/// A successfully-acquired schema: the requested type's FULL nested closure
/// (the type itself PLUS every recursive dependency) as verbatim `.msg` files,
/// plus which rung produced it.
///
/// The engine parses these, verifies the closure is COMPLETE (every referenced
/// nested type is present here, in the workspace `.msg` store, or in the
/// built-in corpus), stages them in memory for the re-resolve pass, and
/// materializes them to disk ONLY inside the consent gate. An INCOMPLETE
/// closure is surfaced loudly and the type stays UNRESOLVABLE — never a silent
/// half-resolved claim.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct AcquiredSchema {
    /// Which rung produced this bundle.
    pub rung: AcquisitionRung,
    /// The requested type's `.msg` plus every nested dependency's `.msg`.
    pub closure: Vec<AcquiredMsg>,
}

/// One rung's failure/skip reason for a type that could NOT be acquired — the
/// loud, per-rung "why" the attach report surfaces (e.g. rung 2's "no type
/// hash in discovery (pre-Iron distro or non-ROS DDS publisher)").
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct RungSkip {
    pub rung: AcquisitionRung,
    pub reason: String,
}

/// The acquisition outcome for ONE requested type.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum AcquisitionOutcome {
    /// The type was acquired with its full nested closure.
    Acquired(AcquiredSchema),
    /// The type could not be acquired — every attempted rung's reason, in
    /// ladder order, for the loud report line.
    Skipped(Vec<RungSkip>),
}

/// One requested type paired with its acquisition outcome (the acquirer's
/// per-type result unit).
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct TypeAcquisition {
    /// The requested canonical `pkg/Type`.
    pub requested: String,
    pub outcome: AcquisitionOutcome,
}

/// The schema-acquisition seam, injectable exactly like
/// [`DdsDiscovery`]: `cerulion ros2 attach` runs it over the DEDUPED set of
/// UNRESOLVABLE ROS type names, stages any acquired schema in memory, then
/// re-resolves. Kept DDS-free so the engine's ladder-driver + tests stay
/// hermetic with canned acquirers; the live wire rung
/// (`~/get_type_description`) implements it in this crate behind the `live`
/// feature, and the binary wires it in.
pub trait SchemaAcquirer {
    /// Attempt to acquire a Cerulion-resolvable `.msg` (with its full nested
    /// closure) for each requested `pkg/Type`. `types` is DEDUPED by the
    /// caller — each name appears at most once, in a deterministic order.
    ///
    /// `discovery` is the SAME [`DiscoveryResult`] the attach flow already
    /// harvested (endpoints + RIHS01 hashes + the `ros_discovery_info` node
    /// table + participant vendors). It is threaded through so the wire rung
    /// CONSUMES it rather than opening a second discovery window:
    /// its skip decisions key off the SAME harvest the report prints.
    /// Every OTHER rung ignores it (the local ament rung reads the filesystem,
    /// the [`NoopAcquirer`] acquires nothing) — pass [`DiscoveryResult::empty`]
    /// when a caller has no discovery context.
    ///
    /// Return one [`TypeAcquisition`] per type ATTEMPTED. A type OMITTED from
    /// the result is treated as "not attempted / no reason recorded" — the
    /// [`NoopAcquirer`] returns an empty `Vec`, so `cerulion ros2 attach`
    /// behaves byte-identically to a no-acquirer run. Never fails as a whole:
    /// per-type failure rides [`AcquisitionOutcome::Skipped`].
    fn acquire(&self, types: &[String], discovery: &DiscoveryResult) -> Vec<TypeAcquisition>;
}

/// The default acquirer wired when no live rung is installed — acquires
/// NOTHING, so `cerulion ros2 attach` resolves types from its local
/// sources alone. The engine's legacy `ros_attach` entry point delegates here; the
/// `_with_acquirer` entry point takes a real [`SchemaAcquirer`].
#[derive(Debug, Default, Clone, Copy)]
pub struct NoopAcquirer;

impl SchemaAcquirer for NoopAcquirer {
    fn acquire(&self, _types: &[String], _discovery: &DiscoveryResult) -> Vec<TypeAcquisition> {
        Vec::new()
    }
}

/// An ordered composition of [`SchemaAcquirer`] rungs (the acquisition
/// ladder). The binary builds the ladder by chaining the concrete
/// rungs in priority order; `cerulion ros2 attach`'s driver sees ONE acquirer.
/// Kept HERE — beside the trait, in the DDS-free vocabulary — so the binary
/// just composes rungs and the engine's ladder-driver stays pure.
///
/// # Batch semantics
///
/// [`acquire`](SchemaAcquirer::acquire) runs each inner acquirer in turn over
/// the set of types NOT YET acquired by an earlier rung — a type an earlier
/// rung resolves is never handed to a later one (each rung gets ONE batch of
/// the still-unacquired types). Per-type [`RungSkip`] reasons ACCUMULATE across
/// rungs in ladder order (the `Vec<RungSkip>` vocabulary exists for exactly
/// this), so the attach report shows every rung that was tried and why it fell
/// through. A type an inner acquirer OMITS from its response is treated as "not
/// attempted" by that rung — it carries forward to the next rung with NO skip
/// reason recorded (identical to how the [`NoopAcquirer`] omits everything). A
/// type acquired by SOME rung wins over any earlier rung's skip; a type every
/// rung omitted is itself omitted from the result (byte-identical to a
/// no-acquirer run for that type). An EMPTY ladder is
/// [`NoopAcquirer`]-equivalent.
pub struct ChainedAcquirer<'a> {
    rungs: Vec<&'a dyn SchemaAcquirer>,
}

impl<'a> ChainedAcquirer<'a> {
    /// Build a ladder from inner acquirers in priority (first-tried) order. An
    /// EMPTY `rungs` acquires nothing — byte-identical to the [`NoopAcquirer`].
    pub fn new(rungs: Vec<&'a dyn SchemaAcquirer>) -> Self {
        Self { rungs }
    }
}

impl SchemaAcquirer for ChainedAcquirer<'_> {
    fn acquire(&self, types: &[String], discovery: &DiscoveryResult) -> Vec<TypeAcquisition> {
        // Types still needing acquisition, in the caller's deduped input order.
        let mut remaining: Vec<String> = types.to_vec();
        // Final acquisitions + accumulated skip reasons, keyed by type.
        let mut acquired: BTreeMap<String, AcquiredSchema> = BTreeMap::new();
        let mut skips: BTreeMap<String, Vec<RungSkip>> = BTreeMap::new();

        for rung in &self.rungs {
            if remaining.is_empty() {
                break;
            }
            // Index this rung's outcomes by requested type (first wins on a
            // rung-side duplicate — mirrors the driver's per-type dedup rule).
            let mut got: BTreeMap<String, AcquisitionOutcome> = BTreeMap::new();
            for ta in rung.acquire(&remaining, discovery) {
                got.entry(ta.requested).or_insert(ta.outcome);
            }
            let mut still: Vec<String> = Vec::with_capacity(remaining.len());
            for t in remaining {
                match got.remove(&t) {
                    // Not attempted by this rung — carry forward, no reason.
                    None => still.push(t),
                    Some(AcquisitionOutcome::Acquired(schema)) => {
                        acquired.entry(t).or_insert(schema);
                    }
                    Some(AcquisitionOutcome::Skipped(reasons)) => {
                        skips.entry(t.clone()).or_default().extend(reasons);
                        still.push(t);
                    }
                }
            }
            // A rung returning a type NOT in
            // its handed batch is the same trust-boundary contract violation
            // the engine driver rejects loudly — but the driver only ever sees
            // this chain's FILTERED output, so a misbehaving inner rung (the
            // remote-reading rungs this seam exists for) could never trip it.
            // Surface the residual here, loudly, and drop it (never staged).
            for unsolicited in got.into_keys() {
                tracing::warn!(
                    unsolicited = %unsolicited,
                    "schema-acquisition rung returned a type that was not in its handed \
                     batch — ignored (never staged)"
                );
            }
            remaining = still;
        }

        // One outcome per ATTEMPTED type, in the caller's input order. Acquired
        // wins over any earlier skip; a type only ever skipped carries its
        // accumulated reasons; a type every rung omitted is itself omitted.
        let mut out = Vec::new();
        for t in types {
            if let Some(schema) = acquired.get(t) {
                out.push(TypeAcquisition {
                    requested: t.clone(),
                    outcome: AcquisitionOutcome::Acquired(schema.clone()),
                });
            } else if let Some(reasons) = skips.get(t) {
                out.push(TypeAcquisition {
                    requested: t.clone(),
                    outcome: AcquisitionOutcome::Skipped(reasons.clone()),
                });
            }
        }
        out
    }
}

#[cfg(test)]
mod acquisition_tests {
    use super::*;
    use std::cell::RefCell;

    /// A recording acquirer: remembers every batch it was handed and returns a
    /// canned outcome per requested type. A type with NO canned entry is
    /// OMITTED from the response (the "not attempted" contract).
    struct RecordingAcquirer {
        responses: BTreeMap<String, AcquisitionOutcome>,
        batches: RefCell<Vec<Vec<String>>>,
    }
    impl RecordingAcquirer {
        fn new(responses: &[(&str, AcquisitionOutcome)]) -> Self {
            Self {
                responses: responses
                    .iter()
                    .map(|(t, o)| (t.to_string(), o.clone()))
                    .collect(),
                batches: RefCell::new(Vec::new()),
            }
        }
        fn batches(&self) -> Vec<Vec<String>> {
            self.batches.borrow().clone()
        }
    }
    impl SchemaAcquirer for RecordingAcquirer {
        fn acquire(&self, types: &[String], _discovery: &DiscoveryResult) -> Vec<TypeAcquisition> {
            self.batches.borrow_mut().push(types.to_vec());
            types
                .iter()
                .filter_map(|t| {
                    self.responses.get(t).map(|o| TypeAcquisition {
                        requested: t.clone(),
                        outcome: o.clone(),
                    })
                })
                .collect()
        }
    }

    fn acq(rung: AcquisitionRung, pkg: &str, ty: &str, text: &str) -> AcquisitionOutcome {
        AcquisitionOutcome::Acquired(AcquiredSchema {
            rung,
            closure: vec![AcquiredMsg {
                package: pkg.to_string(),
                type_name: ty.to_string(),
                msg_text: text.to_string(),
            }],
        })
    }
    fn skip(rung: AcquisitionRung, reason: &str) -> AcquisitionOutcome {
        AcquisitionOutcome::Skipped(vec![RungSkip {
            rung,
            reason: reason.to_string(),
        }])
    }

    /// A MISBEHAVING rung returning a type NOT
    /// in its handed batch — the chain drops it (warned at the chain, since the
    /// engine driver only ever sees the chain's filtered output) and the
    /// legitimate acquisition still flows. The unsolicited type must never
    /// appear in the chain's result.
    #[test]
    fn test_chained_drops_unsolicited_rung_results() {
        struct RogueRung;
        impl SchemaAcquirer for RogueRung {
            fn acquire(
                &self,
                types: &[String],
                _discovery: &DiscoveryResult,
            ) -> Vec<TypeAcquisition> {
                let mut out = vec![TypeAcquisition {
                    // NOT in the handed batch — the trust-boundary violation.
                    requested: "rogue/NotAsked".to_string(),
                    outcome: AcquisitionOutcome::Acquired(AcquiredSchema {
                        rung: AcquisitionRung::WireService,
                        closure: vec![AcquiredMsg {
                            package: "rogue".to_string(),
                            type_name: "NotAsked".to_string(),
                            msg_text: "int32 x\n".to_string(),
                        }],
                    }),
                }];
                // …while legitimately acquiring everything it was asked for.
                out.extend(types.iter().map(|t| {
                    let (pkg, ty) = t.split_once('/').expect("pkg/Type");
                    TypeAcquisition {
                        requested: t.clone(),
                        outcome: acq(AcquisitionRung::WireService, pkg, ty, "int32 a\n"),
                    }
                }));
                out
            }
        }
        let rogue = RogueRung;
        let chain = ChainedAcquirer::new(vec![&rogue]);
        let out = chain.acquire(&["acme/A".to_string()], &DiscoveryResult::empty());
        // Exactly the requested type, acquired; the rogue entry is dropped.
        assert_eq!(out.len(), 1, "{out:?}");
        assert_eq!(out[0].requested, "acme/A");
        assert!(
            matches!(out[0].outcome, AcquisitionOutcome::Acquired(_)),
            "the legitimate acquisition must survive the rogue sibling"
        );
    }

    #[test]
    fn test_local_ament_rung_label_is_plain_language_and_paren_free() {
        assert_eq!(
            AcquisitionRung::LocalAment.label(),
            "local ROS install via ament index"
        );
        // Every label is paren-free so a
        // parenthesized call-site context never renders nested parens.
        for rung in [
            AcquisitionRung::WireService,
            AcquisitionRung::LocalAment,
            AcquisitionRung::BridgeScavenge,
            AcquisitionRung::SshHarvest,
            AcquisitionRung::PublicLookup,
        ] {
            let l = rung.label();
            assert!(
                !l.contains('(') && !l.contains(')'),
                "rung label must be paren-free: {l:?}"
            );
        }
    }

    #[test]
    fn test_chained_first_acquires_second_never_sees_the_type() {
        let first = RecordingAcquirer::new(&[(
            "acme/A",
            acq(AcquisitionRung::LocalAment, "acme", "A", "int32 a\n"),
        )]);
        // Second would answer DIFFERENTLY — proving it is never consulted.
        let second = RecordingAcquirer::new(&[(
            "acme/A",
            acq(
                AcquisitionRung::PublicLookup,
                "acme",
                "A",
                "SHOULD-NOT-BE-USED\n",
            ),
        )]);
        let chain = ChainedAcquirer::new(vec![&first, &second]);
        let out = chain.acquire(&["acme/A".to_string()], &DiscoveryResult::empty());
        assert_eq!(out.len(), 1);
        match &out[0].outcome {
            AcquisitionOutcome::Acquired(s) => {
                assert_eq!(s.rung, AcquisitionRung::LocalAment);
                assert_eq!(s.closure[0].msg_text, "int32 a\n");
            }
            o => panic!("expected Acquired, got {o:?}"),
        }
        assert_eq!(first.batches(), vec![vec!["acme/A".to_string()]]);
        assert!(
            second.batches().is_empty(),
            "an already-acquired type must never reach a later rung"
        );
    }

    #[test]
    fn test_chained_skip_reasons_accumulate_in_ladder_order() {
        let first = RecordingAcquirer::new(&[(
            "acme/A",
            skip(AcquisitionRung::WireService, "no type hash"),
        )]);
        let second = RecordingAcquirer::new(&[(
            "acme/A",
            skip(
                AcquisitionRung::LocalAment,
                "acme not found in any ament prefix",
            ),
        )]);
        let chain = ChainedAcquirer::new(vec![&first, &second]);
        let out = chain.acquire(&["acme/A".to_string()], &DiscoveryResult::empty());
        assert_eq!(out.len(), 1);
        match &out[0].outcome {
            AcquisitionOutcome::Skipped(reasons) => {
                assert_eq!(reasons.len(), 2);
                assert_eq!(reasons[0].rung, AcquisitionRung::WireService);
                assert_eq!(reasons[0].reason, "no type hash");
                assert_eq!(reasons[1].rung, AcquisitionRung::LocalAment);
                assert_eq!(reasons[1].reason, "acme not found in any ament prefix");
            }
            o => panic!("expected Skipped, got {o:?}"),
        }
        assert_eq!(second.batches(), vec![vec!["acme/A".to_string()]]);
    }

    #[test]
    fn test_chained_later_rung_acquires_after_earlier_skip() {
        let first = RecordingAcquirer::new(&[(
            "acme/A",
            skip(AcquisitionRung::WireService, "no type hash"),
        )]);
        let second = RecordingAcquirer::new(&[(
            "acme/A",
            acq(AcquisitionRung::LocalAment, "acme", "A", "int32 a\n"),
        )]);
        let chain = ChainedAcquirer::new(vec![&first, &second]);
        let out = chain.acquire(&["acme/A".to_string()], &DiscoveryResult::empty());
        assert_eq!(out.len(), 1);
        assert!(matches!(
            &out[0].outcome,
            AcquisitionOutcome::Acquired(s) if s.rung == AcquisitionRung::LocalAment
        ));
    }

    #[test]
    fn test_chained_omitted_everywhere_is_omitted_from_result() {
        let first = RecordingAcquirer::new(&[]);
        let second = RecordingAcquirer::new(&[]);
        let chain = ChainedAcquirer::new(vec![&first, &second]);
        let out = chain.acquire(&["acme/Ghost".to_string()], &DiscoveryResult::empty());
        assert!(
            out.is_empty(),
            "a type no rung attempts is omitted (Noop-equivalent)"
        );
        // Both rungs saw the still-unacquired type carried forward.
        assert_eq!(first.batches(), vec![vec!["acme/Ghost".to_string()]]);
        assert_eq!(second.batches(), vec![vec!["acme/Ghost".to_string()]]);
    }

    #[test]
    fn test_empty_chain_is_noop_equivalent() {
        let chain = ChainedAcquirer::new(vec![]);
        let types = vec!["acme/A".to_string(), "acme/B".to_string()];
        let disc = DiscoveryResult::empty();
        assert_eq!(
            chain.acquire(&types, &disc),
            NoopAcquirer.acquire(&types, &disc)
        );
        assert!(chain.acquire(&types, &disc).is_empty());
    }

    #[test]
    fn test_chained_preserves_input_order_and_mixed_outcomes() {
        // Input order B, A, C. first acquires A, skips C, omits B; second
        // acquires B. Output must be in the INPUT order B, A, C.
        let first = RecordingAcquirer::new(&[
            (
                "acme/A",
                acq(AcquisitionRung::LocalAment, "acme", "A", "int32 a\n"),
            ),
            (
                "acme/C",
                skip(
                    AcquisitionRung::LocalAment,
                    "acme/C not found in any ament prefix",
                ),
            ),
        ]);
        let second = RecordingAcquirer::new(&[(
            "acme/B",
            acq(AcquisitionRung::PublicLookup, "acme", "B", "int32 b\n"),
        )]);
        let chain = ChainedAcquirer::new(vec![&first, &second]);
        let out = chain.acquire(
            &[
                "acme/B".to_string(),
                "acme/A".to_string(),
                "acme/C".to_string(),
            ],
            &DiscoveryResult::empty(),
        );
        let names: Vec<&str> = out.iter().map(|ta| ta.requested.as_str()).collect();
        assert_eq!(names, vec!["acme/B", "acme/A", "acme/C"]);
        // Second rung only saw the still-unacquired B and C (A was removed).
        assert_eq!(
            second.batches(),
            vec![vec!["acme/B".to_string(), "acme/C".to_string()]]
        );
    }
}
