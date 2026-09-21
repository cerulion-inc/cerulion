// SPDX-License-Identifier: AGPL-3.0-only
//! `cerulion ros2 attach` — the runtime half of the drop-in ROS-2/DDS attach
//! story.
//!
//! `cerulion ros2 attach --iface <robot-LAN-ip> [--domain N]` runs DDS discovery
//! on the given interface, partitions the discovered topics into RESOLVABLE
//! (a Cerulion codec/schema exists), UNRESOLVABLE (named LOUDLY with the
//! exact remediation), and EXCLUDED-malformed (a candidate topic name the
//! graph layer's own predicate rejects — reported loudly per topic, never
//! silently skipped, never written), generates the `dds_bridge` mapping
//! config + a LEAN one-node bridge graph for the clean resolvable set, and
//! (after the consent ladder) runs it. `--dry-run` stops after the discovery
//! report — the "what would I get" view.
//!
//! # The DDS seam (this module is DDS-free)
//!
//! ALL logic here is PURE over a plain [`DiscoveredEndpoint`] list and never
//! names a `ros2-client`/`rustdds` type. The engine depends on `cerulion_dds`
//! with `default-features = false` (its DDS-free vocabulary — types + trait +
//! error — only), so isolated engine builds/tests carry NO DDS stack. The live
//! discovery backend (`cerulion_dds::LiveDiscovery`, behind that crate's `live`
//! feature) implements [`DdsDiscovery`] and is wired in by the binary. Every
//! function below is therefore oracle-testable without a DDS peer (hand-built
//! endpoint vectors), and if the placement decision flips only the backend crate
//! moves.
//!
//! # The resolvability seam (two-armed)
//!
//! [`resolve_bridge_route`] is the predicate: a `ros_type` routes TYPED when
//! it is one of the four registry types ([`V1_BRIDGE_BINDINGS`] — a mirror of
//! `examples/go2/nodes/dds_bridge/src/registry.rs`; typed WINS), else RAW when a
//! built-in `MessageSchema` resolves it (`native_ros2_messages::BUILTIN_MSGS`,
//! the built-in resolution path, mirroring the bridge's own
//! `generic::schema_supported` gate), else it is UNRESOLVABLE with both
//! remediations named.

use std::collections::{BTreeMap, BTreeSet};
use std::net::IpAddr;
use std::path::{Path, PathBuf};
use std::time::Duration;

// The DDS-free discovery + acquisition vocabulary + the backend seams live in
// the leaf `cerulion_dds` crate (this engine depends on it; the reverse edge
// would cycle). Everything below is PURE over these plain types.
use cerulion_dds::{
    AcquiredMsg, AcquisitionOutcome, DdsDiscovery, DiscoveredEndpoint, DiscoveredNode,
    DiscoveredQos, DiscoveryParams, DiscoveryResult, EndpointKind, NoopAcquirer, QosDurability,
    SchemaAcquirer,
};

// Acquired `.msg` text is parsed with the SAME `parse_rosmsg`
// the store reader + codegen use, and its nested closure is walked over the
// resulting `MessageSchema` — resolved through the codec's SHARED nested-ref
// ladder (`resolve_nested_qname_ladder`) so the completeness check predicts the
// CDR decoder exactly — for the partial-closure completeness check.
use cerulion_core::codegen::{parse_rosmsg, resolve_nested_qname_ladder, FieldType, MessageSchema};

use crate::error::{CliError, CliResult};
use crate::local_harvest::LocalAmentAcquirer;
use crate::schema_store::SchemaStore;

// ─────────────────────────── Normalization (pure) ──────────────────────────

/// Normalize a raw DDS topic name to the canonical ROS/Cerulion topic name.
///
/// ROS 2 over DDS prefixes topic names: `rt/` (topics), `rq/` (service
/// request), `rr/` (service reply). We strip a known prefix and prepend `/`;
/// a name with no known prefix is prepended with `/` verbatim. The result is
/// always a leading-`/` canonical name.
pub fn normalize_dds_topic(dds_topic: &str) -> String {
    for prefix in ["rt/", "rq/", "rr/", "rs/"] {
        if let Some(rest) = dds_topic.strip_prefix(prefix) {
            return format!("/{rest}");
        }
    }
    if let Some(rest) = dds_topic.strip_prefix('/') {
        format!("/{rest}")
    } else {
        format!("/{dds_topic}")
    }
}

/// Normalize a raw DDS type name to the canonical `pkg/Type` form the codec
/// registry matches on.
///
/// Handles the three shapes discovery emits:
/// * IDL-mangled CDR: `sensor_msgs::msg::dds_::PointCloud2_` → `sensor_msgs/PointCloud2`
/// * ROS slash form:   `sensor_msgs/msg/PointCloud2`         → `sensor_msgs/PointCloud2`
/// * ROS colon form:   `sensor_msgs::msg::PointCloud2`       → `sensor_msgs/PointCloud2`
///
/// The `::msg::`/`/msg/` infix and the CDR `dds_::`/trailing-`_` markers are
/// dropped; a form that does not fit (`Foo`, empty) is returned trimmed as-is
/// (it simply will not resolve, surfacing as UNRESOLVABLE).
pub fn normalize_ros_type(type_name: &str) -> String {
    let t = type_name.trim();
    // Split on either separator so both `::` and `/` forms fold together.
    let parts: Vec<&str> = t.split(['/', ':']).filter(|s| !s.is_empty()).collect();
    if parts.len() < 2 {
        return t.to_string();
    }
    let pkg = parts[0];
    // The last segment is the type; strip the CDR `dds_` mangling.
    let mut ty = parts[parts.len() - 1];
    // `dds_::Type_` splits to [.., "dds_", "Type_"]; drop a trailing `_` on the
    // type name (CDR mangling appends it) and ignore a lone `dds_` segment.
    ty = ty.strip_suffix('_').unwrap_or(ty);
    if ty.is_empty() || ty == "dds" {
        return t.to_string();
    }
    format!("{pkg}/{ty}")
}

// ─────────────────────────── Aggregation (pure) ────────────────────────────

/// One discovered ROS topic — endpoints on the same `(ros_topic, ros_type)`
/// folded together with reader/writer counts and a representative QoS.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct DiscoveredTopic {
    /// Canonical ROS/Cerulion topic name (`/utlidar/cloud`).
    pub ros_topic: String,
    /// Canonical `pkg/Type` (`sensor_msgs/PointCloud2`).
    pub ros_type: String,
    /// The raw DDS type name (kept for the report's remediation text).
    pub raw_type: String,
    pub writer_count: usize,
    pub reader_count: usize,
    /// A writer's QoS if any writer was seen, else the first reader's.
    /// REPORT-DISPLAY only — never decision-bearing (the first-writer-wins
    /// representative is an artifact of GUID-keyed DiscoveryDB iteration order;
    /// the typed-port pick reads [`Self::writer_durability_rank`] instead).
    pub qos: DiscoveredQos,
    /// The MINIMUM `durability_streaming_rank` across this topic's
    /// WRITER endpoints only — the ORDER-INDEPENDENT liveness aggregate the
    /// typed-port preference key consumes (min over a multiset is commutative,
    /// so endpoint enumeration order can never flip the winner; any volatile
    /// writer ⇒ the topic is a live stream ⇒ rank 0). Reader-requested QoS
    /// NEVER contributes (a reader's durability is a request, not an offered
    /// liveness signal). `u8::MAX` when the topic has no writers — never
    /// consulted for preference (the key's writer-presence field already
    /// demotes writer-less topics) but deterministic by construction.
    pub writer_durability_rank: u8,
}

/// Fold a raw endpoint list into deduplicated topics. Keyed by
/// `(ros_topic, ros_type)`; a writer's QoS wins over a reader's (the bridge
/// subscribes, so the publisher's offered QoS is the one that matters). Result
/// is sorted by `(ros_topic, ros_type)` for a deterministic report.
pub fn aggregate_topics(endpoints: &[DiscoveredEndpoint]) -> Vec<DiscoveredTopic> {
    // Preserve first-seen raw_type/qos deterministically via an index map.
    use indexmap::IndexMap;
    let mut map: IndexMap<(String, String), DiscoveredTopic> = IndexMap::new();
    for ep in endpoints {
        let ros_topic = normalize_dds_topic(&ep.dds_topic);
        let ros_type = normalize_ros_type(&ep.type_name);
        let key = (ros_topic.clone(), ros_type.clone());
        let entry = map.entry(key).or_insert_with(|| DiscoveredTopic {
            ros_topic,
            ros_type,
            raw_type: ep.type_name.clone(),
            writer_count: 0,
            reader_count: 0,
            qos: ep.qos,
            writer_durability_rank: u8::MAX,
        });
        match ep.kind {
            EndpointKind::Writer => {
                // First writer's QoS becomes the representative (over any
                // reader QoS captured at insert time). Display only — the
                // decision-bearing liveness signal is the min-rank fold below.
                if entry.writer_count == 0 {
                    entry.qos = ep.qos;
                }
                entry.writer_count += 1;
                // Min-fold over WRITERS only — order-independent.
                entry.writer_durability_rank = entry
                    .writer_durability_rank
                    .min(durability_streaming_rank(ep.qos.durability));
            }
            EndpointKind::Reader => entry.reader_count += 1,
        }
    }
    let mut topics: Vec<DiscoveredTopic> = map.into_values().collect();
    topics.sort_by(|a, b| {
        a.ros_topic
            .cmp(&b.ros_topic)
            .then_with(|| a.ros_type.cmp(&b.ros_type))
    });
    topics
}

// ──────────────────────── Resolvability seam (v1) ──────────────────────────

/// A `ros_type` the `dds_bridge` node can carry: its FIXED output port + the
/// Cerulion schema that port publishes. Mirrors
/// `examples/go2/nodes/dds_bridge/src/registry.rs::RosType`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct BridgeBinding {
    /// Canonical `pkg/Type` (the config `ros_type`).
    pub ros_type: &'static str,
    /// The bridge node's fixed output port name.
    pub port: &'static str,
    /// The Cerulion schema that port publishes (the graph's `schema:`).
    pub cerulion_schema: &'static str,
}

/// The v1 supported set — declaration order IS the bridge's fixed port order
/// (`registry.rs::ALL_ROS_TYPES`). The graph generator emits all four ports in
/// THIS order because the bridge macro declares them at compile time and every
/// output proxy is resolved before the node body runs (an omitted port fails
/// every tick).
///
/// `static` (not `const`) so [`resolve_bridge_binding`] can hand out
/// `&'static BridgeBinding` refs (a `const` inlines a fresh temporary array at
/// each use site, whose element refs are not `'static`).
pub static V1_BRIDGE_BINDINGS: [BridgeBinding; 4] = [
    BridgeBinding {
        ros_type: "sensor_msgs/PointCloud2",
        port: "cloud",
        cerulion_schema: "sensor_msgs/PointCloud2",
    },
    BridgeBinding {
        ros_type: "unitree_go/SportModeState",
        port: "odom",
        cerulion_schema: "nav_msgs/Odometry",
    },
    BridgeBinding {
        ros_type: "geometry_msgs/Twist",
        port: "twist",
        cerulion_schema: "geometry_msgs/TwistStamped",
    },
    BridgeBinding {
        ros_type: "unitree_api/Request",
        port: "request_json",
        cerulion_schema: "std_msgs/String",
    },
];

/// The TYPED half of the resolvability seam: exact match against the bridge's
/// four-type registry (each owns one fixed macro port; typed WINS for these
/// types — `dds_bridge::generic::route_mapping`).
pub fn resolve_bridge_binding(ros_type: &str) -> Option<&'static BridgeBinding> {
    V1_BRIDGE_BINDINGS.iter().find(|b| b.ros_type == ros_type)
}

/// The bridge-LOCAL Unitree `.msg` set the `dds_bridge` generic codec
/// ADDITIONALLY carries beyond the built-in corpus — a NAME mirror of
/// `examples/go2/nodes/dds_bridge/src/generic.rs::UNITREE_MSGS`, exactly the way
/// [`V1_BRIDGE_BINDINGS`] mirrors `registry.rs` (keep the two in sync). Consumed
/// ONLY by the registry-type typed-port-loser gate ([`raw_fallback_blocker`]):
/// it makes the CLI's store-closure completeness walk see the SAME nested-ref
/// universe the bridge's runtime codec resolves against, so a store schema
/// referencing e.g. `unitree_go/IMUState` (absent from store + builtins but
/// embedded in the bridge) is not falsely flagged incomplete. NOTE: the
/// bridge's `UNITREE_MSGS` is slated to retire in favor of live acquisition
/// — when it does, retire THIS mirror in the same change (the gate
/// must always match the bridge's ACTUAL codec surface).
const BRIDGE_UNITREE_MSG_NAMES: &[&str] = &[
    "unitree_go/IMUState",
    "unitree_go/SportModeState",
    "unitree_api/RequestIdentity",
    "unitree_api/RequestLease",
    "unitree_api/RequestPolicy",
    "unitree_api/RequestHeader",
    "unitree_api/Request",
];

/// Introspection accessor for the bridge-local Unitree `.msg` name mirror
/// (`BRIDGE_UNITREE_MSG_NAMES`). Exposed so the engine test suite can pin it
/// SET-EQUAL to the REAL `examples/go2/nodes/dds_bridge/src/generic.rs`
/// `UNITREE_MSGS` list (a lockstep pin) — a bridge-side add / retire /
/// rename now fails the engine suite naming both files.
pub fn bridge_unitree_msg_names() -> &'static [&'static str] {
    BRIDGE_UNITREE_MSG_NAMES
}

/// How a resolvable `ros_type` is served by the `dds_bridge`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum BridgeRoute {
    /// One of the four registry types — served by the bridge's FIXED macro
    /// port (deliberate typed projections; typed WINS for these types). The
    /// mapping's `cerulion_topic` is documentation; the graph's `topic:`
    /// override is authoritative.
    Typed(&'static BridgeBinding),
    /// Any other type with a resolvable built-in `MessageSchema` — bridged by
    /// the generic raw codec: the bridge transcodes raw CDR onto a
    /// dynamically-created ingress publisher. The mapping's `cerulion_topic`
    /// is AUTHORITATIVE (no graph port exists for it).
    Raw,
}

/// The attach resolvability CHAIN: the workspace `.msg`
/// schema store FIRST, then the built-in ROS 2 registry.
///
/// This is the widening of the raw arm of [`resolve_bridge_route`]: a type
/// the built-in corpus does not carry can now resolve from a workspace
/// `schemas/<pkg>/msg/<Type>.msg` file (the store every acquisition rung
/// materializes into). Store WINS on a collision (the workspace-wins precedent), and a
/// store entry that shadows a built-in is surfaced loudly ONCE at
/// construction — never silently. The typed-registry arm
/// ([`resolve_bridge_binding`]) is unaffected and still wins for its four
/// types.
///
/// Cheap to hold and pass by reference: it owns the parsed store and is built
/// once per `ros_attach` from the workspace root, then consulted per topic.
/// The acquisition seam re-uses the same handle: after the ladder
/// stages harvested `.msg` schemas in memory ([`stage_all`](Self::stage_all)),
/// the re-resolve pass consults them WITHOUT any file having been written yet
/// (the never-mutate-before-consent floor).
pub struct AttachSchemaChain {
    store: SchemaStore,
    /// Qualified `pkg/Type` names staged IN MEMORY by the acquisition
    /// ladder — resolvable for this run's re-resolve pass, but written to the
    /// workspace store ONLY inside the consent gate. Empty on the
    /// pre-acquisition chain (so the store→builtins behavior is unchanged).
    staged: BTreeSet<String>,
}

impl AttachSchemaChain {
    /// Build from a workspace root, loading `schemas/<pkg>/msg/*.msg`. Any
    /// store entry that shadows a built-in ROS 2 message is `warn!`-logged
    /// here — the boundary where the store-wins inference happens (loud, not
    /// silent, per the house UX rule).
    pub fn from_workspace(workspace_root: &Path) -> Self {
        let store = SchemaStore::load(&workspace_root.join("schemas"));
        for shadowed in store.builtin_shadows() {
            tracing::warn!(
                schema = %shadowed,
                "ros2 attach: the workspace .msg store shadows a built-in ROS 2 message of the \
                 same qualified name — the store definition wins at resolution"
            );
        }
        Self {
            store,
            staged: BTreeSet::new(),
        }
    }

    /// The builtins-only chain (empty store): exactly the behavior of a
    /// workspace with no store. Backs the builtins-only [`resolve_bridge_route`]
    /// entry point, and is what a caller passes [`partition_topics_with`] for
    /// the builtins-only split.
    pub fn builtins_only() -> Self {
        Self {
            store: SchemaStore::empty(),
            staged: BTreeSet::new(),
        }
    }

    /// Stage acquired qualified names IN MEMORY so the
    /// re-resolve pass treats them as resolvable. Idempotent; no file is
    /// touched (materialization rides the consent gate).
    pub fn stage_all(&mut self, names: impl IntoIterator<Item = String>) {
        self.staged.extend(names);
    }

    /// Does the chain resolve `ros_type` (`pkg/Type`) to a raw-codec route?
    /// Store FIRST, then the in-memory staging set, then the
    /// built-in registry. The three are a boolean union; precedence only
    /// matters for `schema info` provenance (store-wins), enforced there.
    pub fn resolves(&self, ros_type: &str) -> bool {
        self.store.resolves(ros_type)
            || self.staged.contains(ros_type)
            || builtin_schema_resolves(ros_type)
    }

    /// True when the on-disk workspace `.msg` store (loaded at construction)
    /// holds ≥1 schema, the signal that the generated bridge
    /// config must point its generic codec at the store. The in-memory staging
    /// set is deliberately NOT counted here: staged schemas are materialized
    /// THIS run and accounted separately via the acquisition report's to-write
    /// set (`ros2 attach` ORs the two).
    pub fn store_non_empty(&self) -> bool {
        !self.store.is_empty()
    }

    /// When `ros_type` resolves by NAME via the
    /// on-disk store, walk the STORE schema's nested references against the
    /// resolvable universe (store ∪ staged ∪ built-ins — the SAME
    /// `resolve_nested_ref` ladder the CDR decoder runs). `Some(missing)`
    /// means the store schema is INCOMPLETE: the bridge codec would still
    /// accept the mapping at load (`knows()` is name-presence) and then fail
    /// per-frame at CDR decode — the silent-failure class the acquired-bundle
    /// completeness walk (`process_acquired` step 3) already kills for ladder
    /// acquisitions; this is the same walk for hand-dropped store files.
    /// `None` = complete, or not a store-held type. Because the universe
    /// includes the in-memory STAGING set, an incompleteness the acquisition
    /// ladder just healed (by staging the missing member) reads as complete on
    /// the post-staging re-resolve.
    pub fn store_closure_missing(&self, ros_type: &str) -> Option<Vec<String>> {
        self.store_closure_missing_with(ros_type, &[])
    }

    /// [`Self::store_closure_missing`] with EXTRA resolvable names unioned into
    /// the walk's universe: the registry-type typed-port-loser gate
    /// passes [`BRIDGE_UNITREE_MSG_NAMES`] so the completeness walk sees the
    /// SAME universe the bridge's runtime codec resolves nested refs against
    /// (store ∪ staged ∪ built-ins ∪ the bridge-embedded UNITREE set) — a store
    /// schema whose only "missing" deps are bridge-embedded is COMPLETE for the
    /// bridge. The plain entry point passes no extras (the partition's raw-arm
    /// check keeps predicting the CLI-visible codec exactly).
    ///
    /// The walk is TRANSITIVE: when a nested ref resolves to
    /// ANOTHER STORE schema, that schema's own nested dependencies are pushed
    /// onto the worklist, because the bridge's CDR decoder must decode the FULL
    /// transitive closure per frame — a store shadow whose direct dep resolves
    /// to a store schema with a missing grandchild would validate at bridge load
    /// (name-presence) and then fail every frame at decode. A ref resolving to a
    /// built-in or a bridge-embedded extra TERMINATES (those closures are
    /// self-contained — the bridge carries them whole). A `visited` set on
    /// qualified names guards cycles (A refs B, B refs A) and duplicate refs.
    fn store_closure_missing_with(&self, ros_type: &str, extra: &[&str]) -> Option<Vec<String>> {
        let stored = self.store.get(ros_type)?;
        let mut universe = self.resolvable_universe();
        universe.extend(extra.iter().map(|s| s.to_string()));
        let mut missing: BTreeSet<String> = BTreeSet::new();
        let mut visited: BTreeSet<String> = BTreeSet::new();
        visited.insert(ros_type.to_string());
        let mut worklist: Vec<NestedRef> = nested_dependencies(&stored.schema);
        while let Some(r) = worklist.pop() {
            match resolve_nested_ref(&r, &universe) {
                None => {
                    missing.insert(missing_ref_label(&r, &universe));
                }
                // A not-yet-visited resolved ref; the guard marks it visited
                // (cycles + duplicate refs guarded). Recurse ONLY when it names
                // a STORE schema (its own closure the codec must also decode).
                // The `else` (store MISS) is where every resolved-but-unstored
                // ref terminates on its FIRST encounter — a built-in, a bridge-
                // embedded extra, OR a STAGED name (staged names live in the
                // universe but not the store). Built-ins / extras are self-
                // contained (the bridge carries them whole); a STAGED name is
                // NOT self-contained by construction, but its own closure is
                // validated separately by the acquisition-bundle completeness
                // check (`process_acquired`, direct-only), not recursed here.
                Some(qname) if visited.insert(qname.clone()) => {
                    if let Some(dep) = self.store.get(&qname) {
                        worklist.extend(nested_dependencies(&dep.schema));
                    }
                }
                // An already-visited ref — a duplicate reference or a cycle back
                // to a schema already walked. Nothing to walk.
                Some(_) => {}
            }
        }
        if missing.is_empty() {
            None
        } else {
            Some(missing.into_iter().collect())
        }
    }

    /// Every qualified `pkg/Type` name resolvable via the store, the in-memory
    /// staging set, and the built-in corpus — the RESOLVABLE UNIVERSE the
    /// nested-reference ladder's unambiguous-bare-suffix rung enumerates.
    /// The completeness walk unions this with
    /// the acquired bundle's OWN closure names before resolving each reference,
    /// so the walk sees exactly what the CDR decoder will have loaded.
    fn resolvable_universe(&self) -> BTreeSet<String> {
        let mut u: BTreeSet<String> = self.store.iter().map(|(q, _)| q.to_string()).collect();
        u.extend(self.staged.iter().cloned());
        for &(pkg, name, _) in native_ros2_messages::BUILTIN_MSGS {
            u.insert(format!("{pkg}/{name}"));
        }
        u
    }
}

/// THE resolvability seam, two-armed: typed registry
/// first (typed wins), then the generic schema chain. This is the builtins-only
/// entry point; [`resolve_bridge_route_with`]
/// threads a store-backed [`AttachSchemaChain`].
pub fn resolve_bridge_route(ros_type: &str) -> Option<BridgeRoute> {
    resolve_bridge_route_with(&AttachSchemaChain::builtins_only(), ros_type)
}

/// The resolvability seam over an explicit [`AttachSchemaChain`]: typed
/// registry first (typed wins), then the store→builtins
/// chain's raw arm.
pub fn resolve_bridge_route_with(chain: &AttachSchemaChain, ros_type: &str) -> Option<BridgeRoute> {
    if let Some(b) = resolve_bridge_binding(ros_type) {
        return Some(BridgeRoute::Typed(b));
    }
    if chain.resolves(ros_type) {
        return Some(BridgeRoute::Raw);
    }
    None
}

/// Does a built-in ROS 2 `MessageSchema` resolve `ros_type` (`pkg/Type`)?
/// The resolution path over `native_ros2_messages::BUILTIN_MSGS` —
/// membership alone suffices because every registry entry parses by
/// build-time invariant (`schema_cmd::parse_builtin_schemas` panics loudly if
/// that codegen invariant is ever broken). Mirrors the bridge's own raw
/// gate (`generic::schema_supported`, whose codec is seeded from the SAME
/// `BUILTIN_MSGS`); the bridge additionally knows its bridge-local
/// `UNITREE_MSGS` extras, so this predicate is CONSERVATIVE — it can miss a
/// type the bridge would accept, but never emits a raw mapping the bridge's
/// config validation rejects (the doomed-run direction).
pub(crate) fn builtin_schema_resolves(ros_type: &str) -> bool {
    match ros_type.split_once('/') {
        Some((pkg, msg)) if !pkg.is_empty() && !msg.is_empty() && !msg.contains('/') => {
            native_ros2_messages::BUILTIN_MSGS
                .iter()
                .any(|&(p, n, _)| p == pkg && n == msg)
        }
        _ => false,
    }
}

/// The supported-set list for UNRESOLVABLE remediation text.
pub fn supported_ros_types() -> String {
    V1_BRIDGE_BINDINGS
        .iter()
        .map(|b| b.ros_type)
        .collect::<Vec<_>>()
        .join(", ")
}

// ─────────────────────────── Partition (pure) ──────────────────────────────

/// A discovered topic that resolves to a bridge route (typed port or the
/// raw-generic codec).
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct ResolvedTopic {
    pub topic: DiscoveredTopic,
    pub route: BridgeRoute,
}

/// The resolvable/unresolvable split of a discovered topic set.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Partitioned {
    pub resolvable: Vec<ResolvedTopic>,
    pub unresolvable: Vec<DiscoveredTopic>,
}

/// Partition topics by the resolvability seam over an explicit
/// [`AttachSchemaChain`] — the store-widened form
/// [`ros_attach`] drives. Pass [`AttachSchemaChain::builtins_only`] for the
/// earlier builtins-only behavior.
///
/// ORDERING CONTRACT (final-gate pin): both output lists preserve the INPUT
/// order — stable, never re-sorted. The input comes from
/// [`aggregate_topics`], which sorts by `(ros_topic, ros_type)`, so the
/// partition's order IS that sort. Everything downstream relies on this as a
/// contract, not an accident: the malformed-exclusion gate and
/// [`build_mappings`]' first-wins dedup rules key their determinism off it,
/// and [`render_discovery_report`] renders both lists verbatim (no
/// independent sort).
///
/// Supplement D: a raw route riding an INCOMPLETE store schema (its nested
/// closure has a gap — [`AttachSchemaChain::store_closure_missing`]) is
/// flipped to UNRESOLVABLE with a loud warn naming the missing dep, instead of
/// being bridged into a mapping that validates at bridge load and fails
/// per-frame at CDR decode.
pub fn partition_topics_with(chain: &AttachSchemaChain, topics: &[DiscoveredTopic]) -> Partitioned {
    let mut resolvable = Vec::new();
    let mut unresolvable = Vec::new();
    for t in topics {
        match resolve_bridge_route_with(chain, &t.ros_type) {
            Some(route) => {
                // A RAW route riding a store
                // schema whose nested closure is INCOMPLETE would validate at
                // bridge load (`knows()` is name-presence) and die per-frame
                // at CDR decode — flip it to UNRESOLVABLE here, loudly. Typed
                // projections never consult the store codec, so they are
                // exempt. Landing in the unresolvable set also hands the type
                // to the acquisition ladder, which can HEAL it by acquiring
                // the missing member (the staged name completes the universe
                // on the post-staging re-partition).
                let store_gap = match route {
                    BridgeRoute::Typed(_) => None,
                    BridgeRoute::Raw => chain.store_closure_missing(&t.ros_type),
                };
                if let Some(missing) = store_gap {
                    tracing::warn!(
                        ros_type = %t.ros_type,
                        missing = %missing.join(", "),
                        "ros2 attach: type resolves by name via the workspace .msg store, but \
                         the store schema's nested closure is INCOMPLETE — kept UNRESOLVABLE \
                         (drop the missing .msg into schemas/<pkg>/msg/, fix the store file, \
                         or let the acquisition ladder fetch it)"
                    );
                    unresolvable.push(t.clone());
                } else {
                    resolvable.push(ResolvedTopic {
                        topic: t.clone(),
                        route,
                    });
                }
            }
            None => unresolvable.push(t.clone()),
        }
    }
    Partitioned {
        resolvable,
        unresolvable,
    }
}

// ──────────────── Malformed-topic exclusion ───────────────────

/// The candidate Cerulion-side topic for a discovered ROS topic:
/// `topic_prefix` + ros_topic (prefix `None` ⇒ mirror). ONE minting site,
/// shared by [`exclude_malformed_topics`] and [`build_mappings`], so the
/// candidate the exclusion gate validates and the mapping that gets written
/// can never diverge.
fn candidate_cerulion_topic(topic_prefix: Option<&str>, ros_topic: &str) -> String {
    match topic_prefix {
        Some(p) => format!("{p}{ros_topic}"),
        None => ros_topic.to_string(),
    }
}

/// A resolvable topic EXCLUDED because its candidate
/// `cerulion_topic` fails the graph layer's own absolute-name predicate. A
/// hostile/nonconforming raw DDS topic name (`*`, `//`, `#`, ...) would
/// otherwise flow into the generated `topic:` override and die at graph load
/// AFTER the consent write — the same doomed-run class the
/// `--topic-prefix` validation refuses. Never silently skipped: each exclusion gets its
/// own report line naming the topic + the predicate's reason verbatim.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct ExcludedMalformedTopic {
    /// The discovered (canonical) ROS topic name.
    pub ros_topic: String,
    /// The topic's resolvable `pkg/Type`.
    pub ros_type: String,
    /// The failing candidate (prefix applied) — what the graph `topic:`
    /// override would have been.
    pub cerulion_topic: String,
    /// `cerulion_core::graph::malformed_absolute_name`'s reason, verbatim.
    pub reason: &'static str,
}

/// Gate every resolvable topic's CANDIDATE `cerulion_topic`
/// (prefix applied — the exact string the generated graph would carry as its
/// `topic:` override) through the graph layer's own
/// `cerulion_core::graph::malformed_absolute_name` predicate (the same single
/// source of truth the `--topic-prefix` probe uses), splitting the set into
/// (clean, excluded). Runs BEFORE [`build_mappings`], so a malformed
/// candidate can never claim a bridge port a valid sibling should keep, and
/// nothing malformed is ever generated or written.
pub fn exclude_malformed_topics(
    resolvable: Vec<ResolvedTopic>,
    topic_prefix: Option<&str>,
) -> (Vec<ResolvedTopic>, Vec<ExcludedMalformedTopic>) {
    let mut clean = Vec::new();
    let mut malformed = Vec::new();
    for r in resolvable {
        let candidate = candidate_cerulion_topic(topic_prefix, &r.topic.ros_topic);
        match cerulion_core::graph::malformed_absolute_name(&candidate) {
            Some(reason) => malformed.push(ExcludedMalformedTopic {
                ros_topic: r.topic.ros_topic.clone(),
                ros_type: r.topic.ros_type.clone(),
                cerulion_topic: candidate,
                reason,
            }),
            None => clean.push(r),
        }
    }
    (clean, malformed)
}

// ─────────────────────── Mapping build (dedup by type) ──────────────────────

/// One DDS→Cerulion mapping the config + graph share. `cerulion_topic` is the
/// resolved wire name (mirror of the ROS topic, optionally prefixed) — for a
/// [`BridgeRoute::Raw`] mapping it is AUTHORITATIVE (the bridge's ingress
/// route publishes on exactly it; no graph port exists).
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct BridgeMapping {
    pub dds_topic: String,
    pub ros_type: String,
    pub cerulion_topic: String,
    pub route: BridgeRoute,
}

/// A resolvable topic that LOST the race for its type's single typed
/// port AND could not be raw-routed — kept NOT bridged, loudly. With the
/// raw-route rescue a typed-port loser normally rides the generic codec instead; only
/// two RARE arms land here (all four registry types are always decodable by the
/// bridge's own generic codec — built-ins for PointCloud2/Twist, the
/// bridge-embedded `UNITREE_MSGS` for SportModeState/Request): (a) a workspace
/// store schema SHADOWS the bridge's entry with an INCOMPLETE nested closure
/// (`raw_fallback_blocker`), or (b) the workspace's VENDORED `dds_bridge`
/// predates `route: raw` and would reject the whole generated config
/// (`degrade_raw_routed_for_old_bridge`). Every other loser degrades to a
/// [`RawRoutedLoser`] instead.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct DroppedDuplicate {
    pub dds_topic: String,
    pub ros_type: String,
    /// The topic that kept the typed port.
    pub kept_dds_topic: String,
    /// Why the loser could NOT be raw-routed (incomplete store shadow, or an
    /// older vendored bridge): the NOT-bridged reason, carrying
    /// its own arm-specific remediation (one fixed suffix
    /// would contradict the store-closure arm).
    pub raw_fallback_reason: String,
}

/// A resolvable topic that LOST the race for its type's single typed
/// port but was RESCUED onto the generic raw codec — bridged, just not through
/// the type's fixed macro port. This is the rescue's headline: a same-type
/// sibling of the typed winner (e.g. the Go2's `/uslam/cloud_map` when
/// `/utlidar/cloud` keeps the `sensor_msgs/PointCloud2` port) rides the generic
/// codec onto its own absolute Cerulion topic instead of being dropped. Surfaced
/// so the report line says HOW it is bridged (and that a per-instance
/// typed port is the eventual home).
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct RawRoutedLoser {
    /// The (canonical) ROS topic that degraded to the generic codec.
    pub ros_topic: String,
    /// Its resolvable `pkg/Type`.
    pub ros_type: String,
    /// The sibling topic that kept the type's typed port.
    pub typed_port_kept_by: String,
}

/// A resolvable topic dropped because its `cerulion_topic` is
/// already claimed by an earlier kept mapping of a DIFFERENT type. Raw DDS
/// legally allows one topic name to carry two types (two independent type
/// registrations) — but keeping both mappings would emit TWO bridge ports
/// publishing ONE Cerulion topic, a single-writer graph collision at run.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct DroppedTopicConflict {
    /// The shared (canonical) topic name.
    pub ros_topic: String,
    /// The DROPPED type.
    pub ros_type: String,
    /// The type that kept the topic.
    pub kept_ros_type: String,
}

// ─────────────────── Typed-port assignment (pure) ──────────────────

/// The outcome of racing same-type siblings for each registry type's ONE typed
/// port ([`assign_typed_ports`]).
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct TypedPortAssignment {
    /// The RE-ROUTED resolvable set: exactly one `Typed` topic per registry type
    /// (the streaming-preferred winner), every other same-type sibling flipped to
    /// `Raw` (bridged via the generic codec) when a schema can carry it. Genuine
    /// `Raw` topics pass through untouched. Losers with NO generic fallback are
    /// REMOVED (they land in [`unbridgeable`](Self::unbridgeable)). Re-sorted by
    /// `(ros_topic, ros_type)` so the mapping build + report stay deterministic
    /// regardless of discovery order. This REPLACES the caller's
    /// `Partitioned::resolvable` for [`build_mappings`] + the report.
    pub resolvable: Vec<ResolvedTopic>,
    /// Losers RESCUED onto the generic codec — for the report's note.
    pub raw_routed: Vec<RawRoutedLoser>,
    /// Losers that could NOT be raw-routed (an incomplete workspace-store
    /// shadow, or, via the caller's old-vendored-bridge degrade, an
    /// older `dds_bridge`), kept NOT bridged, loudly.
    pub unbridgeable: Vec<DroppedDuplicate>,
}

/// How "streaming" (live) a WRITER's offered durability looks, for the
/// typed-port pick. LOWER = more streaming = more preferred for the ONE typed
/// port. `Volatile` is a live stream (0). `Unknown` ranks EQUAL to `Volatile`
/// (0): RTPS ParameterList encoding OMITS default-valued QoS parameters and the
/// DDS-spec default durability IS VOLATILE — so an absent SEDP durability param
/// (surfaced as `Unknown`, e.g. by CycloneDDS on the Go2 itself) means an
/// ordinary volatile writer, and ranking it lower would split identical live
/// streams across ranks by vendor serialization
/// habit. `TransientLocal` / `Transient` / `Persistent` are LATCHED/durable
/// sources — typically a near-static map like the Go2's `/uslam/cloud_map` —
/// the LEAST preferred for a live typed port (1). This is the streaming proxy
/// standing in for the intended (unavailable) per-endpoint activity/rate
/// signal: discovery carries no observed rate, so offered durability is the
/// strongest live-vs-latched discriminator actually available.
fn durability_streaming_rank(d: QosDurability) -> u8 {
    match d {
        QosDurability::Volatile | QosDurability::Unknown => 0,
        QosDurability::TransientLocal | QosDurability::Transient | QosDurability::Persistent => 1,
    }
}

/// The streaming-preference key for the ONE typed port — the candidate that
/// sorts SMALLEST keeps the port. Order (each smaller = more preferred):
///
/// 1. **Writer presence** (`writer_count == 0`; false < true) — a data-bearing
///    topic ALWAYS beats a reader-only one. Without this a stale subscriber
///    (0 writers, volatile-REQUESTING reader) would out-rank the type's only
///    actual publisher on the reader's requested durability and bind the typed
///    port to a topic that can never carry data.
/// 2. **Writer-set durability rank** (`DiscoveredTopic::writer_durability_rank`
///    — the min [`durability_streaming_rank`] over WRITER endpoints only, an
///    order-independent aggregate: any volatile writer ⇒ live stream ⇒ 0) —
///    VOLATILE beats a latched TRANSIENT_LOCAL near-static map. This is the key
///    that saves the Go2 `/utlidar/cloud`(volatile) vs
///    `/uslam/cloud_map`(transient_local) shape, where writer counts tie
///    (1 each) so the lexicographic tie-break ALONE would wrongly keep
///    `/uslam/cloud_map` (it sorts first). Reader-requested QoS never
///    contributes.
/// 3. `Reverse(writer_count)` — more writers beat fewer (a genuinely active
///    multi-publisher topic over a single-writer sibling). The intended primary
///    signal — observed liveliness during the discovery window — is NOT carried
///    on `DiscoveredEndpoint`/`DiscoveryResult` (only writer/reader counts and
///    QoS are), so it is not invented here.
/// 4. `ros_topic` (lexicographic) — the deterministic tie-break, LAST, never
///    first: discovery order never decides.
///
/// The key is a TOTAL order (`ros_topic` is unique within a type after
/// [`aggregate_topics`], and every other component is a pure fold of the
/// endpoint multiset), so the winner is identical for ANY input permutation —
/// the determinism guarantee [`assign_typed_ports`] documents.
fn typed_port_pref_key(t: &DiscoveredTopic) -> (bool, u8, std::cmp::Reverse<usize>, &str) {
    (
        t.writer_count == 0,
        t.writer_durability_rank,
        std::cmp::Reverse(t.writer_count),
        t.ros_topic.as_str(),
    )
}

/// Why the BRIDGE's generic codec cannot SAFELY carry a REGISTRY-type
/// typed-port loser — or `None` when it can (the overwhelmingly common case).
///
/// Consulted ONLY for registry-type losers ([`assign_typed_ports`] races typed
/// groups only), and all four registry types are ALWAYS decodable by TODAY's
/// bridge generic codec: `sensor_msgs/PointCloud2` + `geometry_msgs/Twist` are
/// built-ins, and `unitree_go/SportModeState` + `unitree_api/Request` ship in
/// the bridge-embedded `UNITREE_MSGS` set
/// (`examples/go2/nodes/dds_bridge/src/generic.rs::bridge_schema_set`). So there
/// is deliberately NO resolvability gate here — `AttachSchemaChain::resolves`
/// would be the WRONG universe (store + staged + built-ins; it cannot see
/// `UNITREE_MSGS`) and would produce a factually false "the generic codec has
/// nothing to decode it with" drop for 2 of the 4 registry types on a
/// Unitree Go2 (~19 `unitree_api/Request` topics per
/// attach). When `UNITREE_MSGS` retires in favor of
/// live acquisition, update this gate together with
/// [`BRIDGE_UNITREE_MSG_NAMES`] — it must always match the bridge's ACTUAL
/// codec surface.
///
/// The ONE genuine blocker: a workspace store schema that SHADOWS the bridge's
/// own entry for the type (store wins over built-ins/`UNITREE_MSGS` at the
/// bridge — the workspace-wins shadow semantics) with an INCOMPLETE nested closure — the
/// bridge would accept the `route: raw` mapping by name at load and then fail
/// EVERY frame at CDR decode. The walk is TRANSITIVE: a store
/// shadow whose direct dep resolves to ANOTHER store schema with a missing
/// grandchild is (correctly) INCOMPLETE, because the codec decodes the whole
/// closure per frame. The walk unions [`BRIDGE_UNITREE_MSG_NAMES`] into the
/// universe so a store schema whose nested refs resolve to bridge-embedded
/// types is (correctly) complete. The returned reason carries its own precise
/// remediation (a fixed "drop the type's .msg" suffix
/// contradicted this arm — the root `.msg` is already in the store; the missing
/// NESTED member, or removing the shadow, is the actual fix).
fn raw_fallback_blocker(chain: &AttachSchemaChain, ros_type: &str) -> Option<String> {
    if let Some(missing) = chain.store_closure_missing_with(ros_type, BRIDGE_UNITREE_MSG_NAMES) {
        return Some(format!(
            "the workspace .msg store schema for {ros_type} SHADOWS the bridge's own schema but \
             its nested closure is INCOMPLETE (missing {}) — the bridge would accept the mapping \
             and then fail every frame at CDR decode. Add the missing .msg(s) to \
             schemas/<pkg>/msg/, or remove the incomplete store schema so the bridge's embedded \
             schema serves the type",
            missing.join(", ")
        ));
    }
    None
}

/// Race same-type siblings for each registry type's ONE typed port.
///
/// Bridge v1 binds each of the four registry types ([`V1_BRIDGE_BINDINGS`]) to
/// ONE fixed macro port. Without a port race the FIRST-discovered (lexicographically
/// smallest) topic of a type claims the port and every sibling is DROPPED from
/// the bridge — on a Unitree Go2, `/uslam/cloud_map` alphabetically
/// steals the `PointCloud2` port and the live `/utlidar/cloud` lidar vanishes from
/// the wire. This function does two things instead:
///
/// 1. **Streaming preference** for the ONE typed port (`typed_port_pref_key`):
///    the topic MOST LIKELY to be a live stream keeps the port, not the
///    discovery order — writer presence first (a data-bearing topic always
///    beats a reader-only one), then the writer-set min durability rank
///    (VOLATILE/absent-param > latched), then writer count, lexicographic
///    tie-break LAST.
/// 2. **Raw-route the losers**: every other same-type sibling degrades to the
///    generic raw codec ([`BridgeRoute::Raw`]) on its OWN absolute Cerulion
///    topic — never dropped. All four registry types are ALWAYS decodable by
///    the bridge's generic codec (built-ins for PointCloud2/Twist; the
///    bridge-embedded `UNITREE_MSGS` for SportModeState/Request), so the only
///    loser kept NOT bridged here is one whose type is SHADOWED by an
///    INCOMPLETE workspace store schema (`raw_fallback_blocker`) — loudly,
///    with the precise remediation ([`DroppedDuplicate`]).
///
/// Genuine `Raw` topics (non-registry types already routed raw by the partition)
/// pass through untouched. DETERMINISM: the winner selection is order-independent
/// (total-order key over pure endpoint-multiset folds) and the outputs are
/// re-sorted, so the SAME discovered set in ANY input order — raw endpoint order
/// AND `resolvable` order — yields the SAME assignment.
pub fn assign_typed_ports(
    chain: &AttachSchemaChain,
    resolvable: Vec<ResolvedTopic>,
) -> TypedPortAssignment {
    use indexmap::IndexMap;
    // Group the TYPED candidates by ros_type; genuine Raw topics ride straight
    // into the bridged accumulator (their route is already correct).
    let mut typed_groups: IndexMap<String, Vec<ResolvedTopic>> = IndexMap::new();
    let mut bridged: Vec<ResolvedTopic> = Vec::new();
    for r in resolvable {
        match r.route {
            BridgeRoute::Typed(_) => typed_groups
                .entry(r.topic.ros_type.clone())
                .or_default()
                .push(r),
            BridgeRoute::Raw => bridged.push(r),
        }
    }

    let mut raw_routed: Vec<RawRoutedLoser> = Vec::new();
    let mut unbridgeable: Vec<DroppedDuplicate> = Vec::new();
    for (_ros_type, group) in typed_groups {
        // The streaming-preferred winner (order-independent — total-order key).
        let winner = group
            .iter()
            .min_by(|a, b| typed_port_pref_key(&a.topic).cmp(&typed_port_pref_key(&b.topic)))
            .expect("a typed group always has ≥1 member")
            .topic
            .ros_topic
            .clone();
        for r in group {
            // ros_topic is unique within a ros_type group (aggregate_topics keys
            // by (ros_topic, ros_type)), so this identifies the winner exactly.
            if r.topic.ros_topic == winner {
                bridged.push(r); // keeps its Typed route + the fixed macro port
                continue;
            }
            // A loser: rescue onto the generic codec when a schema can carry it.
            match raw_fallback_blocker(chain, &r.topic.ros_type) {
                None => {
                    tracing::warn!(
                        ros_topic = %r.topic.ros_topic,
                        ros_type = %r.topic.ros_type,
                        typed_port_kept_by = %winner,
                        "ros2 attach: same-type sibling lost the race for the typed port — riding \
                         the generic codec on its own topic instead of being dropped (a \
                         per-instance typed port is the eventual home)"
                    );
                    raw_routed.push(RawRoutedLoser {
                        ros_topic: r.topic.ros_topic.clone(),
                        ros_type: r.topic.ros_type.clone(),
                        typed_port_kept_by: winner.clone(),
                    });
                    // Flip the route so the report + mapping build see it as raw.
                    bridged.push(ResolvedTopic {
                        topic: r.topic,
                        route: BridgeRoute::Raw,
                    });
                }
                Some(reason) => {
                    tracing::warn!(
                        ros_topic = %r.topic.ros_topic,
                        ros_type = %r.topic.ros_type,
                        typed_port_kept_by = %winner,
                        reason = %reason,
                        "ros2 attach: same-type sibling lost the typed port AND its raw fallback is \
                         blocked — kept NOT bridged"
                    );
                    unbridgeable.push(DroppedDuplicate {
                        dds_topic: r.topic.ros_topic.clone(),
                        ros_type: r.topic.ros_type.clone(),
                        kept_dds_topic: winner.clone(),
                        raw_fallback_reason: reason,
                    });
                }
            }
        }
    }

    // Deterministic downstream order for the mapping build + the report.
    bridged.sort_by(|a, b| {
        a.topic
            .ros_topic
            .cmp(&b.topic.ros_topic)
            .then_with(|| a.topic.ros_type.cmp(&b.topic.ros_type))
    });
    raw_routed.sort_by(|a, b| {
        a.ros_topic
            .cmp(&b.ros_topic)
            .then_with(|| a.ros_type.cmp(&b.ros_type))
    });
    unbridgeable.sort_by(|a, b| {
        a.dds_topic
            .cmp(&b.dds_topic)
            .then_with(|| a.ros_type.cmp(&b.ros_type))
    });
    TypedPortAssignment {
        resolvable: bridged,
        raw_routed,
        unbridgeable,
    }
}

/// Does the workspace's VENDORED `dds_bridge` node
/// source understand the `route:` mapping field? The node type is hand-copied
/// into workspaces (the attach flow only checks the directory exists), and an
/// OLDER copy's `#[serde(deny_unknown_fields)]` `TopicMapping` rejects
/// the WHOLE config on an unknown `route` key at graph run — zero topics
/// bridged, strictly worse than dropping the loser. Probe: does
/// `nodes/dds_bridge/src/config.rs` mention `RouteMode` (the enum every
/// newer copy declares as the field's type)?
///
/// The failure shapes are DELIBERATELY split (and the dir stat is strict,
/// so an error is never mistaken for absence):
/// - **Bridge PROVABLY absent** (`nodes/dds_bridge/` stats as `NotFound`) ⇒
///   `true` (fail OPEN): the workspace has no bridge yet — the operator copies
///   the CURRENT node type from the repo (the generated graph's header
///   instructs exactly that), which speaks `route:`.
/// - **The dir stat fails for any OTHER reason** (EACCES on the workspace /
///   nodes dir), OR `nodes/dds_bridge` is present but NOT a directory (a stray
///   FILE) ⇒ `false` (fail CLOSED, loudly): a non-`NotFound` stat error is not
///   proof of absence (a still-runnable older bridge may be behind the
///   permission churn), so fail OPEN requires PROVABLE absence, never a
///   dir-stat error.
/// - **The node dir EXISTS but `config.rs` is unreadable** (EACCES/ACL churn,
///   or a half-copied bridge missing just this file) ⇒ `false` (fail CLOSED,
///   loudly): an OLDER prebuilt cdylib may still run and reject
///   `route: raw` WHOLESALE at graph run. Degrading is never worse than the
///   original drop; writing an unloadable config is. This preserves the ratified
///   floor: never write a `route: raw` the old bridge would reject.
pub fn vendored_bridge_supports_route_mode(workspace_root: &Path) -> bool {
    let bridge_dir = workspace_root.join("nodes").join("dds_bridge");
    // Fail OPEN only when the vendored bridge is PROVABLY absent (a `NotFound`
    // stat). A dir-stat error other than `NotFound` (EACCES on the workspace /
    // nodes dir) is NOT proof of absence (a still-runnable older bridge may
    // be behind the permission churn), and a FILE named `dds_bridge` is a weird
    // half-state; both fail CLOSED into the loud degrade
    // rather than risk a `route: raw` the old bridge rejects wholesale.
    // (Deliberately does NOT reuse `workspace_has_node_type` — its `is_dir()`
    // maps every stat error to `false`, which would fail OPEN on an EACCES.)
    match std::fs::metadata(&bridge_dir) {
        Ok(m) if m.is_dir() => {}
        Err(e) if e.kind() == std::io::ErrorKind::NotFound => return true,
        Ok(_) => {
            tracing::warn!(
                bridge_dir = %bridge_dir.display(),
                "ros2 attach: nodes/dds_bridge exists but is NOT a directory — treating the \
                 vendored bridge as an older copy without `route: raw` support; same-type \
                 siblings that lost the typed-port race \
                 will be degraded (NOT bridged). Re-copy nodes/dds_bridge from the Cerulion repo \
                 (examples/go2/nodes/dds_bridge/) to restore raw routing"
            );
            return false;
        }
        Err(e) => {
            tracing::warn!(
                bridge_dir = %bridge_dir.display(),
                error = %e,
                "ros2 attach: nodes/dds_bridge could not be stat'd (a non-NotFound error is not \
                 proof of absence) — treating the vendored bridge as an older copy without \
                 `route: raw` support; same-type \
                 siblings that lost the typed-port race will be degraded (NOT bridged). Re-copy \
                 nodes/dds_bridge from the Cerulion repo (examples/go2/nodes/dds_bridge/) to restore \
                 raw routing"
            );
            return false;
        }
    }
    let probe = bridge_dir.join("src").join("config.rs");
    match std::fs::read_to_string(&probe) {
        Ok(src) => src.contains("RouteMode"),
        // The bridge dir exists but its config.rs could not be read — a
        // half-copied bridge (NotFound of just this file) or an ACL/EACCES
        // churn over a still-runnable older cdylib. Fail CLOSED into the
        // loud degrade path rather than risk a `route: raw` the old bridge
        // rejects wholesale.
        Err(e) => {
            tracing::warn!(
                probe = %probe.display(),
                error = %e,
                "ros2 attach: a vendored dds_bridge node EXISTS but its config.rs could not be \
                 read — treating the vendored bridge as an older copy without `route: raw` \
                 support; same-type siblings that lost \
                 the typed-port race will be degraded (NOT bridged). Re-copy nodes/dds_bridge from \
                 the Cerulion repo (examples/go2/nodes/dds_bridge/) to restore raw routing"
            );
            false
        }
    }
}

/// The doomed-run guard for an OLDER vendored
/// bridge: strip every raw-routed loser back OUT of the bridged set (the
/// original drop, WITH the real reason) instead of emitting a `route: raw` the
/// old bridge's config parse would reject WHOLESALE. A degraded old-bridge
/// attach is never WORSE than the original drop: the winner + every other topic still
/// bridge; only the losers drop, each named in PORT CONFLICTS with the re-copy
/// remediation. Pure — the caller decides via
/// [`vendored_bridge_supports_route_mode`].
fn degrade_raw_routed_for_old_bridge(assignment: TypedPortAssignment) -> TypedPortAssignment {
    let TypedPortAssignment {
        resolvable,
        raw_routed,
        mut unbridgeable,
    } = assignment;
    // Remove exactly the FLIPPED Raw entries (raw_routed identifies them by
    // (ros_topic, ros_type), unique per aggregate_topics); genuine raw topics
    // and typed winners pass through untouched.
    let resolvable: Vec<ResolvedTopic> = resolvable
        .into_iter()
        .filter(|r| {
            !(matches!(r.route, BridgeRoute::Raw)
                && raw_routed
                    .iter()
                    .any(|l| l.ros_topic == r.topic.ros_topic && l.ros_type == r.topic.ros_type))
        })
        .collect();
    for l in raw_routed {
        tracing::warn!(
            ros_topic = %l.ros_topic,
            ros_type = %l.ros_type,
            typed_port_kept_by = %l.typed_port_kept_by,
            "ros2 attach: typed-port loser NOT bridged — the workspace's vendored dds_bridge \
             does not support `route: raw` and would reject the whole generated config; \
             re-copy nodes/dds_bridge from the Cerulion repo to bridge it via the generic codec"
        );
        unbridgeable.push(DroppedDuplicate {
            dds_topic: l.ros_topic,
            ros_type: l.ros_type,
            kept_dds_topic: l.typed_port_kept_by,
            raw_fallback_reason: OLD_BRIDGE_DEGRADE_REASON.to_string(),
        });
    }
    unbridgeable.sort_by(|a, b| {
        a.dds_topic
            .cmp(&b.dds_topic)
            .then_with(|| a.ros_type.cmp(&b.ros_type))
    });
    TypedPortAssignment {
        resolvable,
        raw_routed: Vec::new(),
        unbridgeable,
    }
}

/// The [`DroppedDuplicate::raw_fallback_reason`] for the old-vendored-bridge
/// degrade arm — one const so the report line and
/// its test oracle cannot drift.
const OLD_BRIDGE_DEGRADE_REASON: &str =
    "this workspace's vendored dds_bridge node does not support `route: raw` and would \
     reject the whole generated config; re-copy nodes/dds_bridge from the Cerulion repo \
     (examples/go2/nodes/dds_bridge/), then re-run attach to bridge it via the generic codec";

/// Build the canonical mapping list from the (already typed-port-assigned)
/// resolvable set.
///
/// ONE dedup pass — by `cerulion_topic`, ALL routes (one DDS topic
/// name carrying two types would collide two publishers onto one single-writer
/// Cerulion topic; the bridge config rejects duplicate raw topics too). Losers land
/// in [`DroppedTopicConflict`].
///
/// The per-type TYPED-port race (each registry type owns ONE fixed port) is NO
/// LONGER done here — it moved to [`assign_typed_ports`], which the
/// caller runs FIRST so this input already carries at most one `Typed` topic per
/// registry type (with same-type losers re-routed to `Raw` or dropped). Calling
/// `build_mappings` on a NON-assigned set with two `Typed` siblings of one type
/// would emit two mappings for the same fixed port (the graph generator would
/// silently pick the first) — production always assigns first, and the pure tests
/// that need the race call [`assign_typed_ports`] before this.
///
/// DETERMINISTIC keep rule: `resolvable` arrives sorted by `(ros_topic, ros_type)`
/// so for a shared `cerulion_topic` the lexicographically-smallest type keeps the
/// topic. `cerulion_topic` = `topic_prefix` + ros_topic (prefix `None` ⇒ mirror).
pub fn build_mappings(
    resolvable: &[ResolvedTopic],
    topic_prefix: Option<&str>,
) -> (Vec<BridgeMapping>, Vec<DroppedTopicConflict>) {
    let mut mappings: Vec<BridgeMapping> = Vec::new();
    let mut topic_conflicts: Vec<DroppedTopicConflict> = Vec::new();
    for r in resolvable {
        let cerulion_topic = candidate_cerulion_topic(topic_prefix, &r.topic.ros_topic);
        // One mapping per cerulion_topic.
        if let Some(kept_ros_type) = mappings
            .iter()
            .find(|m| m.cerulion_topic == cerulion_topic)
            .map(|m| m.ros_type.clone())
        {
            topic_conflicts.push(DroppedTopicConflict {
                ros_topic: r.topic.ros_topic.clone(),
                ros_type: r.topic.ros_type.clone(),
                kept_ros_type,
            });
            continue;
        }
        mappings.push(BridgeMapping {
            dds_topic: r.topic.ros_topic.clone(),
            ros_type: r.topic.ros_type.clone(),
            cerulion_topic,
            route: r.route,
        });
    }
    (mappings, topic_conflicts)
}

// ───────────────── Schema acquisition ladder ──────────────

/// One `.msg` file staged in memory for materialization on consent. The bytes
/// are held (not written) until the consent gate; the re-resolve pass treats
/// its type as resolvable via [`AttachSchemaChain::stage_all`] in the meantime
/// (the never-mutate-before-consent floor).
#[derive(Debug, Clone, PartialEq, Eq)]
struct StagedMsgFile {
    /// Workspace-relative store path `schemas/<pkg>/msg/<Type>.msg` (forward
    /// slashes) — what the report + preview NAME, and the dedup/sort key.
    workspace_rel: String,
    /// Qualified `pkg/Type` this file defines.
    qualified: String,
    /// The `<pkg>` path segment.
    package: String,
    /// The bare `<Type>` (file stem).
    type_name: String,
    /// The `.msg` FILE contents: the acquired payload byte-verbatim except for
    /// exactly one restored trailing newline (`restore_trailing_newline`,
    /// since rcl's REP-2011 `type_sources` embed the text without
    /// the final `'\n'`; the in-memory closure stays wire-verbatim).
    text: String,
    /// Which rung produced it (report provenance).
    rung_label: &'static str,
}

/// The result of running the acquisition ladder over the UNRESOLVABLE set —
/// everything staged IN MEMORY. Nothing here has touched the filesystem; the
/// consent gate materializes [`to_write`](Self::to_write).
#[derive(Debug, Clone, Default)]
struct AcquisitionReport {
    /// The deduped `.msg` files to materialize on consent — the full nested
    /// closure across every acquired type that is NOT already resolvable via
    /// the store/builtins (a built-in nested dep is served from the corpus,
    /// never duplicated into the store). Sorted by store path (deterministic).
    /// The RESOLVABLE-line acquired marker keys off THIS set:
    /// a type staged as a closure sibling of another bundle,
    /// not just a requested root, carries the acquired marker too.
    to_write: Vec<StagedMsgFile>,
    /// Per unresolvable-but-ATTEMPTED type: the loud skip/failure reason lines
    /// (per-rung reasons, partial-closure "missing nested type" reasons, and
    /// cross-bundle CONFLICT-refusal reasons), surfaced under the UNRESOLVABLE
    /// report line. A type the acquirer never touched (the [`NoopAcquirer`]
    /// default) has NO entry — so its line stays byte-identical to the
    /// no-acquisition report. A type that ultimately STAGED (became resolvable) is
    /// pruned from here so its reasons never leak under an UNRESOLVABLE line it
    /// no longer occupies.
    skip_reasons: BTreeMap<String, Vec<String>>,
}

impl AcquisitionReport {
    /// The empty report — no acquisition happened. The legacy no-acquirer
    /// render path passes this, keeping the report byte-identical to today.
    fn none() -> Self {
        Self::default()
    }

    /// The qualified names to STAGE into the chain for the re-resolve pass —
    /// every to-be-written closure member (the already-resolvable members
    /// resolve via store/builtins and need no staging).
    fn newly_resolvable(&self) -> impl Iterator<Item = &str> {
        self.to_write.iter().map(|f| f.qualified.as_str())
    }

    /// The store path a type's OWN `.msg` writes on consent IF it became
    /// resolvable via acquisition this run — keyed off the ACTUAL to-write set,
    /// so a requested root AND a closure sibling staged through another bundle
    /// both resolve here. `None` for a type not
    /// staged this run (store/builtin/typed-resolved).
    fn staged_path(&self, ros_type: &str) -> Option<&str> {
        self.to_write
            .iter()
            .find(|f| f.qualified == ros_type)
            .map(|f| f.workspace_rel.as_str())
    }
}

/// Run the acquisition ladder over the partition's UNRESOLVABLE set,
/// DEDUPED by type — two topics sharing a type = ONE acquisition.
/// `chain` is the PRE-acquisition chain (store + builtins), consulted for the
/// completeness check and to avoid staging types already resolvable.
/// `discovery` is the engine's SINGLE discovery result, threaded through so the
/// wire rung CONSUMES the SAME harvest the report prints (no second discovery
/// window); every other rung ignores it. Returns everything staged in memory;
/// no file is written here.
fn run_acquisition_ladder(
    acquirer: &dyn SchemaAcquirer,
    chain: &AttachSchemaChain,
    unresolvable: &[DiscoveredTopic],
    discovery: &DiscoveryResult,
) -> AcquisitionReport {
    // Dedupe by type, deterministically. Two topics of the same type acquire
    // ONCE; an empty set never invokes the acquirer at all.
    let mut wanted: Vec<String> = unresolvable.iter().map(|t| t.ros_type.clone()).collect();
    wanted.sort();
    wanted.dedup();
    if wanted.is_empty() {
        return AcquisitionReport::none();
    }

    // Thread the engine's SINGLE discovery result through the ladder — the wire
    // rung CONSUMES it (no second discovery window); every other rung ignores it.
    let outcomes = acquirer.acquire(&wanted, discovery);
    let wanted_set: BTreeSet<&str> = wanted.iter().map(String::as_str).collect();
    // Canonicalize the acquirer response.
    // The acquirer is a TRUST BOUNDARY (rungs 3/5 read remote data), so
    // every contract violation is LOUD, never a silent vanish or silent
    // first-win: an UNSOLICITED type (not requested) is rejected, and a
    // DUPLICATE requested key keeps the first but warns on the discard.
    let mut by_type: BTreeMap<String, AcquisitionOutcome> = BTreeMap::new();
    for ta in outcomes {
        if !wanted_set.contains(ta.requested.as_str()) {
            tracing::warn!(
                unsolicited = %ta.requested,
                "ros2 attach: acquirer returned a type that was not requested — ignored (never \
                 staged)"
            );
            continue;
        }
        match by_type.entry(ta.requested.clone()) {
            std::collections::btree_map::Entry::Vacant(e) => {
                e.insert(ta.outcome);
            }
            std::collections::btree_map::Entry::Occupied(_) => {
                tracing::warn!(
                    ros_type = %ta.requested,
                    "ros2 attach: acquirer returned duplicate results for one requested type — \
                     keeping the first, discarding the rest"
                );
            }
        }
    }

    let mut report = AcquisitionReport::default();
    // First phase: process each acquired bundle into a self-contained staging
    // (complete + valid) or record its skip reason. The cross-bundle merge
    // (phase 2) detects byte-different duplicate members before anything is
    // marked resolvable.
    let mut stagings: Vec<BundleStaging> = Vec::new();
    for ros_type in &wanted {
        match by_type.get(ros_type) {
            // Not attempted (the NoopAcquirer / partial-result case): stays
            // unresolvable with NO extra report line — byte-identical to today.
            None => {}
            Some(AcquisitionOutcome::Skipped(reasons)) => {
                report
                    .skip_reasons
                    .insert(ros_type.clone(), skip_reason_lines(ros_type, reasons));
            }
            Some(AcquisitionOutcome::Acquired(schema)) => {
                if let Some(st) = process_acquired(ros_type, schema, chain, &mut report) {
                    stagings.push(st);
                }
            }
        }
    }
    // Second phase: merge the bundles, refusing cross-bundle conflicts, and fill
    // `report.to_write` from the survivors.
    merge_stagings(stagings, &mut report);
    // A wanted type that resolves BY NAME via
    // the store but whose store schema is INCOMPLETE (the partition flipped it
    // into this ladder's input) carries the precise missing-dep report line
    // even when no rung attempted it (the NoopAcquirer arm records nothing).
    // If this ladder run just staged the missing member(s), the post-staging
    // re-resolve heals the type and this line is never rendered (skip lines
    // render only under the UNRESOLVABLE loop).
    for t in &wanted {
        if let Some(missing) = chain.store_closure_missing(t) {
            report
                .skip_reasons
                .entry(t.clone())
                .or_default()
                .push(format!(
                    "resolves by name via the workspace .msg store, but the store schema is \
                 INCOMPLETE — references {}, which is neither in the store nor the built-in \
                 ROS 2 corpus; NOT bridged (drop the missing .msg into schemas/<pkg>/msg/, \
                 or fix the store file)",
                    missing.join(", ")
                ));
        }
    }
    // A type that ended up STAGED (resolvable via acquisition — a requested
    // root OR a closure sibling) must not also carry stale skip lines: those
    // render ONLY under the UNRESOLVABLE loop, which it has left.
    // Drop them so nothing leaks.
    let staged: BTreeSet<String> = report
        .to_write
        .iter()
        .map(|f| f.qualified.clone())
        .collect();
    report
        .skip_reasons
        .retain(|ros_type, _| !staged.contains(ros_type));
    // Deterministic order for the preview/report file list.
    report
        .to_write
        .sort_by(|a, b| a.workspace_rel.cmp(&b.workspace_rel));
    report
}

/// Build the loud per-rung skip lines for a `Skipped` outcome.
/// Each rung's reason fires a `warn!` and a report line;
/// an EMPTY reason string is a rung contract violation surfaced with a
/// rung-named placeholder, and an EMPTY reasons list (`Skipped(vec![])`) yields
/// a single synthesized placeholder — an attempted-and-failed acquisition is
/// NEVER invisible.
fn skip_reason_lines(ros_type: &str, reasons: &[cerulion_dds::RungSkip]) -> Vec<String> {
    if reasons.is_empty() {
        tracing::warn!(
            ros_type = %ros_type,
            "ros2 attach: acquirer skipped a type but recorded NO reason (rung contract \
             violation) — surfacing a placeholder"
        );
        return vec![
            "acquisition attempted but the acquirer recorded no reason (rung contract violation)"
                .to_string(),
        ];
    }
    let mut lines = Vec::with_capacity(reasons.len());
    for r in reasons {
        let label = r.rung.label();
        let reason = if r.reason.trim().is_empty() {
            tracing::warn!(
                ros_type = %ros_type,
                rung = %label,
                "ros2 attach: schema-acquisition rung recorded an EMPTY skip reason — placeholder"
            );
            format!("{label}: (no reason recorded — rung contract violation)")
        } else {
            tracing::warn!(
                ros_type = %ros_type,
                rung = %label,
                reason = %r.reason,
                "ros2 attach: schema-acquisition rung skipped an unresolved type"
            );
            format!("{label}: {}", r.reason)
        };
        lines.push(reason);
    }
    lines
}

/// The staged output of ONE complete, valid acquired bundle:
/// the requested root + the genuinely-new closure members it
/// would materialize. Bundles are self-contained (the acquirer returns each
/// type's FULL closure), so the cross-bundle merge ([`merge_stagings`]) can
/// resolve byte-different duplicates deterministically without rolling back a
/// half-committed report.
struct BundleStaging {
    /// The requested type (bundle root) this staging resolves.
    root: String,
    /// Genuinely-new closure members (not already store/builtin resolvable), in
    /// bundle order.
    members: Vec<StagedMsgFile>,
}

/// Process ONE `Acquired` bundle: VALIDATE every member (trust boundary), parse
/// the closure, verify it is COMPLETE (the partial-closure check), and — only if
/// valid + complete — return the genuinely-new members as a [`BundleStaging`].
/// Any failure (invalid identifier / empty body, parse error, missing requested
/// type, incomplete closure) is surfaced LOUDLY, the type stays UNRESOLVABLE
/// with a precise reason (recorded in `report.skip_reasons`), and `None` is
/// returned — never a silent half-resolved claim, and nothing is marked
/// resolvable here (the merge does that after conflict detection).
fn process_acquired(
    root: &str,
    schema: &cerulion_dds::AcquiredSchema,
    chain: &AttachSchemaChain,
    report: &mut AcquisitionReport,
) -> Option<BundleStaging> {
    let rung = schema.rung.label();

    // 0. VALIDATE every acquired member BEFORE anything else.
    //    The acquirer is a TRUST BOUNDARY (rungs 3/5 read remote
    //    data), and the consent write joins `package`/`type_name` straight into
    //    the store path `schemas/<pkg>/msg/<Type>.msg`: a bad identifier could
    //    steer that write outside `schemas/` (`..`, `/`, absolute) or produce a
    //    file the store reader can never enumerate, and an empty body is a
    //    silent zero-field schema. ANY bad member fails the whole bundle;
    //    nothing stages.
    for m in &schema.closure {
        if let Err(why) = validate_acquired_member(m) {
            let reason = format!(
                "acquired via {rung} but the returned bundle is INVALID — {why}; NOT bridged \
                 (the acquiring rung must return valid ROS package/type identifiers and a \
                 non-empty .msg body, or drop a valid .msg into schemas/<pkg>/msg/ by hand)"
            );
            tracing::warn!(
                ros_type = %root,
                rung = %rung,
                package = %m.package,
                type_name = %m.type_name,
                reason = %why,
                "ros2 attach: acquired bundle member failed validation — kept UNRESOLVABLE"
            );
            report.skip_reasons.insert(root.to_string(), vec![reason]);
            return None;
        }
    }

    // 1. Parse every closure member with the SAME parser the store uses.
    let mut parsed: Vec<MessageSchema> = Vec::with_capacity(schema.closure.len());
    for m in &schema.closure {
        match parse_rosmsg(&m.msg_text, &m.type_name, Some(m.package.as_str())) {
            Ok(s) => parsed.push(s),
            Err(e) => {
                let reason = format!(
                    "acquired via {rung} but the returned .msg for {}/{} did not parse ({e}); \
                     NOT bridged (fix the acquiring rung, or drop a valid .msg into \
                     schemas/{}/msg/ by hand and re-run)",
                    m.package, m.type_name, m.package
                );
                tracing::warn!(
                    ros_type = %root,
                    member = %m.qualified_name(),
                    rung = %rung,
                    error = ?e,
                    "ros2 attach: acquired .msg failed to parse — type kept UNRESOLVABLE"
                );
                report.skip_reasons.insert(root.to_string(), vec![reason]);
                return None;
            }
        }
    }

    // 2. The closure MUST include the requested type itself.
    let closure_names: BTreeSet<String> =
        schema.closure.iter().map(|m| m.qualified_name()).collect();
    if !closure_names.contains(root) {
        let reason = format!(
            "acquired via {rung} but the returned bundle does not include the requested type \
             {root} itself; NOT bridged (fix the acquiring rung, or drop a valid .msg into \
             schemas/ by hand and re-run)"
        );
        tracing::warn!(
            ros_type = %root,
            rung = %rung,
            "ros2 attach: acquired bundle omits the requested type — kept UNRESOLVABLE"
        );
        report.skip_reasons.insert(root.to_string(), vec![reason]);
        return None;
    }

    // 3. Completeness: every referenced nested type must resolve through the
    //    codec's canonical ladder ([`resolve_nested_qname_ladder`]) against the
    //    RESOLVABLE UNIVERSE (this bundle's closure ∪ the workspace store ∪ the
    //    in-memory staging set ∪ the built-in corpus) — the SAME ladder the CDR
    //    decoder will run, so a complete closure using ROS 2's legal bare
    //    `Header` / unambiguous same-suffix references is ACCEPTED (not
    //    wrongly rejected), and a genuine gap is a partial closure (loud +
    //    explicit, type stays UNRESOLVABLE).
    let mut universe = chain.resolvable_universe();
    universe.extend(closure_names.iter().cloned());
    let mut missing: BTreeSet<String> = BTreeSet::new();
    for s in &parsed {
        for r in nested_dependencies(s) {
            if resolve_nested_ref(&r, &universe).is_none() {
                missing.insert(missing_ref_label(&r, &universe));
            }
        }
    }
    if !missing.is_empty() {
        let missing_list = missing.iter().cloned().collect::<Vec<_>>().join(", ");
        let reason = format!(
            "acquired via {rung} but its nested closure is INCOMPLETE — references {missing_list}, \
             which is neither in the acquired bundle, the workspace .msg store, nor the built-in \
             ROS 2 corpus; NOT bridged (acquire the missing type too, or drop its .msg into \
             schemas/)"
        );
        tracing::warn!(
            ros_type = %root,
            rung = %rung,
            missing = %missing_list,
            "ros2 attach: acquired schema has an INCOMPLETE nested closure — kept UNRESOLVABLE"
        );
        report.skip_reasons.insert(root.to_string(), vec![reason]);
        return None;
    }

    // 4. Complete — collect each genuinely-new closure member (a member already
    //    served by the store/builtins is NOT duplicated, so std_msgs/Header &
    //    friends never shadow a builtin — the never-clobber floor, pinned by
    //    local_harvest_test's store-twin test). A member that COLLIDES with a
    //    built-in but carries DIFFERENT bytes is surfaced LOUDLY:
    //    the authoritative built-in wins (as
    //    designed), but the divergence is never silent.
    let mut members = Vec::new();
    for m in &schema.closure {
        let q = m.qualified_name();
        if chain.resolves(&q) {
            warn_on_builtin_text_divergence(&q, &m.msg_text);
            continue;
        }
        members.push(StagedMsgFile {
            workspace_rel: format!("schemas/{}/msg/{}.msg", m.package, m.type_name),
            qualified: q,
            package: m.package.clone(),
            type_name: m.type_name.clone(),
            // Restore POSIX text-file form HERE — the
            // materialization content seam — so the preview, the cross-bundle
            // duplicate compare, and the file write all see the same bytes
            // (the closure's `msg_text` stays wire-verbatim).
            text: restore_trailing_newline(&m.msg_text),
            rung_label: schema.rung.label(),
        });
    }
    Some(BundleStaging {
        root: root.to_string(),
        members,
    })
}

/// Restore POSIX text-file form on an acquired `.msg` payload
/// at the MATERIALIZATION seam. rcl's REP-2011 `type_sources[].raw_file_contents`
/// embeds the `.msg` text WITHOUT the final newline (live-observed: the response
/// payload ends `...string label` bare), so writing the payload literally
/// produced no-newline-at-EOF store files. Rule: if the payload does not end
/// with `'\n'`, append exactly one; if it does, pass it through byte-unchanged
/// (never doubled) — content bytes otherwise verbatim. The result is
/// CONTENT-identical, single-final-newline form — NOT guaranteed byte-identical
/// to the robot's original: rcl strips exactly ONE final newline, so
/// a robot original ending in extra blank lines (`x\n\n` arrives as `x\n`; the
/// vendored `std_msgs/Header` itself ends `0a0a`) is NORMALIZED — the wire
/// destroys that distinction, unrecoverable at any seam. Applied where the
/// staged FILE text is built (so the preview, the cross-bundle duplicate
/// compare, and the write all agree); the in-memory closure
/// (`AcquiredMsg::msg_text`) stays wire-verbatim. An EMPTY payload passes
/// through unchanged — not invented content; it can never reach the write
/// anyway (`validate_acquired_member` rejects empty/whitespace-only bodies
/// upstream). Pure — oracle-tested.
fn restore_trailing_newline(text: &str) -> String {
    if text.is_empty() || text.ends_with('\n') {
        text.to_string()
    } else {
        format!("{text}\n")
    }
}

/// Newline-form-insensitive equality for the cross-bundle
/// duplicate compare. rcl strips exactly ONE final newline from every wire
/// payload (see `restore_trailing_newline`), so two rungs can legitimately
/// stage the SAME schema differing only in trailing newlines (a wire-restored
/// `x\n` vs a local-ament read of a blank-line-terminated original `x\n\n`). A
/// trailing-newline delta is NOT a layout divergence, so it must not trip the
/// content-conflict REFUSAL; every leading/interior byte still compares
/// exactly. Pure — oracle-tested.
fn eq_ignoring_trailing_newlines(a: &str, b: &str) -> bool {
    a.trim_end_matches('\n') == b.trim_end_matches('\n')
}

/// Merge per-bundle stagings into the final write set, detecting cross-bundle
/// CONTENT-different duplicates (the
/// compare ignores trailing-newline-only deltas — `eq_ignoring_trailing_newlines`
/// — since rcl's one-newline strip makes those legitimately coexist across
/// rungs). Identical-content duplicate members dedupe QUIETLY with the
/// FIRST-seen staged text winning the write (as ever); a genuinely
/// content-different duplicate of a shared qualified name REFUSES every
/// requested type whose bundle carries it (deterministic + loud — neither
/// divergent copy ships, since shipping either layout could silently decode
/// the other type's traffic with wrong offsets). Survivors' `.msg` set is
/// exactly their own self-contained bundle's members, deduped in wanted order.
fn merge_stagings(stagings: Vec<BundleStaging>, report: &mut AcquisitionReport) {
    // First staged text per qualified name + which roots carry it + the
    // qualified names seen with two DIFFERENT texts.
    let mut text_of: BTreeMap<String, String> = BTreeMap::new();
    let mut roots_of: BTreeMap<String, BTreeSet<String>> = BTreeMap::new();
    let mut conflicted: BTreeSet<String> = BTreeSet::new();
    for st in &stagings {
        for m in &st.members {
            roots_of
                .entry(m.qualified.clone())
                .or_default()
                .insert(st.root.clone());
            match text_of.entry(m.qualified.clone()) {
                std::collections::btree_map::Entry::Vacant(e) => {
                    e.insert(m.text.clone());
                }
                std::collections::btree_map::Entry::Occupied(e) => {
                    // Newline-form-insensitive — a trailing-newline
                    // delta is not a layout divergence (first-seen text wins).
                    if !eq_ignoring_trailing_newlines(e.get(), &m.text) {
                        conflicted.insert(m.qualified.clone());
                    }
                }
            }
        }
    }
    // Every root whose bundle carries a conflicted member is REFUSED.
    let mut refused: BTreeSet<String> = BTreeSet::new();
    for q in &conflicted {
        if let Some(roots) = roots_of.get(q) {
            refused.extend(roots.iter().cloned());
        }
    }
    // Loud refusal + skip reason per refused root, naming the conflicting types.
    for st in &stagings {
        if !refused.contains(&st.root) {
            continue;
        }
        let bad: Vec<&str> = st
            .members
            .iter()
            .map(|m| m.qualified.as_str())
            .filter(|q| conflicted.contains(*q))
            .collect();
        let reason = format!(
            "acquired but REFUSED — the shared nested type(s) {} were returned with \
             BYTE-DIFFERENT definitions across acquired bundles; shipping either layout could \
             silently decode the other type's traffic with wrong field offsets; NOT bridged \
             (acquire these types from a single consistent source, or drop one authoritative \
             .msg into schemas/ by hand)",
            bad.join(", ")
        );
        tracing::warn!(
            ros_type = %st.root,
            conflicts = %bad.join(", "),
            "ros2 attach: acquired bundle REFUSED — cross-bundle byte-different schema conflict"
        );
        report
            .skip_reasons
            .entry(st.root.clone())
            .or_default()
            .push(reason);
    }
    // Survivors: collect their deduped members into `to_write`.
    let mut seen: BTreeSet<String> = BTreeSet::new();
    for st in stagings {
        if refused.contains(&st.root) {
            continue;
        }
        for m in st.members {
            if seen.insert(m.qualified.clone()) {
                report.to_write.push(m);
            }
        }
    }
}

/// Validate one acquired `.msg` member: the
/// consent write joins `package` + `type_name` straight into the store path
/// `schemas/<pkg>/msg/<Type>.msg`, so both must be valid ROS identifiers (no
/// `/`, `\`, `..`, absolute path, whitespace, or empty — the path-traversal +
/// junk-path classes), and the body must be non-empty (an empty `.msg` parses
/// as a silent zero-field schema).
fn validate_acquired_member(m: &AcquiredMsg) -> Result<(), String> {
    is_ros_identifier(&m.package).map_err(|e| {
        format!(
            "package {:?} is not a valid ROS identifier ({e})",
            m.package
        )
    })?;
    is_ros_identifier(&m.type_name).map_err(|e| {
        format!(
            "type name {:?} is not a valid ROS identifier ({e})",
            m.type_name
        )
    })?;
    if m.msg_text.trim().is_empty() {
        return Err(format!(
            "the .msg body for {}/{} is empty (an empty schema would bridge a real type as a \
             zero-field message)",
            m.package, m.type_name
        ));
    }
    Ok(())
}

/// Warn when an acquired closure member collides with a BUILT-IN ROS 2 message
/// by qualified name but carries DIFFERENT `.msg`
/// content. The built-in (authoritative, vendored) definition is what
/// the bridge decodes with — the acquired variant is discarded — but the
/// divergence must never be silent (loud-never-silent house rule).
/// `BUILTIN_MSGS`'s third tuple element is the exact vendored `.msg` text;
/// the compare ignores trailing-newline-only deltas
/// (`eq_ignoring_trailing_newlines`) — rcl strips the final newline from every
/// wire payload and the vendored `std_msgs/Header` itself ends `0a0a`, so a
/// byte-compare would have fired this warn on EVERY attach whose closure
/// touched a builtin, drowning the real-divergence signal. (A store collision
/// is the intentional never-clobber floor, pinned by `local_harvest_test`'s
/// store-twin test.)
fn warn_on_builtin_text_divergence(qualified: &str, acquired_text: &str) {
    let Some((pkg, name)) = qualified.split_once('/') else {
        return;
    };
    if let Some(&(_, _, builtin_text)) = native_ros2_messages::BUILTIN_MSGS
        .iter()
        .find(|&&(p, n, _)| p == pkg && n == name)
    {
        if builtin_text != acquired_text {
            tracing::warn!(
                schema = %qualified,
                "ros2 attach: acquired .msg for a built-in type differs from the vendored ROS 2 \
                 corpus — the built-in (authoritative) definition is used to bridge it; the \
                 acquired variant is discarded"
            );
        }
    }
}

/// A ROS 2 package / type identifier: non-empty, ASCII, starts with a letter,
/// every other char `[A-Za-z0-9_]`. Rejects `/`, `\`, `..`, absolute paths,
/// whitespace, and empties.
fn is_ros_identifier(s: &str) -> Result<(), &'static str> {
    match s.chars().next() {
        None => return Err("empty"),
        Some(c) if !c.is_ascii_alphabetic() => return Err("must start with an ASCII letter"),
        Some(_) => {}
    }
    if !s.chars().all(|c| c.is_ascii_alphanumeric() || c == '_') {
        return Err("only ASCII letters, digits, and '_' are allowed");
    }
    Ok(())
}

/// One nested-message reference discovered while walking a schema's fields —
/// the RAW reference (bare name + optional source package + the parent it
/// appears in), NOT yet resolved to a qualified name. Resolution is the
/// caller's job so each uses the RIGHT universe: the acquisition-completeness
/// walk runs [`resolve_nested_ref`] through the codec's canonical ladder
/// ([`resolve_nested_qname_ladder`]) — so it predicts the CDR decoder exactly —
/// while the local-ament harvester resolves it to a filesystem harvest target
/// ([`NestedRef::harvest_target`]).
#[derive(Debug, Clone, PartialEq, Eq)]
pub(crate) struct NestedRef {
    /// The referenced bare type name (`Header`, `Point`, `Widget`).
    pub schema_name: String,
    /// The reference's source package when it was qualified
    /// (`geometry_msgs/Point` → `Some("geometry_msgs")`); `None` for a bare
    /// reference (same-package in ROS 2 `.msg` semantics).
    pub package: Option<String>,
    /// The qualified name of the schema this reference appears in (the parent),
    /// used by the same-package + suffix rungs.
    pub parent_qname: String,
}

impl NestedRef {
    /// The same-package DISPLAY name for a reference that resolved NOWHERE (the
    /// missing-dependency reason text): a qualified ref as-is
    /// (`pkg/schema_name`), a bare ref as `<parent_pkg>/schema_name` (the name
    /// the user would add to close the gap), or the bare `schema_name` under a
    /// flat-namespace parent.
    fn display_name(&self) -> String {
        match &self.package {
            Some(p) => format!("{p}/{}", self.schema_name),
            None => match self.parent_qname.rsplit_once('/') {
                Some((parent_pkg, _)) => format!("{parent_pkg}/{}", self.schema_name),
                None => self.schema_name.clone(),
            },
        }
    }

    /// The qualified name a FILESYSTEM harvest (the local-ament rung) resolves
    /// this reference to, following ROS on-disk semantics: a qualified ref
    /// as-is, a bare `Header` as `std_msgs/Header` (matching the codec's Header
    /// rung so a harvest never chases a non-existent
    /// `<pkg>/Header`), and any other bare ref as same-package
    /// `<parent_pkg>/<name>` (rosidl's "bare = same package" rule). The codec's
    /// unambiguous-bare-suffix rung is intentionally NOT applied to a harvest —
    /// an on-disk harvest resolves same-package refs by CONSTRUCTION, and a bare
    /// `Point` on a robot is that package's `Point`, never a same-suffix
    /// builtin.
    pub(crate) fn harvest_target(&self) -> String {
        match &self.package {
            Some(p) => format!("{p}/{}", self.schema_name),
            None if self.schema_name == "Header" => "std_msgs/Header".to_string(),
            None => match self.parent_qname.rsplit_once('/') {
                Some((parent_pkg, _)) => format!("{parent_pkg}/{}", self.schema_name),
                None => self.schema_name.clone(),
            },
        }
    }
}

/// Resolve one nested reference through the codec's canonical ladder against a
/// resolvable universe — the SINGLE shared ladder the completeness walk and the
/// CDR decoder both use (see [`resolve_nested_qname_ladder`]), so they cannot
/// drift.
fn resolve_nested_ref(r: &NestedRef, universe: &BTreeSet<String>) -> Option<String> {
    resolve_nested_qname_ladder(
        &r.schema_name,
        r.package.as_deref(),
        &r.parent_qname,
        |k| universe.contains(k),
        universe.iter().map(String::as_str),
    )
}

/// The human-facing "missing nested type" label for a reference that resolved
/// NOWHERE: a bare reference whose ONLY failure
/// is AMBIGUITY (two+ same-suffix universe members) names the candidates so the
/// user can qualify it; every other gap names the same-package display name
/// (the type to add).
fn missing_ref_label(r: &NestedRef, universe: &BTreeSet<String>) -> String {
    if r.package.is_none() {
        let candidates: Vec<&str> = universe
            .iter()
            .map(String::as_str)
            .filter(|k| k.rsplit('/').next() == Some(r.schema_name.as_str()))
            .collect();
        if candidates.len() > 1 {
            return format!(
                "{} (AMBIGUOUS bare reference — matches {}; qualify it as pkg/{})",
                r.schema_name,
                candidates.join(", "),
                r.schema_name
            );
        }
    }
    r.display_name()
}

/// Collect the RAW nested-message references of every field in `schema`
/// (recursing through array element types so `Point[]` and `Point[3]`
/// contribute their element reference too). References are NOT resolved here —
/// a bare (`package: None`) reference carries its parent's qualified name so the
/// caller can run the codec's canonical ladder (bare `Header` → `std_msgs/Header`,
/// unambiguous suffix, same-package) rather than eagerly guessing `<pkg>/Name`.
///
/// `pub(crate)` so the local-ament harvester
/// ([`crate::local_harvest`]) walks a harvested `.msg`'s nested closure with the
/// SAME reference set the acquisition-completeness check uses (one reference
/// walk, never two that could drift).
pub(crate) fn nested_dependencies(schema: &MessageSchema) -> Vec<NestedRef> {
    let parent_qname = schema.qualified_name();
    let mut out = Vec::new();
    for f in &schema.fields {
        collect_nested_refs(&f.field_type, &parent_qname, &mut out);
    }
    out
}

/// Recursive helper for [`nested_dependencies`] — descends array element types
/// so `Point[]` and `Point[3]` contribute `geometry_msgs/Point` too.
fn collect_nested_refs(ft: &FieldType, parent_qname: &str, out: &mut Vec<NestedRef>) {
    match ft {
        FieldType::Nested {
            schema_name,
            package,
            ..
        } => {
            out.push(NestedRef {
                schema_name: schema_name.clone(),
                package: package.clone(),
                parent_qname: parent_qname.to_string(),
            });
        }
        FieldType::FixedArray { element_type, .. } | FieldType::DynamicArray { element_type } => {
            collect_nested_refs(element_type, parent_qname, out);
        }
        _ => {}
    }
}

// ─────────────────────────── Report render (pure) ──────────────────────────

/// Render the discovery report (`--dry-run` output, and the header of the
/// consent preview). Pure — the binary prints it verbatim. `part.resolvable`
/// must already be the malformed-FILTERED set (see
/// [`exclude_malformed_topics`]) so the RESOLVABLE section never lists a
/// topic the EXCLUDED section disowns.
#[allow(clippy::too_many_arguments)]
pub fn render_discovery_report(
    params: &DiscoveryParams,
    topics: &[DiscoveredTopic],
    part: &Partitioned,
    dropped: &[DroppedDuplicate],
    raw_routed: &[RawRoutedLoser],
    topic_conflicts: &[DroppedTopicConflict],
    malformed: &[ExcludedMalformedTopic],
    own_endpoints_hidden: usize,
) -> String {
    // No-acquisition entry: byte-identical to the
    // no-acquisition report. `ros_attach_with_acquirer` calls the inner form with
    // a live acquisition context.
    render_discovery_report_inner(
        params,
        topics,
        part,
        dropped,
        raw_routed,
        topic_conflicts,
        malformed,
        own_endpoints_hidden,
        &AcquisitionReport::none(),
    )
}

/// The acquisition-aware renderer. `acq` AUGMENTS each
/// acquired RESOLVABLE topic's route marker with the `(…; schema acquired —
/// writes …/msg/….msg on consent)` note (stating HOW it is bridged AND
/// the not-yet-written state), surfaces per-type skip/incomplete/conflict
/// reasons under the UNRESOLVABLE lines, and (when non-empty) adds the ACQUIRED
/// SCHEMAS section naming every `.msg` a consent write would materialize. With
/// [`AcquisitionReport::none`] every acquisition
/// branch is inert, so the output is byte-identical to the no-acquisition report
/// (the regression pin the [`render_discovery_report`] oracle tests
/// enforce).
#[allow(clippy::too_many_arguments)]
fn render_discovery_report_inner(
    params: &DiscoveryParams,
    topics: &[DiscoveredTopic],
    part: &Partitioned,
    dropped: &[DroppedDuplicate],
    raw_routed: &[RawRoutedLoser],
    topic_conflicts: &[DroppedTopicConflict],
    malformed: &[ExcludedMalformedTopic],
    own_endpoints_hidden: usize,
    acq: &AcquisitionReport,
) -> String {
    let iface = if params.only_networks.is_empty() {
        "<none — every interface>".to_string()
    } else {
        params
            .only_networks
            .iter()
            .map(|ip| ip.to_string())
            .collect::<Vec<_>>()
            .join(", ")
    };
    let mut out = format!(
        "DISCOVERED DDS TOPICS (interface {iface}, domain {})\n",
        params.domain_id
    );

    if topics.is_empty() {
        out.push_str(
            "\nno DDS topics discovered — nothing is publishing on this interface/domain, or \
             discovery did not reach the robot. Check: the robot is powered + on this LAN, \
             --iface is the correct local interface IP, --domain matches ROS_DOMAIN_ID, and \
             (multi-homed hosts) that --iface restricts rustdds so CycloneDDS does not drop \
             fragmented discovery data.\n",
        );
        push_own_hidden_footer(&mut out, own_endpoints_hidden);
        return out;
    }

    out.push_str(&format!(
        "\nRESOLVABLE ({}) — bridged onto the Cerulion wire:\n",
        part.resolvable.len()
    ));
    if part.resolvable.is_empty() {
        out.push_str("  (none)\n");
    }
    for r in &part.resolvable {
        let t = &r.topic;
        // Typed vs raw, in plain language, so the split stays
        // clear about HOW each topic is bridged. A topic that
        // became resolvable via acquisition THIS run (a requested root OR a
        // closure sibling staged through another bundle)
        // AUGMENTS the route marker with the "schema acquired"
        // note (augment, never displace — the reader still sees
        // HOW it is bridged AND that the `.msg` is not yet written).
        let route_marker = match r.route {
            BridgeRoute::Typed(_) => "bridged natively",
            BridgeRoute::Raw => "bridged via the generic codec",
        };
        let mut inner = route_marker.to_string();
        if let Some(path) = acq.staged_path(&t.ros_type) {
            inner.push_str(&format!("; schema acquired — writes {path} on consent"));
        }
        // A same-type sibling that lost the typed-port race but was
        // RESCUED onto the generic codec — say so plainly (riding the generic
        // codec because the typed port is taken; per-instance typed ports are
        // not supported), never the "port taken / NOT bridged" wording.
        if let Some(loser) = raw_routed
            .iter()
            .find(|l| l.ros_topic == t.ros_topic && l.ros_type == t.ros_type)
        {
            inner.push_str(&format!(
                "; shares the {} typed port with {} — riding the generic codec until per-instance \
                 typed ports land",
                t.ros_type, loser.typed_port_kept_by
            ));
        }
        let marker = format!("({inner})");
        out.push_str(&format!(
            "  {}  ({})  writers={} readers={}  qos={}  {}\n",
            t.ros_topic,
            t.ros_type,
            t.writer_count,
            t.reader_count,
            t.qos.summary(),
            marker
        ));
    }

    // The `.msg` files a consent write would materialize into
    // the workspace store — NAMED here so `--dry-run`/the preview is explicit
    // about what will be written (nothing is written before the consent gate).
    // Header reconciled with the "(schema acquired …)" marker:
    // the ACQUIRE already happened (during discovery); the pending action is
    // the WRITE, so the section is "ACQUIRED SCHEMAS … written … on consent".
    if !acq.to_write.is_empty() {
        out.push_str(&format!(
            "\nACQUIRED SCHEMAS ({}) — written into the workspace .msg store on consent \
             (--dry-run writes nothing):\n",
            acq.to_write.len()
        ));
        for f in &acq.to_write {
            // The rung label already reads as prose (e.g. "local ROS install
            // via ament index"), so the template must NOT prepend its own
            // "via " — that double-"via"s the line.
            out.push_str(&format!(
                "  {}  ({}, {})\n",
                f.workspace_rel, f.qualified, f.rung_label
            ));
        }
    }

    out.push_str(&format!(
        "\nUNRESOLVABLE ({}) — no Cerulion schema; NOT bridged:\n",
        part.unresolvable.len(),
    ));
    if part.unresolvable.is_empty() {
        out.push_str("  (none)\n");
    }
    for t in &part.unresolvable {
        // Both remediations (the bridge's config-error style): the typed registry
        // set AND the schema-chain path. "Unknown type" means NO SCHEMA,
        // never "no codec".
        out.push_str(&format!(
            "  {}  ({})  — '{}' is neither a typed bridge mapping ({}) nor a built-in \
             ROS 2 message; add a schema for it or exclude the topic \
             (raw DDS type: {})\n",
            t.ros_topic,
            t.ros_type,
            t.ros_type,
            supported_ros_types(),
            t.raw_type
        ));
        // Loud per-rung skip / incomplete-closure reasons for
        // a type the acquirer ATTEMPTED but could not resolve — never silent.
        if let Some(reasons) = acq.skip_reasons.get(&t.ros_type) {
            for reason in reasons {
                out.push_str(&format!("      · acquisition: {reason}\n"));
            }
        }
    }

    // Hostile/nonconforming discovered topic names, excluded
    // by the graph layer's own predicate BEFORE anything is generated.
    if !malformed.is_empty() {
        out.push_str(&format!(
            "\nEXCLUDED — MALFORMED TOPIC NAMES ({}): these would be rejected at graph load; \
             NOT bridged (fix the publisher's topic name, or bridge it manually with a valid \
             `topic:` override):\n",
            malformed.len()
        ));
        for m in malformed {
            out.push_str(&format!(
                "  {}  ({})  — excluded: {}\n",
                m.cerulion_topic, m.ros_type, m.reason
            ));
        }
    }

    // The RARE port conflicts — a same-type sibling that lost the
    // typed port and could NOT be raw-routed (incomplete store shadow, or an
    // older vendored bridge). Rescued losers are bridged and already
    // listed in RESOLVABLE with the "riding the generic codec" marker; only
    // the genuinely-unbridgeable ones land here. Each reason carries its own
    // arm-specific remediation (a fixed suffix here
    // contradicted the incomplete-store-closure arm, telling the operator to
    // drop a .msg that was already in the store).
    if !dropped.is_empty() {
        out.push_str(&format!(
            "\nPORT CONFLICTS ({}) — bridge v1 binds each type to ONE port; these share a \
             type with a kept topic and could not be raw-routed, so they are NOT bridged:\n",
            dropped.len()
        ));
        for d in dropped {
            out.push_str(&format!(
                "  {}  ({})  — typed port taken by {}; {} (per-instance typed ports are the \
                 eventual home)\n",
                d.dds_topic, d.ros_type, d.kept_dds_topic, d.raw_fallback_reason
            ));
        }
    }

    // One DDS topic name legally carrying TWO types.
    if !topic_conflicts.is_empty() {
        out.push_str(&format!(
            "\nTOPIC CONFLICTS ({}) — one DDS topic name carries TWO types (legal in DDS); \
             Cerulion topics are single-writer, so only the first type (lexicographically \
             smallest) is bridged:\n",
            topic_conflicts.len()
        ));
        for c in topic_conflicts {
            out.push_str(&format!(
                "  {}  — dropped type {} (topic kept by {}); to bridge both, edit the \
                 generated config to give one mapping a distinct cerulion_topic\n",
                c.ros_topic, c.ros_type, c.kept_ros_type
            ));
        }
    }

    out.push_str(&format!(
        "\nSummary: {} topic(s) discovered, {} resolvable, {} unresolvable{}{}{}{}.\n",
        topics.len(),
        part.resolvable.len(),
        part.unresolvable.len(),
        // Same-type siblings rescued onto the generic codec (bridged,
        // already counted in `resolvable` — surfaced so the operator sees the
        // typed-port race happened). Silent when none.
        if raw_routed.is_empty() {
            String::new()
        } else {
            format!(
                ", {} same-type sibling(s) on the generic codec",
                raw_routed.len()
            )
        },
        if malformed.is_empty() {
            String::new()
        } else {
            format!(", {} malformed-topic(s) excluded", malformed.len())
        },
        if dropped.is_empty() {
            String::new()
        } else {
            format!(", {} port-conflict(s) dropped", dropped.len())
        },
        if topic_conflicts.is_empty() {
            String::new()
        } else {
            format!(", {} topic-conflict(s) dropped", topic_conflicts.len())
        }
    ));
    push_own_hidden_footer(&mut out, own_endpoints_hidden);
    out
}

/// The one-line footer accounting for the attach
/// participant's OWN endpoints (parameter services etc.) that the GUID
/// filter hid — nothing is silently dropped. Silent when zero.
fn push_own_hidden_footer(out: &mut String, own_endpoints_hidden: usize) {
    if own_endpoints_hidden > 0 {
        out.push_str(&format!(
            "({own_endpoints_hidden} of our own discovery endpoints hidden)\n"
        ));
    }
}

// ───────────────────── Migration report (pure) ──────────────────────────────

/// Inputs to [`render_migration_report`] — plain data the attach flow already
/// holds, bundled so the renderer is unit-testable from synthetic discovery
/// results (no DDS stack, no acquisition ladder, no network, no filesystem).
#[derive(Debug, Clone)]
pub struct MigrationReportInputs<'a> {
    /// The `ros_discovery_info` node table ([`DiscoveryResult::nodes`]).
    /// Empty on a vendor/raw-DDS network (the Go2 class) or when nothing
    /// published it during the window — the cannot-attribute arm.
    pub nodes: &'a [DiscoveredNode],
    /// Every FOREIGN endpoint the discovery window saw — the SAME list the
    /// partition aggregated ([`DiscoveryResult::endpoints`]).
    pub endpoints: &'a [DiscoveredEndpoint],
    /// Canonical `pkg/Type` names resolvable BEFORE this run's acquisition
    /// ladder ran (the pre-acquisition partition's resolvable set) — the
    /// "restartable today" evidence.
    pub locally_resolvable: &'a BTreeSet<String>,
    /// Canonical `pkg/Type` names the generated bridge will actually carry:
    /// the FINAL partition's resolvable set AFTER malformed-topic exclusion
    /// (an acquired type whose only topics were excluded is deliberately NOT
    /// here — labeling it after-write restartable would over-claim, since a
    /// run bridging nothing refuses the consent write entirely). NOT
    /// necessarily a superset of `locally_resolvable` for the same reason;
    /// the classifier checks `locally_resolvable` FIRST, so a locally-known
    /// type keeps its today claim regardless. A type here but NOT locally
    /// resolvable becomes usable once consent materializes the acquired
    /// `.msg` schemas.
    pub resolvable: &'a BTreeSet<String>,
}

/// ROS plumbing endpoints EXCLUDED from the migration judgment: service
/// endpoints (`rq/`/`rr/`/`rs/` — parameter services on every node), the
/// `ros_discovery_info` graph topic, `/rosout`, and `/parameter_events`.
/// Every ROS 2 node carries these under ANY rmw — `rmw_cerulion` serves them
/// natively — and their types (`rcl_interfaces/*`, `rmw_dds_common/*`) are
/// deliberately absent from the built-in corpus, so judging them would mark
/// EVERY stock ROS 2 node unrestartable. They are plumbing, not the data the
/// bridge exists to carry.
fn is_ros_infra_endpoint(dds_topic: &str) -> bool {
    dds_topic.starts_with("rq/")
        || dds_topic.starts_with("rr/")
        || dds_topic.starts_with("rs/")
        || dds_topic == "ros_discovery_info"
        || dds_topic == "rt/rosout"
        || dds_topic == "rt/parameter_events"
}

/// One discovered PROCESS (DDS participant) with the nodes it hosts and the
/// canonical message types its non-infra endpoints carry. The participant is
/// the restart unit: ROS 2 runs one participant per process, so "restart this
/// under `rmw_cerulion`" is judged at participant granularity
/// (the same granularity the node table's join key gives us; see
/// [`DiscoveredNode::participant_prefix`]).
struct MigrationProcess {
    /// The 12-byte DDS participant GUID prefix hosting these nodes — the
    /// restart-unit identity. Carried so the YAML-id scheme can key on
    /// participant + FQN: two DIFFERENT participants hosting a same-named
    /// node (the discovery layer keeps same-named nodes on distinct
    /// participants deliberately) still mint distinct ids.
    prefix: [u8; 12],
    /// Sanitized, sorted fully-qualified node names (`/talker`).
    node_fqns: Vec<String>,
    /// Canonical `pkg/Type` set (non-infra endpoints only), sanitized.
    types: BTreeSet<String>,
}

impl MigrationProcess {
    fn label(&self) -> String {
        self.node_fqns.join(", ")
    }
}

/// FNV-1a 64 (the repo's stock non-cryptographic hash — same recipe as the
/// schema hash and barrier names) — the stable source for the migration YAML
/// block's collision-disambiguation suffix. Private: report rendering only.
fn fnv1a_64(bytes: &[u8]) -> u64 {
    let mut h: u64 = 0xcbf2_9ce4_8422_2325;
    for b in bytes {
        h ^= u64::from(*b);
        h = h.wrapping_mul(0x0000_0100_0000_01b3);
    }
    h
}

/// Graph ids for the migration YAML block's `ros2:` entries — one per
/// `(participant prefix, node FQN)`, in the given order. PROVABLY
/// collision-free (a `nodes:` list with duplicate ids is rejected by the real
/// graph validator, so a "paste-ready" block that aliases would not paste).
///
/// The base id is the FQN with the leading `/` dropped and every character
/// outside `[A-Za-z0-9_]` folded to `_` (a restricted alphabet — remote
/// names carry YAML-meaningful bytes; see the fold's comment) — which is
/// LOSSY (`/a/b` and `/a_b` both fold to `a_b`), AND the SAME FQN can arrive
/// on two different participants. So:
///
/// 1. Any base claimed by more than one entry gets a suffix on EVERY colliding
///    member: `_<low-32-of-FNV1a64(prefix ++ fqn), hex>`. Keying on the
///    PARTICIPANT (not the FQN alone) is what makes two same-FQN-different-
///    participant rows distinct; suffixing ALL colliding members keeps the
///    result independent of enumeration order.
/// 2. A used-set pass then GUARANTEES uniqueness against the two residual
///    aliasing classes a suffix-on-collision scheme alone cannot rule out: a
///    suffixed id (`a_b_b38ee0cc`) can equal ANOTHER entry's bare base (the
///    literal FQN `/a_b_b38ee0cc`), and two distinct `(prefix, fqn)` inputs
///    can — astronomically rarely — share a 32-bit hash. Any candidate
///    already emitted gets a deterministic `_<n>` (n from 2) until free. This
///    pass is the proof of collision-freedom; step 1 is what keeps the common
///    and colliding-base cases clean and order-independent.
fn migration_yaml_ids(entries: &[(&[u8; 12], &str)]) -> Vec<String> {
    // The base id's alphabet is RESTRICTED to `[A-Za-z0-9_]` — the leading
    // `/` is dropped and EVERY other character outside that set (interior
    // `/`s included) folds to `_`. Remote-supplied node names reach this
    // position, and `sanitize_display` neuters only terminal controls: a
    // name like `foo: bar` or `foo #bar` carries YAML-meaningful bytes that
    // would let a hostile node table inject structure into the paste-ready
    // block. The restriction makes the id provably inert in the YAML value
    // position (the emit site double-quotes it as well — see the render
    // loop); the wider lossy fold only widens BASE collisions, which the
    // suffix + used-set machinery below already guarantees against, and the
    // suffix hash keys on the RAW fqn so distinct names keep distinct
    // suffixes. An all-special name folding to nothing gets the `node`
    // floor so no entry ever emits an empty id.
    let base = |fqn: &str| {
        let folded: String = fqn
            .trim_start_matches('/')
            .chars()
            .map(|c| {
                if c.is_ascii_alphanumeric() || c == '_' {
                    c
                } else {
                    '_'
                }
            })
            .collect();
        if folded.is_empty() {
            "node".to_string()
        } else {
            folded
        }
    };
    let mut base_claims: BTreeMap<String, usize> = BTreeMap::new();
    for (_, fqn) in entries {
        *base_claims.entry(base(fqn)).or_default() += 1;
    }
    // Step 1: colliding bases get a participant-keyed suffix; unique stay bare.
    let candidates: Vec<String> = entries
        .iter()
        .map(|(prefix, fqn)| {
            let b = base(fqn);
            if base_claims.get(&b).copied().unwrap_or(0) > 1 {
                let mut buf = prefix.to_vec();
                buf.extend_from_slice(fqn.as_bytes());
                format!("{b}_{:08x}", fnv1a_64(&buf) as u32)
            } else {
                b
            }
        })
        .collect();
    // Step 2: guarantee uniqueness against residual aliasing / hash collision.
    let mut used: BTreeSet<String> = BTreeSet::new();
    candidates
        .into_iter()
        .map(|mut id| {
            if used.contains(&id) {
                let stem = id.clone();
                let mut n = 2u32;
                loop {
                    let next = format!("{stem}_{n}");
                    if !used.contains(&next) {
                        id = next;
                        break;
                    }
                    n += 1;
                }
            }
            used.insert(id.clone());
            id
        })
        .collect()
}

/// Render the MIGRATION section printed at the end of EVERY `ros2 attach`
/// discovery report (automatic, no flag) — the on-ramp that
/// tells the user how to stop needing the bridge. Pure rendering over data
/// attach already holds: no new discovery, no network, never blocks, and the
/// caller's outcome/exit contract is untouched (the string is appended to the
/// report, nothing else).
///
/// Four parts, each explicit about its evidence:
/// 1. Discovered processes grouped by restartability: every type resolvable
///    locally = restartable under `rmw_cerulion` today; types acquired
///    during this attach (any ladder rung — wire or local ament) =
///    restartable once consent materializes the `.msg` schemas;
///    unresolvable types, or no `ros_discovery_info` record (a vendor /
///    raw-DDS process — or a node table this window did not observe, and
///    the wording says so) = stays bridged.
/// 2. The equivalent commands, verbatim: `cerulion ros2 launch` for a
///    user-owned bringup, and a paste-ready `ros2:` graph-entry block (the
///    mixed-graph shape) — with `<package>`/`<executable>` placeholders,
///    because DDS discovery sees endpoints, not launch metadata.
/// 3. The bridged-vs-native cost line, stated as facts.
/// 4. A closing pointer at `cerulion ros2 migrate` (publish-side loaned-API
///    adoption inside the user's own nodes).
///
/// Every remote-supplied string (node names, topic names, type names) is
/// terminal-escape-sanitized via the crate's ONE sanitizer
/// (`topic_cmd::sanitize_display`) — a LAN peer controls all of them.
pub fn render_migration_report(inp: &MigrationReportInputs) -> String {
    use crate::topic_cmd::sanitize_display;

    // Nothing discovered at all ⇒ the EMPTY form. Still a section —
    // "every attach report ends with MIGRATION" is a contract, and a silent
    // absence is indistinguishable from the feature not running.
    if inp.endpoints.is_empty() && inp.nodes.is_empty() {
        return concat!(
            "\nMIGRATION — what could run natively on rmw_cerulion:\n",
            "\nNothing was discovered this window, so there is nothing to judge — see the \
             discovery hints above.\n",
            "\nTo adopt the loaned zero-copy publish API inside your own nodes, see \
             `cerulion ros2 migrate`.\n",
        )
        .to_string();
    }

    // Attribute every non-infra endpoint to its owning participant (the
    // restart unit) via `writer_guid[..12] == participant_prefix`; endpoints
    // with no GUID or no node-table owner are the vendor/raw-DDS orphans.
    let mut hosted_nodes: BTreeMap<[u8; 12], Vec<&DiscoveredNode>> = BTreeMap::new();
    for n in inp.nodes {
        hosted_nodes
            .entry(n.participant_prefix)
            .or_default()
            .push(n);
    }
    let mut participant_types: BTreeMap<[u8; 12], BTreeSet<String>> = BTreeMap::new();
    // Orphan (topic, type) pairs — deduped + sorted by construction.
    let mut orphans: BTreeSet<(String, String)> = BTreeSet::new();
    for ep in inp.endpoints {
        if is_ros_infra_endpoint(&ep.dds_topic) {
            continue;
        }
        let ros_type = sanitize_display(&normalize_ros_type(&ep.type_name));
        let owner = ep.writer_guid.and_then(|g| {
            let prefix: [u8; 12] = g[..12].try_into().expect("16-byte GUID");
            hosted_nodes.contains_key(&prefix).then_some(prefix)
        });
        match owner {
            Some(prefix) => {
                participant_types
                    .entry(prefix)
                    .or_default()
                    .insert(ros_type);
            }
            None => {
                orphans.insert((
                    sanitize_display(&normalize_dds_topic(&ep.dds_topic)),
                    ros_type,
                ));
            }
        }
    }

    // Classify each process: every type locally resolvable ⇒ restartable
    // today (checked FIRST — a locally-resolvable type stays a today claim
    // even if its topics were malformed-excluded from the bridge, because
    // restartable-today is a schema-exists claim, not a bridge-mapping one);
    // else every type in the post-exclusion resolvable set, ≥1 acquired ⇒
    // restartable after the consent write; else stays bridged (types named);
    // no message endpoints seen ⇒ nothing to judge.
    let mut today: Vec<MigrationProcess> = Vec::new();
    let mut after_write: Vec<MigrationProcess> = Vec::new();
    let mut blocked: Vec<(MigrationProcess, Vec<String>)> = Vec::new();
    let mut silent: Vec<MigrationProcess> = Vec::new();
    for (prefix, nodes) in &hosted_nodes {
        let mut node_fqns: Vec<String> = nodes
            .iter()
            .map(|n| sanitize_display(&n.fully_qualified_name()))
            .collect();
        node_fqns.sort();
        let types = participant_types.remove(prefix).unwrap_or_default();
        let proc = MigrationProcess {
            prefix: *prefix,
            node_fqns,
            types,
        };
        if proc.types.is_empty() {
            silent.push(proc);
        } else if proc
            .types
            .iter()
            .all(|t| inp.locally_resolvable.contains(t))
        {
            today.push(proc);
        } else {
            let missing: Vec<String> = proc
                .types
                .iter()
                .filter(|t| !inp.resolvable.contains(*t))
                .cloned()
                .collect();
            if missing.is_empty() {
                after_write.push(proc);
            } else {
                blocked.push((proc, missing));
            }
        }
    }
    // Deterministic order: by the process's first node name.
    let by_first = |p: &MigrationProcess| p.node_fqns.first().cloned().unwrap_or_default();
    today.sort_by_key(by_first);
    after_write.sort_by_key(by_first);
    blocked.sort_by_key(|(p, _)| by_first(p));
    silent.sort_by_key(by_first);

    let mut out = String::from("\nMIGRATION — what could run natively on rmw_cerulion:\n");

    let nothing_restartable = today.is_empty() && after_write.is_empty();
    if nothing_restartable {
        // The everything-stays-bridged line (a robot
        // with zero restartable nodes must say so plainly).
        if inp.nodes.is_empty() {
            out.push_str(
                "\nNo restartable process can be named: no ROS 2 node table was seen \
                 (nothing published ros_discovery_info during the window — vendor/raw-DDS \
                 processes, or the table was simply not observed). Every discovered topic \
                 stays on the dds_bridge; where a topic's type already resolves, its line \
                 below says what a restart would buy.\n",
            );
        } else {
            out.push_str(
                "\nNo process here is restartable under rmw_cerulion yet — every attributed \
                 process stays on the dds_bridge (see the reasons below).\n",
            );
        }
    }

    if !today.is_empty() {
        out.push_str(&format!(
            "\nRESTARTABLE TODAY ({} process(es)) — every message type these nodes use \
             resolves locally; restart each process under rmw_cerulion \
             (RMW_IMPLEMENTATION=rmw_cerulion) and its topics ride the Cerulion wire \
             natively, no bridge hop:\n",
            today.len()
        ));
        for p in &today {
            out.push_str(&format!(
                "  {}  — {}\n",
                p.label(),
                p.types.iter().cloned().collect::<Vec<_>>().join(", ")
            ));
        }
    }

    if !after_write.is_empty() {
        out.push_str(&format!(
            "\nRESTARTABLE AFTER THIS ATTACH WRITES ITS SCHEMAS ({} process(es)) — these \
             nodes use custom types resolved during this attach; consent writes the \
             .msg files, then the same restart applies:\n",
            after_write.len()
        ));
        for p in &after_write {
            out.push_str(&format!(
                "  {}  — {}\n",
                p.label(),
                p.types.iter().cloned().collect::<Vec<_>>().join(", ")
            ));
        }
    }

    if !blocked.is_empty() || !orphans.is_empty() {
        // UNITS: orphans have no participant identity to count (that
        // is what makes them orphans), so they are counted as TOPICS; the
        // blocked group IS attributed, so it is counted as PROCESSES. One
        // bare number over both would claim a unit the data cannot back.
        let mut units: Vec<String> = Vec::new();
        if !orphans.is_empty() {
            units.push(format!("{} topic(s)", orphans.len()));
        }
        if !blocked.is_empty() {
            units.push(format!("{} process(es)", blocked.len()));
        }
        out.push_str(&format!(
            "\nSTAYS BRIDGED ({}) — the dds_bridge keeps carrying these:\n",
            units.join(", ")
        ));
        for (topic, ros_type) in &orphans {
            // Absence-of-evidence wording (never a vendor CLAIM): no node
            // record for this endpoint's participant means EITHER a vendor /
            // raw-DDS process OR a node table this window simply did not
            // observe — the two are indistinguishable from here. What IS
            // distinguishable is the TYPE, in THREE shapes, mirroring the
            // process classification precedence (locally, then acquired, then
            // unknown): a type that resolves LOCALLY gets the today-works hint
            // (schema exists now); one that resolves only AFTER this attach's
            // schemas land gets the after-write hint; an UNKNOWN type gets the
            // plain absence line. Either way it is not "restartable" (no
            // process to name), but a healthy ROS endpoint with a missing node
            // record is never lumped in with a truly unknown one. (Lookup on
            // the sanitized name is exact: sanitizing is identity for clean
            // names, and a control-char name maps to a U+FFFD form that can
            // never match a canonical set entry.)
            let base = format!(
                "  {topic}  ({ros_type})  — no ROS 2 node record seen (vendor/raw DDS, or \
                 the node table was not observed this window)"
            );
            if inp.locally_resolvable.contains(ros_type) {
                out.push_str(&format!(
                    "{base}; the type resolves locally — if this is one of your nodes, \
                     restarting it under rmw_cerulion works\n"
                ));
            } else if inp.resolvable.contains(ros_type) {
                out.push_str(&format!(
                    "{base}; the type resolved during this attach — if this is one of your \
                     nodes, it is restartable after the schemas are written\n"
                ));
            } else {
                out.push_str(&format!("{base}\n"));
            }
        }
        for (p, missing) in &blocked {
            // "unresolvable or excluded": a blocking type is either in the
            // UNRESOLVABLE section or (post-exclusion capture) one whose only
            // topics were malformed-excluded — point at the report, not at
            // one section that may not carry it.
            out.push_str(&format!(
                "  {}  — blocked by unresolvable or excluded type(s): {} (see the report \
                 above)\n",
                p.label(),
                missing.join(", ")
            ));
        }
    }

    if !silent.is_empty() {
        out.push_str(&format!(
            "\nNO MESSAGE ENDPOINTS SEEN ({} process(es)) — these nodes exposed no message \
             endpoints during the window; nothing to judge, nothing bridged:\n",
            silent.len()
        ));
        for p in &silent {
            out.push_str(&format!("  {}\n", p.label()));
        }
    }

    if !nothing_restartable {
        // Part 2: the equivalent commands, verbatim. The YAML block is
        // flush-left so it pastes into a graph file unchanged.
        out.push_str(
            "\nRestart your own bringup natively — one word in front of the launch you \
             already own:\n  cerulion ros2 launch <your-bringup>.launch.py\n\
             Or declare the restartable nodes as ros2: entries in a Cerulion graph and let \
             `cerulion graph run` bring them up beside native nodes (fill in each \
             package/executable — DDS discovery sees endpoints, not launch metadata):\n\n\
             nodes:\n",
        );
        let restartable: Vec<(&[u8; 12], &str)> = today
            .iter()
            .chain(after_write.iter())
            .flat_map(|p| p.node_fqns.iter().map(move |f| (&p.prefix, f.as_str())))
            .collect();
        let ids = migration_yaml_ids(&restartable);
        for ((_, fqn), id) in restartable.iter().zip(&ids) {
            out.push_str(&format!(
                "  - id: \"{id}\"\n    ros2:\n      package: <package>       # the package \
                 that ships {fqn}\n      executable: <executable>\n"
            ));
        }
        // Part 3: what each path buys, as facts.
        out.push_str(
            "\nBridged vs native: a bridged topic pays a per-message CDR decode in the \
             dds_bridge; a native topic is published once on the Cerulion wire — no decode \
             hop, zero-copy eligible. The cost of native is one process restart.\n",
        );
    }

    // Part 4: the publish-side sibling.
    out.push_str(
        "\nTo adopt the loaned zero-copy publish API inside your own nodes, see \
         `cerulion ros2 migrate`.\n",
    );
    out
}

// ─────────────────────────── Config gen (pure) ─────────────────────────────

/// The workspace `.msg` store directory the generated bridge config points its
/// generic codec at. RELATIVE **to the CONFIG FILE's own
/// directory** (supplement A): the `dds_bridge` joins relative `msg_dirs`
/// entries to its config path at load, so the config (which lives in
/// `graphs/`) reaches the workspace store at `<root>/schemas` via `../schemas`
/// — and `cerulion graph run` works from ANY working directory (the CLI's
/// walk-up workspace discovery makes subdir invocation first-class; the old
/// CWD-relative contract doomed the consented auto-run from any non-root CWD).
/// Kept relative rather than absolute so the generated config stays
/// committable + portable (Principle #7 determinism — no machine-specific
/// path baked into a workspace file).
pub const BRIDGE_STORE_MSG_DIR: &str = "../schemas";

/// Generate the `dds_bridge` mapping-config YAML (the `DDS_BRIDGE_CONFIG`
/// file) for the given mappings, plus the top-level `msg_dirs:` key naming the
/// workspace `.msg` schema store(s) the bridge's generic codec must load at run
/// time. Byte-oracle-stable: fixed field order, no serde
/// round-trip. Parses into
/// `examples/go2/nodes/dds_bridge/src/config.rs::BridgeConfig`.
///
/// `qos` is emitted as `best_effort` unconditionally — a best-effort
/// subscriber matches BOTH reliable and best-effort publishers, while a
/// reliable subscriber would NOT match a best-effort publisher; best-effort is
/// the maximum-compatibility default the bridge documents. The discovered
/// per-topic QoS is shown in the report, not baked in.
///
/// The `msg_dirs:` block is emitted iff `msg_dirs` is NON-EMPTY — the caller
/// passes the store dir whenever the store will be NON-EMPTY at run time (it
/// already holds schemas, OR this run materializes acquired ones in the same
/// consent batch). That gate is deliberately CONSERVATIVE, not exact: the
/// store may hold schemas no current mapping resolves through (a stale `.msg`
/// from an earlier robot still triggers emission), which only
/// widens the codec's schema set, never breaks a mapping. An EMPTY slice
/// yields output BYTE-IDENTICAL to a store-less config (the back-compat
/// floor). Paths are emitted verbatim; see [`BRIDGE_STORE_MSG_DIR`] for the
/// config-file-relative resolution contract.
pub fn generate_bridge_config_with_store(
    domain_id: u16,
    only_networks: &[IpAddr],
    mappings: &[BridgeMapping],
    msg_dirs: &[&str],
) -> String {
    let mut out = String::new();
    out.push_str(
        "# Generated by `cerulion ros2 attach`.\n\
         # DDS->Cerulion mapping config for the `dds_bridge` node, consumed via the\n\
         # DDS_BRIDGE_CONFIG env var. Typed mappings ride the bridge's fixed ports\n\
         # (cerulion_topic is documentation; the graph's `topic:` override is\n\
         # authoritative). Raw mappings ride the generic codec: cerulion_topic\n\
         # is AUTHORITATIVE, and max_slice_len defaults to 1 MiB (set it per mapping\n\
         # for larger frames). qos is best_effort (matches both reliable and\n\
         # best-effort publishers). Edit freely.\n",
    );
    out.push_str(&format!("domain_id: {domain_id}\n"));
    if only_networks.is_empty() {
        out.push_str("only_networks: []\n");
    } else {
        out.push_str("only_networks:\n");
        for ip in only_networks {
            out.push_str(&format!("  - {ip}\n"));
        }
    }
    // The workspace `.msg` store(s) the generic codec loads at
    // startup — emitted only when the bridge will need it (see the fn docs).
    if !msg_dirs.is_empty() {
        out.push_str(
            "# msg_dirs: workspace .msg schema store(s) the bridge's generic codec loads at\n\
             # startup — each an ament-mirror `<dir>/<pkg>/msg/<Type>.msg` tree.\n\
             # RELATIVE paths resolve against THIS FILE's directory (the bridge joins them to\n\
             # the config path at load), so `../schemas` is the workspace store beside graphs/\n\
             # and the graph runs from any working directory. A store schema wins over a\n\
             # built-in of the same name (a store schema shadows a built-in).\n",
        );
        out.push_str("msg_dirs:\n");
        for d in msg_dirs {
            out.push_str(&format!("  - {d}\n"));
        }
    }
    out.push_str("mappings:\n");
    for m in mappings {
        out.push_str(&format!("  - dds_topic: {}\n", m.dds_topic));
        out.push_str(&format!("    ros_type: {}\n", m.ros_type));
        out.push_str(&format!("    cerulion_topic: {}\n", m.cerulion_topic));
        out.push_str("    qos: best_effort\n");
        // A REGISTRY-type mapping routed RAW is a typed-port loser
        // rescued onto the generic codec. The bridge routes by `ros_type` (a
        // registry type ⇒ its fixed typed port), so it MUST be told to ride the
        // raw path instead — `route: raw`. A non-registry raw mapping rides the
        // generic codec by default and needs no override (output stays
        // byte-identical to a config without the override for those).
        if matches!(m.route, BridgeRoute::Raw) && resolve_bridge_binding(&m.ros_type).is_some() {
            out.push_str("    route: raw\n");
        }
    }
    out
}

// ─────────────────────────── Robot identity (pure) ─────────────────────────

/// ROS "infrastructure" topics — every robot publishes these, so they carry NO
/// robot identity and must not vote in the shared-namespace derivation
/// ([`derive_robot_identity`]). `/spot/scan` names the robot; `/tf` does not.
const ROS_INFRA_TOPICS: &[&str] = &[
    "/rosout",
    "/parameter_events",
    "/tf",
    "/tf_static",
    "/clock",
    "/diagnostics",
];

/// Does `name` pass the robot-name ALLOW-LIST — a non-empty string of only
/// `[A-Za-z0-9_.-]`? A robot name is a simple identifier, so restricting it to
/// this set rejects — BY CONSTRUCTION — every YAML indicator (`@ ! & % ...`),
/// zenoh key-expression special (`* $ ? #`), the `/` path separator, and all
/// whitespace. The value becomes the attach graph's `prefix:` (the
/// topic-namespace label); it must be a clean scalar so the emitted YAML
/// round-trips. (The network announce identity — the mDNS TXT `robot=` value
/// and the `cerulion_ann/{robot}{topic}` chunk — is resolved SEPARATELY from
/// the hostname / `CERULION_ROBOT_IDENTITY` at runtime, NOT from this value;
/// see [`derive_robot_identity`].) Pure — oracle-tested.
fn is_valid_robot_identity(name: &str) -> bool {
    !name.is_empty()
        && name
            .chars()
            .all(|c| c.is_ascii_alphanumeric() || matches!(c, '_' | '.' | '-'))
}

/// The single first path segment shared by EVERY non-infrastructure topic, or
/// `None` when the topics carry no common namespace (they diverge, there are no
/// non-infra topics at all, or one is malformed — a bare `/`). Infrastructure
/// topics ([`ROS_INFRA_TOPICS`]) are filtered out first — they never carry
/// robot identity. Pure — oracle-tested via [`derive_robot_identity`].
fn shared_first_segment(ros_topics: &[&str]) -> Option<String> {
    let mut common: Option<&str> = None;
    let mut saw_non_infra = false;
    for &topic in ros_topics {
        if ROS_INFRA_TOPICS.contains(&topic) {
            continue;
        }
        // `split('/')` always yields at least one element, so the first segment
        // is well-defined; an empty first segment (a bare `/` or empty topic)
        // means the set has no clean shared namespace — bail to the fallback.
        let first = topic
            .trim_start_matches('/')
            .split('/')
            .next()
            .unwrap_or("");
        if first.is_empty() {
            return None;
        }
        saw_non_infra = true;
        match common {
            None => common = Some(first),
            Some(c) if c == first => {}
            // Two different first segments ⇒ no shared namespace.
            Some(_) => return None,
        }
    }
    if saw_non_infra {
        common.map(str::to_string)
    } else {
        None
    }
}

/// Derive the attach graph's `prefix:` — the robot-name LABEL the generated
/// graph carries. On an attach graph the bridged topics are ABSOLUTE mirrors
/// (never prefixed), so this value drives neither topic naming NOR the network
/// announce identity (which resolves SEPARATELY from the hostname /
/// `CERULION_ROBOT_IDENTITY` at runtime — see
/// `cerulion_cli_engine::graph_cmd::resolve_robot_identity`; the graph prefix
/// is NOT an identity source). It is a human-readable
/// namespace label, and must still be the ROBOT's name — never the graph
/// filename — so the generated YAML reads sensibly and a
/// `--topic-prefix`-less relative wiring (should one ever be added) namespaces
/// under the robot.
///
/// Rules, in order:
///
/// 1. `override_name` (the `--robot-name` flag) wins — trimmed, after the
///    identity allow-list. A value that fails it (empty, or any character
///    outside `[A-Za-z0-9_.-]` — whitespace, `/`, or a YAML/key-expr special)
///    is REJECTED with a loud `warn!` naming it, and the derivation falls back
///    to `hostname_fallback`.
/// 2. Else, if every non-infrastructure ROS topic shares ONE first path segment
///    (a robot that namespaces its nodes under `/spot/...`), that segment IS the
///    identity — a great free name. Infra topics (`/tf`, `/rosout`, ...) are
///    ignored.
/// 3. Else (topics scatter across unrelated segments — the Go2's `/utlidar` +
///    `/lf` + `/uslam` + `/api`, or there is nothing to derive from) →
///    `hostname_fallback`.
///
/// NEVER returns the graph name. At the call site `hostname_fallback` is
/// `cerulion_core::graph::default_prefix("")` (this machine's hostname sans
/// `.local`, else `"localhost"`). NOTE its limitation: attach normally runs ON
/// the robot (the Shape-B deployment), so the machine hostname IS the robot; a
/// REMOTE attach with no shared ROS namespace would pick up the WORKSTATION
/// hostname instead — `--robot-name` is the escape for that case. Pure —
/// oracle-tested.
pub fn derive_robot_identity(
    override_name: Option<&str>,
    ros_topics: &[&str],
    hostname_fallback: &str,
) -> String {
    // 1. Explicit override wins — after the basic identity check.
    if let Some(raw) = override_name {
        let trimmed = raw.trim();
        if is_valid_robot_identity(trimmed) {
            return trimmed.to_string();
        }
        tracing::warn!(
            rejected = %raw,
            fallback = %hostname_fallback,
            "ros2 attach: --robot-name failed the identity check (empty, or not a simple \
             `[A-Za-z0-9_.-]` identifier) — falling back to the hostname identity instead"
        );
        return hostname_fallback.to_string();
    }
    // 2. A shared first path segment across every non-infra topic IS the robot.
    if let Some(namespace) = shared_first_segment(ros_topics) {
        return namespace;
    }
    // 3. No shared namespace ⇒ this machine's hostname (see the NOTE above).
    hostname_fallback.to_string()
}

// ─────────────────────────── Graph gen (pure) ──────────────────────────────

/// The cloud port's SHM slot ceiling — 1 MiB, matching the demo bridge graph
/// (≈21k-point utlidar frames with headroom). Applied to the `cloud` port only.
const CLOUD_MAX_SLICE_LEN: usize = 1_048_576;

/// The graph node id of the `dds_bridge` node — the ONLY node an attach graph
/// declares (no robot-side visualization node is staged; visualization is
/// desk-side, see [`generate_bridge_graph`]).
const BRIDGE_NODE_ID: &str = "bridge";

/// Generate the LEAN one-node bridge graph YAML: a single `dds_bridge` node
/// declaring its FIXED four-port set. A port whose `ros_type` has a mapping
/// publishes on the mapping's `cerulion_topic`; an unmapped port declares a
/// silent default topic (`/dds_bridge/<port>`) but never publishes. Byte-oracle
/// stable. Parses into `cerulion_core::graph::config::GraphConfig`.
///
/// # No visualization node is staged on the robot
///
/// Appending a generic `rerun_sink` node subscribed to every bridged
/// topic would put the Rerun SDK and all archetype conversion ON THE ROBOT. That
/// contradicts the architecture: the desk demands a
/// topic, `cerulion-netd` re-injects it into desk-local SHM, and `cerulion-vizd`
/// — running on the USER'S machine — does the decode + archetype construction.
/// The robot's job is to ship raw frames. A robot-side sink would be a second,
/// redundant conversion path costing robot CPU and dragging ~39 Rerun crates
/// into the robot's build. To SEE this robot's data, run
/// `cerulion viz --robot <name>` (or Cerulion Studio) on your own machine.
///
/// `robot_identity` is the graph `prefix:` (the robot-name topic-namespace
/// LABEL — NOT the network announce identity, which resolves from the hostname
/// / `CERULION_ROBOT_IDENTITY` at runtime; see [`derive_robot_identity`]);
/// `graph_name` is the file stem only (`name:`).
pub fn generate_bridge_graph(
    graph_name: &str,
    robot_identity: &str,
    mappings: &[BridgeMapping],
) -> String {
    // The header names the ONE node type the graph declares and points
    // at the desk-side viz path. It must never advertise a node the graph does
    // not declare below. The invariant is structural: there is only one
    // arm, and only one node.
    let footer =
        "# unmapped ports stay silent. The `dds_bridge` node type must exist in this workspace.\n\
         # To SEE this robot's data, run `cerulion viz --robot <name>` (or Cerulion Studio) on\n\
         # YOUR machine — visualization is desk-side; the robot only ships raw frames.\n";
    let mut out = String::new();
    out.push_str(&format!(
        "# Generated by `cerulion ros2 attach` — the lean one-node bridge graph.\n\
         # Runs the `dds_bridge` node; its DDS mappings come from graphs/{graph_name}.bridge.yaml\n\
         # (set DDS_BRIDGE_CONFIG to it). The four outputs are the bridge's FIXED port set —\n\
         {footer}"
    ));
    // The `prefix:` is the ROBOT-name namespace LABEL,
    // NOT the graph filename and NOT the network announce identity. On an attach
    // graph the bridged topics are absolute mirrors (never prefixed), so this
    // prefix drives no topic names; the announced network identity resolves
    // SEPARATELY from the hostname / `CERULION_ROBOT_IDENTITY` at runtime
    // (`resolve_robot_identity`, which ignores the prefix). Derived by
    // `derive_robot_identity` at the call site.
    //
    // Emit it as a serde-quoted YAML scalar so ANY identity round-trips to
    // EXACTLY `robot_identity` (a raw `@robot`/`!foo` would make the YAML
    // unparseable, or parse to an empty prefix). `derive_robot_identity` already
    // allow-lists to `[A-Za-z0-9_.-]` (a plain scalar), so this is the residual
    // guard for a direct generator call that bypasses the derivation.
    // `serde_yaml::to_string` of a `&str` yields the scalar plus its own
    // trailing newline (e.g. `spot\n`, `'@robot'\n`).
    let prefix_scalar =
        serde_yaml::to_string(robot_identity).expect("a &str always serializes to a YAML scalar");
    out.push_str(&format!("prefix: {prefix_scalar}"));
    out.push_str("nodes:\n");
    out.push_str(&format!("  - id: {BRIDGE_NODE_ID}\n"));
    out.push_str("    type: dds_bridge\n");
    out.push_str("    outputs:\n");
    for binding in V1_BRIDGE_BINDINGS.iter() {
        // TYPED mappings only — raw mappings ride the bridge's
        // dynamically-created ingress publishers and involve NO graph port
        // (the lean one-node graph is unchanged by them).
        let topic = mappings
            .iter()
            .find(|m| matches!(m.route, BridgeRoute::Typed(b) if b.ros_type == binding.ros_type))
            .map(|m| m.cerulion_topic.clone())
            .unwrap_or_else(|| format!("/dds_bridge/{}", binding.port));
        out.push_str(&format!("      - name: {}\n", binding.port));
        out.push_str(&format!("        schema: {}\n", binding.cerulion_schema));
        out.push_str(&format!("        topic: {topic}\n"));
        if binding.port == "cloud" {
            out.push_str(&format!("        max_slice_len: {CLOUD_MAX_SLICE_LEN}\n"));
        }
    }
    out
}

// ─────────────────────────── Orchestration ─────────────────────────────────

/// Options for [`ros_attach`] (parsed from the CLI).
#[derive(Debug, Clone)]
pub struct RosAttachOptions {
    /// `--iface`: the robot-LAN interface IP. Becomes the discovery
    /// `only_networks` restriction AND the generated config's `only_networks`.
    pub iface: IpAddr,
    /// `--domain` (default 0).
    pub domain_id: u16,
    /// Discovery collection window (`--timeout`, default 5 s).
    pub window: Duration,
    /// `--dry-run`: discovery report only; write nothing, run nothing. Wins
    /// over `assume_yes`.
    pub dry_run: bool,
    /// `--yes`: skip the interactive confirm and write (scripts / non-TTY).
    pub assume_yes: bool,
    /// The generated graph name (`--graph-name`, default `attach`). The graph
    /// is `graphs/<name>.yaml`; the config is `graphs/<name>.bridge.yaml`.
    pub graph_name: String,
    /// Optional Cerulion-topic prefix (`--topic-prefix`); `None` ⇒ mirror the
    /// ROS topic name.
    pub topic_prefix: Option<String>,
    /// `--robot-name`: the robot-name LABEL written as the attach graph's
    /// `prefix:`. NOT the network announce identity — the name in `topic list`
    /// ROBOTS / mDNS `_cerulion._tcp` resolves from the hostname /
    /// `CERULION_ROBOT_IDENTITY` at runtime, independent of this.
    /// `None` ⇒ derive the prefix from the bridged ROS topics' shared namespace,
    /// else this machine's hostname (see [`derive_robot_identity`]).
    pub robot_name: Option<String>,
}

/// What [`ros_attach`] did with the workspace / the run.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum AttachOutcome {
    /// `--dry-run`: discovery report only.
    DryRun,
    /// Discovery found no resolvable topic — nothing to bridge (report only,
    /// nothing written).
    NothingResolvable,
    /// The generated graph + config (+ any acquired `.msg` schemas) were
    /// written; the binary should run the graph.
    Written {
        graph_path: PathBuf,
        graph_backup: Option<PathBuf>,
        config_path: PathBuf,
        config_backup: Option<PathBuf>,
        /// The acquired `.msg` schema files materialized into
        /// the workspace store on this consent write (empty when no schema was
        /// acquired). Each carries its `.bak` backup
        /// if a store file already existed at the path.
        schema_writes: Vec<SchemaWrite>,
    },
    /// The interactive confirm was answered "no" — nothing written.
    Declined,
}

/// One acquired `.msg` schema file materialized into the
/// workspace store on a consent write (`AttachOutcome::Written::schema_writes`).
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct SchemaWrite {
    /// The absolute path written (`<root>/schemas/<pkg>/msg/<Type>.msg`).
    pub path: PathBuf,
    /// `Some(.bak)` if a store file already existed at the path and was backed
    /// up before the overwrite (the shared `write_yaml_atomically` `.bak`
    /// convention); `None` for a fresh file.
    pub backup: Option<PathBuf>,
}

/// Everything the binary needs to print + drive after [`ros_attach`].
#[derive(Debug, Clone)]
pub struct RosAttachReport {
    /// The discovery report (always printed).
    pub report: String,
    /// The full consent preview (report + generated files) shown on the
    /// interactive arm; the binary prints it on the non-interactive write arms.
    pub preview: String,
    pub topics: Vec<DiscoveredTopic>,
    pub mappings: Vec<BridgeMapping>,
    pub outcome: AttachOutcome,
    /// True when the confirm provider was invoked (it displays the preview).
    pub preview_shown: bool,
    /// `Some(name)` iff the binary should now `graph run` it (a successful
    /// write on a non-dry-run).
    pub graph_to_run: Option<String>,
    /// The derived graph `prefix:` (the robot-name LABEL) the
    /// generated graph carries (from `--robot-name`, the bridged topics' shared
    /// namespace, or this machine's hostname; see [`derive_robot_identity`]).
    /// This is the topic-namespace label, NOT the runtime network announce
    /// identity (hostname / `CERULION_ROBOT_IDENTITY`). Exposed so callers (and
    /// the full-flow tests) can see the prefix the generated graph was written
    /// with.
    pub robot_identity: String,
}

/// The production `cerulion ros2 attach` acquisition ladder's OWNED rungs.
/// The binary constructs this, borrows a
/// [`cerulion_dds::ChainedAcquirer`] over [`Self::chain`], and passes it to
/// [`ros_attach_with_acquirer`] — keeping `main` thin and this composition
/// unit-testable. The rungs are owned HERE (not returned as a
/// `ChainedAcquirer`, which BORROWS its rungs).
///
/// Ladder order (first-tried): the wire-native `~/get_type_description` rung
/// (when present) THEN the LOCAL ament harvest
/// ([`LocalAmentAcquirer`]). The wire rung is distro-gated and
/// cheaper-when-available, so it is tried before the local filesystem scan. It
/// is held as a `Box<dyn SchemaAcquirer>` set by the binary — the engine stays
/// DDS-free over the acquirer trait, and the hermetic tests compose with a
/// canned wire rung the same way. The store→builtins resolution CHAIN
/// ([`AttachSchemaChain`]) is the acquisition PREDICATE (what already resolves),
/// NOT a rung — the ladder only ever runs over the still-UNRESOLVABLE set (the
/// "C→B→A" ordering: chain-predicate first, then the acquirer rungs).
pub struct AttachAcquirers {
    /// The wire-native rung, when installed (the binary boxes a
    /// `cerulion_dds::WireServiceAcquirer`; hermetic tests box a canned one).
    /// `None` ⇒ the ladder is the local rung alone.
    /// Pub so hermetic tests can compose arbitrary ladders as struct literals;
    /// production code constructs ONLY via [`Self::production`].
    pub wire: Option<Box<dyn SchemaAcquirer>>,
    /// The always-present local ament rung. Pub for the same test-literal
    /// reason as `wire`.
    pub local_ament: LocalAmentAcquirer,
}

impl AttachAcquirers {
    /// Compose over an explicit local rung with NO wire rung — the hermetic
    /// test seam (a `LocalAmentAcquirer::new(prefixes)`, no env read). NOT a
    /// production constructor: the binary calls [`Self::production`], and there
    /// is deliberately no wire-less pub constructor (a `from_env` or a
    /// `with_wire`) that an edit could silently swap in.
    pub fn with_local_ament(local_ament: LocalAmentAcquirer) -> Self {
        Self {
            wire: None,
            local_ament,
        }
    }

    /// The PRODUCTION composition root the binary calls:
    /// the wire-native `~/get_type_description` rung
    /// PREPENDED to the env-derived local ament rung. What is pinned, exactly:
    /// taking the wire rung as a REQUIRED argument means a `production()` call
    /// cannot drop it without a signature-level compile error, AND no wire-less
    /// named constructor exists for a one-line main.rs edit to reach (no `from_env`,
    /// no `with_wire`) — a wire-less binary composition
    /// requires reaching for the documented hermetic test seams
    /// ([`Self::with_local_ament`] / a struct literal), which does not read as
    /// the production idiom. (`cerulion_cli` has no lib target, so `main.rs`
    /// itself cannot be unit-tested; this is the strongest compile-time shape
    /// short of a binary-level e2e.) Pinned by
    /// `ros_attach_test::test_production_acquirers_prepend_wire_rung`.
    pub fn production(wire: Box<dyn SchemaAcquirer>) -> Self {
        Self {
            wire: Some(wire),
            local_ament: LocalAmentAcquirer::from_env(),
        }
    }

    /// The rungs in first-tried priority order, borrowed for
    /// [`cerulion_dds::ChainedAcquirer::new`]: the wire rung (if installed)
    /// FIRST, then the local ament rung.
    pub fn rungs(&self) -> Vec<&dyn SchemaAcquirer> {
        let mut rungs: Vec<&dyn SchemaAcquirer> = Vec::new();
        if let Some(wire) = &self.wire {
            rungs.push(&**wire);
        }
        rungs.push(&self.local_ament);
        rungs
    }

    /// The composed ladder — the ONE acquirer [`ros_attach_with_acquirer`]
    /// consumes.
    pub fn chain(&self) -> cerulion_dds::ChainedAcquirer<'_> {
        cerulion_dds::ChainedAcquirer::new(self.rungs())
    }
}

/// `cerulion ros2 attach` — discover, partition, generate, consent, hand off to
/// run. The no-acquirer entry point (byte-identical to the earlier flow):
/// delegates to [`ros_attach_with_acquirer`] with the [`NoopAcquirer`], which
/// acquires nothing, so the report + writes are unchanged. The binary calls
/// `_with_acquirer` to wire the live wire rung.
pub fn ros_attach(
    discovery: &dyn DdsDiscovery,
    workspace_root: &Path,
    opts: &RosAttachOptions,
    is_tty: bool,
    confirm: &mut dyn FnMut(&str) -> CliResult<bool>,
) -> CliResult<RosAttachReport> {
    ros_attach_with_acquirer(
        discovery,
        &NoopAcquirer,
        workspace_root,
        opts,
        is_tty,
        confirm,
    )
}

/// `cerulion ros2 attach` over an injected [`SchemaAcquirer`]
/// — discover, partition, RUN THE ACQUISITION LADDER over the unresolvable
/// set, re-resolve, generate, consent, hand off to run. Pure but for the
/// injected `discovery` backend, `acquirer`, and `confirm` seam; `is_tty` is
/// threaded (not read here) so the refusal arm is testable.
///
/// The acquisition ladder stages harvested `.msg` schemas IN MEMORY and
/// re-resolves against them, so the report is accurate about resolvability, but
/// NO file is written before consent. Materialized `.msg` files are a THIRD
/// write class on the SAME ladder as the graph + config (the never-mutate
/// floor): `dry_run` → report only (writes NOTHING, not even a `schemas/`
/// dir); no resolvable → report only; `assume_yes` → write graph + config +
/// each acquired `.msg` (+`.bak`); no TTY and no `--yes` → loud `Err` naming
/// `--yes`/`--dry-run`; interactive → `confirm(preview)`.
pub fn ros_attach_with_acquirer(
    discovery: &dyn DdsDiscovery,
    acquirer: &dyn SchemaAcquirer,
    workspace_root: &Path,
    opts: &RosAttachOptions,
    is_tty: bool,
    confirm: &mut dyn FnMut(&str) -> CliResult<bool>,
) -> CliResult<RosAttachReport> {
    validate_graph_name(&opts.graph_name)?;

    // Validate/normalize --topic-prefix BEFORE discovery
    // runs and BEFORE anything can be generated or written. The binary
    // pre-normalizes and prints a stderr note; this is the
    // enforcement backstop (idempotent for already-normalized input; a loud
    // warn when a direct engine caller relies on the normalization here; a
    // hard Err for shapes that stay invalid).
    let normalized_prefix = match opts.topic_prefix.as_deref() {
        Some(raw) => {
            let n = normalize_topic_prefix(raw)?;
            if n.was_normalized {
                tracing::warn!(
                    raw = %raw,
                    normalized = %n.prefix,
                    "ros2 attach: --topic-prefix had no leading '/' — normalized (pass the \
                     leading '/' to silence this warning)"
                );
            }
            Some(n.prefix)
        }
        None => None,
    };

    let params = DiscoveryParams {
        only_networks: vec![opts.iface],
        domain_id: opts.domain_id,
        window: opts.window,
    };
    tracing::info!(
        iface = %opts.iface,
        domain = opts.domain_id,
        window_ms = opts.window.as_millis() as u64,
        "ros2 attach: starting DDS discovery"
    );
    let discovered = discovery.discover(&params).map_err(|e| {
        CliError::Validation(format!(
            "ros2 attach: DDS discovery failed: {e} — check the robot is on this LAN, --iface is \
             the correct local interface IP, and --domain matches the robot's ROS_DOMAIN_ID"
        ))
    })?;
    let topics = aggregate_topics(&discovered.endpoints);
    // Resolve against the workspace `.msg` store FIRST, then
    // the built-in registry — so a type dropped in schemas/<pkg>/msg/ (by hand
    // or by the acquisition ladder) resolves at attach. An
    // empty/absent store reduces to the earlier builtins-only behavior.
    let mut chain = AttachSchemaChain::from_workspace(workspace_root);
    // Pre-acquisition partition: identify the UNRESOLVABLE set the ladder runs
    // over (store + builtins only — no staging yet).
    let pre = partition_topics_with(&chain, &topics);
    // Run the acquisition ladder over the unresolvable types
    // (deduped), stage every acquired schema's closure IN MEMORY, and
    // re-resolve — so the report is accurate about resolvability WITHOUT any file
    // being written before consent (the never-mutate floor).
    let acq = run_acquisition_ladder(acquirer, &chain, &pre.unresolvable, &discovered);
    // The migration section's "restartable today" evidence: the type set that
    // resolved BEFORE the acquisition ladder ran (store + builtins + typed
    // registry). Captured here because `pre` is the last pre-staging view.
    let locally_resolvable_types: BTreeSet<String> = pre
        .resolvable
        .iter()
        .map(|r| r.topic.ros_type.clone())
        .collect();
    let newly: Vec<String> = acq.newly_resolvable().map(str::to_string).collect();
    chain.stage_all(newly);
    let mut part = partition_topics_with(&chain, &topics);
    // Gate every resolvable candidate topic through the graph
    // layer's own absolute-name predicate BEFORE the mapping build — a
    // hostile/nonconforming raw DDS topic name (`*`, `//`, ...) must land in
    // the loudly-reported EXCLUDED bucket, never in a written `topic:`
    // override that dies at graph load after the consent write.
    let (clean_resolvable, malformed) = exclude_malformed_topics(
        std::mem::take(&mut part.resolvable),
        normalized_prefix.as_deref(),
    );
    // The migration section's "the bridge will actually carry it" evidence:
    // the post-staging resolvable type set AFTER malformed-topic exclusion.
    // Captured HERE (not off the pre-exclusion partition) so an acquired type
    // whose only topics were malformed-excluded is never labeled after-write
    // restartable — a run bridging nothing refuses the consent write, so
    // nothing would be materialized for it. The later typed-port passes drop
    // TOPICS, never a type's resolvability, so this is the final type set.
    let resolvable_types: BTreeSet<String> = clean_resolvable
        .iter()
        .map(|r| r.topic.ros_type.clone())
        .collect();
    // Race same-type siblings for each registry type's ONE typed port
    // (streaming preference — writer presence first, then the writer-set min
    // durability rank: a live VOLATILE stream beats a latched TRANSIENT_LOCAL
    // near-static map — then writer count, then lexicographic). The winner
    // keeps its typed port; every same-type sibling DEGRADES to the generic
    // codec on its own topic (never dropped) unless an incomplete store shadow
    // blocks it — then NOT bridged, loudly. Without the race, on a Unitree Go2
    // `/uslam/cloud_map` alphabetically steals the PointCloud2 port and
    // the live `/utlidar/cloud` lidar vanishes from the wire.
    let mut assignment = assign_typed_ports(&chain, clean_resolvable);
    // Doomed-run guard: an OLDER vendored
    // dds_bridge rejects the WHOLE config on the unknown `route` key
    // (deny_unknown_fields), so zero topics are bridged. Degrade to the original drop
    // (loud, still bridging everything else) instead of writing a
    // config the old bridge cannot load.
    let old_bridge_degrade =
        !assignment.raw_routed.is_empty() && !vendored_bridge_supports_route_mode(workspace_root);
    if old_bridge_degrade {
        tracing::warn!(
            losers = assignment.raw_routed.len(),
            "ros2 attach: this workspace's vendored dds_bridge does not support `route: raw` — \
             same-type siblings that lost the typed-port race will NOT be bridged. \
             Re-copy the node type from the Cerulion repo \
             (examples/go2/nodes/dds_bridge/ into nodes/dds_bridge/) to bridge them via the \
             generic codec"
        );
        assignment = degrade_raw_routed_for_old_bridge(assignment);
    }
    part.resolvable = assignment.resolvable;
    let raw_routed = assignment.raw_routed;
    let dropped = assignment.unbridgeable;
    let (mappings, topic_conflicts) =
        build_mappings(&part.resolvable, normalized_prefix.as_deref());
    // Derive the graph `prefix:` — the robot-name
    // namespace LABEL. On an attach graph the bridged topics are absolute
    // mirrors (never prefixed), so this prefix drives no topic names, and the
    // network announce identity resolves SEPARATELY from the hostname /
    // `CERULION_ROBOT_IDENTITY` at runtime (`resolve_robot_identity`, which
    // ignores the prefix). It must still be the ROBOT's name, not the graph
    // filename.
    // Derived from the bridged ROS topics (each mapping's `dds_topic` is the
    // canonical ROS topic), the `--robot-name` override, or this machine's
    // hostname — never the graph name. Computed BEFORE every return arm so the
    // report carries the prefix the generated graph will (or would) use.
    let ros_topics: Vec<&str> = mappings.iter().map(|m| m.dds_topic.as_str()).collect();
    let robot_identity = derive_robot_identity(
        opts.robot_name.as_deref(),
        &ros_topics,
        &cerulion_core::graph::default_prefix(""),
    );
    let mut report = render_discovery_report_inner(
        &params,
        &topics,
        &part,
        &dropped,
        &raw_routed,
        &topic_conflicts,
        &malformed,
        discovered.own_endpoints_hidden,
        &acq,
    );
    // The old-vendored-bridge degrade is a preflight
    // condition the operator must see at decision time (dry-run AND consent
    // preview), naming the incompatibility + the remediation.
    if old_bridge_degrade {
        report.push_str(
            "\nnote: this workspace's vendored `dds_bridge` node does not support \
             `route: raw`, so same-type siblings that lost the typed-port race are NOT bridged \
             (see PORT CONFLICTS) — writing `route: raw` would make the old bridge reject the \
             whole config at graph run. Re-copy the node type from the Cerulion repo \
             (examples/go2/nodes/dds_bridge/ into nodes/dds_bridge/) and re-run attach to bridge \
             them via the generic codec.\n",
        );
    }

    // Automatic, no flag: the MIGRATION section
    // prints on EVERY attach run — after the discovery/partition report,
    // before the consent gate (it rides `report`, which every outcome arm
    // prints and the consent preview embeds). Pure rendering over data this
    // run already holds; never blocks, never touches the network, and the
    // outcome/exit contract is untouched.
    report.push_str(&render_migration_report(&MigrationReportInputs {
        nodes: &discovered.nodes,
        endpoints: &discovered.endpoints,
        locally_resolvable: &locally_resolvable_types,
        resolvable: &resolvable_types,
    }));

    // --dry-run: the "what would I get" view; write nothing, run nothing.
    if opts.dry_run {
        return Ok(RosAttachReport {
            preview: report.clone(),
            report,
            topics,
            mappings,
            outcome: AttachOutcome::DryRun,
            preview_shown: false,
            graph_to_run: None,
            robot_identity,
        });
    }

    // Nothing resolvable ⇒ nothing to bridge. Loud, never a silent empty graph.
    if mappings.is_empty() {
        tracing::warn!(
            "ros2 attach: no resolvable topics — nothing to bridge. Nothing was written. See the \
             UNRESOLVABLE list above; add a schema for the listed types, or point --iface/--domain \
             at the robot."
        );
        return Ok(RosAttachReport {
            preview: report.clone(),
            report,
            topics,
            mappings,
            outcome: AttachOutcome::NothingResolvable,
            preview_shown: false,
            graph_to_run: None,
            robot_identity,
        });
    }

    let graphs_dir = workspace_root.join("graphs");
    let graph_path = graphs_dir.join(format!("{}.yaml", opts.graph_name));
    let config_path = graphs_dir.join(format!("{}.bridge.yaml", opts.graph_name));

    // Emit `msg_dirs:` whenever the store will be NON-EMPTY
    // at run time — it already holds schemas OR this run materializes acquired
    // ones into it (the SAME consent batch — schemas are written before the
    // config). Deliberately conservative: a store schema no
    // current mapping uses still triggers emission, which only widens the
    // bridge codec's schema set. Absent otherwise (today's output is
    // byte-identical when no store is involved — the back-compat floor).
    let store_dirs: &[&str] = if chain.store_non_empty() || !acq.to_write.is_empty() {
        &[BRIDGE_STORE_MSG_DIR]
    } else {
        &[]
    };
    let config_yaml = generate_bridge_config_with_store(
        opts.domain_id,
        &params.only_networks,
        &mappings,
        store_dirs,
    );
    let graph_yaml = generate_bridge_graph(&opts.graph_name, &robot_identity, &mappings);

    // The acquired `.msg` files are a THIRD write class. The
    // preview NAMES + shows each verbatim body (transparent consent, mirroring
    // the graph/config blocks); when none were acquired the block is empty and
    // the "Will write TWO files:" line is byte-identical to the earlier
    // preview.
    let will_write_line = if acq.to_write.is_empty() {
        "Will write TWO files:".to_string()
    } else {
        format!(
            "Will write TWO files + {} acquired schema file(s):",
            acq.to_write.len()
        )
    };
    let mut schema_files_block = String::new();
    for f in &acq.to_write {
        // No template "via " — the rung label is already prose (see the
        // ACQUIRED SCHEMAS render note); otherwise the header double-"via"s.
        schema_files_block.push_str(&format!(
            "── {} (acquired schema, {}) ──\n{}\n",
            f.workspace_rel, f.rung_label, f.text
        ));
    }
    // The graph declares exactly ONE node type. The preview names it
    // and nothing else — a preview naming a type the graph does not declare is
    // the doomed-run class this note exists to prevent.
    let node_types_note = "the `dds_bridge` node type must exist in this workspace.";
    let preview = format!(
        "{report}\n\
         {will_write_line}\n\
         \n\
         ── graphs/{name}.yaml ──\n{graph_yaml}\n\
         ── graphs/{name}.bridge.yaml ──\n{config_yaml}\n\
         {schema_files_block}\
         Then run: cerulion graph run {name} --single-process\n\
         ({node_types_note})\n",
        name = opts.graph_name,
    );

    // The write decision ladder (the consent floor).
    // Write the acquired `.msg` SCHEMAS FIRST, then the graph + config — the
    // graph + config REFERENCE the schemas, so writing the leaf schemas first
    // makes every partial-failure state benign (extra store files, never a
    // dangling reference). The batch is per-file atomic but NOT transactional,
    // so [`write_with_context`] wraps every mid-batch failure with the exact
    // failing path + the files ALREADY written + the re-run remediation.
    let write_files = || -> CliResult<AttachOutcome> {
        // The batch rewrites graphs/<g>.yaml, so it holds the same workspace
        // lock `cerulion-wsd` and the other CLI writers serialize on.
        let _lock =
            crate::workspace_lock::WorkspaceLock::acquire_and_track_gitignore(workspace_root)?;
        let mut schema_writes = Vec::with_capacity(acq.to_write.len());
        let mut written_paths: Vec<PathBuf> = Vec::new();
        for f in &acq.to_write {
            // Build the dest from path COMPONENTS (never string-join) so the
            // store layout is correct on every platform.
            let dest = workspace_root
                .join("schemas")
                .join(&f.package)
                .join("msg")
                .join(format!("{}.msg", f.type_name));
            let backup =
                write_with_context(&dest, &f.text, "acquired schema file", &written_paths)?;
            tracing::info!(
                schema = %f.qualified,
                path = %dest.display(),
                rung = %f.rung_label,
                "ros2 attach: materialized acquired schema into the workspace .msg store"
            );
            written_paths.push(dest.clone());
            schema_writes.push(SchemaWrite { path: dest, backup });
        }
        let graph_backup =
            write_with_context(&graph_path, &graph_yaml, "graph file", &written_paths)?;
        written_paths.push(graph_path.clone());
        let config_backup = write_with_context(
            &config_path,
            &config_yaml,
            "bridge config file",
            &written_paths,
        )?;
        Ok(AttachOutcome::Written {
            graph_path: graph_path.clone(),
            graph_backup,
            config_path: config_path.clone(),
            config_backup,
            schema_writes,
        })
    };

    let mut preview_shown = false;
    let outcome = if opts.assume_yes {
        write_files()?
    } else if !is_tty {
        // Name the THIRD write class (the acquired
        // `.msg` schemas) alongside the graph + config so a CI operator deciding
        // whether to re-run with --yes consents to exactly what --yes writes.
        let schema_note = if acq.to_write.is_empty() {
            String::new()
        } else {
            let paths: Vec<&str> = acq
                .to_write
                .iter()
                .map(|f| f.workspace_rel.as_str())
                .collect();
            format!(
                " plus {} acquired schema file(s) ({})",
                paths.len(),
                paths.join(", ")
            )
        };
        return Err(CliError::Validation(format!(
            "ros2 attach would write '{}' and '{}'{schema_note} but stdin is not a TTY and --yes \
             was not passed — nothing was written. Re-run with --yes to apply non-interactively \
             (scripts/CI), or --dry-run to inspect the discovery report without writing.",
            graph_path.display(),
            config_path.display()
        )));
    } else {
        preview_shown = true;
        if confirm(&preview)? {
            write_files()?
        } else {
            AttachOutcome::Declined
        }
    };

    let graph_to_run = if matches!(outcome, AttachOutcome::Written { .. }) {
        Some(opts.graph_name.clone())
    } else {
        None
    };

    Ok(RosAttachReport {
        report,
        preview,
        topics,
        mappings,
        outcome,
        preview_shown,
        graph_to_run,
        robot_identity,
    })
}

/// The outcome of [`normalize_topic_prefix`].
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct NormalizedPrefix {
    /// The canonical (leading-`/`) prefix to use.
    pub prefix: String,
    /// True when a missing leading `/` was prepended — the caller owes the
    /// user a loud note (the bare-name-resolves-with-note precedent).
    pub was_normalized: bool,
}

/// Validate/normalize a `--topic-prefix` value.
///
/// Unvalidated, a prefix without a leading `/` produces a RELATIVE
/// `cerulion_topic` that `BridgeConfig::validate` AND `validate_graph` reject
/// — but only AFTER the consent ladder has already written both files (a
/// mutated workspace for a run that can never succeed). This runs at the
/// entry, BEFORE anything is generated or written:
///
/// * missing leading `/` → prepended, `was_normalized = true` (the binary
///   prints a stderr `note:`; the engine backstop emits a
///   `tracing::warn!`);
/// * whitespace anywhere, a trailing `/`, or empty-after-normalize (`""` /
///   `"/"`) → loud `Err` with the exact remediation;
/// * anything the GRAPH LAYER's own absolute-name predicate
///   (`cerulion_core::graph::malformed_absolute_name`) would reject — probed
///   with `prefix + "/probe"` and surfaced with ITS reason verbatim, so this
///   reject set can never drift NARROWER than graph load's (interior `//`,
///   zenoh-reserved `* ? # $ @`, and any future rule, for free).
pub fn normalize_topic_prefix(raw: &str) -> CliResult<NormalizedPrefix> {
    if raw.chars().any(char::is_whitespace) {
        return Err(CliError::Validation(format!(
            "ros2 attach: invalid --topic-prefix {raw:?} — whitespace is not allowed in a topic \
             prefix; use a plain segment path like /go2"
        )));
    }
    let (prefix, was_normalized) = if raw.starts_with('/') {
        (raw.to_string(), false)
    } else {
        (format!("/{raw}"), true)
    };
    if prefix == "/" {
        return Err(CliError::Validation(format!(
            "ros2 attach: invalid --topic-prefix {raw:?} — the prefix is empty after \
             normalization; pass a real segment path like /go2, or omit the flag to mirror \
             the ROS topic names"
        )));
    }
    if prefix.ends_with('/') {
        return Err(CliError::Validation(format!(
            "ros2 attach: invalid --topic-prefix {raw:?} — drop the trailing '/' (the prefix \
             is joined to ROS topic names that already start with '/'; a trailing slash would \
             produce a '//' in every generated topic)"
        )));
    }
    // The SINGLE-SOURCE-OF-TRUTH backstop. The graph layer
    // validates every absolute `topic:` override with ONE shape predicate
    // (`cerulion_core::graph::malformed_absolute_name`). A prefix that slips
    // past the edge rules above but fails that predicate (interior '//',
    // zenoh-reserved chars, ...) would write both files and then die at graph
    // load — the exact mutate-for-a-doomed-run class this function exists to kill.
    // Probe the predicate with a topic built from the prefix and surface ITS
    // reason verbatim; a future graph-layer rule is inherited here for free
    // instead of drifting past a second hand-rolled list.
    let probe = format!("{prefix}/probe");
    if let Some(why) = cerulion_core::graph::malformed_absolute_name(&probe) {
        return Err(CliError::Validation(format!(
            "ros2 attach: invalid --topic-prefix {raw:?} — every generated topic would be \
             rejected at graph load: {why}. Use a plain segment path like /go2."
        )));
    }
    Ok(NormalizedPrefix {
        prefix,
        was_normalized,
    })
}

/// Reject a graph name that would escape `graphs/` or produce a bad file name.
/// (The name becomes `graphs/<name>.yaml` and `graphs/<name>.bridge.yaml`.)
fn validate_graph_name(name: &str) -> CliResult<()> {
    if name.is_empty()
        || name.contains('/')
        || name.contains('\\')
        || name.contains("..")
        || name.contains(char::is_whitespace)
    {
        return Err(CliError::Validation(format!(
            "ros2 attach: invalid --graph-name {name:?} — use a plain file-stem (letters, \
             digits, '_', '-'); it becomes graphs/<name>.yaml + graphs/<name>.bridge.yaml"
        )));
    }
    Ok(())
}

/// Write one consent file, converting a mid-batch failure into a LOUD,
/// ACCOUNTABLE error.
/// [`write_yaml_atomically`](crate::graph_cmd::write_yaml_atomically)
/// returns a PATHLESS `io::Error`; this wraps it with the exact destination
/// that failed, the paths ALREADY written this run, and the partial-write
/// remediation. The batch is per-file atomic but NOT transactional, so the
/// error states plainly which files exist and that re-running `ros2 attach`
/// re-writes every file (backing up each existing one with a `.bak`) — the
/// outcome never claims success on a partial write.
fn write_with_context(
    dest: &Path,
    contents: &str,
    file_kind: &str,
    already_written: &[PathBuf],
) -> CliResult<Option<PathBuf>> {
    // `ros2 attach` GENERATES every file it writes from a live discovery pass —
    // it never reads the destination — so there is no prior content this write
    // depends on and nothing for the concurrent-change check to hold.
    crate::graph_cmd::write_yaml_atomically(
        dest,
        contents,
        file_kind,
        crate::graph_cmd::ExpectedPrior::Unchecked,
    )
    .map_err(|e| {
        let done = if already_written.is_empty() {
            "no files were written before this failure".to_string()
        } else {
            format!(
                "these files WERE already written: {}",
                already_written
                    .iter()
                    .map(|p| p.display().to_string())
                    .collect::<Vec<_>>()
                    .join(", ")
            )
        };
        tracing::error!(
            path = %dest.display(),
            file_kind = %file_kind,
            error = %e,
            "ros2 attach: consent write FAILED mid-batch — the workspace may be partially written"
        );
        CliError::Validation(format!(
            "ros2 attach: failed to write the {file_kind} '{}': {e}. {done}. The remaining files \
             were NOT written; the write is not transactional — fix the cause (permissions/disk \
             space) and re-run `cerulion ros2 attach`, which re-writes every file (backing up any \
             existing one with a .bak).",
            dest.display()
        ))
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The trailing-newline restore's oracle vector — the
    /// live no-newline REP-2011 payload gains exactly one `'\n'`; an
    /// already-newlined payload is byte-unchanged (never doubled, trailing
    /// blank lines never trimmed); the empty edge passes through unchanged
    /// (the upstream `validate_acquired_member` emptiness guard keeps it from
    /// the write anyway — no invented behavior).
    #[test]
    fn restore_trailing_newline_oracle() {
        // The live shape: rcl embeds the .msg text without the final newline.
        assert_eq!(
            restore_trailing_newline("uint32 seq\nstring label"),
            "uint32 seq\nstring label\n"
        );
        // Already POSIX form → byte-identical (no doubling).
        assert_eq!(
            restore_trailing_newline("uint32 seq\nstring label\n"),
            "uint32 seq\nstring label\n"
        );
        // Multiple trailing newlines are the payload's own bytes — untouched.
        assert_eq!(restore_trailing_newline("int32 x\n\n"), "int32 x\n\n");
        // Single-line no-newline payload.
        assert_eq!(restore_trailing_newline("int32 x"), "int32 x\n");
        // Empty edge: unchanged (guarded out upstream; nothing invented).
        assert_eq!(restore_trailing_newline(""), "");
    }

    /// The newline-form-insensitive duplicate compare —
    /// trailing-newline-only deltas are EQUAL (rcl's one-newline strip makes
    /// them legitimately coexist across rungs: bare wire payload, restored
    /// `x\n`, blank-line-terminated ament original `x\n\n`); any
    /// leading/interior difference — content, spacing, or a newline anywhere
    /// but the tail — still compares unequal.
    #[test]
    fn eq_ignoring_trailing_newlines_oracle() {
        // Trailing-newline-form deltas: all equal.
        assert!(eq_ignoring_trailing_newlines("int32 x", "int32 x\n"));
        assert!(eq_ignoring_trailing_newlines("int32 x\n", "int32 x\n\n"));
        assert!(eq_ignoring_trailing_newlines("int32 x\n\n\n", "int32 x"));
        assert!(eq_ignoring_trailing_newlines("int32 x\n", "int32 x\n"));
        // Genuine content difference: unequal (the refusal still fires).
        assert!(!eq_ignoring_trailing_newlines("float64 x\n", "float64 y\n"));
        // Interior newlines are content, not form.
        assert!(!eq_ignoring_trailing_newlines(
            "int32 a\nint32 b\n",
            "int32 a\n\nint32 b\n"
        ));
        // Interior/leading whitespace is content.
        assert!(!eq_ignoring_trailing_newlines("int32  x\n", "int32 x\n"));
        // Empty-vs-newline-only: equal in FORM (both empty content — such
        // members are rejected upstream by validate_acquired_member anyway).
        assert!(eq_ignoring_trailing_newlines("", "\n"));
    }

    #[test]
    fn normalize_dds_topic_strips_ros_prefixes() {
        assert_eq!(normalize_dds_topic("rt/utlidar/cloud"), "/utlidar/cloud");
        assert_eq!(normalize_dds_topic("rq/add_two_ints"), "/add_two_ints");
        assert_eq!(normalize_dds_topic("rr/add_two_ints"), "/add_two_ints");
        // No known prefix: prepend '/', collapse a pre-existing leading '/'.
        assert_eq!(normalize_dds_topic("cmd_vel"), "/cmd_vel");
        assert_eq!(normalize_dds_topic("/already"), "/already");
    }

    #[test]
    fn normalize_ros_type_folds_every_discovery_shape() {
        // IDL-mangled CDR form.
        assert_eq!(
            normalize_ros_type("sensor_msgs::msg::dds_::PointCloud2_"),
            "sensor_msgs/PointCloud2"
        );
        // ROS slash + colon forms.
        assert_eq!(
            normalize_ros_type("sensor_msgs/msg/PointCloud2"),
            "sensor_msgs/PointCloud2"
        );
        assert_eq!(
            normalize_ros_type("sensor_msgs::msg::PointCloud2"),
            "sensor_msgs/PointCloud2"
        );
        // Already canonical.
        assert_eq!(
            normalize_ros_type("geometry_msgs/Twist"),
            "geometry_msgs/Twist"
        );
        // Unitree custom type through the mangled form.
        assert_eq!(
            normalize_ros_type("unitree_go::msg::dds_::SportModeState_"),
            "unitree_go/SportModeState"
        );
        // Un-foldable shapes returned trimmed as-is (surface as unresolvable).
        assert_eq!(normalize_ros_type("Foo"), "Foo");
        assert_eq!(normalize_ros_type("  "), "");
    }

    /// `normalize_ros_type` (engine) and
    /// `cerulion_dds::wire::normalize_dds_type` (the leaf duplicate the wire rung
    /// matches endpoints with) MUST agree — a divergence would make the wire
    /// rung's requested `pkg/Type` names miss the endpoints they were derived
    /// from, silently skipping types (surfacing as the new
    /// `TYPE_NOT_DISCOVERED_REASON`). Both are asserted against ONE hand oracle
    /// (a shared table), not just against each other — a shared bug in both
    /// would still fail the oracle. Placed HERE because both fns are visible
    /// (the engine depends on cerulion_dds, and `wire` is always-compiled,
    /// DDS-free).
    #[test]
    fn normalize_dds_and_ros_type_parity_against_hand_oracle() {
        // (raw DDS type name, expected canonical pkg/Type) — the hand oracle,
        // covering the interesting classes: dds_ CDR infix, ::msg:: vs slash,
        // trailing underscore, nested namespaces, already-canonical, non-ROS.
        let cases: &[(&str, &str)] = &[
            // CDR-mangled (dds_ infix + trailing underscore).
            (
                "sensor_msgs::msg::dds_::PointCloud2_",
                "sensor_msgs/PointCloud2",
            ),
            (
                "unitree_go::msg::dds_::SportModeState_",
                "unitree_go/SportModeState",
            ),
            // ROS slash form (::msg:: analogue via /msg/).
            ("sensor_msgs/msg/PointCloud2", "sensor_msgs/PointCloud2"),
            // ROS colon form (::msg::).
            ("sensor_msgs::msg::PointCloud2", "sensor_msgs/PointCloud2"),
            // Service-namespace CDR shape (::srv::).
            (
                "example_interfaces::srv::dds_::AddTwoInts_",
                "example_interfaces/AddTwoInts",
            ),
            // Already canonical.
            ("geometry_msgs/Twist", "geometry_msgs/Twist"),
            // Nested-namespace-looking custom type.
            ("acme_msgs::msg::dds_::Widget_", "acme_msgs/Widget"),
            // Non-ROS / un-foldable names pass through trimmed.
            ("Foo", "Foo"),
        ];
        for (raw, expected) in cases {
            assert_eq!(
                normalize_ros_type(raw),
                *expected,
                "engine normalize_ros_type({raw:?})"
            );
            assert_eq!(
                cerulion_dds::wire::normalize_dds_type(raw),
                *expected,
                "leaf wire::normalize_dds_type({raw:?})"
            );
        }
    }

    #[test]
    fn resolvability_seam_two_armed_typed_then_raw() {
        // The four registry types route TYPED (typed wins).
        for b in V1_BRIDGE_BINDINGS.iter() {
            assert_eq!(
                resolve_bridge_binding(b.ros_type).map(|x| x.port),
                Some(b.port)
            );
            assert!(
                matches!(resolve_bridge_route(b.ros_type), Some(BridgeRoute::Typed(x)) if x.port == b.port),
                "{} must route Typed",
                b.ros_type
            );
        }
        // The raw-codec flip: any OTHER built-in ROS 2 message routes RAW.
        for raw in [
            "sensor_msgs/Imu",
            "sensor_msgs/LaserScan",
            "tf2_msgs/TFMessage",
        ] {
            assert_eq!(
                resolve_bridge_route(raw),
                Some(BridgeRoute::Raw),
                "{raw} is a built-in — must route Raw"
            );
            assert!(resolve_bridge_binding(raw).is_none(), "{raw} is not typed");
        }
        // Near-misses / garbage resolve NOWHERE (exact pkg/Type match; no
        // case forgiveness, no bare names, no unknown packages).
        for miss in [
            "sensor_msgs/pointcloud2",
            "PointCloud2",
            "acme_msgs/Widget",
            "sensor_msgs/msg/Imu",
            "",
        ] {
            assert!(
                resolve_bridge_route(miss).is_none(),
                "{miss:?} must not resolve"
            );
        }
    }

    #[test]
    fn graph_name_validation_rejects_traversal() {
        assert!(validate_graph_name("attach").is_ok());
        assert!(validate_graph_name("go2_attach").is_ok());
        for bad in ["", "a/b", "../x", "a b", "a\\b"] {
            assert!(
                validate_graph_name(bad).is_err(),
                "{bad:?} must be rejected"
            );
        }
    }

    /// The prefix normalizer's oracle vector — the
    /// normalize arm (missing '/' prepended + flagged), the pass-through arm
    /// (canonical input untouched, NOT flagged), and every reject arm
    /// (whitespace / trailing '/' / empty-after-normalize) with the flag
    /// naming `--topic-prefix` in the message.
    #[test]
    fn topic_prefix_normalizer_oracle_vector() {
        // Normalize arm: leading '/' prepended, flagged for the note.
        assert_eq!(
            normalize_topic_prefix("go2").unwrap(),
            NormalizedPrefix {
                prefix: "/go2".to_string(),
                was_normalized: true,
            }
        );
        assert_eq!(
            normalize_topic_prefix("go2/ns").unwrap(),
            NormalizedPrefix {
                prefix: "/go2/ns".to_string(),
                was_normalized: true,
            }
        );
        // Pass-through arm: canonical input is untouched and NOT flagged
        // (idempotence — the engine backstop after the binary pre-normalizes).
        assert_eq!(
            normalize_topic_prefix("/go2").unwrap(),
            NormalizedPrefix {
                prefix: "/go2".to_string(),
                was_normalized: false,
            }
        );
        // Reject arms: whitespace, trailing '/', empty-after-normalize.
        for bad in ["go2/", "/go2/", "a b", "/a b", "", "/", "\t", "go2 "] {
            let err = normalize_topic_prefix(bad).expect_err(&format!("{bad:?} must be rejected"));
            assert!(
                err.to_string().contains("--topic-prefix"),
                "error must name the flag: {err}"
            );
        }
        // The prefix-normalizer backstop arms: shapes the LOCAL edge rules miss but the graph
        // layer's own predicate rejects — the probe backstop must surface the
        // graph predicate's reason verbatim (single source of truth; without
        // the probe both PASS here and doom the generated files at graph load).
        let err = normalize_topic_prefix("/a//b").expect_err("interior '//' must be rejected");
        assert!(
            err.to_string().contains("empty segment"),
            "the graph predicate's reason must surface: {err}"
        );
        assert!(err.to_string().contains("--topic-prefix"), "{err}");
        for bad in ["/cam*", "cam?", "/a#b", "/a$b", "/a@b"] {
            let err = normalize_topic_prefix(bad)
                .expect_err(&format!("{bad:?} must be rejected (zenoh-reserved)"));
            assert!(
                err.to_string().contains("zenoh-reserved"),
                "the graph predicate's reason must surface for {bad:?}: {err}"
            );
        }
    }

    // ─────────────────────────── Graph-gen fixtures ─────────────────────────

    /// A raw-route mapping on `cerulion_topic` — a RAW route rides the bridge's
    /// dynamically-created ingress publishers and touches NO graph port, so it
    /// must leave the emitted graph byte-identical.
    fn raw_mapping(cerulion_topic: &str) -> BridgeMapping {
        BridgeMapping {
            dds_topic: cerulion_topic.to_string(),
            ros_type: "pkg/Type".to_string(),
            cerulion_topic: cerulion_topic.to_string(),
            route: BridgeRoute::Raw,
        }
    }

    // ───────────────── Robot identity derivation ──────────

    #[test]
    fn derive_identity_override_wins_when_valid() {
        // A valid --robot-name overrides everything, TRIMMED — even against a
        // set that would otherwise derive a shared namespace.
        assert_eq!(
            derive_robot_identity(Some("  spot  "), &["/utlidar/x", "/lf/y"], "host"),
            "spot"
        );
    }

    #[test]
    #[tracing_test::traced_test]
    fn derive_identity_rejected_override_falls_back_and_warns() {
        // A name failing the allow-list → fallback + a loud warn naming it.
        assert_eq!(
            derive_robot_identity(Some("bad/name"), &["/spot/a"], "host"),
            "host"
        );
        assert!(logs_contain("--robot-name failed the identity check"));
        assert!(logs_contain("bad/name"));

        // Representative REJECTED classes (each with topics that WOULD
        // otherwise derive `/spot` — so the fallback is the override's rejection,
        // never a lucky derivation). The allow-list is `[A-Za-z0-9_.-]`.
        let topics = &["/spot/a", "/spot/b"];
        for rejected in [
            "",         // empty
            "   ",      // whitespace-only
            "a b",      // interior whitespace
            "@robot",   // YAML indicator
            "!foo",     // YAML tag indicator
            "&anchor",  // YAML anchor indicator
            "%tag",     // YAML directive indicator
            "a*b",      // zenoh key-expr special
            "a$b",      // zenoh key-expr special
            "a?b",      // zenoh key-expr special
            "a#b",      // zenoh key-expr special
            "ns/robot", // slash / path separator
        ] {
            assert_eq!(
                derive_robot_identity(Some(rejected), topics, "host"),
                "host",
                "`{rejected}` must be rejected to the hostname fallback"
            );
        }

        // …and VALID simple-identifier names are ACCEPTED (trimmed), overriding
        // even a derivable namespace (the anti-tautology control).
        for valid in ["go2", "lab-robot_1", "spot.2", "Robot-A"] {
            assert_eq!(
                derive_robot_identity(Some(valid), topics, "host"),
                valid,
                "`{valid}` is a valid simple identifier and must be accepted"
            );
        }
    }

    /// Round-trip (ties to the emit layer): the generated graph's
    /// `prefix:` is a serde-quoted YAML scalar, so even an adversarial identity
    /// that BYPASSED the `derive_robot_identity` allow-list via a direct
    /// generator call round-trips to EXACTLY that identity — never an
    /// unparseable graph (raw `@robot`) nor a silently-empty prefix (`!foo`,
    /// `&anchor`). Hand oracle: parse the generated graph back and confirm
    /// `cfg.prefix == identity` for each.
    #[test]
    fn emitted_prefix_round_trips_for_adversarial_identities() {
        for identity in ["@robot", "!foo", "&anchor", "%tag", "go2", "lab-robot_1"] {
            let yaml = generate_bridge_graph("attach", identity, &[]);
            let cfg: cerulion_core::graph::config::GraphConfig = serde_yaml::from_str(&yaml)
                .unwrap_or_else(|e| {
                    panic!("prefix `{identity}` produced an unparseable graph: {e}\n{yaml}")
                });
            assert_eq!(
                cfg.prefix, identity,
                "the emitted prefix must round-trip to the exact identity `{identity}`"
            );
        }
    }

    #[test]
    fn derive_identity_shared_namespace_wins_infra_ignored() {
        // Every non-infra topic under /spot ⇒ "spot"; /tf + /rosout ignored.
        assert_eq!(
            derive_robot_identity(None, &["/spot/a", "/spot/b", "/tf", "/rosout"], "host"),
            "spot"
        );
    }

    #[test]
    fn derive_identity_no_common_namespace_falls_back() {
        // The Go2 mix scatters across segments ⇒ hostname fallback (never the
        // graph name).
        assert_eq!(
            derive_robot_identity(
                None,
                &[
                    "/utlidar/cloud",
                    "/lf/lowstate",
                    "/uslam/pose",
                    "/api/request",
                ],
                "host"
            ),
            "host"
        );
    }

    #[test]
    fn derive_identity_edge_cases_fall_back_or_derive() {
        // Empty topic set → fallback.
        assert_eq!(derive_robot_identity(None, &[], "host"), "host");
        // A single non-infra topic → its namespace (a great free identity).
        assert_eq!(derive_robot_identity(None, &["/spot/scan"], "host"), "spot");
        // A set that is ONLY infra → fallback (no identity to derive).
        assert_eq!(derive_robot_identity(None, &["/tf"], "host"), "host");
        // A bare "/" (malformed) → no clean namespace → fallback.
        assert_eq!(derive_robot_identity(None, &["/"], "host"), "host");
    }

    #[test]
    fn generate_bridge_graph_matches_frozen_oracle() {
        // The FROZEN in-crate oracle for the ONE emitted graph shape: the
        // lean header (naming `dds_bridge` ONLY, pointing at the desk-side
        // viz path), the four default silent ports, and exactly one node.
        let expected = concat!(
            "# Generated by `cerulion ros2 attach` — the lean one-node bridge graph.\n",
            "# Runs the `dds_bridge` node; its DDS mappings come from graphs/attach.bridge.yaml\n",
            "# (set DDS_BRIDGE_CONFIG to it). The four outputs are the bridge's FIXED port set —\n",
            "# unmapped ports stay silent. The `dds_bridge` node type must exist in this workspace.\n",
            "# To SEE this robot's data, run `cerulion viz --robot <name>` (or Cerulion Studio) on\n",
            "# YOUR machine — visualization is desk-side; the robot only ships raw frames.\n",
            "prefix: attach\n",
            "nodes:\n",
            "  - id: bridge\n",
            "    type: dds_bridge\n",
            "    outputs:\n",
            "      - name: cloud\n",
            "        schema: sensor_msgs/PointCloud2\n",
            "        topic: /dds_bridge/cloud\n",
            "        max_slice_len: 1048576\n",
            "      - name: odom\n",
            "        schema: nav_msgs/Odometry\n",
            "        topic: /dds_bridge/odom\n",
            "      - name: twist\n",
            "        schema: geometry_msgs/TwistStamped\n",
            "        topic: /dds_bridge/twist\n",
            "      - name: request_json\n",
            "        schema: std_msgs/String\n",
            "        topic: /dds_bridge/request_json\n",
        );
        // No mappings ⇒ every port silent-default; a RAW mapping never touches
        // the port list, so /imu yields the SAME graph.
        assert_eq!(generate_bridge_graph("attach", "attach", &[]), expected);
        assert_eq!(
            generate_bridge_graph("attach", "attach", &[raw_mapping("/imu")]),
            expected
        );
    }

    /// `ros2 attach` stages NO visualization node
    /// on the robot — visualization is desk-side (`cerulion-vizd` over the
    /// netd mirror). The pin is deliberately mapping-DENSE: a generator
    /// that emitted a `rerun_sink` node would do so only with >= 1 mapping to
    /// visualize, so a shape with typed AND raw mappings is the one such
    /// code would grow a viz node on. Asserts the ABSENCE three ways —
    /// no `rerun_sink` type, no `id: viz` node, and exactly ONE node parsed
    /// back out of the YAML — plus the header/body consistency invariant:
    /// the emitted header may not advertise a
    /// node type the graph does not declare.
    #[test]
    fn generated_graph_stages_no_visualization_node_on_the_robot() {
        let mappings = [
            BridgeMapping {
                dds_topic: "/utlidar/cloud".to_string(),
                ros_type: "sensor_msgs/PointCloud2".to_string(),
                cerulion_topic: "/go2/utlidar/cloud".to_string(),
                route: BridgeRoute::Typed(&V1_BRIDGE_BINDINGS[0]),
            },
            raw_mapping("/imu_data"),
            raw_mapping("/tf"),
            raw_mapping("/tf_static"),
        ];
        let yaml = generate_bridge_graph("attach", "go2", &mappings);

        assert!(
            !yaml.contains("rerun_sink"),
            "the robot graph must stage no rerun_sink:\n{yaml}"
        );
        assert!(
            !yaml.contains("id: viz"),
            "the robot graph must declare no viz node:\n{yaml}"
        );
        // The typed mapping still reaches its port (anti-tautology: the graph
        // is a REAL bridge graph, not an empty one that trivially has no viz).
        assert!(
            yaml.contains("topic: /go2/utlidar/cloud"),
            "the typed mapping must still drive its port:\n{yaml}"
        );

        let cfg: cerulion_core::graph::config::GraphConfig =
            serde_yaml::from_str(&yaml).expect("the generated graph must parse");
        assert_eq!(
            cfg.nodes.len(),
            1,
            "an attach graph declares exactly one node (the bridge):\n{yaml}"
        );
        assert_eq!(cfg.nodes[0].node_type, "dds_bridge");

        // Header/body consistency, as SET EQUALITY over the types the header
        // actually names — not a fixed candidate list. A loop that
        // iterated `["dds_bridge", "rerun_sink"]` would have BOTH iterations
        // dominated by assertions above (`dds_bridge` by the `node_type` check,
        // `rerun_sink` by the absence check, which makes its `if` unreachable),
        // so a header naming a THIRD undeclared type would pass silently.
        let advertised = header_advertised_node_types(&leading_comment_block(&yaml));
        let declared: BTreeSet<String> = cfg.nodes.iter().map(|n| n.node_type.clone()).collect();
        assert_eq!(
            advertised, declared,
            "the header's advertised node types and the graph's declared ones must \
             be the SAME SET (a header naming a type nothing declares sends the user \
             hunting for a node crate that is not staged; a header naming none leaves \
             the one required crate undocumented):\n{yaml}"
        );
    }

    /// The leading `#` comment block of a generated graph.
    fn leading_comment_block(yaml: &str) -> String {
        yaml.lines()
            .take_while(|l| l.starts_with('#'))
            .collect::<Vec<_>>()
            .join("\n")
    }

    /// The node TYPES a generated header advertises.
    ///
    /// The emitter's ONE convention for naming a node type is a backtick-quoted
    /// token immediately followed by the word `node` — "Runs the `dds_bridge`
    /// node;", "The `dds_bridge` node type must exist". Everything else the
    /// header backticks is a command or a path (`cerulion ros2 attach`,
    /// `cerulion viz --robot <name>`), and is deliberately NOT collected: a
    /// bare "contains this substring" scan would make the invariant depend on
    /// prose rather than on what the header claims the graph declares.
    fn header_advertised_node_types(header: &str) -> BTreeSet<String> {
        let mut out = BTreeSet::new();
        let mut rest = header;
        while let Some(open) = rest.find('`') {
            let after = &rest[open + 1..];
            let Some(close) = after.find('`') else { break };
            let token = &after[..close];
            let tail = &after[close + 1..];
            let next_word = tail
                .split_whitespace()
                .next()
                .map(|w| w.trim_end_matches(|c: char| !c.is_alphanumeric()));
            if next_word == Some("node") && !token.is_empty() {
                out.insert(token.to_string());
            }
            rest = tail;
        }
        out
    }

    /// Oracle vectors for [`header_advertised_node_types`] — including the
    /// CRAFTED third type a fixed-list loop cannot see, and the two false
    /// positives a substring scan would produce.
    #[test]
    fn header_node_type_scan_collects_exactly_the_types_named_as_nodes() {
        let real = leading_comment_block(&generate_bridge_graph("attach", "go2", &[]));
        assert_eq!(
            header_advertised_node_types(&real),
            BTreeSet::from(["dds_bridge".to_string()]),
            "the generated header advertises exactly one node type"
        );

        // A crafted third type: the shape a fixed-list loop passes silently.
        let crafted = format!("{real}\n# Also runs the `phantom_sink` node.");
        assert_eq!(
            header_advertised_node_types(&crafted),
            BTreeSet::from(["dds_bridge".to_string(), "phantom_sink".to_string()]),
            "a third advertised type must be collected"
        );

        // Commands and paths in backticks are NOT node types.
        assert!(header_advertised_node_types(
            "# run `cerulion viz --robot <name>` (or Studio) and read `graphs/x.yaml` first"
        )
        .is_empty());
        // `node` must be the NEXT word, and an unterminated backtick terminates
        // the scan rather than swallowing the rest of the header.
        assert!(header_advertised_node_types("# the `dds_bridge` crate is a node").is_empty());
        assert!(header_advertised_node_types("# `unterminated node").is_empty());
        // Punctuation after the word is tolerated; the empty token is not one.
        assert_eq!(
            header_advertised_node_types("# runs the `a` node; and the `b` node,"),
            BTreeSet::from(["a".to_string(), "b".to_string()])
        );
        assert!(header_advertised_node_types("# `` node").is_empty());
    }
}
