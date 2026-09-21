// SPDX-License-Identifier: AGPL-3.0-only
//! YAML schema types for graph configuration.
//!
//! These types map directly to the graph YAML format.
//! The graph YAML file is the single source of truth (Principle #5).

use serde::{Deserialize, Serialize};

/// The identity served for a config nothing could name.
///
/// Reachable only when a YAML string is parsed with NO file to stem it from
/// (a bag's embedded `graph.yaml`, a hand-built literal) AND that YAML carried
/// no legacy [`name:`](GraphConfig::name) either. Serving a placeholder rather
/// than an empty string keeps a log field or an iceoryx2 node name readable
/// instead of silently truncating to nothing.
pub const UNNAMED_GRAPH: &str = "unnamed";

/// Top-level graph configuration parsed from YAML.
///
/// `#[serde(deny_unknown_fields)]`: graph YAML is the most
/// user-authored file format in the product, and until this attribute landed a
/// misspelled key (`proces_groups:`, `dpeth: 32`) was SILENTLY DROPPED by
/// serde while the graph ran with the intended setting absent. The macro side
/// has long rejected an unknown attribute with a `compile_error!` naming the
/// closed set; this is the YAML side of the same contract. Kept in
/// lockstep by `crates/cerulion_core/tests/config_deny_unknown_fields_test.rs`.
#[derive(Debug, Clone, Deserialize, Serialize)]
#[serde(deny_unknown_fields)]
pub struct GraphConfig {
    /// DEPRECATED: **the file stem is the
    /// graph name.** This key is OPTIONAL and IGNORED.
    ///
    /// It only ever did run-identity bookkeeping that duplicated the stem the
    /// CLI already resolved the graph by (`graphs/<stem>.yaml`), with nothing
    /// keeping the two in sync — so a graph whose file was renamed kept
    /// stamping run directories, bag stems, ring tags and `graph=` log fields
    /// with a name that matched no file on disk.
    ///
    /// It is KEPT on the struct so an old YAML still parses and still
    /// ROUND-TRIPS through `node stage` / `graph partition` rather than having
    /// a line silently deleted out from under the author. It is never read for
    /// identity: read [`identity()`](GraphConfig::identity) instead. Exactly
    /// two places touch it, both in the load path and both documented there —
    /// [`parse_graph_raw`](crate::graph::parse_graph_raw) SEEDS the identity
    /// from it (the only identity a stemless parse can know) and
    /// [`adopt_file_stem_identity`](crate::graph::adopt_file_stem_identity)
    /// warns when it DIVERGES from the stem.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub name: Option<String>,
    /// The graph's IDENTITY: the FILE STEM the CLI resolved it by.
    ///
    /// Every identity surface derives from this — run directories, bag stems,
    /// trace/state ring tags, Flashback capture names, `graph=` log fields,
    /// cost-snapshot and projection keys. NOT serialized (it is a property of
    /// how the config was LOADED, not of the document), so a config built from
    /// a YAML string with no file behind it carries whatever
    /// [`parse_graph_raw`](crate::graph::parse_graph_raw) could seed.
    ///
    /// Empty means UNKNOWN. Read it through
    /// [`identity()`](GraphConfig::identity), which serves [`UNNAMED_GRAPH`]
    /// rather than an empty string.
    #[serde(skip)]
    pub identity: String,
    #[serde(default, skip_serializing_if = "String::is_empty")]
    pub prefix: String,
    pub nodes: Vec<NodeDef>,
    /// ABSOLUTE topic names opted into multiple
    /// publishers (e.g. `multi_publisher_topics: [/tf]`). A listed topic
    /// with an in-graph producer relaxes the no-double-producer rule and
    /// provisions the shared loose publisher cap
    /// (`transport::MULTI_PUBLISHER_LOOSE_MAX`) instead of single-writer
    /// — there is deliberately NO per-topic count knob (you can't always
    /// know how many publishers a `/tf`-class topic will carry, and a
    /// shared constant keeps cross-graph open requirements consistent).
    /// Entries must be absolute (`/...`): derived names embed the node id
    /// and cannot be shared by construction. Validated at graph load;
    /// a listed topic with NO in-graph producer warns (external topics
    /// already admit multiple publishers — the listing has no effect).
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub multi_publisher_topics: Vec<String>,
    /// Cross-process partition declaration: a MAP of
    /// NAMED process groups (group name → node ids). Groups carry SEMANTIC
    /// names (`perception`, `local_planner`, …), so rank is NOT name-sorted.
    /// When present, EVERY node must appear in EXACTLY ONE group, and
    /// `cerulion graph run` deploys the groups exactly as written.
    ///
    /// Absent does NOT mean single-process. Under `cerulion graph run` on Unix
    /// with the real clock, an unpartitioned graph has a partition DERIVED for
    /// it (fused from `graphs/<name>.costs.yaml` when `cerulion graph profile`
    /// has written one, else one process per node) and runs multi-process.
    /// The derivation is written into the file only with consent (`--yes`, or
    /// `y` at the prompt); otherwise it is held in memory and the file is left
    /// untouched. `--single-process` opts out, and the virtual and external
    /// clocks and non-Unix hosts run one process. Only a caller that builds a
    /// `GraphRuntime` from this config directly, as the framework's tests do,
    /// gets one process from an absent block.
    ///
    /// `process_rank` (the cross-process trace-merge tiebreaker + barrier
    /// ordering) is the order the groups are LISTED here — the first group is
    /// rank 0, the next rank 1, and so on (preserved via the insertion-ordered
    /// `IndexMap`) — UNLESS [`process_group_order`](Self::process_group_order)
    /// is provided, in which case that explicit list defines the rank order.
    ///
    /// Written by hand, or by `cerulion graph partition <name>`, which emits
    /// this same shape (preview, then confirm; the previous file is kept as
    /// `.bak`).
    #[serde(default, skip_serializing_if = "indexmap::IndexMap::is_empty")]
    pub process_groups: indexmap::IndexMap<String, Vec<String>>,
    /// OPTIONAL explicit cross-process rank order — the
    /// group names from [`process_groups`](Self::process_groups) in the order
    /// you want them ranked (the first name is rank 0). When non-empty it MUST
    /// be a permutation of `process_groups`'s keys: every group appears EXACTLY
    /// once, no unknown names, no duplicates (enforced at graph load by
    /// `validate_process_groups`). When empty/absent, rank falls back to the
    /// `process_groups` declaration (listing) order.
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub process_group_order: Vec<String>,
    /// OPTIONAL per-node DAG-level assignment — a MAP of
    /// node id → level index (`level_assignments:` in graph YAML, mirroring
    /// `process_groups:`). When present, the runtime builds its executor
    /// levels FROM THIS MAP instead of the derived Kahn levelization
    /// (`GraphTopology::derive_levels`) — the baked output of the
    /// cost-aware refinement (`graph partition` emits it; hand-editable).
    /// `None` = absent = today's Kahn levelization, byte-identical.
    ///
    /// Hard contract, validated LOUDLY at graph build (hand-edited files are
    /// untrusted input — see `GraphTopology::levels_from_assignments`):
    /// every key names a graph node; EVERY node is covered (a partial map is
    /// ambiguous about intent and rejects — delete the whole block to fall
    /// back to Kahn); every trigger edge is strictly level-increasing; the
    /// assigned levels form a contiguous `0..K` range with no empty level
    /// (the multi-process barrier advances one generation per level).
    ///
    /// The refinement OBJECTIVE's pins (sinks stay ASAP, uncosted nodes stay
    /// put) deliberately do NOT apply here: a user may hand-delay a sink.
    /// The invariants above are the only hard contract.
    ///
    /// Within-level fire order is NOT taken from this map's ordering — it is
    /// always graph (`nodes:`) declaration order, same as the Kahn path
    /// (Principle #5/#7).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub level_assignments: Option<indexmap::IndexMap<String, usize>>,
    /// OPTIONAL cross-machine networking surface. The block RESTRICTS
    /// networking; it does not switch it on.
    ///
    /// Under `cerulion graph run` on the real clock, an absent block (and a
    /// block whose `mode` is `disabled`) means the PERMISSIVE default: the CLI
    /// starts one separate gateway process that opens the machine's zenoh
    /// session, listens on port 7683 (`CERULION_GATEWAY_PORT` overrides it),
    /// scouts the LAN and announces every topic the graph produces. An enabled
    /// block (`mode: peer` or `client`) replaces that with a STRICT posture:
    /// the `connect` / `listen` locators used verbatim, no scouting, and only
    /// the listed `egress` and `ingress` topics crossing. No value of this
    /// block means "no network": that is `graph run --network off` (or
    /// `CERULION_NETWORK=off`). The virtual and external clocks never start a
    /// gateway.
    ///
    /// The graph process itself opens no zenoh session in any of these cases;
    /// the gateway is its own process. When present the block declares this
    /// graph's role in the zenoh mesh (`mode`), the
    /// endpoints to reach (`connect`/`listen`), and which topics leave
    /// (`egress`) or enter (`ingress`) over the network. Validated at
    /// graph-load by `validate_graph` (the `validate_network_block` arm in
    /// `super::validation`); the transport wiring that CONSUMES it is
    /// `GraphConfig::network_transport_config`, and the block itself is parse +
    /// validate only.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub network: Option<NetworkBlock>,
}

impl GraphConfig {
    /// The graph's identity: the file stem.
    ///
    /// THE read path for every identity surface. Serves [`UNNAMED_GRAPH`] when
    /// [`identity`](Self::identity) is empty, which is reachable only for a
    /// config parsed from a YAML string with no file behind it that also
    /// carried no legacy `name:` — so a caller never has to decide what an
    /// empty graph name renders as, and no surface can silently truncate to
    /// nothing.
    ///
    /// Deliberately NOT the deprecated [`name`](Self::name) field: that key is
    /// optional-and-ignored, and reading it for identity is exactly the
    /// file-stem/`name:` divergence this decision removed.
    pub fn identity(&self) -> &str {
        if self.identity.is_empty() {
            UNNAMED_GRAPH
        } else {
            &self.identity
        }
    }

    /// ONE definition of "is this
    /// resolved topic opted into multiple publishers" — membership is
    /// tested at three layers (validation, topology build, runtime
    /// provisioning), and a single predicate keeps them from diverging.
    pub fn is_multi_publisher(&self, topic: &str) -> bool {
        self.multi_publisher_topics.iter().any(|t| t == topic)
    }

    /// True when this graph FILE declares cross-process partitions. False
    /// does not mean the run is single-process: `cerulion graph run` derives a
    /// partition for an unpartitioned graph (see
    /// [`process_groups`](Self::process_groups)). For a runtime built directly
    /// from a config with no groups, the barrier participant-map derivation is
    /// a no-op.
    pub fn has_process_groups(&self) -> bool {
        !self.process_groups.is_empty()
    }

    /// The NATIVE nodes — everything the runtime schedules. Excludes
    /// [`ros2:`](NodeDef::ros2) entries, which `cerulion graph run` spawns as
    /// supervised processes instead.
    pub fn native_nodes(&self) -> impl Iterator<Item = &NodeDef> {
        self.nodes.iter().filter(|n| !n.is_ros2())
    }

    /// The [`ros2:`](NodeDef::ros2) entries, in declaration order.
    pub fn ros2_nodes(&self) -> impl Iterator<Item = &NodeDef> {
        self.nodes.iter().filter(|n| n.is_ros2())
    }

    /// True when the graph declares at least one [`ros2:`](NodeDef::ros2)
    /// entry.
    pub fn has_ros2_nodes(&self) -> bool {
        self.nodes.iter().any(NodeDef::is_ros2)
    }

    /// REMOVE the [`ros2:`](NodeDef::ros2) entries from `nodes`, returning
    /// them in declaration order — the ONE seam `cerulion graph run` / `graph
    /// levels` use to split a mixed graph into the native half the runtime
    /// builds and the ROS 2 half the run spawns. After this the config is a
    /// plain native graph (`GraphRuntime::build` refuses a config that still
    /// carries ros2 entries, so the split cannot be forgotten silently).
    pub fn take_ros2_nodes(&mut self) -> Vec<NodeDef> {
        let (ros2, native): (Vec<NodeDef>, Vec<NodeDef>) = std::mem::take(&mut self.nodes)
            .into_iter()
            .partition(NodeDef::is_ros2);
        self.nodes = native;
        ros2
    }

    /// The ONE funnel from the graph's declared `network:`
    /// block to the transport layer's
    /// [`NetworkConfig`](crate::transport::network::NetworkConfig) —
    /// `TransportConfig.network` should be populated from THIS method for a
    /// graph run (the CLI's `graph_run` does; programmatic embedders should
    /// too). Returns `None` for an absent block AND for `mode: disabled`
    /// (both mean local-only — no zenoh session opens), so callers can
    /// assign the result directly.
    ///
    /// Design decision: the returned config carries `robot_identity`
    /// resolved from the machine HOSTNAME (or the `CERULION_ROBOT_IDENTITY`
    /// override) via [`crate::graph::robot_identity_from_env`] — the SAME
    /// resolver the CLI gateway path uses, so both agree by construction. The
    /// robot's network identity is DECOUPLED from the graph prefix: the prefix
    /// drives topic naming (frozen in the YAML for replay determinism), while
    /// identity is a network-only concern resolved at runtime (never the graph
    /// prefix — before that decision it doubled as one, which is why a `prefix: go2`
    /// graph on host `ubuntu` announced as `go2`). The hostname resolver never
    /// returns empty (it falls back to `localhost`), so identity is always
    /// present and an announce never fails for want of one.
    pub fn network_transport_config(&self) -> Option<crate::transport::network::NetworkConfig> {
        self.network
            .as_ref()
            .and_then(NetworkBlock::to_network_config)
            .map(|mut cfg| {
                cfg.robot_identity = Some(crate::graph::robot_identity_from_env());
                cfg
            })
    }

    /// True when the graph declares a PRESENT and ENABLED
    /// (`mode: peer|client`) `network:` block — the predicate the run path
    /// uses for the network-vs-multiprocess gate and the `--network off`
    /// kill-switch notices.
    pub fn has_enabled_network(&self) -> bool {
        self.network.as_ref().is_some_and(NetworkBlock::is_enabled)
    }
}

/// Zenoh session role declared by a graph's `network:` block.
///
/// The graph-YAML surface deliberately exposes a SUBSET of the transport
/// layer's `ZenohMode` (in `crate::transport`): `router` is intentionally
/// NOT exposed in v1 (a graph declares participants, not routers), and a
/// `disabled` sentinel is added so the block can be parsed but treated as
/// network-OFF (inert). Keeping this enum in the graph layer keeps the YAML
/// vocabulary decoupled from the transport type — the mapping onto the
/// transport `ZenohMode` is [`NetworkBlock::to_network_config`].
///
/// `#[serde(rename_all = "lowercase")]` makes the accepted strings
/// `disabled` / `peer` / `client`; an unknown string (e.g. `router`) is
/// rejected loudly by serde with an error that lists the valid values.
///
/// `#[serde(deny_unknown_fields)]` rides along for UNIFORMITY with the other
/// graph-YAML types (so the source walk needs no special case). On a
/// unit-variant enum it denies nothing: an unknown VALUE is already rejected
/// by the variant matcher, which is what the paragraph above describes.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default, Deserialize, Serialize)]
#[serde(rename_all = "lowercase", deny_unknown_fields)]
pub enum NetworkMode {
    /// The block imposes NO restriction, exactly like an absent block. The
    /// default when `mode:` is omitted. Under `cerulion graph run` on the real
    /// clock that is the PERMISSIVE default (a gateway process, LAN scouting,
    /// every produced topic announced), NOT "no network": the local-only
    /// switch is `graph run --network off`. Declaring `egress`/`ingress` under
    /// this mode is a graph-load error.
    #[default]
    Disabled,
    /// Peer mode: participate in the zenoh mesh topology.
    Peer,
    /// Client mode: connect to a router, do not route traffic.
    Client,
}

/// The optional top-level `network:` block (see
/// `GraphConfig::network`). Absent means no restriction: `cerulion graph run`
/// applies its permissive default, and only `--network off` runs local-only.
///
/// `connect` and `listen` map 1:1 onto zenoh locator strings
/// (e.g. `"tcp/192.168.1.10:7447"`), and 1:1 onto
/// `NetworkConfig::connect_endpoints` / `NetworkConfig::listen_endpoints`
/// (in `crate::transport`): `connect` are remote endpoints to dial,
/// `listen` are local endpoints to bind. Because multicast scouting is off
/// by default (the safe default in `NetworkConfig::default`), a config-only
/// cross-machine deployment needs one side to `listen` and the other to
/// `connect`. `listen` is an addition to the original design example
/// (which showed only `connect`) precisely because the config-only
/// cross-machine story is impossible without it.
///
/// `egress` topics LEAVE this machine (each must be produced by an in-graph
/// node); `ingress` topics ARRIVE from the network (each must NOT have an
/// in-graph producer — they are external sources). Every topic name must be
/// canonical absolute (leading `/`). The full contract is enforced at
/// graph-load by the `validate_network_block` arm in `super::validation`.
///
/// `#[serde(deny_unknown_fields)]`: a misspelled key is a loud parse error,
/// never a silently dropped setting — see [`GraphConfig`] for the rationale.
#[derive(Debug, Clone, Default, PartialEq, Eq, Deserialize, Serialize)]
#[serde(deny_unknown_fields)]
pub struct NetworkBlock {
    /// This graph's role in the zenoh mesh. Defaults to
    /// [`NetworkMode::Disabled`] when omitted.
    #[serde(default)]
    pub mode: NetworkMode,
    /// Remote zenoh locators to connect to (e.g. `["tcp/192.168.1.10:7447"]`).
    /// Maps 1:1 onto `NetworkConfig::connect_endpoints`.
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub connect: Vec<String>,
    /// Local zenoh locators to listen on (e.g. `["tcp/0.0.0.0:7447"]`).
    /// Maps 1:1 onto `NetworkConfig::listen_endpoints`.
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub listen: Vec<String>,
    /// Canonical absolute topics this graph EXPORTS to the network. Each
    /// must be produced by an in-graph node (validated at graph-load).
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub egress: Vec<String>,
    /// Canonical absolute topics this graph IMPORTS from the network. Each
    /// must NOT have an in-graph producer — they are external sources
    /// (validated at graph-load).
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub ingress: Vec<String>,
}

impl NetworkBlock {
    /// True when the block's `mode` actually enables the
    /// network (`peer` / `client`). A `disabled` block is parsed but INERT
    /// — validation additionally rejects declared `egress`/`ingress` lists
    /// under it (see `super::validation`).
    pub fn is_enabled(&self) -> bool {
        self.mode != NetworkMode::Disabled
    }

    /// Map this block onto the transport layer's
    /// [`NetworkConfig`](crate::transport::network::NetworkConfig).
    ///
    /// PURE field mapping — no session opens here (the transport's
    /// `NetworkManager` is lazy):
    ///
    /// | `network:` block | `NetworkConfig` |
    /// |---|---|
    /// | `mode: peer` | `ZenohMode::Peer` |
    /// | `mode: client` | `ZenohMode::Client` |
    /// | `mode: disabled` | `None` (no transport network at all) |
    /// | `connect` | `connect_endpoints` (1:1) |
    /// | `listen` | `listen_endpoints` (1:1) |
    /// | — | scouting knobs: NOT exposed in v1 — both stay the transport default OFF (nothing is discovered by accident) |
    pub fn to_network_config(&self) -> Option<crate::transport::network::NetworkConfig> {
        use crate::transport::network::{NetworkConfig, ZenohMode};
        let mode = match self.mode {
            NetworkMode::Disabled => return None,
            NetworkMode::Peer => ZenohMode::Peer,
            NetworkMode::Client => ZenohMode::Client,
        };
        Some(NetworkConfig {
            mode,
            connect_endpoints: self.connect.clone(),
            listen_endpoints: self.listen.clone(),
            // Scouting deliberately rides the transport default (OFF) —
            // v1 exposes no scouting knobs in graph YAML.
            ..NetworkConfig::default()
        })
    }
}

/// Definition of a single node in the graph.
///
/// Trigger policy lives ONLY on the node (via `#[cerulion_node(...)]`
/// attributes or `#[input(trigger)]` fields) — the macro is the single
/// source of truth, not graph YAML. The runtime reads the macro's
/// declared `MacroPolicy` from the cdylib's `cerulion_node_info()`
/// JSON. A policy cleanup removed the prior
/// optional `policy:` field on this struct: a YAML-side override of
/// node behavior was a second source of truth that diverged from what
/// the user wrote in the macro, with no compile-time check and no
/// "is this YAML actually doing anything" feedback. The cleanup
/// removes the override path entirely; nodes that need different
/// policies in different graph instantiations should be different
/// node types or use `External` + host-driven `trigger_external`.
///
/// `#[serde(deny_unknown_fields)]`: a misspelled key is a loud parse error,
/// never a silently dropped setting — see [`GraphConfig`] for the rationale.
#[derive(Debug, Clone, Deserialize, Serialize)]
#[serde(deny_unknown_fields)]
pub struct NodeDef {
    pub id: String,
    /// The native node TYPE (`nodes/<type>/`). `#[serde(default)]` because a
    /// [`ros2:`](Self::ros2) entry carries no type — `validate_graph` enforces
    /// that exactly one of `type:` / `ros2:` is declared, so an omitted
    /// `type:` on a native node is still a loud load error (it just arrives
    /// from validation instead of serde).
    #[serde(rename = "type", default, skip_serializing_if = "String::is_empty")]
    pub node_type: String,
    #[serde(default)]
    pub inputs: Vec<InputDef>,
    #[serde(default)]
    pub outputs: Vec<OutputDef>,
    /// A stock ROS 2 process declared as a graph entry (the mixed-stack
    /// shape: native nodes and ROS 2 nodes in ONE graph file). `cerulion
    /// graph run` SPAWNS it as a supervised child process on Cerulion
    /// transport (`RMW_IMPLEMENTATION=rmw_cerulion`, the same staged env
    /// `cerulion ros2 run` uses); it is NOT scheduled — no trigger policy, no
    /// DAG level, no barrier participation, no determinism claim. Its topics
    /// meet native nodes on the shared transport BY NAME, so `inputs:` /
    /// `outputs:` are rejected on such an entry (the wiring is not modelled
    /// in v1). Mutually exclusive with `type:`.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub ros2: Option<Ros2NodeDef>,
}

impl NodeDef {
    /// True for a [`ros2:`](Self::ros2) entry — an opaque ROS 2 process the
    /// run SPAWNS rather than a native node the runtime SCHEDULES.
    pub fn is_ros2(&self) -> bool {
        self.ros2.is_some()
    }
}

/// The `ros2:` block of a graph entry — how to launch one stock ROS 2
/// process. Two shapes, mutually exclusive (validated at graph load):
///
/// - `package` + `executable` (+ optional `args`, `params_file`, `params`)
///   ⇒ `ros2 run <package> <executable> [args...] --ros-args [--params-file
///   <file>] [-p k:=v ...]`;
/// - `launch` ⇒ `ros2 launch <file> [args...]` (`params` / `params_file`
///   are rejected here — a launch file carries its own parameters; pass
///   launch arguments through `args`).
///
/// `#[serde(deny_unknown_fields)]`: a misspelled key is a loud parse error,
/// never a silently dropped setting — see [`GraphConfig`] for the rationale.
#[derive(Debug, Clone, Default, Deserialize, Serialize)]
#[serde(deny_unknown_fields)]
pub struct Ros2NodeDef {
    /// The ROS 2 package (`ros2 run <package> ...`). Requires `executable`.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub package: Option<String>,
    /// The executable inside `package` (`ros2 run <package> <executable>`).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub executable: Option<String>,
    /// Extra arguments, forwarded verbatim — the executable's own args in the
    /// `package`/`executable` form, launch arguments (`name:=value`) in the
    /// `launch` form.
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub args: Vec<String>,
    /// A ROS parameters YAML file (`--ros-args --params-file <file>`).
    /// `package`/`executable` form only.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub params_file: Option<String>,
    /// Inline ROS parameters (`--ros-args -p key:=value` per entry, in
    /// declaration order). Values must be SCALARS (string / number / bool);
    /// nested maps are rejected at graph load. `package`/`executable` form
    /// only.
    #[serde(default, skip_serializing_if = "indexmap::IndexMap::is_empty")]
    pub params: indexmap::IndexMap<String, serde_yaml::Value>,
    /// A launch file (`ros2 launch <file> [args...]`) — the ALTERNATIVE to
    /// `package`/`executable`. Resolved relative to the workspace root when
    /// relative.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub launch: Option<String>,
}

impl Ros2NodeDef {
    /// The `ros2` argv this entry spawns with (everything after the `ros2`
    /// program name). Pure; assumes a VALIDATED shape (`validate_graph`
    /// rejects anything else) — a non-scalar param value renders as an empty
    /// string rather than panicking, so an unvalidated config still cannot
    /// crash the spawner. `launch_path` lets the caller hand in the resolved
    /// launch file (relative paths are resolved against the workspace root).
    pub fn argv(&self, launch_path: Option<&str>) -> Vec<String> {
        if let Some(launch) = &self.launch {
            let mut argv = vec![
                "launch".to_string(),
                launch_path.unwrap_or(launch.as_str()).to_string(),
            ];
            argv.extend(self.args.iter().cloned());
            return argv;
        }
        let mut argv = vec![
            "run".to_string(),
            self.package.clone().unwrap_or_default(),
            self.executable.clone().unwrap_or_default(),
        ];
        argv.extend(self.args.iter().cloned());
        if self.params_file.is_some() || !self.params.is_empty() {
            argv.push("--ros-args".to_string());
            if let Some(file) = &self.params_file {
                argv.push("--params-file".to_string());
                argv.push(file.clone());
            }
            for (key, value) in &self.params {
                argv.push("-p".to_string());
                argv.push(format!(
                    "{key}:={}",
                    scalar_param_value(value).unwrap_or_default()
                ));
            }
        }
        argv
    }
}

/// Render a `params:` value as the text after `key:=` — `Some` for a scalar
/// (string verbatim; number / bool via their canonical display), `None` for
/// anything else (null, sequence, map — rejected by `validate_graph`).
pub fn scalar_param_value(value: &serde_yaml::Value) -> Option<String> {
    match value {
        serde_yaml::Value::String(s) => Some(s.clone()),
        serde_yaml::Value::Number(n) => Some(n.to_string()),
        serde_yaml::Value::Bool(b) => Some(b.to_string()),
        _ => None,
    }
}

/// Input port definition.
///
/// `#[serde(deny_unknown_fields)]`: a misspelled key is a loud parse error,
/// never a silently dropped setting — see [`GraphConfig`] for the rationale.
#[derive(Debug, Clone, Deserialize, Serialize)]
#[serde(deny_unknown_fields)]
pub struct InputDef {
    pub name: String,
    pub source: String,
}

/// Default buffer size when graph YAML omits `max_slice_len:` AND
/// the schema marker carries no `<T as ShmMessage>::MAX_SLICE_LEN`
/// (tier-3 of the 3-tier resolution ladder).
///
/// 128 MiB — the safe upper bound for stock ROS2 messages (8K RGB
/// frame ≈ 95 MiB fits with headroom). The launch bump
/// raised this from 16 MiB to stay congruent with the codegen catch-all
/// (`TIER_HUGE` in `variable_schema_max_slice_len`); it is FREE because
/// iceoryx2 `Static` pools are lazy/demand-paged on Linux+macOS
/// (resident = working set, not the reservation). Before the ladder this was
/// 64 KiB, which silently truncated `sensor_msgs/Image` and similar
/// payloads at runtime; the old 16 MiB still hard-failed `PayloadTooLarge`
/// for 4K/8K frames. The 3-tier ladder:
///
/// 1. Explicit `max_slice_len:` in graph YAML wins.
/// 2. `<T as ShmMessage>::MAX_SLICE_LEN` (per-schema codegen-emitted
///    default) consulted via `OutputMeta::max_slice_len_default`.
/// 3. This constant — 128 MiB — final fallback. A `tracing::warn!`
///    fires when this tier resolves so users see they're using the
///    coarse default; setting `max_slice_len:` per-topic in YAML or
///    using a schema with a populated trait const silences it.
pub const DEFAULT_MAX_SLICE_LEN: usize = 128 * 1024 * 1024;

/// Output port definition.
///
/// `#[serde(deny_unknown_fields)]`: a misspelled key is a loud parse error,
/// never a silently dropped setting — see [`GraphConfig`] for the rationale.
#[derive(Debug, Clone, Deserialize, Serialize)]
#[serde(deny_unknown_fields)]
pub struct OutputDef {
    pub name: String,
    #[serde(default, deserialize_with = "normalize_schema_separator")]
    pub schema: String,
    /// Maximum SHM slot size (header + payload) in bytes. Optional —
    /// resolved by the runtime's 3-tier ladder when omitted (see
    /// `super::runtime::resolve_max_slice_len` — file-private fn).
    /// Typed as `Option<usize>` here because YAML deserialization is
    /// lenient; the resolver narrows the final value to `u32`
    /// since `WireHeader::total_size: u32` caps
    /// representable slot sizes at 4 GiB. Values above `u32::MAX` are
    /// rejected at graph-load by `validate_graph`.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub max_slice_len: Option<usize>,
    /// ABSOLUTE topic override. When set, this output
    /// publishes to the given topic verbatim instead of the derived
    /// `/{prefix}/{node_id}/{name}` — for externally-fixed global names
    /// (`/tf`-style). The value MUST be absolute (leading `/`);
    /// `validate_graph` rejects relative values with a did-you-mean so
    /// this never becomes a second relative-naming mechanism.
    /// Single-writer provisioning applies like any produced topic: two
    /// nodes (or two graphs) publishing the same `topic:` collide at
    /// build / port creation UNLESS the topic is listed in the graph's
    /// `multi_publisher_topics` opt-in (the multi-publisher escape hatch).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub topic: Option<String>,
    /// Publisher-side ring buffer depth for **late-joiner replay**: how
    /// many of the most recently published frames the publisher
    /// retains so that subscribers joining AFTER the publisher started
    /// can read them. Existing subscribers do not consume from this
    /// buffer — they read from the live channel.
    ///
    /// **Default `0` (Volatile semantics): late joiners see nothing
    /// from before they joined.** Set `> 0` only for **state-like**
    /// topics — robot pose, configuration, mode flags, calibration —
    /// where a late joiner needs the most recent value to function.
    /// For **stream-like** topics (sensor data at 10-1000 Hz, control
    /// outputs, joint states), leave at 0: replaying stale frames into
    /// a fresh subscriber is usually wrong and reserves SHM that goes
    /// unused.
    ///
    /// # ROS2 mapping
    ///
    /// ROS2 splits this concept across two QoS policies:
    ///
    /// | Cerulion | ROS2 equivalent |
    /// |---|---|
    /// | `history_size = 0` | `Durability::VOLATILE` (default) |
    /// | `history_size = N` (N > 0) | `Durability::TRANSIENT_LOCAL` + publisher `History::KEEP_LAST(N)` |
    ///
    /// The defaults match: ROS2 is also `VOLATILE`-by-default for the
    /// same reason (don't reserve memory for replay that most topics
    /// don't need).
    ///
    /// # Contract with subscriber `depth`
    ///
    /// Subscribers carry their own buffer depth via
    /// `InputMeta.depth` (`crates/cerulion_core/src/graph/node.rs`): the
    /// in-flight receive queue depth, governed by the
    /// `BackpressurePolicy` on overflow. The two fields interact ONLY
    /// at subscriber-join time:
    ///
    /// - **`subscriber.depth >= history_size`** — the late joiner's
    ///   queue can absorb the full replay; common case.
    /// - **`subscriber.depth < history_size`**: the late joiner is
    ///   delivered only the NEWEST `depth` frames of the history (iceoryx2
    ///   truncates the replay to the subscriber's queue depth, per
    ///   consumer). The OLDEST `history_size - depth` frames are never
    ///   delivered to that consumer. The graph build logs a `warn` naming
    ///   the topic, the node and the input for each such edge. It is a
    ///   configuration error for any subscriber meant to consume the whole
    ///   replay.
    ///
    /// After the initial drain the two fields decouple: `depth`
    /// governs steady-state in-flight buffering; `history_size` is
    /// only consulted again the next time a subscriber joins.
    ///
    /// Omitted from serialized YAML when `0` (the VOLATILE default) so
    /// generated graph files don't carry the noise `history_size: 0` on
    /// every output — `#[serde(default)]` round-trips the absent key
    /// back to `0`.
    #[serde(default, skip_serializing_if = "is_zero")]
    pub history_size: usize,
}

/// serde `skip_serializing_if` predicate: omit a `usize` field when it
/// holds its zero default. Keeps generated YAML free of `field: 0` noise
/// for fields whose `0` is the documented default (e.g. `history_size`).
fn is_zero(n: &usize) -> bool {
    *n == 0
}

/// Normalize schema separator from `::` to `/` during deserialization.
///
/// Accepts both `sensor_msgs::Image` and `sensor_msgs/Image`, storing
/// the canonical `/`-separated form.
fn normalize_schema_separator<'de, D>(deserializer: D) -> Result<String, D::Error>
where
    D: serde::Deserializer<'de>,
{
    let s = String::deserialize(deserializer)?;
    Ok(s.replace("::", "/"))
}

#[cfg(test)]
mod network_block_tests {
    //! Parse-shape oracle tests for the `network:` block.
    //! These exercise the serde surface directly (`serde_yaml::from_str`),
    //! so they carry no hostname/prefix-defaulting dependency — the field
    //! mapping and mode-string parsing are pinned in isolation.
    use super::*;

    /// A no-`network:` YAML parses to `network == None` AND leaves every
    /// other field untouched (the byte-identical-when-absent contract).
    #[test]
    fn network_block_absent_parses_to_none() {
        let yaml = r#"
name: local_only
prefix: robo
nodes:
  - id: cam
    type: camera
    outputs:
      - name: cloud
        schema: sensor_msgs/PointCloud2
"#;
        let config: GraphConfig = serde_yaml::from_str(yaml).unwrap();
        assert_eq!(config.network, None, "absent block must parse to None");
        // The rest of the struct is untouched by the field add. NOTE this is a
        // RAW `serde_yaml::from_str`, not `parse_graph_raw`, so the file-stem
        // identity SEED never runs — the deprecated key is all this
        // config carries, which is exactly what is asserted.
        assert_eq!(config.name.as_deref(), Some("local_only"));
        assert_eq!(
            config.identity, "",
            "`identity` is `serde(skip)`: a raw deserialize leaves it UNKNOWN"
        );
        assert_eq!(config.prefix, "robo");
        assert_eq!(config.nodes.len(), 1);
        assert_eq!(config.nodes[0].id, "cam");
        assert_eq!(config.nodes[0].outputs[0].name, "cloud");
    }

    /// A full block parses every field to the expected oracle values, and
    /// `mode: peer` maps to [`NetworkMode::Peer`].
    #[test]
    fn network_full_block_parses_all_fields() {
        let yaml = r#"
name: go2
prefix: go2
nodes:
  - id: utlidar
    type: lidar
    outputs:
      - name: cloud
        schema: sensor_msgs/PointCloud2
network:
  mode: peer
  connect:
    - tcp/192.168.123.99:7447
  listen:
    - tcp/0.0.0.0:7447
  egress:
    - /go2/utlidar/cloud
  ingress:
    - /go2/cmd_vel/keyboard
"#;
        let config: GraphConfig = serde_yaml::from_str(yaml).unwrap();
        let net = config.network.expect("block present");
        // Hand-built oracle — never a self-compare.
        let expected = NetworkBlock {
            mode: NetworkMode::Peer,
            connect: vec!["tcp/192.168.123.99:7447".to_string()],
            listen: vec!["tcp/0.0.0.0:7447".to_string()],
            egress: vec!["/go2/utlidar/cloud".to_string()],
            ingress: vec!["/go2/cmd_vel/keyboard".to_string()],
        };
        assert_eq!(net, expected);
    }

    /// `listen` and `connect` BOTH parse (a block that
    /// carries both, not only `connect`).
    #[test]
    fn network_listen_and_connect_both_parse() {
        let yaml = r#"
name: t
prefix: p
nodes:
  - id: n
    type: t
network:
  mode: client
  connect:
    - tcp/10.0.0.1:7447
    - tcp/10.0.0.2:7447
  listen:
    - tcp/0.0.0.0:7447
"#;
        let config: GraphConfig = serde_yaml::from_str(yaml).unwrap();
        let net = config.network.expect("block present");
        assert_eq!(net.mode, NetworkMode::Client);
        assert_eq!(
            net.connect,
            vec![
                "tcp/10.0.0.1:7447".to_string(),
                "tcp/10.0.0.2:7447".to_string()
            ]
        );
        assert_eq!(net.listen, vec!["tcp/0.0.0.0:7447".to_string()]);
        assert!(net.egress.is_empty());
        assert!(net.ingress.is_empty());
    }

    /// `mode: disabled` parses to [`NetworkMode::Disabled`].
    #[test]
    fn network_mode_disabled_parses() {
        let yaml = r#"
name: t
prefix: p
nodes:
  - id: n
    type: t
network:
  mode: disabled
"#;
        let config: GraphConfig = serde_yaml::from_str(yaml).unwrap();
        assert_eq!(config.network.unwrap().mode, NetworkMode::Disabled);
    }

    /// A present block that OMITS `mode:` defaults to
    /// [`NetworkMode::Disabled`] (the loud-safe default; validation then
    /// rejects any declared egress/ingress under it).
    #[test]
    fn network_mode_defaults_to_disabled_when_omitted() {
        let yaml = r#"
name: t
prefix: p
nodes:
  - id: n
    type: t
network:
  connect:
    - tcp/10.0.0.1:7447
"#;
        let config: GraphConfig = serde_yaml::from_str(yaml).unwrap();
        let net = config.network.expect("block present");
        assert_eq!(net.mode, NetworkMode::Disabled);
        assert_eq!(net.connect, vec!["tcp/10.0.0.1:7447".to_string()]);
    }

    /// An unknown `mode:` string (`router` is deliberately not exposed in
    /// v1) is rejected loudly by serde, and the error lists the valid
    /// values.
    #[test]
    fn network_unknown_mode_rejected_listing_valid_values() {
        let yaml = r#"
name: t
prefix: p
nodes:
  - id: n
    type: t
network:
  mode: router
"#;
        let result: Result<GraphConfig, _> = serde_yaml::from_str(yaml);
        let err = result
            .expect_err("router is not a valid v1 mode")
            .to_string();
        assert!(
            err.contains("unknown variant") && err.contains("router"),
            "serde error must name the bad variant; got: {err}"
        );
        // The error lists the valid values so the user knows the fix.
        assert!(
            err.contains("peer") && err.contains("client") && err.contains("disabled"),
            "serde error must list the valid modes; got: {err}"
        );
    }

    /// `deny_unknown_fields`: a misspelled KEY is rejected as
    /// loudly as a misspelled VARIANT, on every one of the six graph-YAML
    /// types, with the offending key NAMED.
    ///
    /// This sits directly beside `network_unknown_mode_rejected_listing_valid_values`
    /// because an ASYMMETRY between them is what makes the class a trap: with
    /// only the variant path rejected AND test-pinned, the surrounding
    /// silence reads as strictness, while an unknown KEY would be dropped in total
    /// silence with no test asserting otherwise. The two paths are
    /// pinned symmetrically.
    ///
    /// The spellings are realistic — each is a near miss of a REAL key
    /// whose default is consequential, not an invented `zzz`.
    #[test]
    fn a_misspelled_key_is_rejected_naming_it_on_every_graph_type() {
        // (type under test, the near-miss document, the key it must name)
        let cases: &[(&str, String, &str)] = &[
            (
                "GraphConfig",
                // `netwrok:` — the FAIL-OPEN one. Accepted, it leaves
                // `network: None`, which `resolve_run_network` reads as "no
                // block" and answers with the PERMISSIVE default: every
                // produced topic announced to the LAN, scouting on. The exact
                // opposite of what the author wrote.
                "name: t\nprefix: p\nnodes:\n  - id: n\n    type: t\nnetwrok:\n  mode: peer\n"
                    .to_string(),
                "netwrok",
            ),
            (
                "GraphConfig (nodes)",
                "name: t\nprefix: p\nnods:\n  - id: n\n    type: t\nnodes:\n  - id: n\n    type: t\n"
                    .to_string(),
                "nods",
            ),
            (
                "NetworkBlock",
                // `egres:` — the FAIL-CLOSED one. Accepted, it leaves `egress`
                // at its `#[serde(default)]` empty, so the graph installs a
                // deny-all allow-list and exports NOTHING, in silence.
                "name: t\nprefix: p\nnodes:\n  - id: n\n    type: t\nnetwork:\n  mode: peer\n  \
                 egres:\n    - /a/b\n"
                    .to_string(),
                "egres",
            ),
            (
                "NodeDef",
                "name: t\nprefix: p\nnodes:\n  - id: n\n    type: t\n    inputz: []\n".to_string(),
                "inputz",
            ),
            (
                // `sorce:` shadows a REQUIRED field, so serde reports the
                // MISSING one before it gets to the unknown one. Still a loud
                // refusal, and a more actionable message — it names the key
                // the author meant rather than the one they typed — so the
                // needle is `source`, not `sorce`. Required fields are not
                // the silent class (a typo there is a
                // loud missing-field serde error); this case is here to pin
                // that `deny_unknown_fields` does not REPLACE that behaviour.
                "InputDef",
                r#"
name: t
prefix: p
nodes:
  - id: n
    type: t
    inputs:
      - name: i
        sorce: a/b
"#
                .to_string(),
                "source",
            ),
            (
                "OutputDef",
                // `histroy_size:` — silently disables late-joiner replay on a
                // pose/mode/calibration topic, so a late subscriber waits
                // forever for a value that will never be replayed.
                r#"
name: t
prefix: p
nodes:
  - id: n
    type: t
    outputs:
      - name: o
        schema: sensor_msgs/Image
        histroy_size: 4
"#
                .to_string(),
                "histroy_size",
            ),
        ];

        for (ty, yaml, key) in cases {
            let err = serde_yaml::from_str::<GraphConfig>(yaml)
                .err()
                .unwrap_or_else(|| {
                    panic!("{ty}: `{key}:` must be REFUSED, not silently dropped:\n{yaml}")
                })
                .to_string();
            // Either arm is a LOUD refusal naming a key the author can act on:
            // `unknown field` for a typo of an OPTIONAL key (the silent class
            // `deny_unknown_fields` closes), `missing field` where the typo shadowed a
            // REQUIRED one (loud on its own).
            assert!(
                (err.contains("unknown field") || err.contains("missing field"))
                    && err.contains(key),
                "{ty}: the refusal must NAME `{key}`; got: {err}"
            );
        }
    }

    /// ANTI-TAUTOLOGY for the arm above: the CORRECT spellings all parse, and
    /// the values really land. Without this, "every misspelling is refused" is
    /// satisfied by a parser that refuses everything — and `history_size` is
    /// asserted on its VALUE (the positive control), since the
    /// whole point of that key is that its silent default is consequential.
    #[test]
    fn the_correct_spellings_parse_and_their_values_land() {
        let yaml = r#"
name: t
prefix: p
nodes:
  - id: n
    type: t
    inputs:
      - name: i
        source: a/b
    outputs:
      - name: o
        schema: sensor_msgs/Image
        history_size: 4
network:
  mode: peer
  egress:
    - /p/n/o
"#;
        let config: GraphConfig = serde_yaml::from_str(yaml).expect("the correct spellings parse");
        assert_eq!(config.nodes.len(), 1);
        assert_eq!(config.nodes[0].inputs[0].source, "a/b");
        // The value parses correctly: a `histroy_size:` typo would leave this 0
        // (VOLATILE), which is what makes the refusal above load-bearing.
        assert_eq!(config.nodes[0].outputs[0].history_size, 4);
        let net = config.network.expect("block present");
        assert_eq!(net.mode, NetworkMode::Peer);
        assert_eq!(net.egress, vec!["/p/n/o".to_string()]);
    }

    /// Round-trip: a serialized-then-parsed block is byte-stable, and an
    /// absent block does NOT introduce a `network:` key on serialize
    /// (`skip_serializing_if`).
    #[test]
    fn network_absent_block_omitted_on_serialize() {
        let yaml = r#"
name: t
prefix: p
nodes:
  - id: n
    type: t
"#;
        let config: GraphConfig = serde_yaml::from_str(yaml).unwrap();
        let out = serde_yaml::to_string(&config).unwrap();
        assert!(
            !out.contains("network:"),
            "absent network must not serialize a `network:` key; got:\n{out}"
        );
    }

    // ---- NetworkBlock → transport NetworkConfig mapping
    // (pure, oracle-vector). ----

    use crate::transport::network::ZenohMode;

    /// Peer mode maps every field 1:1 and keeps scouting OFF (the v1
    /// no-scouting-knobs contract).
    #[test]
    fn to_network_config_peer_maps_fields_and_keeps_scouting_off() {
        let block = NetworkBlock {
            mode: NetworkMode::Peer,
            connect: vec!["tcp/192.168.123.99:7447".to_string()],
            listen: vec!["tcp/0.0.0.0:7447".to_string()],
            egress: vec!["/go2/utlidar/cloud".to_string()],
            ingress: vec!["/go2/cmd_vel/keyboard".to_string()],
        };
        let cfg = block.to_network_config().expect("peer mode is enabled");
        assert_eq!(cfg.mode, ZenohMode::Peer);
        assert_eq!(
            cfg.connect_endpoints,
            vec!["tcp/192.168.123.99:7447".to_string()]
        );
        assert_eq!(cfg.listen_endpoints, vec!["tcp/0.0.0.0:7447".to_string()]);
        assert!(!cfg.multicast_scouting, "scouting must stay OFF (v1)");
        assert!(!cfg.gossip_scouting, "gossip scouting must stay OFF (v1)");
    }

    /// Client mode maps to `ZenohMode::Client`.
    #[test]
    fn to_network_config_client_maps_mode() {
        let block = NetworkBlock {
            mode: NetworkMode::Client,
            connect: vec!["tcp/10.0.0.1:7447".to_string()],
            ..NetworkBlock::default()
        };
        let cfg = block.to_network_config().expect("client mode is enabled");
        assert_eq!(cfg.mode, ZenohMode::Client);
        assert_eq!(cfg.connect_endpoints, vec!["tcp/10.0.0.1:7447".to_string()]);
        assert!(cfg.listen_endpoints.is_empty());
    }

    /// `mode: disabled` maps to `None` — the block is inert, no transport
    /// network config exists at all (no zenoh session will open).
    #[test]
    fn to_network_config_disabled_is_none() {
        let block = NetworkBlock {
            mode: NetworkMode::Disabled,
            connect: vec!["tcp/10.0.0.1:7447".to_string()],
            ..NetworkBlock::default()
        };
        assert!(
            block.to_network_config().is_none(),
            "disabled mode must map to None"
        );
        assert!(!block.is_enabled());
    }

    /// The `GraphConfig`-level funnel: absent block → `None`; enabled block
    /// → `Some` with the mapped fields; `has_enabled_network` agrees.
    #[test]
    fn graph_config_network_transport_config_funnel() {
        let absent: GraphConfig = serde_yaml::from_str(
            r#"
name: t
prefix: p
nodes:
  - id: n
    type: t
"#,
        )
        .unwrap();
        assert!(absent.network_transport_config().is_none());
        assert!(!absent.has_enabled_network());

        let enabled: GraphConfig = serde_yaml::from_str(
            r#"
name: t
prefix: p
nodes:
  - id: n
    type: t
network:
  mode: peer
  connect:
    - tcp/192.168.123.99:7447
"#,
        )
        .unwrap();
        assert!(enabled.has_enabled_network());
        let cfg = enabled
            .network_transport_config()
            .expect("enabled block maps to Some");
        assert_eq!(cfg.mode, ZenohMode::Peer);
        assert_eq!(
            cfg.connect_endpoints,
            vec!["tcp/192.168.123.99:7447".to_string()]
        );
        // Design decision: the funnel stamps the resolved network identity
        // (hostname or `CERULION_ROBOT_IDENTITY` override), DECOUPLED from the
        // graph prefix — NEVER the prefix "p". The env-controlled override +
        // decoupling oracle lives in the engine's `network_run_gate_test.rs`
        // (it owns the env serialization); here we pin that the funnel routes
        // through the shared resolver and never re-couples to the prefix.
        assert_eq!(
            cfg.robot_identity,
            Some(crate::graph::robot_identity_from_env())
        );
        assert_ne!(
            cfg.robot_identity.as_deref(),
            Some("p"),
            "identity must NOT be the graph prefix"
        );

        let disabled: GraphConfig = serde_yaml::from_str(
            r#"
name: t
prefix: p
nodes:
  - id: n
    type: t
network:
  mode: disabled
"#,
        )
        .unwrap();
        assert!(disabled.network_transport_config().is_none());
        assert!(!disabled.has_enabled_network());
    }
}
