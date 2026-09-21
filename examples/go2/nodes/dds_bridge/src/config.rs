// SPDX-License-Identifier: AGPL-3.0-only
//! The bridge mapping config — parse + validation (no DDS, no I/O beyond
//! the one `load_from_env` file read).
//!
//! A config file declares the DDS side of the bridge: the domain, the
//! interface restriction, and a list of `{dds_topic, ros_type,
//! cerulion_topic}` mappings. Every `ros_type` must resolve on ONE of two
//! paths:
//!
//! - **Typed projection** — one of the four registry types
//!   ([`crate::registry::RosType`]), served by the node's fixed macro ports
//!   (deliberate remaps, e.g. `SportModeState`→Odometry). Typed wins for
//!   these types ([`crate::generic::route_mapping`]).
//! - **Raw-generic** — ANY other type with a `MessageSchema` in the bridge
//!   codec set ([`crate::generic::schema_supported`]): the pump transcodes
//!   its raw CDR through the generic codec onto a dynamically-created
//!   ingress publisher ([`crate::generic::RawIngressRoute`]) — zero
//!   per-type code.
//!
//! A type on NEITHER path is a LOUD error naming both the registry set and
//! the schema-chain remediation — "unknown type" means NO SCHEMA, never "no
//! codec".
//!
//! `cerulion_topic` semantics differ per path. For TYPED mappings it is the
//! DECLARED wire name only — the GRAPH YAML remains the source of truth for
//! the actual port topics (Principle #5; keep them in sync — see
//! `graphs/bridge.yaml` and `graphs/go2.bridge.yaml`). For RAW mappings
//! it is AUTHORITATIVE: the route publishes on exactly this absolute topic
//! (no graph port exists for it; consumers wire it via an absolute
//! `source:` reference, like any external topic).

use std::net::IpAddr;
use std::path::{Path, PathBuf};

use cerulion_core::codegen::CdrCodec;
use cerulion_core::wire::MaxSliceLen;
use serde::Deserialize;
use thiserror::Error;

use crate::registry::RosType;

/// Slice capacity for a RAW mapping's ingress publisher when the mapping
/// omits `max_slice_len`: 1 MiB. Generous on purpose — iceoryx2 `Static`
/// pools reserve lazily (demand-paged virtual space, not RAM), and a frame
/// LARGER than the cap fails its publish loudly (counted + flood-latched),
/// never silently truncates.
pub const DEFAULT_RAW_MAX_SLICE_LEN: MaxSliceLen = MaxSliceLen::const_new(1 << 20);

/// The environment variable naming the mapping-config file path — the demo's
/// env-config convention (`cerulion_viz::stream::ADDR_ENV` precedent). There is NO
/// default path: a generic bridge cannot guess its mapping, so an unset var is
/// a loud error at `external_source` time.
pub const CONFIG_ENV: &str = "DDS_BRIDGE_CONFIG";

/// Per-mapping DDS subscription QoS. `BestEffort` is the DEFAULT: a
/// best-effort SUB matches both reliable and best-effort publishers, while a
/// reliable SUB will NOT match a best-effort publisher (the DDS QoS
/// matching rules) — maximum ingress compatibility with unknown robot QoS.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum QosMode {
    #[default]
    BestEffort,
    Reliable,
}

/// How a mapping routes through the bridge. `Auto` (the default)
/// applies the type rule — a REGISTRY type ([`RosType`]) rides its fixed
/// typed projection port, any other schema-resolvable type rides the generic
/// raw codec. `Raw` FORCES the generic raw codec even for a registry type:
/// `cerulion ros2 attach` writes it on a same-type sibling that lost the race
/// for the type's ONE typed port, so the loser is bridged (on its own
/// cerulion_topic) instead of dropped. The forced mapping still needs a
/// resolvable schema (all four registry types have one), and — being routed
/// raw — it does NOT contend for the fixed port, so two registry-type siblings
/// (one `Auto`/typed, one `Raw`) validate side by side.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum RouteMode {
    /// Registry type ⇒ typed projection port; else generic raw codec.
    #[default]
    Auto,
    /// Force the generic raw codec (a registry-type sibling that lost the
    /// typed-port race).
    Raw,
}

/// One DDS→Cerulion topic mapping.
#[derive(Debug, Clone, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct TopicMapping {
    /// The ROS-side topic name (ros2-client adds the DDS `rt/` prefix).
    pub dds_topic: String,
    /// The ROS message type as `pkg/Type` (e.g. `sensor_msgs/PointCloud2`).
    /// Must resolve in the codec registry.
    pub ros_type: String,
    /// The Cerulion-side topic. For TYPED mappings: documentation of the
    /// route (the graph's `topic:` override is authoritative; keep them in
    /// sync). For RAW mappings: AUTHORITATIVE — the route publishes on
    /// exactly this absolute topic (module docs).
    pub cerulion_topic: String,
    /// DDS subscription QoS (default: best_effort — see [`QosMode`]).
    #[serde(default)]
    pub qos: QosMode,
    /// RAW mappings only: the ingress publisher's slice capacity in bytes
    /// (default [`DEFAULT_RAW_MAX_SLICE_LEN`]). Must be ≥ 32 (the wire
    /// header). REJECTED on a typed mapping — typed ports take their cap
    /// from the graph's `OutputDef`, so a config value there would be a
    /// silently-ignored knob.
    #[serde(default)]
    pub max_slice_len: Option<u32>,
    /// Force the generic raw codec even for a registry type (default
    /// [`RouteMode::Auto`]). Set to `raw` by `cerulion ros2 attach` on a
    /// same-type sibling that lost the race for the type's ONE typed port, so
    /// it is bridged on its own cerulion_topic rather than dropped.
    #[serde(default)]
    pub route: RouteMode,
}

impl TopicMapping {
    /// Does this mapping ride the RAW generic codec? True when it is FORCED raw
    /// (`route: raw`) OR its `ros_type` is not a registry projection
    /// type. The single predicate [`BridgeConfig::validate`] +
    /// [`BridgeConfig::typed_mappings`]/[`BridgeConfig::raw_mappings`] share, so
    /// the load-time partition and the pump's run-time partition can never
    /// diverge.
    fn is_raw_route(&self) -> bool {
        self.route == RouteMode::Raw || RosType::parse(&self.ros_type).is_none()
    }
}

/// The parsed + validated bridge config.
#[derive(Debug, Clone, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct BridgeConfig {
    /// DDS domain id (must match the robot's `ROS_DOMAIN_ID`; Go2 default 0).
    #[serde(default)]
    pub domain_id: u16,
    /// Restrict rustdds to these local interface IPs. Resolution precedence is
    /// the `cerulion_go2_dds` contract (`ParticipantConfig::resolve_only_networks`):
    /// config value → `GO2_IFACE` env → none (LOUD warn — a multi-homed host
    /// WILL fail CycloneDDS discovery; see lib/cerulion_go2_dds).
    #[serde(default)]
    pub only_networks: Vec<IpAddr>,
    /// Workspace `.msg` schema store directories the generic codec loads at
    /// startup — each an ament-mirror
    /// `<dir>/<pkg>/msg/<Type>.msg` tree. `cerulion ros2 attach` writes this key
    /// (`msg_dirs: [../schemas]`) when the run needs the store; a store schema
    /// WINS over a built-in / `UNITREE_MSGS` constant of the same qualified name
    /// (shadow semantics). RELATIVE entries resolve against the CONFIG
    /// FILE's own directory ([`Self::from_yaml`] joins them to the config path
    /// at load), so a `graph run` works from ANY working
    /// directory; absolute entries are used verbatim.
    ///
    /// Three-state on purpose (`Option<Vec>`, not a bare `Vec`): ABSENT ⇒ no
    /// store (built-ins + `UNITREE_MSGS` only); present but
    /// EMPTY ⇒ a loud [`ConfigError::EmptyMsgDirs`] (a written key must mean
    /// something); present + non-empty ⇒ each dir must be a readable directory.
    #[serde(default)]
    pub msg_dirs: Option<Vec<PathBuf>>,
    /// The DDS→Cerulion mappings (at least one required).
    pub mappings: Vec<TopicMapping>,
}

/// A config load/validation failure. Every variant is self-diagnosing: it
/// names the file/field and, for type errors, the full supported set.
#[derive(Debug, Error)]
pub enum ConfigError {
    #[error(
        "{env} is not set — the generic dds_bridge node needs a mapping config file \
         (e.g. {env}=graphs/go2.bridge.yaml). See nodes/dds_bridge docs",
        env = CONFIG_ENV
    )]
    EnvUnset,
    #[error("failed to read bridge config {path:?}: {source}")]
    Read {
        path: String,
        source: std::io::Error,
    },
    #[error("failed to parse bridge config {path:?} as YAML: {source}")]
    Parse {
        path: String,
        source: serde_yaml::Error,
    },
    #[error(
        "bridge config has NO mappings — an ingress bridge with nothing to bridge is a \
         config mistake, not a valid state; add at least one {{dds_topic, ros_type, \
         cerulion_topic}} entry"
    )]
    EmptyMappings,
    #[error(
        "bridge config mapping {index} ({dds_topic:?}): unknown ros_type {ros_type:?} — \
         neither a typed projection (registry: {supported}) nor schema-resolvable (no \
         MessageSchema in the bridge codec set — \"unknown type\" means NO SCHEMA, never \
         \"no codec\"). Add the type's .msg to the schema registry \
         (native_ros2_messages BUILTIN_MSGS, the bridge's UNITREE_MSGS, or the \
         auto-schema chain)"
    )]
    UnknownRosType {
        index: usize,
        dds_topic: String,
        ros_type: String,
        supported: String,
    },
    #[error(
        "bridge config mapping {index}: {field} must be a non-empty topic name starting \
         with '/' (got {value:?})"
    )]
    BadTopic {
        index: usize,
        field: &'static str,
        value: String,
    },
    #[error(
        "bridge config mappings {first} and {second} both map ros_type {ros_type:?} — \
         the typed path binds each ros_type to ONE fixed output port ({port:?}), so at most one \
         mapping per type (multi-instance configs are not supported)"
    )]
    DuplicateRosType {
        first: usize,
        second: usize,
        ros_type: String,
        port: &'static str,
    },
    #[error(
        "bridge config mappings {first} and {second} (both RAW) publish the same \
         cerulion_topic {topic:?} — two ingress publishers on one topic would interleave \
         their sequence counters into a corrupt stream; give each raw mapping its own \
         cerulion_topic"
    )]
    DuplicateRawCerulionTopic {
        first: usize,
        second: usize,
        topic: String,
    },
    #[error(
        "bridge config mapping {index}: max_slice_len {value} is invalid — it must be at \
         least 32 (the wire-header size); omit it for the {default}-byte default",
        default = DEFAULT_RAW_MAX_SLICE_LEN
    )]
    BadMaxSliceLen { index: usize, value: u32 },
    #[error(
        "bridge config mapping {index} ({ros_type:?}): max_slice_len applies only to RAW \
         mappings — a typed projection's slice cap comes from the graph's OutputDef, so a \
         config value here would be a silently-ignored knob; remove it"
    )]
    MaxSliceLenOnTypedMapping { index: usize, ros_type: String },
    #[error(
        "bridge config has an EMPTY msg_dirs list — a written msg_dirs key must name at \
         least one workspace .msg store directory; OMIT the key entirely for no store \
         (built-ins + UNITREE_MSGS only), or list the store dir (e.g. `schemas`)"
    )]
    EmptyMsgDirs,
    #[error(
        "bridge config msg_dirs[{index}] {path:?} is not a readable directory: {source} — \
         the .msg schema store the generic codec loads must exist. RELATIVE msg_dirs entries \
         resolve against the config file's own directory (`cerulion ros2 attach` writes \
         `../schemas`, the workspace store beside graphs/); re-run `cerulion ros2 attach` to \
         regenerate the store + config, or fix/remove the path"
    )]
    BadMsgDir {
        index: usize,
        path: String,
        source: std::io::Error,
    },
}

impl BridgeConfig {
    /// Parse + validate a config from YAML text. `path` is the config file's
    /// own path: it provides error context AND is the resolution base for
    /// RELATIVE `msg_dirs` entries (each is joined to the config
    /// path's parent directory BEFORE validation, so the store resolves from
    /// any working directory; the bridge knows its config path via
    /// [`CONFIG_ENV`]). A path with no parent component (e.g. a bare filename,
    /// or the tests' `"test"` placeholder) leaves relative entries untouched
    /// (CWD-relative resolution — the degenerate fallback).
    pub fn from_yaml(text: &str, path: &str) -> Result<Self, ConfigError> {
        let mut cfg: BridgeConfig = serde_yaml::from_str(text).map_err(|e| ConfigError::Parse {
            path: path.to_string(),
            source: e,
        })?;
        // Resolve RELATIVE msg_dirs entries
        // against the CONFIG FILE's directory. `cerulion graph run` (and
        // attach's consented auto-run) inherit the user's CWD, and the CLI
        // supports invocation from any workspace subdirectory — a CWD-relative
        // store path would doom those runs AFTER consent with a BadMsgDir whose
        // remediation cannot fix it.
        if let Some(dirs) = &mut cfg.msg_dirs {
            let base = Path::new(path).parent().unwrap_or(Path::new(""));
            if !base.as_os_str().is_empty() {
                for d in dirs.iter_mut() {
                    if d.is_relative() {
                        *d = base.join(&*d);
                    }
                }
            }
        }
        cfg.validate()?;
        Ok(cfg)
    }

    /// Read + parse + validate the file at `path`.
    pub fn from_file(path: &Path) -> Result<Self, ConfigError> {
        let text = std::fs::read_to_string(path).map_err(|e| ConfigError::Read {
            path: path.display().to_string(),
            source: e,
        })?;
        Self::from_yaml(&text, &path.display().to_string())
    }

    /// Resolve the config path from [`CONFIG_ENV`] and load it. The production
    /// entry point (`DdsBridge::external_source`); unset env is a loud error.
    pub fn load_from_env() -> Result<Self, ConfigError> {
        let path = std::env::var(CONFIG_ENV).map_err(|_| ConfigError::EnvUnset)?;
        Self::from_file(Path::new(&path))
    }

    /// The validation pass: at least one mapping; every topic well-formed;
    /// every ros_type either a typed projection or schema-resolvable
    /// (see the module docs); at most one TYPED mapping per ros_type (each
    /// type owns one fixed output port); RAW mappings may repeat a
    /// ros_type (two DDS topics of one type are legitimate) but never a
    /// cerulion_topic; `max_slice_len` only on RAW mappings, ≥ 32.
    ///
    /// The generic codec is built LAZILY — a config using only the four
    /// typed projections never pays the registry parse.
    fn validate(&self) -> Result<(), ConfigError> {
        if self.mappings.is_empty() {
            return Err(ConfigError::EmptyMappings);
        }
        // Validate the optional `.msg` store dirs FIRST —
        // independent of whether any raw mapping builds the codec, because a
        // written `msg_dirs` key must mean something even in an all-typed
        // config. A present-but-empty list is a config mistake; a listed dir
        // that does not exist / is unreadable is loud (re-run attach to fix).
        if let Some(dirs) = &self.msg_dirs {
            if dirs.is_empty() {
                return Err(ConfigError::EmptyMsgDirs);
            }
            for (index, d) in dirs.iter().enumerate() {
                // `read_dir` catches nonexistent + permission-denied + not-a-dir
                // (ENOTDIR) in ONE probe — exactly "does not exist or is
                // unreadable"; a valid (even empty) dir succeeds.
                if let Err(source) = std::fs::read_dir(d) {
                    return Err(ConfigError::BadMsgDir {
                        index,
                        path: d.display().to_string(),
                        source,
                    });
                }
            }
        }
        let mut codec: Option<cerulion_core::codegen::CdrCodec> = None;
        let mut seen_typed: Vec<(RosType, usize)> = Vec::new();
        let mut seen_raw_topics: Vec<(&str, usize)> = Vec::new();
        for (index, m) in self.mappings.iter().enumerate() {
            for (field, value) in [
                ("dds_topic", &m.dds_topic),
                ("cerulion_topic", &m.cerulion_topic),
            ] {
                if !value.starts_with('/') || value.len() < 2 {
                    return Err(ConfigError::BadTopic {
                        index,
                        field,
                        value: value.clone(),
                    });
                }
            }
            // A mapping FORCED raw (`route: raw`) validates on the RAW
            // arm even for a registry type — so a same-type sibling that lost the
            // typed-port race does NOT trip `DuplicateRosType` against the winner
            // and rides the generic codec instead. `is_raw_route()` is the single
            // predicate the pump's partition shares.
            if m.is_raw_route() {
                // Raw-generic arm: FORCED raw (registry type) OR a non-registry
                // schema-resolvable type. Accepted IFF a schema resolves it.
                // Seed the codec with the workspace `.msg`
                // store so a store-only type validates as a RAW mapping. THE
                // SAME seam the pump builds at run time ([`Self::runtime_codec`])
                // — so a store-only mapping that validates here is guaranteed
                // transcodable at drain time (they can't diverge on which
                // dirs seed the codec).
                let codec = codec.get_or_insert_with(|| self.runtime_codec());
                if !crate::generic::schema_supported(codec, &m.ros_type) {
                    return Err(ConfigError::UnknownRosType {
                        index,
                        dds_topic: m.dds_topic.clone(),
                        ros_type: m.ros_type.clone(),
                        supported: RosType::supported_list(),
                    });
                }
                if let Some(n) = m.max_slice_len {
                    if MaxSliceLen::try_new(n).is_none() {
                        return Err(ConfigError::BadMaxSliceLen { index, value: n });
                    }
                }
                if let Some((_, first)) = seen_raw_topics
                    .iter()
                    .find(|(t, _)| *t == m.cerulion_topic.as_str())
                {
                    return Err(ConfigError::DuplicateRawCerulionTopic {
                        first: *first,
                        second: index,
                        topic: m.cerulion_topic.clone(),
                    });
                }
                seen_raw_topics.push((m.cerulion_topic.as_str(), index));
            } else {
                // Typed projection arm: a REGISTRY type on the default `Auto`
                // route (typed WINS — crate::generic::route_mapping). Each
                // registry type owns ONE fixed output port; a second Auto mapping
                // of the same type is DuplicateRosType (a same-type sibling must
                // ride `route: raw` to be bridged).
                let ros_type = RosType::parse(&m.ros_type)
                    .expect("is_raw_route() == false implies a registry type");
                if let Some((_, first)) = seen_typed.iter().find(|(t, _)| *t == ros_type) {
                    return Err(ConfigError::DuplicateRosType {
                        first: *first,
                        second: index,
                        ros_type: m.ros_type.clone(),
                        port: ros_type.port_name(),
                    });
                }
                if m.max_slice_len.is_some() {
                    return Err(ConfigError::MaxSliceLenOnTypedMapping {
                        index,
                        ros_type: m.ros_type.clone(),
                    });
                }
                seen_typed.push((ros_type, index));
            }
        }
        Ok(())
    }

    /// The validated TYPED `(RosType, mapping)` pairs, in declaration order
    /// (the pump's fixed-port subscription set). RAW mappings are excluded —
    /// they ride [`Self::raw_mappings`]. (Named "typed", not "resolved":
    /// non-registry types resolve too, so "resolved" does not
    /// imply "typed".) A registry-type mapping FORCED raw
    /// (`route: raw`) is excluded here — it rides the raw set, not its fixed
    /// port — via the shared [`TopicMapping::is_raw_route`] predicate.
    pub fn typed_mappings(&self) -> Vec<(RosType, &TopicMapping)> {
        self.mappings
            .iter()
            .filter(|m| !m.is_raw_route())
            .filter_map(|m| RosType::parse(&m.ros_type).map(|t| (t, m)))
            .collect()
    }

    /// The validated RAW mappings — non-registry schema-resolvable types AND
    /// registry-type mappings FORCED raw (`route: raw`), in
    /// declaration order — the pump's raw-generic route set. Uses the
    /// SAME [`TopicMapping::is_raw_route`] predicate `validate` +
    /// [`Self::typed_mappings`] use, so the load-time and run-time partitions
    /// cannot diverge.
    pub fn raw_mappings(&self) -> Vec<&TopicMapping> {
        self.mappings.iter().filter(|m| m.is_raw_route()).collect()
    }

    /// The workspace `.msg` store dirs the generic codec loads:
    /// the validated `msg_dirs`, or an empty slice when the key is absent
    /// (no store; built-ins + `UNITREE_MSGS` only). The single accessor both
    /// `validate` and the pump use, so the codec is seeded identically at
    /// load-check and at run time.
    pub fn effective_msg_dirs(&self) -> &[PathBuf] {
        self.msg_dirs.as_deref().unwrap_or(&[])
    }

    /// THE single run-time codec seam: the generic CDR codec
    /// seeded with THIS config's [`Self::effective_msg_dirs`]. Both `validate`
    /// (the load-time raw-mapping acceptance check) and the pump's drain thread
    /// (`crate::pump`, at run time) build the codec HERE, so a store-only raw
    /// mapping that validates at load is guaranteed transcodable at run time —
    /// the two can never diverge on which store dirs seed the codec. Pinned by
    /// the `runtime_codec_*` tests. Constructing it is cheap only relative to a
    /// drain — it parses the whole built-in + store schema set, so callers keep
    /// ONE codec for the process lifetime rather than rebuilding per frame.
    pub fn runtime_codec(&self) -> cerulion_core::codegen::CdrCodec {
        crate::generic::bridge_codec_with_store(self.effective_msg_dirs())
    }

    /// A RAW mapping's effective slice capacity (validated at load; None →
    /// [`DEFAULT_RAW_MAX_SLICE_LEN`]).
    ///
    /// This is the CONFIGURED value only. A mapping that omits
    /// `max_slice_len:` — which is every mapping `cerulion ros2 attach` writes —
    /// should be narrowed to its type's own budget by
    /// [`Self::raw_slice_len_for_route`]; call that at route-open time, where a
    /// codec is in hand.
    pub fn raw_slice_len(m: &TopicMapping) -> MaxSliceLen {
        m.max_slice_len
            .and_then(MaxSliceLen::try_new)
            .unwrap_or(DEFAULT_RAW_MAX_SLICE_LEN)
    }

    /// The slice a RAW route is actually created with — the
    /// configured value, narrowed to what this route's SCHEMA can produce.
    ///
    /// # Why a route's slice is a frame-loss question
    ///
    /// The slice sets the route's receive-queue DEPTH
    /// (`cerulion_core::transport::ingress_route_buffer_depth`:
    /// `clamp(64 MiB / slice, 16, 1024)`), the depth is pinned at service CREATE
    /// and immutable after, and the depth is the loss boundary — a frame the
    /// recorder has not drained when the queue is full is reclaimed in SHM at
    /// the next commit. Every `ros2 attach` route omits
    /// `max_slice_len:`, so without per-route sizing they would ALL sit at
    /// [`DEFAULT_RAW_MAX_SLICE_LEN`] = 1 MiB ⇒ depth 64 ⇒ 38.6 ms of absorption
    /// at 1.66 kHz, regardless of whether the route carries 157-byte transforms
    /// or 46 KiB point clouds. Measured on a Go2: drive passes ran past 100 ms,
    /// and at depth 64 the two ~1.66 kHz routes lost ~1.4 % of their frames.
    ///
    /// # Precedence
    ///
    /// EXPLICIT WINS. A mapping that spells `max_slice_len:` is taken verbatim —
    /// an operator who sized a route knows something this rule does not, and
    /// silently narrowing their number would be the same class of surprise as
    /// ignoring it. Only the DEFAULT is narrowed, and only downward
    /// (`cerulion_core::codegen::route_slice_budget_capped` is a `min`), so this
    /// can never make a route's queue shallower than the default gives it.
    ///
    /// An unresolvable type keeps the configured value: a route whose schema the
    /// codec does not know cannot be given a derived size, and
    /// [`crate::generic::RawIngressRoute::open`] refuses such a route anyway.
    pub fn raw_slice_len_for_route(m: &TopicMapping, codec: &CdrCodec) -> MaxSliceLen {
        let configured = Self::raw_slice_len(m);
        if m.max_slice_len.is_some() {
            return configured;
        }
        codec
            .route_slice_budget(&m.ros_type, configured)
            .unwrap_or(configured)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const HAPPY: &str = r#"
domain_id: 0
only_networks: ["192.168.123.99"]
mappings:
  - dds_topic: /utlidar/cloud
    ros_type: sensor_msgs/PointCloud2
    cerulion_topic: /go2/utlidar/cloud
    qos: best_effort
  - dds_topic: /sportmodestate
    ros_type: unitree_go/SportModeState
    cerulion_topic: /go2/odom
    qos: reliable
"#;

    #[test]
    fn happy_config_parses_and_resolves() {
        let cfg = BridgeConfig::from_yaml(HAPPY, "test").expect("happy config");
        assert_eq!(cfg.domain_id, 0);
        assert_eq!(
            cfg.only_networks,
            vec!["192.168.123.99".parse::<IpAddr>().unwrap()]
        );
        assert_eq!(cfg.mappings.len(), 2);
        assert_eq!(cfg.mappings[0].qos, QosMode::BestEffort);
        assert_eq!(cfg.mappings[1].qos, QosMode::Reliable);
        let typed = cfg.typed_mappings();
        assert_eq!(typed[0].0, RosType::PointCloud2);
        assert_eq!(typed[1].0, RosType::SportModeState);
        assert!(cfg.raw_mappings().is_empty(), "all-typed config has no raw");
    }

    #[test]
    fn qos_defaults_to_best_effort_and_domain_to_zero() {
        let cfg = BridgeConfig::from_yaml(
            r#"
mappings:
  - dds_topic: /cmd_vel
    ros_type: geometry_msgs/Twist
    cerulion_topic: /go2/twist_in
"#,
            "test",
        )
        .expect("minimal config");
        assert_eq!(cfg.domain_id, 0);
        assert!(cfg.only_networks.is_empty());
        assert_eq!(cfg.mappings[0].qos, QosMode::BestEffort);
    }

    #[test]
    fn unknown_ros_type_is_loud_and_names_both_remediations() {
        // Only a type on NEITHER path (not a projection, no schema)
        // errs — and the error names the registry set AND the schema-chain
        // remediation ("no schema", never "no codec").
        let err = BridgeConfig::from_yaml(
            r#"
mappings:
  - dds_topic: /bogus
    ros_type: totally/Bogus
    cerulion_topic: /go2/bogus
"#,
            "test",
        )
        .unwrap_err();
        assert!(matches!(err, ConfigError::UnknownRosType { .. }));
        let msg = err.to_string();
        assert!(msg.contains("totally/Bogus"), "names the offender: {msg}");
        // Names the FULL typed-projection registry set.
        for t in [
            "sensor_msgs/PointCloud2",
            "unitree_go/SportModeState",
            "geometry_msgs/Twist",
            "unitree_api/Request",
        ] {
            assert!(msg.contains(t), "registry set must name {t}: {msg}");
        }
        // Names the schema-chain remediation.
        assert!(msg.contains("no MessageSchema"), "{msg}");
        assert!(msg.contains("never \"no codec\""), "{msg}");
        assert!(
            msg.contains("auto-schema chain"),
            "extension pointer: {msg}"
        );
    }

    #[test]
    fn schema_resolvable_type_is_accepted_as_a_raw_mapping() {
        // The non-registry relaxation: sensor_msgs/Imu is outside the 4-type registry
        // but has a builtin schema → accepted, partitioned to the raw set.
        let cfg = BridgeConfig::from_yaml(
            r#"
mappings:
  - dds_topic: /imu
    ros_type: sensor_msgs/Imu
    cerulion_topic: /go2/imu
"#,
            "test",
        )
        .expect("a schema-resolvable type is a valid RAW mapping");
        assert!(cfg.typed_mappings().is_empty());
        let raw = cfg.raw_mappings();
        assert_eq!(raw.len(), 1);
        assert_eq!(raw[0].ros_type, "sensor_msgs/Imu");
        assert_eq!(
            BridgeConfig::raw_slice_len(raw[0]),
            DEFAULT_RAW_MAX_SLICE_LEN,
            "omitted max_slice_len takes the default"
        );
    }

    #[test]
    fn mixed_config_partitions_typed_and_raw_in_declaration_order() {
        let cfg = BridgeConfig::from_yaml(
            r#"
mappings:
  - dds_topic: /utlidar/cloud
    ros_type: sensor_msgs/PointCloud2
    cerulion_topic: /go2/cloud
  - dds_topic: /imu
    ros_type: sensor_msgs/Imu
    cerulion_topic: /go2/imu
    max_slice_len: 65536
  - dds_topic: /joints
    ros_type: sensor_msgs/JointState
    cerulion_topic: /go2/joints
"#,
            "test",
        )
        .expect("mixed config validates");
        let typed = cfg.typed_mappings();
        assert_eq!(typed.len(), 1);
        assert_eq!(typed[0].0, RosType::PointCloud2);
        let raw = cfg.raw_mappings();
        assert_eq!(raw.len(), 2);
        assert_eq!(raw[0].ros_type, "sensor_msgs/Imu");
        assert_eq!(raw[1].ros_type, "sensor_msgs/JointState");
        assert_eq!(BridgeConfig::raw_slice_len(raw[0]).get(), 65536);
    }

    #[test]
    fn duplicate_raw_cerulion_topic_is_rejected_but_duplicate_raw_type_is_not() {
        // Two raw mappings of the SAME type on DIFFERENT topics: legitimate
        // (two DDS topics of one type).
        let ok = BridgeConfig::from_yaml(
            r#"
mappings:
  - dds_topic: /imu_front
    ros_type: sensor_msgs/Imu
    cerulion_topic: /go2/imu_front
  - dds_topic: /imu_rear
    ros_type: sensor_msgs/Imu
    cerulion_topic: /go2/imu_rear
"#,
            "test",
        );
        assert!(ok.is_ok(), "raw mappings may repeat a ros_type: {ok:?}");

        // The SAME cerulion_topic twice: two ingress publishers would
        // interleave sequences into a corrupt stream — loud error.
        let err = BridgeConfig::from_yaml(
            r#"
mappings:
  - dds_topic: /imu_front
    ros_type: sensor_msgs/Imu
    cerulion_topic: /go2/imu
  - dds_topic: /imu_rear
    ros_type: sensor_msgs/Imu
    cerulion_topic: /go2/imu
"#,
            "test",
        )
        .unwrap_err();
        assert!(matches!(err, ConfigError::DuplicateRawCerulionTopic { .. }));
        let msg = err.to_string();
        assert!(msg.contains("mappings 0 and 1"), "{msg}");
        assert!(msg.contains("/go2/imu"), "{msg}");
    }

    #[test]
    fn max_slice_len_arms_are_validated() {
        // Below the 32-byte wire-header floor → loud error.
        let err = BridgeConfig::from_yaml(
            r#"
mappings:
  - dds_topic: /imu
    ros_type: sensor_msgs/Imu
    cerulion_topic: /go2/imu
    max_slice_len: 8
"#,
            "test",
        )
        .unwrap_err();
        assert!(matches!(err, ConfigError::BadMaxSliceLen { value: 8, .. }));
        assert!(err.to_string().contains("at least 32"), "{err}");

        // On a TYPED mapping → rejected (would be a silently-ignored knob;
        // typed ports take their cap from the graph's OutputDef).
        let err = BridgeConfig::from_yaml(
            r#"
mappings:
  - dds_topic: /utlidar/cloud
    ros_type: sensor_msgs/PointCloud2
    cerulion_topic: /go2/cloud
    max_slice_len: 65536
"#,
            "test",
        )
        .unwrap_err();
        assert!(matches!(err, ConfigError::MaxSliceLenOnTypedMapping { .. }));
        assert!(err.to_string().contains("OutputDef"), "{err}");
    }

    #[test]
    fn empty_mappings_is_loud() {
        let err = BridgeConfig::from_yaml("mappings: []\n", "test").unwrap_err();
        assert!(matches!(err, ConfigError::EmptyMappings));
        assert!(err.to_string().contains("NO mappings"));
    }

    #[test]
    fn missing_field_is_a_loud_parse_error() {
        // `cerulion_topic` omitted → serde missing-field error surfaced with
        // the file path context.
        let err = BridgeConfig::from_yaml(
            r#"
mappings:
  - dds_topic: /utlidar/cloud
    ros_type: sensor_msgs/PointCloud2
"#,
            "go2.bridge.yaml",
        )
        .unwrap_err();
        let msg = err.to_string();
        assert!(msg.contains("go2.bridge.yaml"), "names the file: {msg}");
        assert!(msg.contains("cerulion_topic"), "names the field: {msg}");
    }

    #[test]
    fn unknown_key_is_rejected_not_ignored() {
        // deny_unknown_fields: a typo'd key is a loud parse error, never a
        // silently-ignored knob.
        let err = BridgeConfig::from_yaml(
            r#"
domain: 7
mappings:
  - dds_topic: /x
    ros_type: geometry_msgs/Twist
    cerulion_topic: /y/x
"#,
            "test",
        )
        .unwrap_err();
        assert!(err.to_string().contains("domain"), "{err}");
    }

    #[test]
    fn bad_topic_shapes_are_loud() {
        for (yaml, field) in [
            (
                "mappings:\n  - dds_topic: utlidar\n    ros_type: geometry_msgs/Twist\n    cerulion_topic: /ok\n",
                "dds_topic",
            ),
            (
                "mappings:\n  - dds_topic: /ok\n    ros_type: geometry_msgs/Twist\n    cerulion_topic: \"/\"\n",
                "cerulion_topic",
            ),
        ] {
            let err = BridgeConfig::from_yaml(yaml, "test").unwrap_err();
            assert!(
                err.to_string().contains(field),
                "expected {field} in: {err}"
            );
        }
    }

    #[test]
    fn duplicate_ros_type_is_rejected_naming_both_indices_and_the_port() {
        let err = BridgeConfig::from_yaml(
            r#"
mappings:
  - dds_topic: /a
    ros_type: geometry_msgs/Twist
    cerulion_topic: /go2/a
  - dds_topic: /b
    ros_type: geometry_msgs/Twist
    cerulion_topic: /go2/b
"#,
            "test",
        )
        .unwrap_err();
        let msg = err.to_string();
        assert!(msg.contains("mappings 0 and 1"), "{msg}");
        assert!(msg.contains("twist"), "names the port: {msg}");
    }

    // ───────────────────────── Route: raw ─────────────────────────

    /// A same-type sibling FORCED raw (`route: raw`) coexists with the
    /// typed winner of the SAME registry type — it does NOT trip
    /// `DuplicateRosType` and rides the RAW partition (its own cerulion_topic).
    /// This is what `cerulion ros2 attach` writes for a typed-port loser so it is
    /// bridged via the generic codec instead of dropped.
    #[test]
    fn route_raw_registry_sibling_coexists_with_typed_winner() {
        let cfg = BridgeConfig::from_yaml(
            r#"
mappings:
  - dds_topic: /utlidar/cloud
    ros_type: sensor_msgs/PointCloud2
    cerulion_topic: /utlidar/cloud
  - dds_topic: /uslam/cloud_map
    ros_type: sensor_msgs/PointCloud2
    cerulion_topic: /uslam/cloud_map
    route: raw
"#,
            "test",
        )
        .expect("a route: raw sibling must validate beside the typed winner");
        let typed = cfg.typed_mappings();
        assert_eq!(typed.len(), 1, "only the Auto mapping is typed");
        assert_eq!(typed[0].0, RosType::PointCloud2);
        assert_eq!(typed[0].1.dds_topic, "/utlidar/cloud");
        let raw = cfg.raw_mappings();
        assert_eq!(
            raw.len(),
            1,
            "the route: raw sibling rides the raw partition"
        );
        assert_eq!(raw[0].dds_topic, "/uslam/cloud_map");
        assert_eq!(raw[0].ros_type, "sensor_msgs/PointCloud2");
        assert_eq!(raw[0].route, RouteMode::Raw);
    }

    /// `route: raw` is NOT a bypass of the schema requirement — a
    /// forced-raw mapping whose type has no schema is still `UnknownRosType`
    /// (the raw path needs a codec-known type to decode).
    #[test]
    fn route_raw_still_requires_a_resolvable_schema() {
        let err = BridgeConfig::from_yaml(
            r#"
mappings:
  - dds_topic: /widget
    ros_type: acme_msgs/Widget
    cerulion_topic: /widget
    route: raw
"#,
            "test",
        )
        .unwrap_err();
        assert!(matches!(err, ConfigError::UnknownRosType { .. }), "{err}");
    }

    /// Two `route: raw` registry-type siblings on DISTINCT
    /// cerulion_topics both ride the raw partition (raw siblings may share a
    /// type, like non-registry raw mappings); a SHARED cerulion_topic is still
    /// rejected (the raw single-writer rule).
    #[test]
    fn two_route_raw_registry_siblings_share_a_type_but_not_a_topic() {
        let cfg = BridgeConfig::from_yaml(
            r#"
mappings:
  - dds_topic: /a
    ros_type: sensor_msgs/PointCloud2
    cerulion_topic: /a
    route: raw
  - dds_topic: /b
    ros_type: sensor_msgs/PointCloud2
    cerulion_topic: /b
    route: raw
"#,
            "test",
        )
        .expect("two route: raw siblings on distinct topics validate");
        assert_eq!(cfg.raw_mappings().len(), 2);
        assert!(cfg.typed_mappings().is_empty());
        let err = BridgeConfig::from_yaml(
            r#"
mappings:
  - dds_topic: /a
    ros_type: sensor_msgs/PointCloud2
    cerulion_topic: /shared
    route: raw
  - dds_topic: /b
    ros_type: sensor_msgs/PointCloud2
    cerulion_topic: /shared
    route: raw
"#,
            "test",
        )
        .unwrap_err();
        assert!(
            matches!(err, ConfigError::DuplicateRawCerulionTopic { .. }),
            "{err}"
        );
    }

    /// The default route is `Auto` — a config that omits `route`
    /// gets the type rule (a registry type ⇒ typed port), so a config
    /// written without the key is unaffected by it.
    #[test]
    fn route_defaults_to_auto_registry_type_stays_typed() {
        let cfg = BridgeConfig::from_yaml(HAPPY, "test").expect("happy config");
        assert_eq!(cfg.mappings[0].route, RouteMode::Auto);
        assert_eq!(cfg.typed_mappings().len(), 2, "both registry types typed");
        assert!(cfg.raw_mappings().is_empty());
    }

    /// `max_slice_len` is legal on a `route: raw`
    /// REGISTRY-type mapping — the exact knob the raw-cap documentation tells
    /// an operator to set when a raw-routed PointCloud2's frames exceed the
    /// 1 MiB default. A check keyed on the registry type alone would reject it as
    /// `MaxSliceLenOnTypedMapping`; the forced-raw arm must validate it as a
    /// RAW mapping (accepted, visible via `raw_slice_len`) and reject a bad
    /// value as `BadMaxSliceLen` — NEVER `MaxSliceLenOnTypedMapping` (a
    /// refactor re-keying the check on `RosType::parse(..).is_some()` instead
    /// of arm placement would reject the operator's only remediation).
    #[test]
    fn route_raw_registry_mapping_accepts_max_slice_len() {
        let cfg = BridgeConfig::from_yaml(
            r#"
mappings:
  - dds_topic: /utlidar/cloud
    ros_type: sensor_msgs/PointCloud2
    cerulion_topic: /utlidar/cloud
  - dds_topic: /uslam/cloud_map
    ros_type: sensor_msgs/PointCloud2
    cerulion_topic: /uslam/cloud_map
    route: raw
    max_slice_len: 4194304
"#,
            "test",
        )
        .expect("route: raw + max_slice_len on a registry type must validate");
        let raw = cfg.raw_mappings();
        assert_eq!(raw.len(), 1);
        assert_eq!(raw[0].max_slice_len, Some(4_194_304));
        assert_eq!(
            BridgeConfig::raw_slice_len(raw[0]).get(),
            4_194_304,
            "the per-mapping override reaches the ingress publisher's cap"
        );
        // Bad-value twin: rejected as a RAW-arm value error, not as a typed
        // mapping carrying the knob.
        let err = BridgeConfig::from_yaml(
            r#"
mappings:
  - dds_topic: /uslam/cloud_map
    ros_type: sensor_msgs/PointCloud2
    cerulion_topic: /uslam/cloud_map
    route: raw
    max_slice_len: 8
"#,
            "test",
        )
        .unwrap_err();
        assert!(
            matches!(err, ConfigError::BadMaxSliceLen { value: 8, .. }),
            "must be BadMaxSliceLen (raw arm), never MaxSliceLenOnTypedMapping: {err}"
        );
    }

    #[test]
    fn env_unset_error_names_the_var_and_an_example() {
        let msg = ConfigError::EnvUnset.to_string();
        assert!(msg.contains(CONFIG_ENV));
        assert!(msg.contains("go2.bridge.yaml"));
    }

    // ─────────────────── Msg_dirs store ────────────────────

    /// Write `<store_dir>/<pkg>/msg/<Type>.msg` = `text` — a fake `.msg` store.
    fn write_store_msg(store_dir: &Path, pkg: &str, ty: &str, text: &str) {
        let dir = store_dir.join(pkg).join("msg");
        std::fs::create_dir_all(&dir).unwrap();
        std::fs::write(dir.join(format!("{ty}.msg")), text).unwrap();
    }

    #[test]
    fn msg_dirs_absent_is_todays_behavior() {
        // No msg_dirs key ⇒ None ⇒ empty effective dirs (built-ins + UNITREE).
        let cfg = BridgeConfig::from_yaml(HAPPY, "test").expect("happy config");
        assert!(cfg.msg_dirs.is_none());
        assert!(cfg.effective_msg_dirs().is_empty());
    }

    #[test]
    fn msg_dirs_empty_list_is_loud() {
        // A written but EMPTY msg_dirs key means nothing — loud error.
        let err = BridgeConfig::from_yaml(
            r#"
msg_dirs: []
mappings:
  - dds_topic: /cmd_vel
    ros_type: geometry_msgs/Twist
    cerulion_topic: /go2/twist_in
"#,
            "test",
        )
        .unwrap_err();
        assert!(matches!(err, ConfigError::EmptyMsgDirs), "{err:?}");
        assert!(err.to_string().contains("EMPTY msg_dirs"), "{err}");
    }

    #[test]
    fn msg_dirs_nonexistent_path_is_loud() {
        let err = BridgeConfig::from_yaml(
            r#"
msg_dirs:
  - /cerulion/definitely/not/a/real/store/dir
mappings:
  - dds_topic: /cmd_vel
    ros_type: geometry_msgs/Twist
    cerulion_topic: /go2/twist_in
"#,
            "test",
        )
        .unwrap_err();
        assert!(
            matches!(err, ConfigError::BadMsgDir { index: 0, .. }),
            "{err:?}"
        );
        let msg = err.to_string();
        assert!(
            msg.contains("/cerulion/definitely/not/a/real/store/dir"),
            "names the path: {msg}"
        );
        assert!(
            msg.contains("cerulion ros2 attach"),
            "names the remediation: {msg}"
        );
        // The error states the config-relative
        // resolution contract (relative entries join the config file's dir).
        assert!(
            msg.contains("config file's own directory"),
            "names the config-relative contract: {msg}"
        );
    }

    #[test]
    fn relative_msg_dirs_resolve_against_the_config_file_dir() {
        // THE doomed-subdir-run regression pin.
        // The store lives at <ws>/schemas; the config at <ws>/graphs/… names
        // `../schemas`. The test process's CWD is the CRATE root (not <ws>),
        // so CWD-relative resolution would FAIL this config's validation
        // with BadMsgDir; config-relative resolution loads the store from any
        // CWD.
        let ws = tempfile::tempdir().unwrap();
        write_store_msg(
            &ws.path().join("schemas"),
            "acme",
            "Widget",
            "int32 id\nfloat64 value\n",
        );
        std::fs::create_dir_all(ws.path().join("graphs")).unwrap();
        let config_path = ws.path().join("graphs").join("attach.bridge.yaml");
        let yaml = "msg_dirs:\n  - ../schemas\nmappings:\n  - dds_topic: /widget\n    \
                    ros_type: acme/Widget\n    cerulion_topic: /go2/widget\n";
        let cfg = BridgeConfig::from_yaml(yaml, &config_path.display().to_string())
            .expect("relative msg_dirs must resolve against the config file's dir");
        // The effective dir is the JOINED path (config parent + ../schemas)…
        assert_eq!(
            cfg.effective_msg_dirs(),
            &[ws.path().join("graphs").join("../schemas")]
        );
        // …and the run-time codec loads the store through it (the store-only
        // raw mapping validated above, which already proves the read_dir probe
        // hit the real store).
        assert!(crate::generic::schema_supported(
            &cfg.runtime_codec(),
            "acme/Widget"
        ));

        // Absolute entries are used VERBATIM (never re-joined).
        let abs_store = tempfile::tempdir().unwrap();
        write_store_msg(abs_store.path(), "acme", "Widget", "int32 id\n");
        let yaml_abs = format!(
            "msg_dirs:\n  - {}\nmappings:\n  - dds_topic: /widget\n    ros_type: acme/Widget\n    \
             cerulion_topic: /go2/widget\n",
            abs_store.path().display()
        );
        let cfg_abs = BridgeConfig::from_yaml(&yaml_abs, &config_path.display().to_string())
            .expect("absolute msg_dirs entry");
        assert_eq!(
            cfg_abs.effective_msg_dirs(),
            &[abs_store.path().to_path_buf()]
        );

        // Degenerate fallback: a parent-less `path` (the tests' "test"
        // placeholder) leaves a relative entry untouched — which then fails
        // the readability probe from this CWD, loudly (never a silent guess).
        let err = BridgeConfig::from_yaml(
            "msg_dirs:\n  - definitely_not_a_dir_here\nmappings:\n  - dds_topic: /widget\n    \
             ros_type: acme/Widget\n    cerulion_topic: /go2/widget\n",
            "test",
        )
        .unwrap_err();
        assert!(
            matches!(err, ConfigError::BadMsgDir { index: 0, .. }),
            "{err:?}"
        );
    }

    #[test]
    fn msg_dirs_valid_empty_dir_is_accepted_with_a_typed_mapping() {
        // A valid (even empty) store dir is honored; the typed mapping needs no
        // store, so validation passes.
        let store = tempfile::tempdir().unwrap();
        let yaml = format!(
            "msg_dirs:\n  - {}\nmappings:\n  - dds_topic: /cmd_vel\n    ros_type: \
             geometry_msgs/Twist\n    cerulion_topic: /go2/twist_in\n",
            store.path().display()
        );
        let cfg = BridgeConfig::from_yaml(&yaml, "test").expect("valid msg_dirs dir");
        assert_eq!(cfg.effective_msg_dirs().len(), 1);
    }

    #[test]
    fn store_only_raw_type_validates_and_codec_knows_it_via_msg_dirs() {
        // The end-to-end "codec knows() a store-only type" pin THROUGH config
        // validation: a type NEITHER built-in NOR a typed projection is accepted
        // as a RAW mapping because msg_dirs supplies its schema.
        let store = tempfile::tempdir().unwrap();
        write_store_msg(store.path(), "acme", "Widget", "int32 id\nfloat64 value\n");
        assert!(
            RosType::parse("acme/Widget").is_none(),
            "Widget is not a typed projection"
        );
        let yaml = format!(
            "msg_dirs:\n  - {}\nmappings:\n  - dds_topic: /widget\n    ros_type: acme/Widget\n    \
             cerulion_topic: /go2/widget\n",
            store.path().display()
        );
        let cfg = BridgeConfig::from_yaml(&yaml, "test").expect("store-only raw type validates");
        assert_eq!(cfg.raw_mappings().len(), 1);
        // The codec seeded from the SAME store KNOWS it (the bridge gate)...
        let codec = crate::generic::bridge_codec_with_store(cfg.effective_msg_dirs());
        assert!(crate::generic::schema_supported(&codec, "acme/Widget"));
        // ...and a no-store codec does NOT (proves the store is load-bearing).
        assert!(!crate::generic::schema_supported(
            &crate::generic::bridge_codec(),
            "acme/Widget"
        ));
    }

    #[test]
    fn runtime_codec_knows_store_only_type_and_matches_validate() {
        // The pump's run-time codec seam
        // ([`BridgeConfig::runtime_codec`]) and config validation are built from
        // the SAME `effective_msg_dirs`. A store-only type that VALIDATES as a
        // raw mapping (validate's codec saw the store) is ALSO known by the
        // run-time codec (the pump's codec sees the store) — so it transcodes at
        // drain time, never dies validated-at-load / dead-at-runtime.
        let store = tempfile::tempdir().unwrap();
        write_store_msg(store.path(), "acme", "Widget", "int32 id\nfloat64 value\n");
        let yaml = format!(
            "msg_dirs:\n  - {}\nmappings:\n  - dds_topic: /widget\n    ros_type: acme/Widget\n    \
             cerulion_topic: /go2/widget\n",
            store.path().display()
        );
        // validate ACCEPTED the store-only raw mapping...
        let cfg = BridgeConfig::from_yaml(&yaml, "test").expect("store-only raw type validates");
        assert_eq!(cfg.raw_mappings().len(), 1);
        // ...and the RUN-TIME codec (the pump's seam) ALSO knows it.
        assert!(crate::generic::schema_supported(
            &cfg.runtime_codec(),
            "acme/Widget"
        ));

        // A no-store config's run-time codec does NOT know it (store load-bearing).
        let no_store = BridgeConfig::from_yaml(HAPPY, "test").expect("no-store config");
        assert!(!crate::generic::schema_supported(
            &no_store.runtime_codec(),
            "acme/Widget"
        ));
    }

    // =====================================================================
    // Per-route slice sizing.
    //
    // Each oracle is stated as the pair that actually matters: the SLICE the
    // route is created with, and the receive-queue DEPTH that slice buys it
    // through `cerulion_core::transport::ingress_route_buffer_depth`. A slice
    // assertion alone would not say whether the change did anything, because
    // the depth rule clamps.
    // =====================================================================

    /// The absorption a depth buys at the Go2's ~1.66 kHz Point-LIO rate — the
    /// rate the per-route sizing is about. Stated here so each oracle below can
    /// say what its number MEANS.
    fn absorbs_ms_at_1660hz(depth: usize) -> f64 {
        1000.0 * depth as f64 / 1660.0
    }

    fn raw_mapping(ros_type: &str, max_slice_len: Option<u32>) -> TopicMapping {
        TopicMapping {
            dds_topic: "/probe".to_string(),
            ros_type: ros_type.to_string(),
            cerulion_topic: "/go2/probe".to_string(),
            qos: QosMode::BestEffort,
            max_slice_len,
            route: RouteMode::Raw,
        }
    }

    /// A route whose type is recursively FIXED gets its EXACT frame size, and
    /// that reaches the depth CAP.
    ///
    /// Hand oracle: `geometry_msgs/Twist` is two `Vector3`s = 6 × f64 = 48 bytes
    /// of fixed section, so the frame is `WireHeader::SIZE` (32) + 48 = 80.
    /// `clamp(64 MiB / 80, 16, 1024)` = 1024, the cap.
    #[test]
    fn a_fixed_type_route_gets_its_exact_frame_size_and_the_depth_cap() {
        let codec = crate::generic::bridge_codec_with_store(&[]);
        let m = raw_mapping("geometry_msgs/Twist", None);

        // The CONFIGURED-only value, for contrast — the bridge default.
        assert_eq!(BridgeConfig::raw_slice_len(&m), DEFAULT_RAW_MAX_SLICE_LEN);
        assert_eq!(
            cerulion_core::transport::ingress_route_buffer_depth(
                DEFAULT_RAW_MAX_SLICE_LEN.get() as usize
            ),
            64
        );

        let slice = BridgeConfig::raw_slice_len_for_route(&m, &codec);
        assert_eq!(slice.get(), 80, "32-byte header + 48-byte Twist");
        let depth = cerulion_core::transport::ingress_route_buffer_depth(slice.get() as usize);
        assert_eq!(depth, 1024, "the cap: 64 MiB / 80 B is far past it");
        assert!(
            absorbs_ms_at_1660hz(depth) > 600.0,
            "617 ms of absorption, against 38.6 ms at the bridge default"
        );
    }

    /// A VARIABLE type takes its declared tier when the tier is BELOW the
    /// bridge default.
    ///
    /// These are the TWO ~1.66 kHz Go2 topics per-route sizing exists for — `/state_estimation`
    /// (`nav_msgs/Odometry`) and `/tf` (`tf2_msgs/TFMessage`) — and they
    /// land on DIFFERENT tiers, which is why each carries its own hand oracle
    /// rather than sharing a loop:
    ///
    /// * `/tf` is TIER_SMALL = 256 KiB → `clamp(64 MiB / 256 KiB, 16, 1024)` =
    ///   256. It only reaches this row at all because the shared tier table
    ///   puts TFMessage at SMALL, not MEDIUM. At MEDIUM its 4 MiB is above the
    ///   bridge default, the `min` leaves it at 1 MiB and depth 64, and a
    ///   burst bench measures it losing 12.67 % / 13.15 %.
    /// * `/state_estimation` is TIER_TINY = 16 KiB → `clamp(64 MiB / 16 KiB,
    ///   16, 1024)` = 4096 clamped to the 1024 CAP. The tier table puts
    ///   `nav_msgs/Odometry` at TINY, not SMALL: its wire size is near-constant
    ///   (688 B of fixed section including two 288-B covariance matrices, plus
    ///   two frame strings ≈ 1 KB worst), so the type never needs 256 KiB and
    ///   a SMALL tier would leave 4x of depth on the table.
    ///
    /// The pair being at different depths is load-bearing for the test, not an
    /// accident of the tier table: a single shared expectation would pass a rule
    /// that ignored the schema and returned one bucket for everything below the
    /// default.
    #[test]
    fn a_variable_type_below_the_default_takes_its_tier() {
        let codec = crate::generic::bridge_codec_with_store(&[]);

        // `/tf` — TIER_SMALL.
        let m = raw_mapping("tf2_msgs/TFMessage", None);
        let slice = BridgeConfig::raw_slice_len_for_route(&m, &codec);
        assert_eq!(slice.get() as usize, 256 * 1024, "TFMessage: TIER_SMALL");
        let depth = cerulion_core::transport::ingress_route_buffer_depth(slice.get() as usize);
        assert_eq!(depth, 256, "TFMessage");
        assert!(
            (absorbs_ms_at_1660hz(depth) - 154.2).abs() < 0.5,
            "TFMessage: 154 ms, against 38.6 ms at the bridge default — past the measured \
             107.9 ms worst drive pass"
        );

        // `/state_estimation` — TIER_TINY.
        let m = raw_mapping("nav_msgs/Odometry", None);
        let slice = BridgeConfig::raw_slice_len_for_route(&m, &codec);
        assert_eq!(slice.get() as usize, 16 * 1024, "Odometry: TIER_TINY");
        let depth = cerulion_core::transport::ingress_route_buffer_depth(slice.get() as usize);
        assert_eq!(
            depth, 1024,
            "Odometry: the cap — 64 MiB / 16 KiB is far past it"
        );
        assert!(
            absorbs_ms_at_1660hz(depth) > 600.0,
            "Odometry: 617 ms, against 38.6 ms at the bridge default and 154 ms at TIER_SMALL"
        );
    }

    /// A VARIABLE type whose tier is AT OR ABOVE the bridge default keeps the
    /// conservative slice, unchanged.
    ///
    /// `sensor_msgs/PointCloud2` (TIER_HUGE = 128 MiB) is the guard the sizing
    /// rule needs by name: adopting the tier instead of `min`-ing it would give a
    /// point-cloud route a 128 MiB slice and a depth of 16 — SHALLOWER than
    /// the default, the exact opposite of the intent.
    ///
    /// `nav_msgs/Path` (TIER_MEDIUM = 4 MiB) covers the tier immediately above
    /// the bridge default, which is where the boundary actually sits. It is also
    /// the only MEDIUM row in this arm (TFMessage resolves to TIER_SMALL):
    /// without it a mutation that emptied the medium arm would leave the
    /// boundary untested.
    ///
    /// Neither row is inherently at risk of overflow — both are large-payload
    /// types and therefore low-rate, which is why leaving them at depth 64 is the
    /// intended shape rather than a residual.
    #[test]
    fn a_variable_type_at_or_above_the_default_keeps_the_conservative_slice() {
        let codec = crate::generic::bridge_codec_with_store(&[]);

        for ros_type in ["sensor_msgs/PointCloud2", "nav_msgs/Path"] {
            let m = raw_mapping(ros_type, None);
            let slice = BridgeConfig::raw_slice_len_for_route(&m, &codec);
            assert_eq!(
                slice, DEFAULT_RAW_MAX_SLICE_LEN,
                "{ros_type} must be left at the bridge default, never widened to its tier"
            );
            assert_eq!(
                cerulion_core::transport::ingress_route_buffer_depth(slice.get() as usize),
                64,
                "{ros_type} keeps the default depth"
            );
        }
    }

    /// An EXPLICIT `max_slice_len:` is taken verbatim — never narrowed by the
    /// schema budget.
    ///
    /// Driven in the direction that can actually fail: a 4 MiB explicit value on
    /// a type whose derived budget is 80 bytes. A rule that `min`-ed
    /// unconditionally would return 80 here and silently overrule an operator
    /// who sized the route on purpose.
    #[test]
    fn an_explicit_slice_is_never_narrowed_by_the_schema_budget() {
        let codec = crate::generic::bridge_codec_with_store(&[]);
        let m = raw_mapping("geometry_msgs/Twist", Some(4 * 1024 * 1024));

        assert_eq!(
            BridgeConfig::raw_slice_len_for_route(&m, &codec).get() as usize,
            4 * 1024 * 1024
        );
        // ...and the same type WITHOUT the explicit value does derive, so the
        // test cannot pass by the derivation being wired to nothing.
        assert_eq!(
            BridgeConfig::raw_slice_len_for_route(
                &raw_mapping("geometry_msgs/Twist", None),
                &codec
            )
            .get(),
            80
        );
    }

    /// A type the codec cannot resolve keeps the configured value rather than
    /// inventing one.
    ///
    /// `RawIngressRoute::open` refuses such a route anyway (the no-schema
    /// refusal), so this arm is about the sizing function never fabricating a
    /// number for a type it could not look up — and never panicking on one.
    #[test]
    fn an_unresolvable_type_keeps_the_configured_slice() {
        let codec = crate::generic::bridge_codec_with_store(&[]);

        assert_eq!(
            BridgeConfig::raw_slice_len_for_route(&raw_mapping("acme/NoSuchType", None), &codec),
            DEFAULT_RAW_MAX_SLICE_LEN
        );
        assert_eq!(
            BridgeConfig::raw_slice_len_for_route(
                &raw_mapping("acme/NoSuchType", Some(65536)),
                &codec
            )
            .get(),
            65536
        );
    }

    /// A store-only type — a `.msg` acquired from the robot
    /// at attach time — is sized by the SAME rule, from the store's own schema.
    ///
    /// This is the arm that proves the rule reaches the routes `cerulion ros2
    /// attach` actually generates: those types are not in the built-in corpus at
    /// all, so a derivation that only worked for built-ins would leave every
    /// robot-specific route (on the Go2, every `unitree_go/*` topic) at the
    /// bridge default while only the built-in rows are sized.
    ///
    /// Hand oracle: `acme/Widget` is `int32 id` (4) + `float64 value` (8) with
    /// the f64 aligned to 8, so the fixed section is 16 bytes and the frame is
    /// 48. `clamp(64 MiB / 48, 16, 1024)` = 1024.
    #[test]
    fn a_store_only_type_is_sized_from_its_store_schema() {
        let store = tempfile::tempdir().unwrap();
        write_store_msg(store.path(), "acme", "Widget", "int32 id\nfloat64 value\n");
        let yaml = format!(
            "msg_dirs:\n  - {}\nmappings:\n  - dds_topic: /widget\n    ros_type: acme/Widget\n    \
             cerulion_topic: /go2/widget\n",
            store.path().display()
        );
        let cfg = BridgeConfig::from_yaml(&yaml, "test").expect("store-only raw type validates");
        let codec = cfg.runtime_codec();
        let m = &cfg.raw_mappings()[0];

        let slice = BridgeConfig::raw_slice_len_for_route(m, &codec);
        assert_eq!(slice.get(), 48, "32-byte header + a 16-byte fixed section");
        assert_eq!(
            cerulion_core::transport::ingress_route_buffer_depth(slice.get() as usize),
            1024
        );

        // A codec WITHOUT the store cannot resolve it, and falls back rather
        // than fabricating — the anti-tautology half: the assertion above is
        // about the store being consulted, not about 48 being a constant.
        assert_eq!(
            BridgeConfig::raw_slice_len_for_route(m, &crate::generic::bridge_codec_with_store(&[])),
            DEFAULT_RAW_MAX_SLICE_LEN
        );
    }
}
