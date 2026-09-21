// SPDX-License-Identifier: AGPL-3.0-only
//! The GENERIC, schema-driven registry seam.
//!
//! One mechanism serves every type: a
//! single [`CdrCodec`](cerulion_core::codegen::CdrCodec) built over the
//! embedded ROS 2 message registry transcodes ANY resolvable `pkg/Type`
//! from a DDS CDR payload straight into a Cerulion wire frame — no
//! per-message-type Rust struct, no hand-written mapping code. The four
//! serde structs + CDR codecs in `cerulion_go2_dds` are ORACLES (see the
//! byte-equivalence tests below), not the mechanism.
//!
//! # "Unknown type" means "no schema", never "no codec"
//!
//! [`schema_supported`] resolves a type iff a [`MessageSchema`] exists for it
//! — the whole `native_ros2_messages::BUILTIN_MSGS` registry plus any
//! workspace schemas the codec is built with. There is no closed enum: adding
//! a schema (the auto-schema chain, or a workspace `.msg`) is all a
//! new type needs.
//!
//! # How the decoded frame is published (the integration choice)
//!
//! [`decode_to_wire_frame`] returns a complete Cerulion wire frame (32-byte
//! `WireHeader` + payload) whose `schema_hash` is the SOURCE schema's — ready
//! for `CerulionPublisher::publish_raw` on a dynamically-created publisher.
//! The precedent is the network→local injector
//! (`TransportManager::create_ingress_publisher` → `publish_raw`): one
//! zero-copy publisher per discovered topic, no typed macro port, fully
//! config/discovery-driven. That is exactly the sink `cerulion ros2 attach`
//! wires — one publisher per `{dds_topic → cerulion_topic}` mapping,
//! its frames minted by this codec.
//!
//! The existing [`DdsBridge`](crate::DdsBridge) node keeps its four TYPED
//! macro ports because those carry deliberate, application-level PROJECTIONS
//! (`SportModeState`→`nav_msgs/Odometry` with the quaternion reorder,
//! `Twist`→`TwistStamped`, `Request`→JSON `std_msgs/String`) — remaps, NOT
//! pure schema transcodes, so the generic codec (a faithful pass-through)
//! does not subsume them. [`route_mapping`] is the dispatch rule: the four
//! projection types stay TYPED (typed wins — their value IS the projection);
//! any OTHER schema-resolvable type takes the raw-generic path.
//!
//! The raw path has two halves, both in this module: the DDS side ([`raw`]
//! — a rustdds `DeserializerAdapter` capturing verbatim CDR bodies; its
//! module doc records what rustdds and ros2-client do here) and
//! the Cerulion side ([`RawIngressRoute`] — a dynamically-created
//! egress-suppressed ingress publisher fed by this codec's frames). The
//! pump's drain thread wires them (see
//! `pump::drain_raw_from_stream`): config mappings partition via
//! [`route_mapping`], one [`raw::RawReader`] per raw mapping is created ON
//! the drain thread, drained via `as_async_stream()` exactly like the typed
//! subscriptions, and each [`raw::RawSample`] feeds
//! [`RawIngressRoute::publish_cdr_body`] (body + codec → frame →
//! `publish_raw`).

pub mod raw;

use std::path::{Path, PathBuf};

use cerulion_core::codegen::{parse_rosmsg, CdrCodec, CdrCodecError, CdrEndianness, MessageSchema};
use cerulion_core::transport::TransportManager;
use cerulion_core::wire::MaxSliceLen;
use cerulion_core::{CerulionPublisher, TransportError};

use crate::registry::RosType;

/// Unitree custom `.msg` schemas — the two ingress types NOT in the embedded
/// ROS 2 registry (`unitree_go`/`unitree_api`). In production the
/// auto-schema chain (or a workspace `schemas/*.yaml`) supplies these; here
/// they demonstrate that a NON-builtin type flows through the identical
/// generic mechanism with only a schema. Field order/types are transcribed
/// from `cerulion_go2_dds::messages` (the field-verified IDL mirror), so the
/// codec's CDR interpretation is byte-identical to the hand structs'.
const UNITREE_MSGS: &[(&str, &str, &str)] = &[
    (
        "unitree_go",
        "IMUState",
        "float32[4] quaternion\nfloat32[3] gyroscope\nfloat32[3] accelerometer\nfloat32[3] rpy\nint8 temperature\n",
    ),
    (
        "unitree_go",
        "SportModeState",
        "builtin_interfaces/Time stamp\nuint32 error_code\nIMUState imu_state\nuint8 mode\nfloat32 progress\nuint8 gait_type\nfloat32 foot_raise_height\nfloat32[3] position\nfloat32 body_height\nfloat32[3] velocity\nfloat32 yaw_speed\nfloat32[4] range_obstacle\nint16[4] foot_force\nfloat32[12] foot_position_body\nfloat32[12] foot_speed_body\n",
    ),
    ("unitree_api", "RequestIdentity", "int64 id\nint64 api_id\n"),
    ("unitree_api", "RequestLease", "int64 id\n"),
    ("unitree_api", "RequestPolicy", "int32 priority\nbool noreply\n"),
    (
        "unitree_api",
        "RequestHeader",
        "RequestIdentity identity\nRequestLease lease\nRequestPolicy policy\n",
    ),
    (
        "unitree_api",
        "Request",
        "RequestHeader header\nstring parameter\nuint8[] binary\n",
    ),
];

/// Parse the codec's schema set: every embedded ROS 2 built-in plus the
/// Unitree custom types. A built-in that fails to parse is skipped with a
/// loud warning (a corrupt built-in is a codegen bug, not a runtime state);
/// a Unitree schema that fails to parse is a programming error here and
/// panics (the text above is a compile-time constant).
fn bridge_schema_set() -> Vec<MessageSchema> {
    let mut set: Vec<MessageSchema> = Vec::new();
    for &(package, name, text) in native_ros2_messages::BUILTIN_MSGS {
        match parse_rosmsg(text, name, Some(package)) {
            Ok(s) => set.push(s),
            Err(e) => tracing::warn!(
                package,
                name,
                error = ?e,
                "dds_bridge generic codec: failed to parse a built-in .msg; skipping it"
            ),
        }
    }
    for &(package, name, text) in UNITREE_MSGS {
        set.push(parse_rosmsg(text, name, Some(package)).unwrap_or_else(|e| {
            panic!("built-in Unitree schema {package}/{name} failed to parse: {e:?}")
        }));
    }
    set
}

/// [`bridge_schema_set`] PLUS every workspace `.msg` store schema found under
/// `msg_dirs`. Store schemas are appended LAST, so a store
/// definition WINS over a colliding built-in / `UNITREE_MSGS` entry via
/// [`CdrCodec::new`]'s last-insert-wins dedup (shadow semantics) — and
/// every such shadow fires a loud `warn!`, never a silent override. An EMPTY
/// `msg_dirs` yields exactly [`bridge_schema_set`] (the back-compat floor).
///
/// Across MULTIPLE dirs a LATER dir's definition wins over an EARLIER dir's of
/// the same qualified name (the same last-insert-wins rule, and each such
/// cross-dir shadow also warns). An exact-DUPLICATE dir entry (a config typo
/// listing one store dir twice) is read ONCE — a `debug!` note, never a
/// self-shadow `warn!` flood.
fn bridge_schema_set_with_store(msg_dirs: &[PathBuf]) -> Vec<MessageSchema> {
    let mut set = bridge_schema_set();
    if msg_dirs.is_empty() {
        return set;
    }
    // Names already in the set (built-ins + UNITREE, and earlier store dirs) —
    // a store schema colliding with any of these shadows it (store wins) loudly.
    let mut seen: std::collections::BTreeSet<String> =
        set.iter().map(|s| s.qualified_name()).collect();
    // Dedupe the dir list (exact path). A config typo listing
    // the SAME store dir twice must NOT re-read it — a re-read would fire a
    // shadow `warn!` for EVERY schema in the dir (a flood) and stay behaviorally
    // sane only via CdrCodec last-insert-wins. Reading once is the smallest
    // correct fix; DISTINCT dirs are all still processed (later-dir-wins on a
    // genuine cross-dir collision).
    let mut seen_dirs: std::collections::BTreeSet<&PathBuf> = std::collections::BTreeSet::new();
    for dir in msg_dirs {
        if !seen_dirs.insert(dir) {
            tracing::debug!(
                dir = %dir.display(),
                "dds_bridge generic codec: duplicate msg_dirs entry — reading it once"
            );
            continue;
        }
        let stored_schemas = read_store_schemas(dir);
        // A listed store dir contributing ZERO schemas is LOUD.
        // `msg_dirs` is written exactly because the store held content at
        // attach time — an emptied-but-still-present dir passes the config's
        // readability probe, store-only raw mappings then fail as a generic
        // UnknownRosType, and a store schema that was SHADOWING a
        // built-in silently reverts to the built-in layout (wrong decode, no
        // collision, no shadow warn). This is the breadcrumb for both.
        if stored_schemas.is_empty() {
            tracing::warn!(
                dir = %dir.display(),
                "dds_bridge generic codec: msg_dirs store contributed ZERO schemas — this \
                 config was generated expecting store content; if schemas were moved or \
                 deleted, store-only raw mappings will fail and any store shadow of a \
                 built-in has silently reverted (re-run `cerulion ros2 attach` to regenerate)"
            );
        }
        for stored in stored_schemas {
            let qualified = stored.qualified_name();
            if !seen.insert(qualified.clone()) {
                tracing::warn!(
                    schema = %qualified,
                    dir = %dir.display(),
                    "dds_bridge generic codec: workspace .msg store shadows a built-in / \
                     UNITREE (or earlier store) schema of the same qualified name — the store \
                     definition wins"
                );
            }
            set.push(stored);
        }
    }
    set
}

/// Build the generic CDR codec over the bridge's built-in + `UNITREE_MSGS`
/// schema set (NO workspace store). The store-aware form is
/// [`bridge_codec_with_store`]; this is its `&[]` shim, kept for the many
/// test/helper callers that need no store.
pub fn bridge_codec() -> CdrCodec {
    bridge_codec_with_store(&[])
}

/// [`bridge_codec`] seeded ALSO with the workspace `.msg` store dirs — the
/// production entry point (config validation + the pump route
/// here with the config's `msg_dirs`). Surfaces any nested-resolution warnings
/// loudly (a warning means a schema references a type absent from the set — a
/// decode of that type would fail). An EMPTY slice is byte-identical to
/// [`bridge_codec`].
pub fn bridge_codec_with_store(msg_dirs: &[PathBuf]) -> CdrCodec {
    let (codec, warnings) = CdrCodec::new(bridge_schema_set_with_store(msg_dirs));
    for w in &warnings {
        tracing::warn!(warning = %w, "dds_bridge generic codec: schema resolution warning");
    }
    codec
}

/// Read every `<store_dir>/<pkg>/msg/<Type>.msg` into a parsed [`MessageSchema`]
/// — the bridge's own mirror of `cerulion_cli_engine::schema_store` (the two
/// are kept in agreement by the mirror-duty test pin; the demo bridge
/// cannot depend on the CLI engine). The package name comes FROM THE PATH (a
/// `.msg` carries none); enumeration is sorted (deterministic); and the
/// robustness contract mirrors the store reader: a missing/unreadable dir or a
/// `.msg` that fails to read/parse is a `warn!` + skip — never a hard failure
/// (one bad file must not sink the rest). A bad `msg_dirs` ENTRY is already
/// rejected loudly at config validation ([`crate::config::BridgeConfig`]); this
/// reader is defensive on top.
fn read_store_schemas(store_dir: &Path) -> Vec<MessageSchema> {
    let mut out: Vec<MessageSchema> = Vec::new();
    let entries = match std::fs::read_dir(store_dir) {
        Ok(e) => e,
        Err(e) => {
            tracing::warn!(
                dir = %store_dir.display(),
                error = %e,
                "dds_bridge store: msg_dirs entry unreadable — skipped"
            );
            return out;
        }
    };
    // (pkg, <store_dir>/<pkg>/msg) for every package subdir carrying a msg/ dir.
    // Mirror parity: the enumeration-level skips warn like the
    // engine reader's (`cerulion_cli_engine::schema_store`) — an entry the
    // engine counted (making attach declare the type RESOLVABLE) that this
    // reader drops silently would surface only as a downstream UnknownRosType
    // with no breadcrumb.
    let mut pkg_dirs: Vec<(String, PathBuf)> = Vec::new();
    for dirent in entries {
        let entry = match dirent {
            Ok(e) => e,
            Err(e) => {
                tracing::warn!(
                    dir = %store_dir.display(),
                    error = %e,
                    "dds_bridge store: unreadable directory entry — skipped"
                );
                continue;
            }
        };
        let path = entry.path();
        if !path.is_dir() {
            continue;
        }
        let msg_dir = path.join("msg");
        if !msg_dir.is_dir() {
            continue;
        }
        let Some(pkg) = path.file_name().and_then(|n| n.to_str()) else {
            tracing::warn!(
                path = %path.display(),
                "dds_bridge store: non-UTF-8 package directory name — skipped"
            );
            continue;
        };
        pkg_dirs.push((pkg.to_string(), msg_dir));
    }
    pkg_dirs.sort();
    for (pkg, msg_dir) in pkg_dirs {
        let mut files: Vec<PathBuf> = Vec::new();
        match std::fs::read_dir(&msg_dir) {
            Ok(entries) => {
                for dirent in entries {
                    match dirent {
                        Ok(d) => {
                            let p = d.path();
                            if p.extension().and_then(|x| x.to_str()) == Some("msg") {
                                files.push(p);
                            }
                        }
                        Err(e) => tracing::warn!(
                            dir = %msg_dir.display(),
                            error = %e,
                            "dds_bridge store: unreadable directory entry — skipped"
                        ),
                    }
                }
            }
            Err(e) => {
                tracing::warn!(
                    dir = %msg_dir.display(),
                    error = %e,
                    "dds_bridge store: msg/ directory unreadable — skipped"
                );
                continue;
            }
        }
        files.sort();
        for f in files {
            let Some(stem) = f.file_stem().and_then(|s| s.to_str()) else {
                tracing::warn!(
                    file = %f.display(),
                    "dds_bridge store: .msg file has a non-UTF-8 name — skipped"
                );
                continue;
            };
            let text = match std::fs::read_to_string(&f) {
                Ok(t) => t,
                Err(e) => {
                    tracing::warn!(
                        file = %f.display(),
                        error = %e,
                        "dds_bridge store: could not read .msg — skipped"
                    );
                    continue;
                }
            };
            match parse_rosmsg(&text, stem, Some(pkg.as_str())) {
                Ok(s) => out.push(s),
                Err(e) => tracing::warn!(
                    file = %f.display(),
                    error = ?e,
                    "dds_bridge store: failed to parse .msg — skipped"
                ),
            }
        }
    }
    out
}

/// True iff `ros_type` (`pkg/Type`) resolves to a schema the codec can
/// transcode — the acceptance-critical "no schema" (never "no codec") signal
/// the config layer should gate on.
pub fn schema_supported(codec: &CdrCodec, ros_type: &str) -> bool {
    codec.knows(ros_type)
}

/// Decode a full DDS payload (encapsulation header + CDR body) of type
/// `ros_type` into a Cerulion wire frame ready for `publish_raw`. The generic
/// ingress mechanism — no per-type code. `sequence`/`timestamp_ns` stamp the
/// frame's `WireHeader`.
pub fn decode_to_wire_frame(
    codec: &CdrCodec,
    ros_type: &str,
    dds_payload: &[u8],
    sequence: u32,
    timestamp_ns: u64,
) -> Result<Vec<u8>, CdrCodecError> {
    codec.decode_dds_payload(ros_type, dds_payload, sequence, timestamp_ns)
}

// ---------------------------------------------------------------------------
// Mapping routing: which path serves a config ros_type
// ---------------------------------------------------------------------------

/// How a config mapping's `ros_type` routes through the bridge.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum MappingRoute {
    /// One of the four deliberate TYPED projection ports
    /// ([`crate::registry::RosType`] — `SportModeState`→Odometry etc.).
    /// Typed WINS over raw for these types (they all have schemas too, but
    /// their value IS the projection, not the pass-through).
    TypedPort(RosType),
    /// Any other type with a [`MessageSchema`] in the bridge codec — the
    /// raw-generic path ([`RawIngressRoute`]), zero per-type code.
    RawGeneric,
    /// No projection port AND no schema — refuse loudly (and the refusal
    /// means "no schema", never "no codec").
    Unknown,
}

/// Route a config `ros_type` — the CANONICAL dispatch rule (pure): typed
/// projection first, then schema-resolved raw-generic, else
/// [`MappingRoute::Unknown`]. Config validation and the pump's
/// `typed_mappings`/`raw_mappings` partition implement exactly this rule for the
/// DEFAULT (`RouteMode::Auto`) mappings.
///
/// Caveat: a mapping may FORCE the raw path with `route: raw`
/// ([`crate::config::RouteMode::Raw`]) even for a registry type — that per-
/// MAPPING override cannot be expressed here (this fn sees only the type), so it
/// lives at the mapping level ([`crate::config::TopicMapping::is_raw_route`],
/// which `validate` + the accessors share). This fn stays the type-level
/// precedence pin; a caller needing the override-aware decision consults the
/// mapping's route field.
pub fn route_mapping(codec: &CdrCodec, ros_type: &str) -> MappingRoute {
    if let Some(t) = RosType::parse(ros_type) {
        return MappingRoute::TypedPort(t);
    }
    if codec.knows(ros_type) {
        return MappingRoute::RawGeneric;
    }
    MappingRoute::Unknown
}

// ---------------------------------------------------------------------------
// The dynamic-publisher half: raw-generic ingress routes
// ---------------------------------------------------------------------------

/// A raw-route failure. Every variant names the route so a dead mapping is
/// diagnosable on sight; the caller (the pump wiring) owns log policy — this
/// type never logs on the publish path (no flood). The `source` payloads are
/// BOXED so the per-sample publish path's `Result` stays lean
/// (`TransportError`/`CdrCodecError` are large enums — clippy
/// `result_large_err`); Display text and the `source()` chain are unchanged
/// by the Box.
#[derive(Debug, thiserror::Error)]
pub enum RawRouteError {
    /// `ros_type` has no [`MessageSchema`] in the bridge codec set — the
    /// acceptance signal: "unknown type" means NO SCHEMA, never "no
    /// codec".
    #[error(
        "ros_type {ros_type:?} has no MessageSchema in the bridge codec set — \"unknown \
         type\" means NO SCHEMA, never \"no codec\". Add the type's .msg text to \
         the schema registry (native_ros2_messages BUILTIN_MSGS, the bridge's UNITREE_MSGS, \
         or the auto-schema chain)"
    )]
    UnknownType { ros_type: String },
    /// The local ingress publisher could not be created (topic collision,
    /// iceoryx2 provisioning failure, egress-loop refusal — see
    /// `TransportManager::create_ingress_publisher`).
    #[error(
        "raw route {ros_type:?} -> {cerulion_topic:?}: failed to create the local ingress \
         publisher: {source}"
    )]
    PublisherCreate {
        ros_type: String,
        cerulion_topic: String,
        source: Box<TransportError>,
    },
    /// The DDS payload failed to transcode (truncated/hostile/corrupt CDR, or
    /// an unsupported encapsulation). The sample is dropped; the route's
    /// sequence is NOT burned (gap-free published streams — the
    /// commit-time rule).
    #[error(
        "raw route {ros_type:?} -> {cerulion_topic:?}: CDR decode failed (sample dropped, \
         sequence not burned): {source}"
    )]
    Decode {
        ros_type: String,
        cerulion_topic: String,
        source: Box<CdrCodecError>,
    },
    /// The decoded frame failed to publish locally (e.g. loan-pool
    /// exhaustion). The sample is dropped; the sequence is NOT burned.
    #[error(
        "raw route {ros_type:?} -> {cerulion_topic:?}: local publish failed (sample \
         dropped, sequence not burned): {source}"
    )]
    Publish {
        ros_type: String,
        cerulion_topic: String,
        source: Box<TransportError>,
    },
}

/// One raw-generic DDS→Cerulion ingress route: a dynamically-created local
/// publisher fed by [`CdrCodec`]-decoded wire frames — the
/// network-ingress precedent applied to DDS (`create_ingress_publisher` →
/// `publish_raw`).
///
/// Like the zenoh ingress, frames flow OFF the node-tick path (the pump's
/// drain machinery publishes them directly), so the route is egress-
/// suppressed by construction (an ingress publisher never registers a
/// network bridge flag — no DDS→SHM→zenoh→SHM loops) and the topic is
/// provisioned like a foreign External writer (absolute `source:` consumers
/// attach exactly as they do to `/ext/…` topics).
///
/// # Sequence discipline
///
/// The route owns the wire `sequence` counter and consumes it at COMMIT: a
/// decode or publish failure drops the sample WITHOUT burning the sequence,
/// so downstream consumers see gap-free streams and a gap genuinely means
/// transport loss, not a bridge-side decode failure.
///
/// # Timestamps
///
/// `timestamp_ns` is caller-provided per publish (this half deliberately
/// reads no clocks — pure of wall time, directly oracle-testable). The pump
/// wiring stamps the TRANSPORT clock at drain-publish time — consistent with
/// the typed ports' loan-time stamps and deliberately NOT the DDS writer's
/// `source_timestamp` (a remote wall clock; rationale in the lib.rs raw-path
/// docs).
pub struct RawIngressRoute {
    ros_type: String,
    cerulion_topic: String,
    publisher: CerulionPublisher,
    sequence: u32,
    published_total: u64,
    decode_failures_total: u64,
}

impl RawIngressRoute {
    /// Open a raw route: validate `ros_type` resolves in the codec (the
    /// no-schema refusal), then create the egress-suppressed local ingress
    /// publisher on `cerulion_topic` (the network-ingress precedent; provisioned
    /// like a foreign External writer; a topic owned by an in-graph
    /// single-writer producer is rejected by iceoryx2).
    pub fn open(
        transport: &TransportManager,
        codec: &CdrCodec,
        ros_type: &str,
        cerulion_topic: &str,
        max_slice_len: MaxSliceLen,
    ) -> Result<Self, RawRouteError> {
        if !codec.knows(ros_type) {
            return Err(RawRouteError::UnknownType {
                ros_type: ros_type.to_string(),
            });
        }
        let publisher = transport
            .create_ingress_publisher(cerulion_topic, max_slice_len)
            .map_err(|source| RawRouteError::PublisherCreate {
                ros_type: ros_type.to_string(),
                cerulion_topic: cerulion_topic.to_string(),
                source: Box::new(source),
            })?;
        tracing::info!(
            ros_type = %ros_type,
            topic = %cerulion_topic,
            max_slice_len = %max_slice_len,
            "dds_bridge raw route: generic ingress publisher created"
        );
        // Make the raw route REMOTELY DEMANDABLE. Push its
        // registration onto the `__cerulion/gateway_topics` control channel so
        // the (separate) gateway process announces + serves demand for
        // it exactly like a YAML-declared output — the whole point of this:
        // a `ros2 attach` robot's ~90 raw routes were invisible to remote viz.
        // BEST-EFFORT by design: local DDS→SHM bridging must never die because
        // the registration can't be queued. NOTE the API is NETWORK-FREE (it
        // opens a local iceoryx2 control service, no zenoh) — its two real
        // failure classes are (a) a registration-hostile NAME (reserved
        // namespace / over MAX_REG_TOPIC_LEN: per-route, each loudly
        // actionable) and (b) a PROCESS-WIDE local control-plane failure (the
        // control service can't open: identical for all ~90 routes, so it is
        // warn-LATCHED once per process rather than flooding the log).
        // Either way the route stays local-only and bridging continues. The
        // ORDER matters: the publisher above was created FIRST, so
        // `create_ingress_publisher`'s create-time egress loop-check ran
        // before any egress flag could exist for this topic (and that flag
        // lives in the GATEWAY process's bridge manager anyway, which this
        // worker-side check never reads).
        match codec.schema_hash(ros_type) {
            Some(schema_hash) => {
                match transport.register_dynamic_egress_topic(cerulion_topic, schema_hash) {
                    Ok(_) => {}
                    Err(error @ TransportError::InvalidTransportConfig { .. }) => {
                        // Per-NAME refusal (reserved / over-long) — at most a
                        // handful, each independently actionable: loud every time.
                        tracing::warn!(
                            ros_type = %ros_type,
                            topic = %cerulion_topic,
                            error = %error,
                            "dds_bridge raw route: dynamic egress registration REFUSED \
                             (registration-hostile name) — the route stays LOCAL-ONLY \
                             (remote demand cannot discover or grant it); local DDS→SHM \
                             bridging is unaffected"
                        );
                    }
                    Err(error) => {
                        // Process-wide control-plane failure (the local
                        // `__cerulion/gateway_topics` service can't open) —
                        // identical for every route, so warn ONCE per process
                        // and demote repeats to debug (the ~90-line flood).
                        static CONTROL_PLANE_WARNED: std::sync::atomic::AtomicBool =
                            std::sync::atomic::AtomicBool::new(false);
                        if !CONTROL_PLANE_WARNED.swap(true, std::sync::atomic::Ordering::Relaxed) {
                            tracing::warn!(
                                ros_type = %ros_type,
                                topic = %cerulion_topic,
                                error = %error,
                                "dds_bridge raw route: dynamic egress registration failed \
                                 (LOCAL control-plane — the __cerulion/gateway_topics \
                                 service could not be opened; NOT a network condition). \
                                 This and every later raw route stay LOCAL-ONLY; local \
                                 DDS→SHM bridging is unaffected. Further identical \
                                 failures log at debug"
                            );
                        } else {
                            tracing::debug!(
                                ros_type = %ros_type,
                                topic = %cerulion_topic,
                                error = %error,
                                "dds_bridge raw route: dynamic egress registration failed \
                                 (control-plane, suppressed repeat) — route LOCAL-ONLY"
                            );
                        }
                    }
                }
            }
            None => {
                // Unreachable in practice: `codec.knows(ros_type)` gated above,
                // and every known layout carries a hash. Guarded loudly anyway —
                // a silent skip here would silently strand the route local-only.
                tracing::warn!(
                    ros_type = %ros_type,
                    topic = %cerulion_topic,
                    "dds_bridge raw route: no schema hash for a codec-known type — \
                     dynamic egress registration skipped, route stays LOCAL-ONLY"
                );
            }
        }
        Ok(Self {
            ros_type: ros_type.to_string(),
            cerulion_topic: cerulion_topic.to_string(),
            publisher,
            sequence: 0,
            published_total: 0,
            decode_failures_total: 0,
        })
    }

    /// Transcode one full DDS payload (encapsulation header + CDR body) and
    /// publish the resulting Cerulion wire frame locally. Returns the
    /// recipient count from `publish_raw`. Sequence is consumed only on a
    /// successful publish (see the type docs).
    pub fn publish_dds_payload(
        &mut self,
        codec: &CdrCodec,
        dds_payload: &[u8],
        timestamp_ns: u64,
    ) -> Result<usize, RawRouteError> {
        let frame = codec
            .decode_dds_payload(&self.ros_type, dds_payload, self.sequence, timestamp_ns)
            .map_err(|source| {
                self.decode_failures_total += 1;
                RawRouteError::Decode {
                    ros_type: self.ros_type.clone(),
                    cerulion_topic: self.cerulion_topic.clone(),
                    source: Box::new(source),
                }
            })?;
        self.commit_frame(frame)
    }

    /// Transcode one raw CDR BODY (encapsulation already stripped — the shape
    /// [`raw::RawSample`] carries, since rustdds parses the 4-byte header off
    /// before the adapter runs) and publish it. The pump-facing entry point:
    /// the pump's drain wiring calls
    /// `route.publish_cdr_body(&codec, sample.endianness, &sample.body, ts)`.
    /// Same sequence discipline as [`Self::publish_dds_payload`].
    pub fn publish_cdr_body(
        &mut self,
        codec: &CdrCodec,
        endianness: CdrEndianness,
        cdr_body: &[u8],
        timestamp_ns: u64,
    ) -> Result<usize, RawRouteError> {
        let frame = codec
            .decode(
                &self.ros_type,
                endianness,
                cdr_body,
                self.sequence,
                timestamp_ns,
            )
            .map_err(|source| {
                self.decode_failures_total += 1;
                RawRouteError::Decode {
                    ros_type: self.ros_type.clone(),
                    cerulion_topic: self.cerulion_topic.clone(),
                    source: Box::new(source),
                }
            })?;
        self.commit_frame(frame)
    }

    /// Publish a decoded frame + consume the sequence at COMMIT (shared tail
    /// of both publish entry points — the discipline lives in ONE
    /// place).
    fn commit_frame(&mut self, frame: Vec<u8>) -> Result<usize, RawRouteError> {
        let recipients =
            self.publisher
                .publish_raw(&frame)
                .map_err(|source| RawRouteError::Publish {
                    ros_type: self.ros_type.clone(),
                    cerulion_topic: self.cerulion_topic.clone(),
                    source: Box::new(source),
                })?;
        self.sequence = self.sequence.wrapping_add(1);
        self.published_total += 1;
        Ok(recipients)
    }

    /// The route's ROS type (`pkg/Type`).
    pub fn ros_type(&self) -> &str {
        &self.ros_type
    }

    /// The Cerulion topic this route publishes on.
    pub fn cerulion_topic(&self) -> &str {
        &self.cerulion_topic
    }

    /// Frames published (== sequences consumed). Principle #3 observability;
    /// the pump renders these next to its hop counters.
    pub fn published_total(&self) -> u64 {
        self.published_total
    }

    /// Samples dropped to a CDR decode failure (sequence not burned).
    pub fn decode_failures_total(&self) -> u64 {
        self.decode_failures_total
    }

    /// The sequence the NEXT successful publish will stamp.
    pub fn next_sequence(&self) -> u32 {
        self.sequence
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use cerulion_core::codegen::{CdrEndianness, FrameValueKind, FrameWalker};
    use cerulion_core::wire::WireHeader;
    use cerulion_go2_dds::cdr::{
        encode_point_cloud2, encode_request, encode_sport_mode_state, encode_twist, CDR_LE_HEADER,
    };
    use cerulion_go2_dds::messages::{
        Header, ImuState, PointCloud2, PointField, Request, RequestHeader, RequestIdentity,
        RequestLease, RequestPolicy, SportModeState, Time, Twist, Vector3, POINT_FIELD_FLOAT32,
        SPORT_API_ID_MOVE,
    };

    fn walker() -> FrameWalker {
        let (w, _) = FrameWalker::new(bridge_schema_set());
        w
    }

    /// The core anti-regression oracle: the generic codec's decode∘encode
    /// reproduces the EXACT CDR body the hand codec produced (byte-for-byte),
    /// so the hand codec is a redundant oracle of the generic mechanism.
    fn assert_hand_cdr_round_trips(codec: &CdrCodec, qname: &str, hand_payload: &[u8]) -> Vec<u8> {
        assert_eq!(&hand_payload[..4], &CDR_LE_HEADER, "hand payload is CDR_LE");
        let body = &hand_payload[4..];
        let frame = codec
            .decode(qname, CdrEndianness::Little, body, 0, 0)
            .expect("generic decode of hand CDR");
        // Frame header carries the SOURCE schema's hash (ready for publish_raw).
        let hdr = WireHeader::read_from_buf(&frame).unwrap();
        assert_eq!(hdr.schema_hash, codec.schema_hash(qname).unwrap());
        assert_eq!(hdr.total_size as usize, frame.len());
        // Round-trip: encode(decode(hand_cdr)) == hand_cdr, byte-exact.
        let re = codec.encode(qname, CdrEndianness::Little, &frame).unwrap();
        assert_eq!(
            re, body,
            "generic encode must reproduce the hand codec's CDR"
        );
        frame
    }

    #[test]
    fn twist_generic_codec_byte_matches_hand_codec() {
        let codec = bridge_codec();
        let twist = Twist {
            linear: Vector3 {
                x: 0.5,
                y: 0.0,
                z: 0.0,
            },
            angular: Vector3 {
                x: 0.0,
                y: 0.0,
                z: 0.25,
            },
        };
        let payload = encode_twist(&twist).unwrap();
        let frame = assert_hand_cdr_round_trips(&codec, "geometry_msgs/Twist", &payload);
        // Walker cross-check against the hand struct's field values.
        let w = walker();
        let fv = w.walk("geometry_msgs/Twist", &frame).unwrap();
        match fv.field("linear") {
            Some(FrameValueKind::Nested(n)) => {
                assert_eq!(n.field("x"), Some(&FrameValueKind::F64(0.5)));
            }
            other => panic!("expected linear Nested, got {other:?}"),
        }
        match fv.field("angular") {
            Some(FrameValueKind::Nested(n)) => {
                assert_eq!(n.field("z"), Some(&FrameValueKind::F64(0.25)));
            }
            other => panic!("expected angular Nested, got {other:?}"),
        }
    }

    #[test]
    fn pointcloud2_generic_codec_byte_matches_hand_codec() {
        let codec = bridge_codec();
        // The worst-case wire shape: Header (variable nested) + PointField[]
        // (variable-nested array) + uint8[] data.
        let pc = PointCloud2 {
            header: Header {
                stamp: Time {
                    sec: 12,
                    nanosec: 34,
                },
                frame_id: "lidar".to_string(),
            },
            height: 1,
            width: 2,
            fields: vec![
                PointField {
                    name: "x".to_string(),
                    offset: 0,
                    datatype: POINT_FIELD_FLOAT32,
                    count: 1,
                },
                PointField {
                    name: "y".to_string(),
                    offset: 4,
                    datatype: POINT_FIELD_FLOAT32,
                    count: 1,
                },
                PointField {
                    name: "z".to_string(),
                    offset: 8,
                    datatype: POINT_FIELD_FLOAT32,
                    count: 1,
                },
            ],
            is_bigendian: false,
            point_step: 12,
            row_step: 24,
            data: (0u8..24).collect(),
            is_dense: true,
        };
        let payload = encode_point_cloud2(&pc).unwrap();
        let frame = assert_hand_cdr_round_trips(&codec, "sensor_msgs/PointCloud2", &payload);
        // Walker cross-check on the framework-decodable fields.
        let w = walker();
        let fv = w.walk("sensor_msgs/PointCloud2", &frame).unwrap();
        assert_eq!(fv.field("height"), Some(&FrameValueKind::U32(1)));
        assert_eq!(fv.field("width"), Some(&FrameValueKind::U32(2)));
        assert_eq!(fv.field("point_step"), Some(&FrameValueKind::U32(12)));
        assert_eq!(
            fv.field("data"),
            Some(&FrameValueKind::Bytes(&(0u8..24).collect::<Vec<_>>()))
        );
        match fv.field("header") {
            Some(FrameValueKind::Nested(h)) => {
                assert_eq!(h.field("frame_id"), Some(&FrameValueKind::Str("lidar")));
            }
            other => panic!("expected header Nested, got {other:?}"),
        }
    }

    #[test]
    fn sportmodestate_generic_codec_byte_matches_hand_codec() {
        // A NON-builtin type (unitree_go/SportModeState), fixed-array-heavy
        // with a nested IMUState carrying repr(C) trailing padding — the
        // generic codec handles the CDR↔repr(C) padding divergence.
        let codec = bridge_codec();
        assert!(schema_supported(&codec, "unitree_go/SportModeState"));
        let s = SportModeState {
            stamp: Time {
                sec: 100,
                nanosec: 250_000_000,
            },
            error_code: 0xDEAD_BEEF,
            imu_state: ImuState {
                quaternion: [1.0, 0.0, 0.0, 0.0],
                gyroscope: [0.1, 0.2, 0.3],
                accelerometer: [0.0, 0.0, 9.81],
                rpy: [0.01, -0.02, 0.03],
                temperature: 42,
            },
            mode: 1,
            progress: 0.5,
            gait_type: 2,
            foot_raise_height: 0.08,
            position: [1.5, -2.5, 0.3],
            body_height: 0.32,
            velocity: [0.4, 0.0, 0.0],
            yaw_speed: 0.25,
            range_obstacle: [1.0, 2.0, 3.0, 4.0],
            foot_force: [10, 20, 30, 40],
            foot_position_body: [0.1; 12],
            foot_speed_body: [0.2; 12],
        };
        let payload = encode_sport_mode_state(&s).unwrap();
        let frame = assert_hand_cdr_round_trips(&codec, "unitree_go/SportModeState", &payload);
        // Walker cross-check on a couple of fields (error_code fixed@8, the
        // nested imu_state's temperature).
        let w = walker();
        let fv = w.walk("unitree_go/SportModeState", &frame).unwrap();
        assert_eq!(
            fv.field("error_code"),
            Some(&FrameValueKind::U32(0xDEAD_BEEF))
        );
        match fv.field("imu_state") {
            Some(FrameValueKind::Nested(imu)) => {
                assert_eq!(imu.field("temperature"), Some(&FrameValueKind::I8(42)));
            }
            other => panic!("expected imu_state Nested, got {other:?}"),
        }
    }

    #[test]
    fn request_generic_codec_byte_matches_hand_codec() {
        // A NON-builtin type (unitree_api/Request): nested fixed header +
        // string + uint8[].
        let codec = bridge_codec();
        assert!(schema_supported(&codec, "unitree_api/Request"));
        let req = Request {
            header: RequestHeader {
                identity: RequestIdentity {
                    id: 42,
                    api_id: SPORT_API_ID_MOVE,
                },
                lease: RequestLease { id: 0 },
                policy: RequestPolicy {
                    priority: 0,
                    noreply: false,
                },
            },
            parameter: r#"{"x":0.1,"y":0.0,"z":0.0}"#.to_string(),
            binary: vec![0xDE, 0xAD, 0xBE, 0xEF],
        };
        let payload = encode_request(&req).unwrap();
        let frame = assert_hand_cdr_round_trips(&codec, "unitree_api/Request", &payload);
        let w = walker();
        let fv = w.walk("unitree_api/Request", &frame).unwrap();
        assert_eq!(
            fv.field("parameter"),
            Some(&FrameValueKind::Str(r#"{"x":0.1,"y":0.0,"z":0.0}"#))
        );
        assert_eq!(
            fv.field("binary"),
            Some(&FrameValueKind::Bytes(&[0xDE, 0xAD, 0xBE, 0xEF]))
        );
        // Cross-check the nested fixed header value survived.
        match fv.field("header") {
            Some(FrameValueKind::Nested(h)) => match h.field("identity") {
                Some(FrameValueKind::Nested(id)) => {
                    assert_eq!(
                        id.field("api_id"),
                        Some(&FrameValueKind::I64(SPORT_API_ID_MOVE))
                    );
                }
                other => panic!("expected identity Nested, got {other:?}"),
            },
            other => panic!("expected header Nested, got {other:?}"),
        }
    }

    #[test]
    fn a_type_the_hand_registry_never_had_flows_with_zero_per_type_code() {
        // sensor_msgs/Imu — a real built-in the go2 hand registry does NOT
        // support — decoded from a HAND-built CDR body to a HAND oracle, proving
        // "any standard-registry type flows with zero per-type code".
        let codec = bridge_codec();
        assert!(schema_supported(&codec, "sensor_msgs/Imu"));
        assert!(
            crate::registry::RosType::parse("sensor_msgs/Imu").is_none(),
            "Imu is deliberately absent from the hand registry"
        );

        // Imu = Header header, Quaternion orientation, float64[9]
        // orientation_covariance, Vector3 angular_velocity, float64[9]
        // angular_velocity_covariance, Vector3 linear_acceleration, float64[9]
        // linear_acceleration_covariance. Header is the only variable field.
        let mut body = Vec::new();
        // header.stamp{sec=5, nanosec=6}
        body.extend_from_slice(&5i32.to_le_bytes());
        body.extend_from_slice(&6u32.to_le_bytes());
        // header.frame_id = "imu_link"
        body.extend_from_slice(&9u32.to_le_bytes()); // len incl NUL
        body.extend_from_slice(b"imu_link\0"); // 9 bytes; body offset now 8+4+9=21
                                               // orientation Quaternion{x,y,z,w} aligns to 8 (pad 21→24)
        while body.len() % 8 != 0 {
            body.push(0);
        }
        for v in [0.0f64, 0.0, 0.0, 1.0] {
            body.extend_from_slice(&v.to_le_bytes());
        }
        // orientation_covariance float64[9]
        for _ in 0..9 {
            body.extend_from_slice(&0.0f64.to_le_bytes());
        }
        // angular_velocity Vector3{0.1,0.2,0.3}
        for v in [0.1f64, 0.2, 0.3] {
            body.extend_from_slice(&v.to_le_bytes());
        }
        // angular_velocity_covariance [9]
        for _ in 0..9 {
            body.extend_from_slice(&0.0f64.to_le_bytes());
        }
        // linear_acceleration Vector3{0.0,0.0,9.81}
        for v in [0.0f64, 0.0, 9.81] {
            body.extend_from_slice(&v.to_le_bytes());
        }
        // linear_acceleration_covariance [9]
        for _ in 0..9 {
            body.extend_from_slice(&0.0f64.to_le_bytes());
        }

        let frame = codec
            .decode("sensor_msgs/Imu", CdrEndianness::Little, &body, 0, 0)
            .expect("Imu decodes");
        let w = walker();
        let fv = w.walk("sensor_msgs/Imu", &frame).unwrap();
        match fv.field("header") {
            Some(FrameValueKind::Nested(h)) => {
                assert_eq!(h.field("frame_id"), Some(&FrameValueKind::Str("imu_link")));
            }
            other => panic!("expected header Nested, got {other:?}"),
        }
        match fv.field("orientation") {
            Some(FrameValueKind::Nested(q)) => {
                assert_eq!(q.field("w"), Some(&FrameValueKind::F64(1.0)));
            }
            other => panic!("expected orientation Nested, got {other:?}"),
        }
        match fv.field("angular_velocity") {
            Some(FrameValueKind::Nested(v)) => {
                assert_eq!(v.field("x"), Some(&FrameValueKind::F64(0.1)));
                assert_eq!(v.field("z"), Some(&FrameValueKind::F64(0.3)));
            }
            other => panic!("expected angular_velocity Nested, got {other:?}"),
        }
        // Round-trips byte-exact (the hand body IS canonical XCDR1).
        let re = codec
            .encode("sensor_msgs/Imu", CdrEndianness::Little, &frame)
            .unwrap();
        assert_eq!(re, body);
    }

    #[test]
    fn adversarial_inputs_error_matching_the_hand_codec() {
        use cerulion_go2_dds::cdr::decode_twist;
        let codec = bridge_codec();

        // (a) Truncated body: 3 bytes of a Twist. Both codecs Err.
        let short = [0x01, 0x02, 0x03];
        assert!(decode_twist(&{
            let mut p = CDR_LE_HEADER.to_vec();
            p.extend_from_slice(&short);
            p
        })
        .is_err());
        assert!(codec
            .decode("geometry_msgs/Twist", CdrEndianness::Little, &short, 0, 0)
            .is_err());

        // (b) Hostile string length in a PointCloud2 frame_id.
        let mut hostile = Vec::new();
        hostile.extend_from_slice(&0i32.to_le_bytes()); // stamp.sec
        hostile.extend_from_slice(&0u32.to_le_bytes()); // stamp.nanosec
        hostile.extend_from_slice(&0xFFFF_FFFFu32.to_le_bytes()); // frame_id len
        hostile.extend_from_slice(&[0x41, 0x42]); // 2 bytes, not 4 GiB
        assert!(hostile.len() < 50);
        assert!(codec
            .decode(
                "sensor_msgs/PointCloud2",
                CdrEndianness::Little,
                &hostile,
                0,
                0
            )
            .is_err());

        // (c) Non-UTF-8 in a std_msgs/String data field.
        assert!(schema_supported(&codec, "std_msgs/String"));
        let mut bad = Vec::new();
        bad.extend_from_slice(&2u32.to_le_bytes()); // len 2
        bad.extend_from_slice(&[0xFF, 0x00]); // invalid utf8 + NUL
        assert!(codec
            .decode("std_msgs/String", CdrEndianness::Little, &bad, 0, 0)
            .is_err());

        // (d) Big-endian Twist decodes fine (parity with the hand codec's BE
        // arm), and byte-matches an LE decode of the same values.
        let mut be = Vec::new();
        let mut le = Vec::new();
        for v in [1.0f64, 2.0, 3.0, 4.0, 5.0, 6.0] {
            be.extend_from_slice(&v.to_be_bytes());
            le.extend_from_slice(&v.to_le_bytes());
        }
        let bf = codec
            .decode("geometry_msgs/Twist", CdrEndianness::Big, &be, 0, 0)
            .unwrap();
        let lf = codec
            .decode("geometry_msgs/Twist", CdrEndianness::Little, &le, 0, 0)
            .unwrap();
        assert_eq!(bf, lf);
    }

    #[test]
    fn schema_support_means_no_schema_not_no_codec() {
        let codec = bridge_codec();
        // Every built-in the go2 sinks care about resolves.
        for t in [
            "geometry_msgs/Twist",
            "sensor_msgs/PointCloud2",
            "sensor_msgs/Imu",
            "sensor_msgs/JointState",
            "nav_msgs/Odometry",
            "std_msgs/String",
            "std_msgs/Header",
            "tf2_msgs/TFMessage",
        ] {
            assert!(schema_supported(&codec, t), "{t} must resolve");
        }
        // The Unitree customs resolve too (schema-provided, not builtin).
        assert!(schema_supported(&codec, "unitree_go/SportModeState"));
        assert!(schema_supported(&codec, "unitree_api/Request"));
        // A truly unknown type is "no schema" — the error the config surfaces.
        assert!(!schema_supported(&codec, "totally/Bogus"));
        assert_eq!(
            decode_to_wire_frame(&codec, "totally/Bogus", &[0, 1, 0, 0, 0, 0, 0, 0], 0, 0),
            Err(CdrCodecError::UnknownSchema("totally/Bogus".to_string()))
        );
    }

    #[test]
    fn decode_is_deterministic() {
        let codec = bridge_codec();
        let twist = Twist {
            linear: Vector3 {
                x: 1.0,
                y: 2.0,
                z: 3.0,
            },
            angular: Vector3 {
                x: 4.0,
                y: 5.0,
                z: 6.0,
            },
        };
        let payload = encode_twist(&twist).unwrap();
        let a = decode_to_wire_frame(&codec, "geometry_msgs/Twist", &payload, 9, 10).unwrap();
        let b = decode_to_wire_frame(&codec, "geometry_msgs/Twist", &payload, 9, 10).unwrap();
        assert_eq!(a, b);
    }

    #[test]
    fn route_mapping_typed_wins_over_raw_for_all_four_projection_types() {
        // The precedence pin: every projection type ALSO has a schema in the
        // codec (PointCloud2/Twist are builtins; SportModeState/Request ride
        // UNITREE_MSGS), so without typed-first dispatch they would
        // double-route. Their value IS the projection — typed wins.
        let codec = bridge_codec();
        for t in crate::registry::ALL_ROS_TYPES {
            assert!(
                schema_supported(&codec, t.ros_type_str()),
                "{} must have a schema (making the precedence non-vacuous)",
                t.ros_type_str()
            );
            assert_eq!(
                route_mapping(&codec, t.ros_type_str()),
                MappingRoute::TypedPort(t),
                "{} must route TYPED",
                t.ros_type_str()
            );
        }
    }

    #[test]
    fn route_mapping_schema_resolvable_types_route_raw_generic() {
        // The types the hand registry never had: builtins + a Unitree custom
        // type outside the projection set.
        let codec = bridge_codec();
        for ros_type in [
            "sensor_msgs/Imu",
            "sensor_msgs/JointState",
            "geometry_msgs/Vector3",
            "std_msgs/String",
            "unitree_go/IMUState",
        ] {
            assert_eq!(
                route_mapping(&codec, ros_type),
                MappingRoute::RawGeneric,
                "{ros_type} must route raw-generic"
            );
        }
    }

    #[test]
    fn route_mapping_no_schema_is_unknown() {
        let codec = bridge_codec();
        assert_eq!(
            route_mapping(&codec, "totally/Bogus"),
            MappingRoute::Unknown
        );
        // Near-miss spellings of a projection type do NOT silently fall
        // through to raw/typed — exact-match discipline (registry docs).
        assert_eq!(
            route_mapping(&codec, "sensor_msgs/msg/PointCloud2"),
            MappingRoute::Unknown
        );
    }

    // ─────────────────── msg_dirs store ────────────────────

    /// Write `<store_dir>/<pkg>/msg/<Type>.msg` = `text` — a fake `.msg` store.
    fn write_store_msg(store_dir: &Path, pkg: &str, ty: &str, text: &str) {
        let dir = store_dir.join(pkg).join("msg");
        std::fs::create_dir_all(&dir).unwrap();
        std::fs::write(dir.join(format!("{ty}.msg")), text).unwrap();
    }

    #[test]
    fn store_only_type_is_known_by_bridge_codec() {
        // A type NEITHER built-in NOR UNITREE flows through the generic codec
        // purely from a workspace `.msg` store dir (the mirror-duty BRIDGE side).
        let store = tempfile::tempdir().unwrap();
        write_store_msg(store.path(), "acme", "Widget", "int32 id\nfloat64 value\n");
        let dirs = vec![store.path().to_path_buf()];

        // The schema set gained exactly the one store type over the no-store set.
        let base = bridge_schema_set();
        let with_store = bridge_schema_set_with_store(&dirs);
        assert_eq!(with_store.len(), base.len() + 1);
        assert!(with_store
            .iter()
            .any(|s| s.qualified_name() == "acme/Widget"));

        // The codec KNOWS it; a no-store codec does not (store is load-bearing).
        let codec = bridge_codec_with_store(&dirs);
        assert!(schema_supported(&codec, "acme/Widget"));
        assert!(!schema_supported(&bridge_codec(), "acme/Widget"));
        assert_eq!(
            route_mapping(&codec, "acme/Widget"),
            MappingRoute::RawGeneric
        );
    }

    #[test]
    fn empty_msg_dirs_is_byte_identical_to_the_no_store_set() {
        // Back-compat floor: an empty store slice yields EXACTLY the no-store set.
        let base = bridge_schema_set();
        let with_empty = bridge_schema_set_with_store(&[]);
        assert_eq!(with_empty.len(), base.len());
        let base_names: std::collections::BTreeSet<String> =
            base.iter().map(|s| s.qualified_name()).collect();
        let empty_names: std::collections::BTreeSet<String> =
            with_empty.iter().map(|s| s.qualified_name()).collect();
        assert_eq!(base_names, empty_names);
        // Spot-check the codec: headline types resolve, a store-only one does not.
        let codec = bridge_codec_with_store(&[]);
        assert!(schema_supported(&codec, "geometry_msgs/Twist"));
        assert!(schema_supported(&codec, "unitree_go/SportModeState"));
        assert!(!schema_supported(&codec, "acme/Widget"));
    }

    #[test]
    fn store_schema_shadows_a_builtin_and_wins() {
        // Collision precedence: a store std_msgs/String with a
        // DIFFERENT layout WINS over the built-in (CdrCodec last-insert-wins);
        // the codec serves the STORE definition's schema_hash.
        let store = tempfile::tempdir().unwrap();
        // Built-in std_msgs/String is `string data`; the store adds a field so
        // its layout (and schema_hash) DIFFER.
        write_store_msg(
            store.path(),
            "std_msgs",
            "String",
            "string data\nint32 extra\n",
        );
        let dirs = vec![store.path().to_path_buf()];

        let store_codec = bridge_codec_with_store(&dirs);
        let base_codec = bridge_codec();
        // Both know the type...
        assert!(schema_supported(&store_codec, "std_msgs/String"));
        assert!(schema_supported(&base_codec, "std_msgs/String"));
        // ...but the store definition WON: the hash differs from the built-in's.
        assert_ne!(
            store_codec.schema_hash("std_msgs/String"),
            base_codec.schema_hash("std_msgs/String"),
            "the store's redefinition must win over the built-in"
        );
        // The winning hash equals a codec built from ONLY the store definition.
        let store_schema =
            parse_rosmsg("string data\nint32 extra\n", "String", Some("std_msgs")).unwrap();
        let (only_store, _) = CdrCodec::new(vec![store_schema]);
        assert_eq!(
            store_codec.schema_hash("std_msgs/String"),
            only_store.schema_hash("std_msgs/String"),
            "the codec serves the STORE definition of the shadowed type"
        );
    }

    /// Capture `tracing` output (at `max` level and above) emitted on THIS
    /// thread while running `f`, as one formatted string — a self-contained
    /// in-memory subscriber (this isolated demo workspace has no `tracing-test`
    /// dep). Thread-scoped via `with_default`, so parallel tests never
    /// cross-contaminate; uses the already-present `tracing-subscriber` dev-dep.
    fn capture_tracing<F: FnOnce()>(max: tracing::Level, f: F) -> String {
        use std::io::Write;
        use std::sync::{Arc, Mutex};
        #[derive(Clone)]
        struct Buf(Arc<Mutex<Vec<u8>>>);
        impl Write for Buf {
            fn write(&mut self, b: &[u8]) -> std::io::Result<usize> {
                self.0.lock().unwrap().extend_from_slice(b);
                Ok(b.len())
            }
            fn flush(&mut self) -> std::io::Result<()> {
                Ok(())
            }
        }
        impl<'a> tracing_subscriber::fmt::MakeWriter<'a> for Buf {
            type Writer = Buf;
            fn make_writer(&'a self) -> Buf {
                self.clone()
            }
        }
        let buf = Buf(Arc::new(Mutex::new(Vec::new())));
        let sub = tracing_subscriber::fmt()
            .with_writer(buf.clone())
            .with_max_level(max)
            .with_ansi(false)
            .finish();
        tracing::subscriber::with_default(sub, f);
        let bytes = buf.0.lock().unwrap().clone();
        String::from_utf8(bytes).expect("utf8 log capture")
    }

    #[test]
    fn store_shadow_of_a_builtin_fires_a_loud_warn() {
        // The store-shadows-a-builtin path must fire
        // the documented loud `warn!`, not silently override. (The existing
        // `store_schema_shadows_a_builtin_and_wins` pins only hash precedence.)
        let store = tempfile::tempdir().unwrap();
        write_store_msg(
            store.path(),
            "std_msgs",
            "String",
            "string data\nint32 extra\n",
        );
        let dirs = vec![store.path().to_path_buf()];
        let logs = capture_tracing(tracing::Level::WARN, || {
            let _ = bridge_schema_set_with_store(&dirs);
        });
        assert!(
            logs.contains("store definition wins"),
            "the builtin shadow must fire a loud warn: {logs}"
        );
        assert!(
            logs.contains("std_msgs/String"),
            "the warn must name the shadowed schema: {logs}"
        );
    }

    #[test]
    fn store_shadow_of_a_unitree_msg_fires_warn_and_wins() {
        // The go2 field-failure class — a store
        // schema shadowing a UNITREE type with a DRIFTED layout must fire the
        // loud warn AND win on hash. `unitree_go/SportModeState` is the exact
        // type the warn exists to catch; the store redefines it with a
        // trivially-different 1-field layout (nothing references it, so no
        // nested-resolution cascade).
        let store = tempfile::tempdir().unwrap();
        write_store_msg(
            store.path(),
            "unitree_go",
            "SportModeState",
            "uint32 error_code\n",
        );
        let dirs = vec![store.path().to_path_buf()];

        let logs = capture_tracing(tracing::Level::WARN, || {
            let _ = bridge_schema_set_with_store(&dirs);
        });
        assert!(
            logs.contains("store definition wins"),
            "the UNITREE shadow must fire a loud warn: {logs}"
        );
        assert!(
            logs.contains("unitree_go/SportModeState"),
            "the warn must name the shadowed UNITREE type: {logs}"
        );

        // The store definition WON on hash (vs the UNITREE built-in).
        let store_codec = bridge_codec_with_store(&dirs);
        assert!(schema_supported(&store_codec, "unitree_go/SportModeState"));
        assert_ne!(
            store_codec.schema_hash("unitree_go/SportModeState"),
            bridge_codec().schema_hash("unitree_go/SportModeState"),
            "the store redefinition must win over the UNITREE built-in"
        );
        let store_schema =
            parse_rosmsg("uint32 error_code\n", "SportModeState", Some("unitree_go")).unwrap();
        let (only_store, _) = CdrCodec::new(vec![store_schema]);
        assert_eq!(
            store_codec.schema_hash("unitree_go/SportModeState"),
            only_store.schema_hash("unitree_go/SportModeState"),
            "the codec serves the STORE definition of the shadowed UNITREE type"
        );
    }

    #[test]
    fn duplicate_msg_dir_is_read_once_no_shadow_flood() {
        // A config typo listing the SAME store dir
        // twice is read ONCE — no self-shadow `warn!` flood, and the schema set
        // is identical to listing it once (last-insert-wins would otherwise mask
        // the flood behaviorally while still spamming the log).
        let store = tempfile::tempdir().unwrap();
        write_store_msg(store.path(), "acme", "Widget", "int32 id\nfloat64 value\n");
        let once = vec![store.path().to_path_buf()];
        let twice = vec![store.path().to_path_buf(), store.path().to_path_buf()];

        assert_eq!(
            bridge_schema_set_with_store(&once).len(),
            bridge_schema_set_with_store(&twice).len(),
            "a duplicate dir must not add duplicate schemas"
        );
        let logs = capture_tracing(tracing::Level::WARN, || {
            let _ = bridge_schema_set_with_store(&twice);
        });
        assert!(
            !logs.contains("store definition wins"),
            "a duplicate dir must not self-shadow-flood: {logs}"
        );
        assert!(schema_supported(
            &bridge_codec_with_store(&twice),
            "acme/Widget"
        ));
    }

    #[test]
    fn later_msg_dir_wins_on_cross_dir_collision() {
        // Two DISTINCT dirs both defining
        // acme/Widget with DIFFERENT layouts — the LATER-listed dir's definition
        // wins (last-insert-wins), pinned by schema_hash so a loop-order
        // regression flipping to first-wins fails.
        let dir1 = tempfile::tempdir().unwrap();
        let dir2 = tempfile::tempdir().unwrap();
        write_store_msg(dir1.path(), "acme", "Widget", "int32 id\n");
        write_store_msg(dir2.path(), "acme", "Widget", "int32 id\nfloat64 value\n");
        let dirs = vec![dir1.path().to_path_buf(), dir2.path().to_path_buf()];

        let codec = bridge_codec_with_store(&dirs);
        let later = parse_rosmsg("int32 id\nfloat64 value\n", "Widget", Some("acme")).unwrap();
        let earlier = parse_rosmsg("int32 id\n", "Widget", Some("acme")).unwrap();
        let (later_only, _) = CdrCodec::new(vec![later]);
        let (earlier_only, _) = CdrCodec::new(vec![earlier]);
        assert_eq!(
            codec.schema_hash("acme/Widget"),
            later_only.schema_hash("acme/Widget"),
            "the LATER dir's definition must win"
        );
        assert_ne!(
            codec.schema_hash("acme/Widget"),
            earlier_only.schema_hash("acme/Widget"),
            "the earlier dir's definition must NOT win"
        );
    }

    #[test]
    fn read_store_schemas_skips_bad_content_without_sinking_the_rest() {
        // The skip-not-sink robustness contract — one
        // hand-mangled `.msg` (or an empty/msg-less pkg dir, or a stray non-.msg
        // file) must NOT take down the rest of the user-editable store.
        let store = tempfile::tempdir().unwrap();
        // A GOOD schema...
        write_store_msg(store.path(), "acme", "Good", "int32 id\nfloat64 value\n");
        // ...beside a MALFORMED one (a single-token line has no field name —
        // `parse_rosmsg` rejects it deterministically).
        write_store_msg(store.path(), "acme", "Broken", "garbage_no_field_name\n");
        // A pkg with an EMPTY msg/ dir (zero schemas, must not error).
        std::fs::create_dir_all(store.path().join("emptypkg").join("msg")).unwrap();
        // A pkg dir with NO msg/ subdir (skipped, not an error).
        std::fs::create_dir_all(store.path().join("nomsgpkg").join("srv")).unwrap();
        // A stray NON-.msg file inside a real msg/ dir (ignored by extension).
        std::fs::write(
            store.path().join("acme").join("msg").join("NOTES.txt"),
            "hi",
        )
        .unwrap();
        // A stray file at the store ROOT (not a dir → skipped).
        std::fs::write(store.path().join("loose.txt"), "hi").unwrap();

        let dirs = vec![store.path().to_path_buf()];
        let logs = capture_tracing(tracing::Level::WARN, || {
            let _ = bridge_schema_set_with_store(&dirs);
        });

        // The GOOD schema loaded; the malformed one did NOT; the codec built.
        let codec = bridge_codec_with_store(&dirs);
        assert!(
            schema_supported(&codec, "acme/Good"),
            "the good schema must survive the bad sibling"
        );
        assert!(
            !schema_supported(&codec, "acme/Broken"),
            "the malformed schema must be skipped, not loaded"
        );
        // The parse failure was LOUD (a skip, not a silent swallow, not a sink).
        assert!(
            logs.contains("failed to parse .msg"),
            "the malformed .msg must warn-and-skip: {logs}"
        );
    }

    #[test]
    fn empty_store_dir_contributing_zero_schemas_warns() {
        // A listed dir yielding ZERO schemas fires the loud
        // emptied-store breadcrumb (config validation accepts a valid-but-empty
        // dir by design — read_dir succeeds — so this warn is the ONLY signal
        // that a previously-acquired store was moved/deleted; a store shadow of
        // a built-in reverts SILENTLY otherwise).
        let store = tempfile::tempdir().unwrap(); // exists, readable, EMPTY
        let dirs = vec![store.path().to_path_buf()];
        let logs = capture_tracing(tracing::Level::WARN, || {
            let _ = bridge_schema_set_with_store(&dirs);
        });
        assert!(
            logs.contains("contributed ZERO schemas"),
            "an emptied store dir must warn: {logs}"
        );

        // Control (anti-tautology): a dir WITH content does not fire it.
        write_store_msg(store.path(), "acme", "Widget", "int32 id\n");
        let logs = capture_tracing(tracing::Level::WARN, || {
            let _ = bridge_schema_set_with_store(&dirs);
        });
        assert!(
            !logs.contains("contributed ZERO schemas"),
            "a populated store dir must not fire the empty-store warn: {logs}"
        );
    }
}
