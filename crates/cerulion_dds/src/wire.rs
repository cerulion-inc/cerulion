// SPDX-License-Identifier: AGPL-3.0-only
//! The wire rung's PURE decision engine.
//!
//! The live `~/get_type_description` rung ([`crate::wire_acquirer`]) is a thin
//! DDS I/O shell over the functions HERE: everything that decides *which* node
//! to call, *how* to correlate the reply, *how* a service response becomes
//! Cerulion-resolvable `.msg` text, and *how* the per-attempt call phase folds
//! (retry / wait-failure cache / wall cap) is pure, DDS-free, and oracle-tested
//! with hand-built inputs. That keeps the humble/no-live builds green (no
//! `ros2-client`/`rustdds` here) and makes the load-bearing logic testable
//! without a live ROS 2 peer (the live service round-trip itself is designed
//! against the REP-2011 spec + the ros2-client 0.10 source and validated
//! against a live ROS 2 Jazzy peer).
//!
//! The pipeline the live shell runs:
//!
//! 1. CONSUME the attach discovery result the engine already harvested:
//!    endpoints (+ RIHS01 `type_hash` + `writer_guid`), the `ros_discovery_info`
//!    node table, and remote participant vendor ids. The wire rung runs NO
//!    second discovery window — it is handed the engine's [`DiscoveryResult`].
//! 2. [`plan_service_calls`] — per requested `pkg/Type`: the hash SOURCE is
//!    publisher-preferred with subscriber-fallback (GUID-independent — hash
//!    PRESENCE alone decides reason (ii)); the call TARGETS are the union of
//!    owning nodes joined from ALL of the type's endpoints, writers first then
//!    readers (a reader-side participant with a node-table entry is a valid
//!    `get_type_description` target when the writer's PEI was lost). Yields
//!    an ordered list of [`CallAttempt`]s (node +
//!    hash + the vendor-appropriate correlation mapping try-order), or a loud,
//!    DISTINCT [`RungSkip`] for each of the three not-callable shapes (never
//!    discovered / no hash / no owning node).
//! 3. For each attempt the shell calls the service (bounded by per-call budgets
//!    and one overall call-phase wall cap); its `type_sources` become an
//!    [`AcquiredMsg`] closure via [`sources_to_closure`].
//!
//! [`DiscoveryResult`]: crate::DiscoveryResult

use std::collections::BTreeSet;
use std::time::Instant;

use crate::{
    AcquiredMsg, AcquisitionRung, DiscoveredEndpoint, DiscoveredNode, EndpointKind, RungSkip,
};

// ───────────────────────────── Skip wording ────────────────────────────────
//
// The wire rung's three DISTINCT not-callable shapes get three DISTINCT,
// precise skip reasons (one shared NO_TYPE_HASH_REASON would
// conflate them). Each states what discovery actually observed
// and names its likeliest cause + what the user can do. The exact wording is
// pinned against hand-pasted literals in `tests::test_skip_reason_wording_literals`.

/// Skip reason (i): the requested type was NOT seen on ANY endpoint in the
/// discovery window. Nothing on the robot advertised it (a topic that stopped
/// publishing, too short a window, or — since the wire rung consumes the
/// engine's harvest — a normalize_dds_type / normalize_ros_type mismatch), so
/// there is no endpoint to trace to an owning node.
pub const TYPE_NOT_DISCOVERED_REASON: &str =
    "this type was not seen on any endpoint in the discovery window — nothing on the robot \
     advertised it during discovery (a topic that stopped publishing, or too short a window) — \
     the wire rung has no endpoint to trace to a get_type_description-serving node";

/// Skip reason (ii): the type's endpoints DO exist but carry NO RIHS01 hash — a
/// pre-Iron distro or a non-ROS DDS peer (the server looks up
/// `get_type_description` BY HASH, so a missing hash is unusable). Hash
/// PRESENCE is judged GUID-INDEPENDENTLY: an
/// endpoint whose backend surfaced a hash but no GUID still counts as "hash
/// seen" — it falls to reason (iii), never here. Wording deliberately
/// unchanged (the issue-pinned literal).
pub const NO_TYPE_HASH_REASON: &str =
    "no type hash in discovery (pre-Iron distro or non-ROS DDS publisher)";

/// Skip reason (iii): the type's endpoints DO carry a hash but NO endpoint of
/// the type could be mapped to an owning ROS node — its GUID joins no
/// `ros_discovery_info` entry (PEI absent, dropped, or unparsed — e.g. a distro
/// whose `ParticipantEntitiesInfo` GID width does not match this build), or the
/// backend surfaced no endpoint GUID at all — so
/// no `get_type_description` service can be located.
pub const NO_OWNING_NODE_REASON: &str =
    "the endpoint advertises a type hash but could not be mapped to an owning ROS node \
     via ros_discovery_info (no ParticipantEntitiesInfo for its writer GUID) — cannot locate a \
     get_type_description service";

/// Build a single wire-rung [`RungSkip`].
pub fn wire_skip(reason: impl Into<String>) -> RungSkip {
    RungSkip {
        rung: AcquisitionRung::WireService,
        reason: reason.into(),
    }
}

// ─────────────────────────── Node name joining ─────────────────────────────

/// Join a ROS node namespace + base name into a fully-qualified name, mirroring
/// ROS 2 name composition. A root namespace (`""` or `"/"`) yields `/<name>`; a
/// nested namespace (`/robot`) yields `/robot/<name>`. Any trailing slash on
/// the namespace is normalized away. Pure — oracle-tested.
pub fn join_node_fqn(namespace: &str, name: &str) -> String {
    let ns = namespace.trim_end_matches('/');
    if ns.is_empty() {
        format!("/{name}")
    } else if let Some(rest) = ns.strip_prefix('/') {
        format!("/{rest}/{name}")
    } else {
        format!("/{ns}/{name}")
    }
}

// ───────────────────────── USER_DATA key=value parse ───────────────────────

/// Strip the CDR sequence encapsulation from a `USER_DATA` blob, if present —
/// the live-acceptance fix.
///
/// rustdds's `Parameter.value` is "the CDR encapsulation of the Parameter
/// type": for USER_DATA (a DDS `sequence<octet>`) that is
/// `[u32 length][content][padding to 4]`, NOT the naked `key=value;` string.
/// Live capture from the Jazzy container (FastDDS writer, via a throwaway
/// probe): `user_data.len() == 88`, bytes =
/// `[51 00 00 00, 74 79 70 65 68 61 73 68 3d 52 49 48 53 30 31 5f ...]` —
/// i.e. LE length `0x51 == 81`, then the ASCII `typehash=RIHS01_<64 hex>;`
/// (exactly 81 bytes: 9 + 7 + 64 + 1), then 3 padding bytes (81 → 84 padded,
/// plus the 4-byte prefix = 88). A naked-string parse would read the 4
/// prefix bytes into the first key, so EVERY live hash parse would return `None`
/// ("no type hash in discovery" for every type on the network, with
/// real user_data sitting in the snapshot).
///
/// Tolerant detect-and-strip rule: if `len >= 4`, read the first 4 bytes as
/// u32 LE — if `prefix <= len - 4` AND `(len - 4) - prefix <= 3` (padding
/// tolerance), the content is `bytes[4..4 + prefix]`; else try the same with
/// u32 BE (a BE-encapsulated SEDP peer); else FALL THROUGH and treat the
/// WHOLE slice as naked content (hermetic callers + any rmw that hands naked
/// strings). Safety of the fallback: a naked `typehash=...`/`enclave=...`
/// string starts with ASCII (e.g. 't' = 0x74), so its first-4-bytes-as-u32 is
/// ~2 billion in either endianness — never within 3 of a realistic length, so
/// a false strip is structurally implausible. Pure — oracle-tested with the
/// live-captured shape and its BE / naked / beyond-tolerance twins.
fn strip_user_data_encapsulation(user_data: &[u8]) -> &[u8] {
    let Some(body_len) = user_data.len().checked_sub(4) else {
        return user_data; // shorter than a length prefix — naked
    };
    let head: [u8; 4] = user_data[..4].try_into().expect("len >= 4");
    let candidates = [
        u32::from_le_bytes(head) as usize,
        u32::from_be_bytes(head) as usize,
    ];
    for prefix in candidates {
        if prefix <= body_len && body_len - prefix <= 3 {
            return &user_data[4..4 + prefix];
        }
    }
    user_data // no plausible encapsulation — naked content
}

/// Parse a DDS endpoint `USER_DATA` blob into its `key=value` entries. The rmw
/// encodes user data as a `;`-terminated list of `key=value` pairs
/// (`enclave=/;typehash=RIHS01_…;`), and rustdds hands the blob to us
/// CDR-ENCAPSULATED (`[u32 length][content][padding]`) — stripped first via
/// `strip_user_data_encapsulation`; naked content is accepted too (the
/// fall-through). This is TOLERANT: a trailing `;` is optional, empty segments
/// are dropped, a segment with no `=` is ignored, and non-UTF-8 bytes yield an
/// empty result (never a panic). Later duplicate keys override earlier ones.
/// Pure — oracle-tested with hand-built byte vectors.
pub fn parse_user_data_entries(user_data: &[u8]) -> Vec<(String, String)> {
    let content = strip_user_data_encapsulation(user_data);
    let Ok(text) = std::str::from_utf8(content) else {
        return Vec::new();
    };
    let mut out: Vec<(String, String)> = Vec::new();
    for segment in text.split(';') {
        let segment = segment.trim();
        if segment.is_empty() {
            continue;
        }
        let Some((key, value)) = segment.split_once('=') else {
            continue; // no '=' — not a key=value entry, ignore
        };
        let key = key.trim().to_string();
        let value = value.trim().to_string();
        if key.is_empty() {
            continue;
        }
        // Last value for a repeated key wins.
        if let Some(existing) = out.iter_mut().find(|(k, _)| *k == key) {
            existing.1 = value;
        } else {
            out.push((key, value));
        }
    }
    out
}

/// Extract the RIHS01 `typehash` value from an endpoint's `USER_DATA`, or `None`
/// when absent/empty (pre-Iron / non-ROS peer). Unknown keys (`enclave=…`) are
/// ignored. The value is returned VERBATIM (trimmed) — no shape validation
/// here; the caller feeds it to the service `type_hash` field and the server
/// validates the hash. Pure — oracle-tested.
pub fn parse_type_hash_from_user_data(user_data: &[u8]) -> Option<String> {
    parse_user_data_entries(user_data)
        .into_iter()
        .find(|(k, _)| k == "typehash")
        .map(|(_, v)| v)
        .filter(|v| !v.is_empty())
}

// ──────────────────────── DDS type-name normalization ──────────────────────

/// Normalize a raw DDS type name to canonical `pkg/Type`, mirroring the
/// engine's `normalize_ros_type` (kept HERE, duplicated, because `cerulion_dds`
/// is the leaf crate and cannot depend on the engine). Handles the CDR-mangled
/// (`sensor_msgs::msg::dds_::PointCloud2_`), slash (`sensor_msgs/msg/…`), and
/// colon (`sensor_msgs::msg::…`) forms; the `msg` infix and the CDR `dds_`
/// segment + trailing `_` are dropped. A form that does not fit is returned
/// trimmed as-is (it simply will not match a requested `pkg/Type`). Pure —
/// oracle-tested against the engine's shapes.
pub fn normalize_dds_type(type_name: &str) -> String {
    let t = type_name.trim();
    let parts: Vec<&str> = t.split(['/', ':']).filter(|s| !s.is_empty()).collect();
    if parts.len() < 2 {
        return t.to_string();
    }
    let pkg = parts[0];
    let mut ty = parts[parts.len() - 1];
    ty = ty.strip_suffix('_').unwrap_or(ty);
    if ty.is_empty() || ty == "dds" {
        return t.to_string();
    }
    format!("{pkg}/{ty}")
}

// ─────────────────────── Vendor → correlation mapping ──────────────────────

/// The DDS-RPC request/reply correlation mapping the wire rung will use, as a
/// DDS-free enum (the live shell maps this to `ros2-client`'s `ServiceMapping`).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum WireMapping {
    /// "Enhanced" DDS-RPC mapping — eProsima Fast DDS (`rmw_fastrtps`, the ROS 2
    /// default rmw).
    Enhanced,
    /// The CycloneDDS-specific mapping (`rmw_cyclonedds`).
    Cyclone,
}

/// RTPS vendor id for eProsima Fast DDS / FastRTPS.
pub const VENDOR_EPROSIMA: [u8; 2] = [0x01, 0x0F];
/// RTPS vendor id for Eclipse Cyclone DDS.
pub const VENDOR_CYCLONE: [u8; 2] = [0x01, 0x10];

/// Pick the correlation-mapping TRY-ORDER for a participant's vendor id: Fast
/// DDS ⇒ Enhanced only, Cyclone ⇒ Cyclone only, and any UNKNOWN vendor ⇒ try
/// Enhanced then Cyclone (the requests are idempotent reads, so trying both is
/// safe). Pure — oracle-tested decision table.
pub fn mapping_order_for_vendor(vendor_id: [u8; 2]) -> Vec<WireMapping> {
    match vendor_id {
        VENDOR_EPROSIMA => vec![WireMapping::Enhanced],
        VENDOR_CYCLONE => vec![WireMapping::Cyclone],
        _ => vec![WireMapping::Enhanced, WireMapping::Cyclone],
    }
}

/// The try-order when the owning participant's vendor is UNKNOWN (not in the
/// participant table) — Enhanced then Cyclone.
pub fn mapping_order_unknown_vendor() -> Vec<WireMapping> {
    vec![WireMapping::Enhanced, WireMapping::Cyclone]
}

// ─────────────────────── Endpoint → owning node join ───────────────────────

/// Indices into `nodes` of every node whose participant hosts `writer_guid`
/// (i.e. `node.participant_prefix == writer_guid[..12]`) — the owning-node
/// candidates for a discovered publisher endpoint. Because `ros2-client` does
/// not expose per-node writer GUIDs, the match is at PARTICIPANT granularity:
/// every node in the endpoint's participant is a candidate (any that holds the
/// type answers its `get_type_description` — the r727 retry set). Deterministic
/// (input order). Pure — oracle-tested.
pub fn nodes_owning_writer(writer_guid: [u8; 16], nodes: &[DiscoveredNode]) -> Vec<usize> {
    let prefix: [u8; 12] = writer_guid[..12].try_into().expect("16 >= 12");
    nodes
        .iter()
        .enumerate()
        .filter(|(_, n)| n.participant_prefix == prefix)
        .map(|(i, _)| i)
        .collect()
}

/// The vendor id of the participant with GUID prefix `prefix`, or `None` if no
/// participant with that prefix was discovered. The prefix-keyed core (a call
/// TARGET node's participant is known only by its prefix —
/// `DiscoveredNode::participant_prefix`); [`participant_vendor`] is the
/// endpoint-GUID convenience over it. Pure — oracle-tested.
pub fn participant_vendor_of_prefix(
    prefix: [u8; 12],
    participants: &[crate::DiscoveredParticipant],
) -> Option<[u8; 2]> {
    participants
        .iter()
        .find(|p| p.guid_prefix == prefix)
        .map(|p| p.vendor_id)
}

/// The vendor id of the participant owning `writer_guid` (matched by the guid's
/// first 12 bytes = the participant GUID prefix), or `None` if no participant
/// with that prefix was discovered. Pure — oracle-tested.
pub fn participant_vendor(
    writer_guid: [u8; 16],
    participants: &[crate::DiscoveredParticipant],
) -> Option<[u8; 2]> {
    let prefix: [u8; 12] = writer_guid[..12].try_into().expect("16 >= 12");
    participant_vendor_of_prefix(prefix, participants)
}

// ───────────────────────────── Call planning ───────────────────────────────

/// One attempt at `~/get_type_description` for a requested type: the owning
/// node's fully-qualified name, the RIHS01 hash to look up, and the ordered
/// correlation mappings to try (first that connects wins).
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct CallAttempt {
    /// The owning node's fully-qualified name (`/talker`) — the service is at
    /// `<node_fqn>/get_type_description`.
    pub node_fqn: String,
    /// The RIHS01 type hash the request looks up (server ignores the name).
    pub type_hash: String,
    /// Correlation mappings to try, in order (vendor-derived; unknown ⇒ both).
    pub mapping_order: Vec<WireMapping>,
}

/// The plan for ONE requested type: the ordered attempts to try (retry the next
/// node holding the same hash on failure).
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct TypeCallPlan {
    /// The requested canonical `pkg/Type`.
    pub requested: String,
    /// Attempts in try-order, deduped by `(node_fqn, type_hash)`.
    pub attempts: Vec<CallAttempt>,
}

/// The distinct non-empty RIHS01 hashes carried by an endpoint iterator, in
/// first-seen order (deterministic). GUID-INDEPENDENT:
/// hash PRESENCE alone decides skip reason (ii) — an endpoint whose
/// backend surfaced a hash but no GUID still counts as "hash seen" (it simply
/// contributes no call target, falling to reason (iii) when nothing else does).
fn distinct_hashes<'a>(endpoints: impl Iterator<Item = &'a DiscoveredEndpoint>) -> Vec<&'a String> {
    let mut out: Vec<&'a String> = Vec::new();
    for e in endpoints {
        if let Some(h) = &e.type_hash {
            if !h.is_empty() && !out.contains(&h) {
                out.push(h);
            }
        }
    }
    out
}

/// Select the hash SOURCE for one requested type — PREFER-WRITER,
/// FALL-BACK-TO-READER: use the PUBLISHER (Writer) endpoints' hashes when any
/// publisher carries one, otherwise fall back to SUBSCRIBER (Reader) endpoints'
/// hashes (a robot-side subscriber — e.g. a `/cmd_vel` reader whose schema the
/// teleop direction needs). On a writer/reader hash SKEW the WRITER set wins by
/// construction (readers are only consulted when NO publisher carried a hash).
/// Returns `(distinct hashes in first-seen order, from_writers)` — the hash
/// SOURCE only; call TARGETS are chosen independently by
/// [`candidate_call_nodes`]. Pure but for a
/// `debug` note on the reader fallback.
fn select_hashes<'a>(matching: &[&'a DiscoveredEndpoint], req: &str) -> (Vec<&'a String>, bool) {
    let writers = distinct_hashes(
        matching
            .iter()
            .copied()
            .filter(|e| e.kind == EndpointKind::Writer),
    );
    if !writers.is_empty() {
        return (writers, true);
    }
    let readers = distinct_hashes(
        matching
            .iter()
            .copied()
            .filter(|e| e.kind == EndpointKind::Reader),
    );
    if !readers.is_empty() {
        tracing::debug!(
            ros_type = %req,
            "wire acquirer: no PUBLISHER carried a type hash for this type; falling back to a \
             SUBSCRIBER endpoint's hash (a robot-side reader, e.g. a /cmd_vel subscriber whose \
             schema the teleop direction needs)"
        );
    }
    (readers, false)
}

/// The hash-skew warn: when the SELECTED hash source for one type
/// carries MORE THAN ONE distinct RIHS01 hash, the graph is version-skewed (two
/// machines publishing the same type after a partial rebuild — a known ROS 2
/// field failure). Selection stays deterministic (every `(node, hash)` pair is
/// planned; attempts sort by `(node_fqn, type_hash)` and the first successful
/// response wins), but the skew must be LOUD: frames from the losing publisher
/// may not match the acquired layout. Returns the warn line naming every hash +
/// the source kind, or `None` when the source agrees. Pure — oracle-tested.
pub fn hash_skew_warning(req: &str, hashes: &[&String], from_writers: bool) -> Option<String> {
    if hashes.len() < 2 {
        return None;
    }
    let listed = hashes
        .iter()
        .map(|h| h.as_str())
        .collect::<Vec<_>>()
        .join(", ");
    let source = if from_writers {
        "PUBLISHERS"
    } else {
        "SUBSCRIBERS (reader-fallback source)"
    };
    Some(format!(
        "wire acquirer: {source} of {req} DISAGREE on the RIHS01 type hash ({listed}) — a \
         version-skewed graph (e.g. a partial rebuild across machines). Every (node, hash) pair \
         is tried in deterministic (node, hash) order and the FIRST successful response wins, so \
         frames from the losing publisher may not match the acquired layout; align the machines \
         to one interface version"
    ))
}

/// Candidate call-TARGET node indices for one requested type: the UNION of
/// [`nodes_owning_writer`] joins over ALL matching endpoints' GUIDs — WRITER
/// endpoints first, then READER endpoints — deduped in that order.
/// The hash SOURCE is writer-preferred
/// ([`select_hashes`]), but the call TARGET set must NOT be gated on the hash
/// source's participant having a node-table entry: the server looks the type up
/// BY HASH, so a reader-side node whose PEI survived is a perfectly good
/// `get_type_description` target when the writer's PEI was lost (the bounded(8)
/// join-burst drop). A hashless endpoint's node is a
/// valid target too — it demonstrably uses the type. Pure — pinned via
/// [`plan_service_calls`] oracles.
fn candidate_call_nodes(matching: &[&DiscoveredEndpoint], nodes: &[DiscoveredNode]) -> Vec<usize> {
    let mut out: Vec<usize> = Vec::new();
    let ordered = matching
        .iter()
        .filter(|e| e.kind == EndpointKind::Writer)
        .chain(matching.iter().filter(|e| e.kind == EndpointKind::Reader));
    for e in ordered {
        if let Some(g) = e.writer_guid {
            for idx in nodes_owning_writer(g, nodes) {
                if !out.contains(&idx) {
                    out.push(idx);
                }
            }
        }
    }
    out
}

/// Plan the wire-rung service calls for a deduped set of requested `pkg/Type`
/// names, given the harvested discovery. Returns the callable plans plus, for
/// every type with NO callable attempt, the loud per-type [`RungSkip`] — with a
/// DISTINCT reason for each of the three not-callable shapes: (i) the type was
/// never seen among the endpoints ([`TYPE_NOT_DISCOVERED_REASON`]); (ii) it was
/// seen but no endpoint carried a hash ([`NO_TYPE_HASH_REASON`] — hash PRESENCE
/// judged GUID-independently); (iii) it has a hash but no owning-node
/// join candidate anywhere among its endpoints ([`NO_OWNING_NODE_REASON`]). The
/// live shell executes the plans and turns a per-type total call failure into a
/// further skip. Pure but for the loud skew warn — oracle-tested.
///
/// For each requested type: its endpoints (by normalized type name) supply the
/// hash SOURCE publisher-first, subscriber-fallback (`select_hashes`; ≥2
/// distinct hashes in the selected source is a version-skew — warned loudly);
/// the call TARGETS are the union of owning nodes joined from ALL
/// of the type's endpoints, writers first then readers
/// (`candidate_call_nodes`); each `(target node, hash)` becomes a
/// [`CallAttempt`] with the mapping order derived from the TARGET node's
/// participant vendor (deduped, deterministic).
pub fn plan_service_calls(
    requested: &[String],
    endpoints: &[DiscoveredEndpoint],
    nodes: &[DiscoveredNode],
    participants: &[crate::DiscoveredParticipant],
) -> (Vec<TypeCallPlan>, Vec<(String, RungSkip)>) {
    let mut plans: Vec<TypeCallPlan> = Vec::new();
    let mut skips: Vec<(String, RungSkip)> = Vec::new();

    for req in requested {
        // (i) The type was never seen on any endpoint in the window.
        let matching: Vec<&DiscoveredEndpoint> = endpoints
            .iter()
            .filter(|e| normalize_dds_type(&e.type_name) == *req)
            .collect();
        if matching.is_empty() {
            skips.push((req.clone(), wire_skip(TYPE_NOT_DISCOVERED_REASON)));
            continue;
        }

        // (ii) Seen, but no endpoint carried a usable hash (publisher-first;
        // GUID-independent).
        let (hashes, from_writers) = select_hashes(&matching, req);
        if hashes.is_empty() {
            skips.push((req.clone(), wire_skip(NO_TYPE_HASH_REASON)));
            continue;
        }
        // A version-skewed source set is loud, never silent.
        if let Some(msg) = hash_skew_warning(req, &hashes, from_writers) {
            tracing::warn!("{msg}");
        }

        // Call TARGETS: union of owning nodes over ALL endpoints of the type,
        // writers first then readers — independent of which
        // endpoints supplied the hashes.
        let targets = candidate_call_nodes(&matching, nodes);

        let mut attempts: Vec<CallAttempt> = Vec::new();
        for &idx in &targets {
            let node_fqn = nodes[idx].fully_qualified_name();
            // Mapping order from the TARGET node's participant vendor (the node
            // answering the service is what the correlation mapping must match
            // — not the hash-source writer's participant).
            let prefix = nodes[idx].participant_prefix;
            let mapping_order = match participant_vendor_of_prefix(prefix, participants) {
                Some(vendor) => mapping_order_for_vendor(vendor),
                None => mapping_order_unknown_vendor(),
            };
            for hash in &hashes {
                // Dedupe by (node, hash) — a node publishing many topics of one
                // type, or same-named nodes resolving to one fqn, is ONE call.
                if attempts
                    .iter()
                    .any(|a| a.node_fqn == node_fqn && a.type_hash == **hash)
                {
                    continue;
                }
                attempts.push(CallAttempt {
                    node_fqn: node_fqn.clone(),
                    type_hash: (*hash).clone(),
                    mapping_order: mapping_order.clone(),
                });
            }
        }

        // (iii) Had a hash but no reachable owning node — loud, distinct reason.
        if attempts.is_empty() {
            skips.push((req.clone(), wire_skip(NO_OWNING_NODE_REASON)));
            continue;
        }
        // Deterministic try-order.
        attempts.sort_by(|a, b| {
            a.node_fqn
                .cmp(&b.node_fqn)
                .then_with(|| a.type_hash.cmp(&b.type_hash))
        });
        plans.push(TypeCallPlan {
            requested: req.clone(),
            attempts,
        });
    }
    (plans, skips)
}

// ─────────────────────── Response → `.msg` closure ─────────────────────────

/// One entry of a `GetTypeDescription` response's `type_sources[]`, reduced to
/// the DDS-free fields the closure mapping needs. The live shell decodes the
/// CDR `TypeSource` into this; the mapping below is pure.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct WireTypeSource {
    /// `PACKAGE/NAMESPACE/TYPENAME` (`sensor_msgs/msg/PointCloud2`) or
    /// `PACKAGE/TYPENAME` — the type this source defines.
    pub type_name: String,
    /// The source encoding (`"msg"`, `"idl"`, `"dynamic"`, `"implicit"`, …).
    /// Only `"msg"` sources become `.msg` closure members.
    pub encoding: String,
    /// The verbatim original source-file contents (comments + whitespace
    /// preserved) — byte-copied into the store.
    pub raw_file_contents: String,
}

/// Split a `TypeSource.type_name` into `(package, TypeName)`. Accepts the ROS
/// `pkg/msg/Type` (3-segment) and the bare `pkg/Type` (2-segment) shapes; the
/// interior `msg`/`srv`/`action`/`idl` namespace segment is dropped. `None` for
/// a name that does not fit. Pure — oracle-tested.
pub fn split_type_source_name(qualified: &str) -> Option<(&str, &str)> {
    let parts: Vec<&str> = qualified.split('/').filter(|s| !s.is_empty()).collect();
    match parts.as_slice() {
        [pkg, ty] => Some((pkg, ty)),
        [pkg, _ns, ty] => Some((pkg, ty)),
        _ => None,
    }
}

/// Turn a service response's `type_sources[]` into an [`AcquiredMsg`] closure:
/// keep every `encoding == "msg"` source (verbatim), map each to its
/// `(package, TypeName)` + text. The REQUESTED `pkg/Type` MUST appear among the
/// msg sources (else the response omitted it — a loud `Err`). `idl`/`dynamic`/
/// `implicit` sources are skipped (the structured `TypeDescription` covers
/// idl-only types, which are not supported; this rung ships the `.msg` closure). Pure —
/// oracle-tested (encoding filter, requested-missing, byte-verbatim, bad name).
pub fn sources_to_closure(
    requested: &str,
    sources: &[WireTypeSource],
) -> Result<Vec<AcquiredMsg>, String> {
    let mut closure: Vec<AcquiredMsg> = Vec::new();
    let mut saw_requested = false;
    for src in sources {
        if src.encoding != "msg" {
            continue;
        }
        let Some((pkg, ty)) = split_type_source_name(&src.type_name) else {
            return Err(format!(
                "the get_type_description response carried a type source with an unparseable \
                 type name {:?} (expected pkg/Type or pkg/msg/Type)",
                src.type_name
            ));
        };
        let qualified = format!("{pkg}/{ty}");
        if qualified == requested {
            saw_requested = true;
        }
        closure.push(AcquiredMsg {
            package: pkg.to_string(),
            type_name: ty.to_string(),
            msg_text: src.raw_file_contents.clone(),
        });
    }
    if closure.is_empty() {
        return Err(format!(
            "the get_type_description response for {requested} carried no msg-encoded type \
             sources (only idl/dynamic/implicit) — not bridged"
        ));
    }
    if !saw_requested {
        return Err(format!(
            "the get_type_description response omitted the requested type's .msg source \
             ({requested} was not among the returned msg sources)"
        ));
    }
    Ok(closure)
}

// ─────────────────── Call-phase orchestration (pure) ──────────────────────
//
// The live shell ([`crate::wire_acquirer`]) drives the DDS I/O; these pure
// helpers carry the per-attempt DECISIONS ON THE PRODUCTION CALL PATH (so they
// are oracle-tested here, not left to live-network tests): the successful=false
// classification, the wait-vs-call attempt classification feeding the per-node
// `wait_for_service` failure cache, the last-error retention (real answers
// beat bookkeeping) + retry fold, the per-participant node-table-miss warn
// predicate + its build-aware wording, and the overall call-phase wall-cap
// predicate.

/// One service-call attempt's DDS-free result, produced by the live shell's
/// `call_one` and folded by [`apply_attempt_outcome`].
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum AttemptResult {
    /// The call returned these `type_sources` — the type is resolved.
    Resolved(Vec<WireTypeSource>),
    /// The node was reachable (a server answered on the service topics) but the
    /// call did not yield sources: a client-create error, a transport error, or
    /// `successful == false`. Retry the next node holding the same hash.
    CallFailed(String),
    /// `wait_for_service` timed out for every correlation mapping — no
    /// `get_type_description` server for this node. The node is cached so a
    /// LATER type's plan is not re-waited against the same dead node.
    WaitTimedOut(String),
}

/// Fold one attempt's [`AttemptResult`] into the plan-resolution state on the
/// production call path: on `Resolved` return the sources (the caller stops
/// retrying); on `CallFailed` retain the reason and return `None` (try the next
/// attempt); on `WaitTimedOut` ADD `node_fqn` to the per-attach wait-failure
/// cache AND retain the reason. Pure — oracle-tested.
pub fn apply_attempt_outcome(
    outcome: AttemptResult,
    node_fqn: &str,
    wait_failed: &mut BTreeSet<String>,
    last_reason: &mut Option<String>,
) -> Option<Vec<WireTypeSource>> {
    match outcome {
        AttemptResult::Resolved(sources) => Some(sources),
        AttemptResult::CallFailed(reason) => {
            *last_reason = Some(reason);
            None
        }
        AttemptResult::WaitTimedOut(reason) => {
            wait_failed.insert(node_fqn.to_string());
            *last_reason = Some(reason);
            None
        }
    }
}

/// Whether an attempt against `node_fqn` must be SKIPPED because that node
/// already failed `wait_for_service` earlier in this attach — returns the loud
/// skip reason to retain, or `None` when the node is not cached. Keyed by the
/// node's `~/get_type_description` service FQN. Pure — oracle-tested.
pub fn wait_cache_skip_reason(wait_failed: &BTreeSet<String>, node_fqn: &str) -> Option<String> {
    if wait_failed.contains(node_fqn) {
        Some(format!(
            "node {node_fqn} already failed wait_for_service earlier in this attach (no \
             get_type_description server appeared) — not re-waited"
        ))
    } else {
        None
    }
}

/// Classify a `get_type_description` response: `Ok(())` when the server reported
/// success, else `Err(reason)` naming the node + the server's `failure_reason`.
/// The `!successful` arm is the "inverted-successful" guard the live shell uses
/// to move to the next candidate node. Pure — oracle-tested.
pub fn classify_call_response(
    successful: bool,
    failure_reason: &str,
    node_fqn: &str,
) -> Result<(), String> {
    if successful {
        Ok(())
    } else {
        Err(format!(
            "{node_fqn}/get_type_description reported failure: {failure_reason}"
        ))
    }
}

/// Whether the wire rung's overall call-phase wall cap has elapsed (checked
/// between plans + attempts so a LAN full of dead/silent nodes cannot make the
/// rung run unbounded, on top of the per-call budgets). Pure — the `Instant`
/// pair keeps it oracle-testable without real time (mirrors
/// `discovery::collect_deadline_reached`).
pub fn call_phase_budget_exceeded(now: Instant, deadline: Instant) -> bool {
    now >= deadline
}

/// The per-attempt failure classification feeding
/// the wait-failure cache, extracted from the live shell's `call_one` tail so it
/// is oracle-testable (the module contract: every cache decision lives in
/// `wire` and is oracle-tested). A node is
/// classified [`AttemptResult::WaitTimedOut`] — and thus CACHED as serverless
/// for the rest of the attach — ONLY when at least one mapping's
/// `wait_for_service` timed out AND no mapping ever connected. Every other
/// failure shape is [`AttemptResult::CallFailed`] (transient — never poisons
/// the cache): a server that answered the wait but failed the call, a mixed
/// wait-timeout-then-wait-ok across mappings, or a pure client-create failure
/// (neither flag set). Pure — 4-cell truth table oracle-tested.
pub fn classify_attempt_failure(
    any_wait_ok: bool,
    any_wait_timeout: bool,
    reason: String,
) -> AttemptResult {
    if any_wait_timeout && !any_wait_ok {
        AttemptResult::WaitTimedOut(reason)
    } else {
        AttemptResult::CallFailed(reason)
    }
}

/// Fill `last_reason` with a BOOKKEEPING reason
/// (wall-cap elapsed, wait-cache skip, response-cache negative) ONLY when no
/// better reason exists yet. A real attempt outcome — especially a live
/// server's `successful == false` diagnosis ("Type not currently in use by
/// this node"), the single most diagnostic datum in the attach (it proves the
/// service works and the hash resolved) — must NEVER be overwritten by
/// budget/cache bookkeeping. Real outcomes keep overwriting each other via
/// [`apply_attempt_outcome`] (the latest real result wins); bookkeeping only
/// fills silence. Pure — oracle-tested.
pub fn retain_bookkeeping_reason(last_reason: &mut Option<String>, bookkeeping: String) {
    if last_reason.is_none() {
        *last_reason = Some(bookkeeping);
    }
}

/// Whether to emit the node-table-LOSS warn (a PER-PARTICIPANT predicate,
/// not `nodes.is_empty()`): true iff at least one
/// discovered endpoint carries a non-empty RIHS01 hash AND a GUID whose
/// participant prefix has NO `ros_discovery_info` node-table entry — a callable
/// hash the wire rung cannot route. PER-PARTICIPANT, not global-empty, because
/// (a) the bounded(8) status channel loses the TAIL of a join burst while the
/// draining consumer keeps the head, so the common loss shape is a PARTIAL
/// table (a globally-empty one is the rare case), and (b) any surviving entry —
/// including our own node's looped-back PEI before the discovery-side own-node
/// filter — would mask an emptiness check while the one PEI that mattered was
/// dropped. A hash-bearing endpoint with NO GUID does not fire this (it has no
/// participant to miss; reason (iii) covers it). Pure — oracle-tested.
pub fn should_warn_node_table_miss(
    endpoints: &[DiscoveredEndpoint],
    nodes: &[DiscoveredNode],
) -> bool {
    endpoints.iter().any(|e| {
        if e.type_hash.as_deref().is_none_or(|h| h.is_empty()) {
            return false;
        }
        let Some(g) = e.writer_guid else {
            return false;
        };
        let prefix: [u8; 12] = g[..12].try_into().expect("16 >= 12");
        !nodes.iter().any(|n| n.participant_prefix == prefix)
    })
}

/// The node-table-loss warn line: names BOTH
/// candidate causes — the bounded(8) status-channel drop on the node/PEI path,
/// AND a GID-width mismatch between this build and the announcing peers
/// (`ros_discovery_info` is width-encoded: 16-byte gids from Iron on, 24-byte
/// before; a build on the other width cannot decode the PEI at all) — instead
/// of a channel-drop-only diagnosis whose "re-run" advice could never
/// help a width-mismatched build. `build_is_pre_iron_gid` is the COMPILED width
/// of the calling build (the live shell passes
/// `COMPILED_ROS_DISTRO < RosDistro::Iron`), which decides which cause is
/// structural: hash-bearing peers are by definition Iron+ (16-byte), so a
/// pre-Iron (24-byte `humble`) build can NEVER decode their PEIs — re-running
/// cannot help there — while an Iron+ build shares their width, leaving the
/// channel drop / window timing as the live causes. Pure — wording pinned by
/// hand-pasted literals in the module tests.
pub fn node_table_miss_warning(build_is_pre_iron_gid: bool) -> String {
    if build_is_pre_iron_gid {
        "wire acquirer: discovered endpoints carry RIHS01 type hashes but their participants \
         have NO ros_discovery_info node-table entry — those publishers cannot be mapped to \
         their get_type_description service. This build is the PRE-IRON (24-byte-GID) `humble` \
         opt-in and structurally CANNOT decode Iron+ peers' 16-byte ros_discovery_info — for \
         hash-bearing (Iron+) publishers that is the certain cause, and re-running cannot help. \
         Rebuild cerulion_dds with the default `jazzy` feature to attach to Iron+ robots. (The \
         bounded(8) status-channel drop is the other cause class, but the GID width blocks \
         first here.)"
            .to_string()
    } else {
        "wire acquirer: discovered endpoints carry RIHS01 type hashes but their participants \
         have NO ros_discovery_info node-table entry — those publishers cannot be mapped to \
         their get_type_description service. Two cause classes: (a) the DDS status channel \
         (async_channel::bounded(8), try_send DROP) dropped part of the node-announcement \
         burst, or the announcement landed after the discovery window — re-run `cerulion ros2 \
         attach` (a fresh window often catches it) or lengthen --timeout; (b) a GID-width \
         mismatch between builds — not in play here, since hash-bearing peers are Iron+ and \
         share this build's 16-byte ros_discovery_info encoding."
            .to_string()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::DiscoveredParticipant;
    use crate::{DiscoveredQos, EndpointKind, QosDurability, QosReliability};
    use tracing_test::traced_test;

    // ─────────────────────────── join_node_fqn ─────────────────────────────

    #[test]
    fn test_join_node_fqn_shapes() {
        assert_eq!(join_node_fqn("/", "talker"), "/talker");
        assert_eq!(join_node_fqn("", "talker"), "/talker");
        assert_eq!(join_node_fqn("/robot", "talker"), "/robot/talker");
        assert_eq!(join_node_fqn("/a/b", "talker"), "/a/b/talker");
        // Trailing slash normalized.
        assert_eq!(join_node_fqn("/robot/", "talker"), "/robot/talker");
        // Namespace without a leading slash still yields an absolute name.
        assert_eq!(join_node_fqn("robot", "talker"), "/robot/talker");
    }

    // ───────────────────── USER_DATA parse oracles ─────────────────────────

    #[test]
    fn test_user_data_typehash_present() {
        let ud = b"typehash=RIHS01_0123abcd;";
        assert_eq!(
            parse_type_hash_from_user_data(ud),
            Some("RIHS01_0123abcd".to_string())
        );
    }

    #[test]
    fn test_user_data_typehash_absent() {
        // Only an enclave key — no typehash.
        let ud = b"enclave=/;";
        assert_eq!(parse_type_hash_from_user_data(ud), None);
        // Completely empty.
        assert_eq!(parse_type_hash_from_user_data(b""), None);
    }

    #[test]
    fn test_user_data_multiple_keys_and_no_trailing_semicolon() {
        // Multiple keys, typehash in the middle, and NO trailing ';'.
        let ud = b"enclave=/robot;typehash=RIHS01_ffff;securitykey=abc";
        assert_eq!(
            parse_type_hash_from_user_data(ud),
            Some("RIHS01_ffff".to_string())
        );
        // Full entry parse: order + values preserved, unknown keys kept.
        assert_eq!(
            parse_user_data_entries(ud),
            vec![
                ("enclave".to_string(), "/robot".to_string()),
                ("typehash".to_string(), "RIHS01_ffff".to_string()),
                ("securitykey".to_string(), "abc".to_string()),
            ]
        );
    }

    #[test]
    fn test_user_data_malformed_entries_ignored() {
        // A segment with no '=' is ignored; an empty typehash value → None.
        assert_eq!(parse_type_hash_from_user_data(b"typehash;enclave=/"), None);
        assert_eq!(parse_type_hash_from_user_data(b"typehash=;"), None);
        // Leading/empty segments dropped, still finds the hash.
        assert_eq!(
            parse_type_hash_from_user_data(b";;typehash=RIHS01_ab;;"),
            Some("RIHS01_ab".to_string())
        );
        // Non-UTF-8 → empty (never a panic).
        assert!(parse_user_data_entries(&[0xff, 0xfe, 0x00]).is_empty());
    }

    #[test]
    fn test_user_data_duplicate_key_last_wins() {
        let ud = b"typehash=RIHS01_first;typehash=RIHS01_second;";
        assert_eq!(
            parse_type_hash_from_user_data(ud),
            Some("RIHS01_second".to_string())
        );
    }

    // ─────────── USER_DATA CDR-encapsulation strip (the live fix) ───────────

    /// A 64-hex-char RIHS01 digest (the REP-2011 hash body length) — fixed,
    /// hand-typed, so every encapsulation oracle below is byte-deterministic.
    const HEX64: &str = "0123456789abcdef0123456789abcdef0123456789abcdef0123456789abcdef";

    /// Hand-build the CDR-encapsulated USER_DATA blob rustdds hands us:
    /// `[u32 length prefix][content][padding]`.
    fn encapsulated(prefix: [u8; 4], content: &[u8], padding: &[u8]) -> Vec<u8> {
        let mut ud = prefix.to_vec();
        ud.extend_from_slice(content);
        ud.extend_from_slice(padding);
        ud
    }

    /// (a) THE live-captured shape (Jazzy container, FastDDS
    /// writer via a throwaway probe): 88 bytes = LE length 0x51 (= 81), the
    /// 81-byte ASCII `typehash=RIHS01_<64 hex>;` (9 + 7 + 64 + 1), 3 padding
    /// bytes. The observed capture began
    /// `[51 00 00 00, 74 79 70 65 68 61 73 68 3d 52 49 48 53 30 31 5f ...]` —
    /// the first 20 hand-built bytes are asserted to MATCH that observation,
    /// so this oracle is provably the live shape, not a self-invented one.
    /// A naked-string parse returns None here (the 4 prefix bytes corrupt the
    /// first key) — the "no type hash in discovery" false-skip on every live type.
    #[test]
    fn test_user_data_cdr_encapsulated_le_live_shape_parses() {
        let content = format!("typehash=RIHS01_{HEX64};");
        assert_eq!(content.len(), 0x51, "the live capture's content length");
        let ud = encapsulated([0x51, 0, 0, 0], content.as_bytes(), &[0, 0, 0]);
        assert_eq!(ud.len(), 88, "the live capture's total length");
        // Provenance cross-check: first 20 bytes == the observed capture.
        assert_eq!(
            &ud[..20],
            &[
                0x51, 0x00, 0x00, 0x00, // LE length prefix
                0x74, 0x79, 0x70, 0x65, 0x68, 0x61, 0x73, 0x68, // "typehash"
                0x3d, 0x52, 0x49, 0x48, 0x53, 0x30, 0x31, 0x5f, // "=RIHS01_"
            ]
        );
        assert_eq!(
            parse_type_hash_from_user_data(&ud),
            Some(format!("RIHS01_{HEX64}"))
        );
    }

    /// (b) The BE twin (`[00 00 00 51]` prefix — a BE-encapsulated SEDP peer):
    /// the LE read of that head (0x51000000) fails the padding tolerance, the
    /// BE read (0x51) passes, and the hash parses identically.
    #[test]
    fn test_user_data_cdr_encapsulated_be_parses() {
        let content = format!("typehash=RIHS01_{HEX64};");
        let ud = encapsulated([0, 0, 0, 0x51], content.as_bytes(), &[0, 0, 0]);
        assert_eq!(
            parse_type_hash_from_user_data(&ud),
            Some(format!("RIHS01_{HEX64}"))
        );
    }

    /// (c) Naked (un-encapsulated) content still parses — the back-compat pin
    /// for hermetic callers / an rmw handing naked strings. A naked string's
    /// first 4 ASCII bytes read as a ~2-billion u32 in either endianness, so
    /// the strip's fall-through is taken (the existing naked-input oracles
    /// above ride the same arm).
    #[test]
    fn test_user_data_naked_content_still_parses() {
        let ud = format!("typehash=RIHS01_{HEX64};");
        assert_eq!(
            parse_type_hash_from_user_data(ud.as_bytes()),
            Some(format!("RIHS01_{HEX64}"))
        );
    }

    /// (d) A prefix-vs-length mismatch BEYOND the 3-byte padding tolerance is
    /// NOT stripped — the blob is treated as naked, and since a garbage
    /// binary head makes no `typehash=` key, the parse yields None (never a
    /// mis-slice). Prefix 0x50 = 80 against an 84-byte body is off by 4 — the
    /// sharpest just-beyond-tolerance boundary.
    #[test]
    fn test_user_data_prefix_beyond_padding_tolerance_treated_naked() {
        let content = format!("typehash=RIHS01_{HEX64};");
        let ud = encapsulated([0x50, 0, 0, 0], content.as_bytes(), &[0, 0, 0]);
        assert_eq!(
            parse_type_hash_from_user_data(&ud),
            None,
            "an implausible prefix must fall through to naked (and parse as garbage)"
        );
        // Within-tolerance control: padding 0 (exact length) strips fine.
        let exact = b"typehash=x;";
        let ud = encapsulated([exact.len() as u8, 0, 0, 0], exact, &[]);
        assert_eq!(parse_type_hash_from_user_data(&ud), Some("x".to_string()));
    }

    /// (e) Empty and shorter-than-a-prefix slices: no panic, no hash.
    #[test]
    fn test_user_data_short_slices_are_safe() {
        assert_eq!(parse_type_hash_from_user_data(b""), None);
        assert_eq!(parse_type_hash_from_user_data(&[0x51]), None);
        assert_eq!(parse_type_hash_from_user_data(&[0x51, 0]), None);
        assert_eq!(parse_type_hash_from_user_data(&[0x51, 0, 0]), None);
        // The all-zero 4-byte blob: prefix 0, body 0 — strips to EMPTY content.
        assert_eq!(parse_type_hash_from_user_data(&[0, 0, 0, 0]), None);
    }

    /// (f) Participant-style multi-entry content INSIDE an encapsulation
    /// (`enclave=` first, then `typehash=`): the strip exposes the full entry
    /// list and the leading non-typehash pair does not derail the split.
    #[test]
    fn test_user_data_cdr_encapsulated_multi_entry_parses() {
        let content = b"enclave=/robot;typehash=RIHS01_ab;"; // 34 bytes
        assert_eq!(content.len(), 34);
        // 34 → padded to 36: 2 padding bytes.
        let ud = encapsulated([34, 0, 0, 0], content, &[0, 0]);
        assert_eq!(
            parse_type_hash_from_user_data(&ud),
            Some("RIHS01_ab".to_string())
        );
        assert_eq!(
            parse_user_data_entries(&ud),
            vec![
                ("enclave".to_string(), "/robot".to_string()),
                ("typehash".to_string(), "RIHS01_ab".to_string()),
            ]
        );
    }

    // ─────────────────────── normalize_dds_type ────────────────────────────

    #[test]
    fn test_normalize_dds_type_all_forms() {
        assert_eq!(
            normalize_dds_type("sensor_msgs::msg::dds_::PointCloud2_"),
            "sensor_msgs/PointCloud2"
        );
        assert_eq!(
            normalize_dds_type("sensor_msgs/msg/PointCloud2"),
            "sensor_msgs/PointCloud2"
        );
        assert_eq!(
            normalize_dds_type("sensor_msgs::msg::PointCloud2"),
            "sensor_msgs/PointCloud2"
        );
        assert_eq!(
            normalize_dds_type("unitree_go::msg::dds_::SportModeState_"),
            "unitree_go/SportModeState"
        );
        // A form that does not fit is returned trimmed.
        assert_eq!(normalize_dds_type("Foo"), "Foo");
    }

    // ─────────────────── vendor → mapping decision table ───────────────────

    #[test]
    fn test_mapping_order_decision_table() {
        assert_eq!(
            mapping_order_for_vendor(VENDOR_EPROSIMA),
            vec![WireMapping::Enhanced]
        );
        assert_eq!(
            mapping_order_for_vendor(VENDOR_CYCLONE),
            vec![WireMapping::Cyclone]
        );
        // Unknown vendor → try both, Enhanced first.
        assert_eq!(
            mapping_order_for_vendor([0x01, 0x01]), // RTI Connext
            vec![WireMapping::Enhanced, WireMapping::Cyclone]
        );
        assert_eq!(
            mapping_order_for_vendor([0x00, 0x00]), // unknown
            vec![WireMapping::Enhanced, WireMapping::Cyclone]
        );
        assert_eq!(
            mapping_order_unknown_vendor(),
            vec![WireMapping::Enhanced, WireMapping::Cyclone]
        );
    }

    // ──────────────────────── guid → node join ─────────────────────────────

    /// Build a node under participant prefix `pfx` (a fill byte).
    fn node(ns: &str, name: &str, pfx: u8) -> DiscoveredNode {
        DiscoveredNode {
            namespace: ns.to_string(),
            name: name.to_string(),
            participant_prefix: [pfx; 12],
        }
    }

    /// A 16-byte GUID under participant prefix `pfx` with entity tail `tail`.
    fn guid(pfx: u8, tail: u8) -> [u8; 16] {
        let mut g = [pfx; 16];
        g[12..].copy_from_slice(&[tail; 4]);
        g
    }

    #[test]
    fn test_nodes_owning_writer_join() {
        // alpha + gamma share participant prefix 0x0a; beta is on 0x0b.
        let nodes = vec![
            node("/", "alpha", 0x0a),
            node("/robot", "beta", 0x0b),
            node("/", "gamma", 0x0a),
        ];
        // A writer on participant 0x0b → only beta (index 1).
        assert_eq!(nodes_owning_writer(guid(0x0b, 1), &nodes), vec![1]);
        // A writer on participant 0x0a → both co-hosted nodes (0, 2), in order.
        assert_eq!(nodes_owning_writer(guid(0x0a, 7), &nodes), vec![0, 2]);
        // Different entity tail, same participant → still both.
        assert_eq!(nodes_owning_writer(guid(0x0a, 9), &nodes), vec![0, 2]);
        // Unknown participant → none.
        assert!(nodes_owning_writer(guid(0x0c, 1), &nodes).is_empty());
    }

    #[test]
    fn test_participant_vendor_by_prefix() {
        let mut g = [0u8; 16];
        g[..12].copy_from_slice(&[0xaa; 12]);
        let parts = vec![
            DiscoveredParticipant {
                guid_prefix: [0xaa; 12],
                vendor_id: VENDOR_CYCLONE,
            },
            DiscoveredParticipant {
                guid_prefix: [0xbb; 12],
                vendor_id: VENDOR_EPROSIMA,
            },
        ];
        assert_eq!(participant_vendor(g, &parts), Some(VENDOR_CYCLONE));
        // A GUID whose prefix is unknown → None (caller tries both mappings).
        assert_eq!(participant_vendor([0xcc; 16], &parts), None);
    }

    // ──────────────────────── split_type_source_name ──────────────────────

    #[test]
    fn test_split_type_source_name() {
        assert_eq!(
            split_type_source_name("sensor_msgs/msg/PointCloud2"),
            Some(("sensor_msgs", "PointCloud2"))
        );
        assert_eq!(
            split_type_source_name("acme_msgs/Widget"),
            Some(("acme_msgs", "Widget"))
        );
        assert_eq!(split_type_source_name("Widget"), None);
        assert_eq!(split_type_source_name(""), None);
        assert_eq!(split_type_source_name("a/b/c/d"), None);
    }

    // ───────────────────── sources_to_closure oracles ─────────────────────

    fn src(name: &str, encoding: &str, text: &str) -> WireTypeSource {
        WireTypeSource {
            type_name: name.to_string(),
            encoding: encoding.to_string(),
            raw_file_contents: text.to_string(),
        }
    }

    #[test]
    fn test_sources_to_closure_happy_encoding_filter_and_byte_verbatim() {
        // A .msg with comments + a constant — must survive byte-verbatim.
        const WIDGET: &str = "# a widget\nuint8 KIND_A=1\nint32 id\nacme/Vec2 where\n";
        const VEC2: &str = "float64 x\nfloat64 y\n";
        let sources = vec![
            src("acme_msgs/msg/Widget", "msg", WIDGET),
            // An idl twin of the same type — must be SKIPPED (not a .msg).
            src("acme_msgs/msg/Widget", "idl", "module acme_msgs { ... };"),
            src("acme/msg/Vec2", "msg", VEC2),
            // A dynamic/implicit source — skipped.
            src("builtin_interfaces/msg/Time", "implicit", ""),
        ];
        let closure = sources_to_closure("acme_msgs/Widget", &sources).expect("closure");
        // Only the two msg sources, byte-verbatim (comments + constant kept).
        assert_eq!(closure.len(), 2);
        assert_eq!(closure[0].qualified_name(), "acme_msgs/Widget");
        assert_eq!(closure[0].msg_text, WIDGET);
        assert_eq!(closure[1].qualified_name(), "acme/Vec2");
        assert_eq!(closure[1].msg_text, VEC2);
    }

    #[test]
    fn test_sources_to_closure_requested_missing_is_error() {
        // The response returned only a DEP, not the requested type itself.
        let sources = vec![src("acme/msg/Vec2", "msg", "float64 x\n")];
        let err = sources_to_closure("acme_msgs/Widget", &sources).unwrap_err();
        assert!(err.contains("omitted the requested type"), "{err}");
        assert!(err.contains("acme_msgs/Widget"), "{err}");
    }

    #[test]
    fn test_sources_to_closure_no_msg_sources_is_error() {
        // idl-only response → error (this rung ships .msg text).
        let sources = vec![src("acme_msgs/msg/Widget", "idl", "module ...")];
        let err = sources_to_closure("acme_msgs/Widget", &sources).unwrap_err();
        assert!(err.contains("no msg-encoded type sources"), "{err}");
    }

    // ──────────────────────── plan_service_calls ──────────────────────────

    fn ep_kind(
        topic: &str,
        ty: &str,
        hash: Option<&str>,
        guid: Option<[u8; 16]>,
        kind: EndpointKind,
    ) -> DiscoveredEndpoint {
        DiscoveredEndpoint {
            dds_topic: topic.to_string(),
            type_name: ty.to_string(),
            qos: DiscoveredQos {
                reliability: QosReliability::Reliable,
                durability: QosDurability::Volatile,
            },
            kind,
            type_hash: hash.map(str::to_string),
            writer_guid: guid,
        }
    }

    fn ep(topic: &str, ty: &str, hash: Option<&str>, guid: Option<[u8; 16]>) -> DiscoveredEndpoint {
        ep_kind(topic, ty, hash, guid, EndpointKind::Writer)
    }

    #[test]
    fn test_plan_no_hash_skips_with_pinned_wording() {
        // Endpoint publishing the type but WITHOUT a hash (pre-Iron / non-ROS).
        let eps = vec![ep(
            "rt/widget",
            "acme_msgs::msg::dds_::Widget_",
            None,
            Some([1u8; 16]),
        )];
        let (plans, skips) = plan_service_calls(&["acme_msgs/Widget".to_string()], &eps, &[], &[]);
        assert!(plans.is_empty());
        assert_eq!(skips.len(), 1);
        assert_eq!(skips[0].0, "acme_msgs/Widget");
        assert_eq!(skips[0].1.reason, NO_TYPE_HASH_REASON);
        assert_eq!(skips[0].1.rung, AcquisitionRung::WireService);
    }

    #[test]
    fn test_plan_hash_but_no_owning_node_skips_distinctly() {
        let g = [3u8; 16];
        let eps = vec![ep(
            "rt/widget",
            "acme_msgs::msg::dds_::Widget_",
            Some("RIHS01_ab"),
            Some(g),
        )];
        // No node table → the writer maps to nothing.
        let (plans, skips) = plan_service_calls(&["acme_msgs/Widget".to_string()], &eps, &[], &[]);
        assert!(plans.is_empty());
        assert_eq!(skips.len(), 1);
        assert_eq!(skips[0].1.reason, NO_OWNING_NODE_REASON);
    }

    #[test]
    fn test_plan_happy_single_node_uses_vendor_mapping() {
        let g = guid(0x0a, 1);
        let eps = vec![ep(
            "rt/widget",
            "acme_msgs::msg::dds_::Widget_",
            Some("RIHS01_ab"),
            Some(g),
        )];
        let nodes = vec![node("/", "widget_pub", 0x0a)];
        let parts = vec![DiscoveredParticipant {
            guid_prefix: [0x0a; 12],
            vendor_id: VENDOR_CYCLONE, // Cyclone → Cyclone-only mapping
        }];
        let (plans, skips) =
            plan_service_calls(&["acme_msgs/Widget".to_string()], &eps, &nodes, &parts);
        assert!(skips.is_empty());
        assert_eq!(plans.len(), 1);
        assert_eq!(plans[0].requested, "acme_msgs/Widget");
        assert_eq!(plans[0].attempts.len(), 1);
        let a = &plans[0].attempts[0];
        assert_eq!(a.node_fqn, "/widget_pub");
        assert_eq!(a.type_hash, "RIHS01_ab");
        assert_eq!(a.mapping_order, vec![WireMapping::Cyclone]);
    }

    #[test]
    fn test_plan_unknown_vendor_tries_both_mappings() {
        let g = guid(5, 1);
        let eps = vec![ep(
            "rt/w",
            "acme_msgs::msg::dds_::Widget_",
            Some("RIHS01_ab"),
            Some(g),
        )];
        let nodes = vec![node("/", "pub", 5)];
        // Empty participant table → vendor unknown → try both.
        let (plans, _skips) =
            plan_service_calls(&["acme_msgs/Widget".to_string()], &eps, &nodes, &[]);
        assert_eq!(
            plans[0].attempts[0].mapping_order,
            vec![WireMapping::Enhanced, WireMapping::Cyclone]
        );
    }

    #[test]
    fn test_plan_dedup_and_retry_ordering() {
        // Two participants hold the SAME hash → two attempts (retry). A second
        // endpoint on the SAME participant collapses to one attempt.
        let g1a = guid(1, 1);
        let g1b = guid(1, 2); // same participant (prefix 1), different entity
        let g2 = guid(2, 1);
        let eps = vec![
            ep(
                "rt/a",
                "acme_msgs::msg::dds_::Widget_",
                Some("RIHS01_ab"),
                Some(g1a),
            ),
            // Second endpoint, same participant prefix 1 → deduped to one node.
            ep(
                "rt/a2",
                "acme_msgs::msg::dds_::Widget_",
                Some("RIHS01_ab"),
                Some(g1b),
            ),
            ep(
                "rt/b",
                "acme_msgs::msg::dds_::Widget_",
                Some("RIHS01_ab"),
                Some(g2),
            ),
        ];
        let nodes = vec![node("/", "zeta", 1), node("/", "alpha", 2)];
        let (plans, _skips) =
            plan_service_calls(&["acme_msgs/Widget".to_string()], &eps, &nodes, &[]);
        assert_eq!(plans.len(), 1);
        // Two attempts (one per node), deduped, sorted by node_fqn: alpha, zeta.
        let fqns: Vec<&str> = plans[0]
            .attempts
            .iter()
            .map(|a| a.node_fqn.as_str())
            .collect();
        assert_eq!(fqns, vec!["/alpha", "/zeta"]);
    }

    /// Reason (i) — a requested type that is NOT among ANY
    /// discovered endpoint gets the DISTINCT `TYPE_NOT_DISCOVERED_REASON`, not
    /// the conflated no-hash reason. Endpoint set carries a DIFFERENT type.
    #[test]
    fn test_plan_type_never_discovered_skips_distinctly() {
        let eps = vec![ep(
            "rt/other",
            "other_msgs::msg::dds_::Gadget_",
            Some("RIHS01_ab"),
            Some([1u8; 16]),
        )];
        let (plans, skips) = plan_service_calls(&["acme_msgs/Widget".to_string()], &eps, &[], &[]);
        assert!(plans.is_empty());
        assert_eq!(skips.len(), 1);
        assert_eq!(skips[0].0, "acme_msgs/Widget");
        assert_eq!(skips[0].1.reason, TYPE_NOT_DISCOVERED_REASON);
        // And it is DISTINCT from the no-hash reason (one shared reason would conflate the two).
        assert_ne!(skips[0].1.reason, NO_TYPE_HASH_REASON);
    }

    /// The three skip reasons are pinned against
    /// HAND-PASTED literals (not the same const production uses) so a wording
    /// drift fails here, and the three are provably distinct.
    #[test]
    fn test_skip_reason_wording_literals() {
        assert_eq!(
            TYPE_NOT_DISCOVERED_REASON,
            "this type was not seen on any endpoint in the discovery window — nothing on the \
             robot advertised it during discovery (a topic that stopped publishing, or too short \
             a window) — the wire rung has no endpoint to trace to a get_type_description-serving \
             node"
        );
        assert_eq!(
            NO_TYPE_HASH_REASON,
            "no type hash in discovery (pre-Iron distro or non-ROS DDS publisher)"
        );
        assert_eq!(
            NO_OWNING_NODE_REASON,
            "the endpoint advertises a type hash but could not be mapped to an owning ROS node \
             via ros_discovery_info (no ParticipantEntitiesInfo for its writer GUID) — cannot \
             locate a get_type_description service"
        );
        // All three distinct.
        assert_ne!(TYPE_NOT_DISCOVERED_REASON, NO_TYPE_HASH_REASON);
        assert_ne!(NO_TYPE_HASH_REASON, NO_OWNING_NODE_REASON);
        assert_ne!(TYPE_NOT_DISCOVERED_REASON, NO_OWNING_NODE_REASON);
    }

    // ──────── endpoint hash policy (prefer-writer source; union targets) ────

    /// A type with BOTH a hashed
    /// publisher (writer H1) and a hashed subscriber (reader H2) uses the
    /// WRITER hash — the reader's HASH is never consulted — while the reader's
    /// NODE joins the call-TARGET union as a secondary attempt carrying the
    /// writer hash (targets are hash-source-independent).
    #[test]
    fn test_plan_prefers_writer_hash_over_reader_on_skew() {
        let gw = guid(0x0a, 1);
        let gr = guid(0x0b, 1);
        let eps = vec![
            ep_kind(
                "rt/w",
                "acme_msgs::msg::dds_::Widget_",
                Some("RIHS01_writer"),
                Some(gw),
                EndpointKind::Writer,
            ),
            ep_kind(
                "rt/w",
                "acme_msgs::msg::dds_::Widget_",
                Some("RIHS01_reader"),
                Some(gr),
                EndpointKind::Reader,
            ),
        ];
        let nodes = vec![node("/", "pub", 0x0a), node("/", "sub", 0x0b)];
        let (plans, skips) =
            plan_service_calls(&["acme_msgs/Widget".to_string()], &eps, &nodes, &[]);
        assert!(skips.is_empty());
        assert_eq!(plans.len(), 1);
        // BOTH nodes are targets (writer's first before the sort; deterministic
        // (node, hash) order after) — but EVERY attempt carries the WRITER
        // hash; "RIHS01_reader" appears nowhere.
        let pairs: Vec<(&str, &str)> = plans[0]
            .attempts
            .iter()
            .map(|a| (a.node_fqn.as_str(), a.type_hash.as_str()))
            .collect();
        assert_eq!(
            pairs,
            vec![("/pub", "RIHS01_writer"), ("/sub", "RIHS01_writer")]
        );
    }

    /// When the type has ONLY reader (subscriber) endpoints
    /// carrying a hash — a robot-side `/cmd_vel` subscriber, no publisher on the
    /// wire — the rung FALLS BACK to the reader's hash (the teleop direction).
    #[test]
    fn test_plan_falls_back_to_reader_hash_when_no_writer() {
        let gr = guid(0x0c, 1);
        let eps = vec![ep_kind(
            "rt/cmd_vel",
            "geometry_msgs::msg::dds_::Twist_",
            Some("RIHS01_sub"),
            Some(gr),
            EndpointKind::Reader,
        )];
        let nodes = vec![node("/", "controller", 0x0c)];
        let (plans, skips) =
            plan_service_calls(&["geometry_msgs/Twist".to_string()], &eps, &nodes, &[]);
        assert!(skips.is_empty());
        assert_eq!(plans.len(), 1);
        assert_eq!(plans[0].attempts.len(), 1);
        assert_eq!(plans[0].attempts[0].node_fqn, "/controller");
        assert_eq!(plans[0].attempts[0].type_hash, "RIHS01_sub");
    }

    /// A writer WITHOUT a hash and a
    /// reader WITH a hash → the reader hash is the SOURCE (no usable publisher
    /// hash to prefer), and BOTH nodes are call targets (the hashless writer's
    /// node demonstrably uses the type, so it stays in the union — writers
    /// first, then the deterministic sort).
    #[test]
    fn test_plan_hashless_writer_falls_back_to_reader_hash() {
        let gw = guid(0x0d, 1);
        let gr = guid(0x0e, 1);
        let eps = vec![
            ep_kind(
                "rt/t",
                "acme_msgs::msg::dds_::Widget_",
                None, // publisher advertised no hash
                Some(gw),
                EndpointKind::Writer,
            ),
            ep_kind(
                "rt/t",
                "acme_msgs::msg::dds_::Widget_",
                Some("RIHS01_r"),
                Some(gr),
                EndpointKind::Reader,
            ),
        ];
        let nodes = vec![node("/", "pub", 0x0d), node("/", "sub", 0x0e)];
        let (plans, skips) =
            plan_service_calls(&["acme_msgs/Widget".to_string()], &eps, &nodes, &[]);
        assert!(skips.is_empty());
        assert_eq!(plans.len(), 1);
        let pairs: Vec<(&str, &str)> = plans[0]
            .attempts
            .iter()
            .map(|a| (a.node_fqn.as_str(), a.type_hash.as_str()))
            .collect();
        assert_eq!(pairs, vec![("/pub", "RIHS01_r"), ("/sub", "RIHS01_r")]);
    }

    /// The GUID-less hashed endpoint:
    /// an endpoint with `type_hash: Some(..)` but `writer_guid: None` — a
    /// backend that could not surface the endpoint GUID — must route to
    /// [`NO_OWNING_NODE_REASON`] (the hash WAS in discovery; there is just no
    /// join candidate), NEVER to [`NO_TYPE_HASH_REASON`] (whose "no type hash
    /// in discovery" wording this input factually violates).
    #[test]
    fn test_plan_hash_present_guid_absent_routes_to_no_owning_node() {
        let eps = vec![ep(
            "rt/widget",
            "acme_msgs::msg::dds_::Widget_",
            Some("RIHS01_x"),
            None, // hash present, GUID absent — the un-joinable shape
        )];
        let nodes = vec![node("/", "bystander", 0x77)];
        let (plans, skips) =
            plan_service_calls(&["acme_msgs/Widget".to_string()], &eps, &nodes, &[]);
        assert!(plans.is_empty());
        assert_eq!(skips.len(), 1);
        assert_eq!(skips[0].0, "acme_msgs/Widget");
        assert_eq!(skips[0].1.reason, NO_OWNING_NODE_REASON);
        assert_ne!(
            skips[0].1.reason, NO_TYPE_HASH_REASON,
            "hash-present/GUID-absent must not claim the hash was missing"
        );
    }

    /// The partial-PEI-loss shape: writer on participant A carries hash H but
    /// A's PEI was LOST
    /// (no node entry); a reader on participant B also carries H and B's PEI
    /// survived. The plan must call B's node with H — a writer-hash
    /// gate on the target set would block the routable reader target and the
    /// type would skip as NO_OWNING_NODE although calling /S resolves it.
    #[test]
    fn test_plan_reader_side_node_is_call_target_when_writer_unroutable() {
        let gw = guid(0x0a, 1); // participant A — PEI lost, no node entry
        let gr = guid(0x0b, 1); // participant B — PEI survived
        let eps = vec![
            ep_kind(
                "rt/w",
                "acme_msgs::msg::dds_::Widget_",
                Some("RIHS01_h"),
                Some(gw),
                EndpointKind::Writer,
            ),
            ep_kind(
                "rt/w",
                "acme_msgs::msg::dds_::Widget_",
                Some("RIHS01_h"),
                Some(gr),
                EndpointKind::Reader,
            ),
        ];
        // Node table holds ONLY the reader-side participant's node.
        let nodes = vec![node("/", "subscriber_node", 0x0b)];
        let (plans, skips) =
            plan_service_calls(&["acme_msgs/Widget".to_string()], &eps, &nodes, &[]);
        assert!(skips.is_empty(), "resolvable via the reader-side node");
        assert_eq!(plans.len(), 1);
        assert_eq!(plans[0].attempts.len(), 1);
        assert_eq!(plans[0].attempts[0].node_fqn, "/subscriber_node");
        assert_eq!(plans[0].attempts[0].type_hash, "RIHS01_h");
    }

    /// `mapping_order`
    /// derives from the TARGET node's participant vendor, NOT the hash-source
    /// writer's — the DISCRIMINATING arm (every other
    /// vendor oracle puts source and target on the SAME participant, or passes
    /// an empty participant table, so reverting the derivation to the writer's
    /// vendor stayed green). Shape: hash-source writer on an eProsima
    /// participant whose PEI was LOST (no node entry — so the reader-side node
    /// is the only candidate); target node on a CYCLONE participant. The one
    /// attempt must carry the CYCLONE order — the node ANSWERING the service
    /// is what the correlation mapping must match; the writer's eProsima
    /// (Enhanced-only) order leaking in fails this assert.
    #[test]
    fn test_plan_mapping_order_derives_from_target_not_hash_source() {
        let gw = guid(0x0a, 1); // hash-source writer — participant A (eProsima)
        let gr = guid(0x0b, 1); // reader — participant B (Cyclone), the target
        let eps = vec![
            ep_kind(
                "rt/w",
                "acme_msgs::msg::dds_::Widget_",
                Some("RIHS01_h"),
                Some(gw),
                EndpointKind::Writer,
            ),
            ep_kind(
                "rt/w",
                "acme_msgs::msg::dds_::Widget_",
                Some("RIHS01_h"),
                Some(gr),
                EndpointKind::Reader,
            ),
        ];
        // ONLY the reader-side participant has a node-table entry.
        let nodes = vec![node("/", "sub", 0x0b)];
        let parts = vec![
            DiscoveredParticipant {
                guid_prefix: [0x0a; 12],
                vendor_id: VENDOR_EPROSIMA,
            },
            DiscoveredParticipant {
                guid_prefix: [0x0b; 12],
                vendor_id: VENDOR_CYCLONE,
            },
        ];
        let (plans, skips) =
            plan_service_calls(&["acme_msgs/Widget".to_string()], &eps, &nodes, &parts);
        assert!(skips.is_empty());
        assert_eq!(plans.len(), 1);
        assert_eq!(plans[0].attempts.len(), 1);
        assert_eq!(plans[0].attempts[0].node_fqn, "/sub");
        assert_eq!(plans[0].attempts[0].type_hash, "RIHS01_h");
        assert_eq!(
            plans[0].attempts[0].mapping_order,
            vec![WireMapping::Cyclone],
            "the TARGET participant's (Cyclone) order must win — a revert to the \
             hash-source writer's vendor yields the eProsima Enhanced-only order"
        );
    }

    /// The vendor-SWAPPED control for the arm above: same shape, vendors
    /// exchanged (writer's lost participant = Cyclone, target's = eProsima) →
    /// the order flips to Enhanced-only, proving the assertion TRACKS the
    /// target's vendor rather than pinning a constant.
    #[test]
    fn test_plan_mapping_order_target_vendor_control_flips() {
        let gw = guid(0x0a, 1);
        let gr = guid(0x0b, 1);
        let eps = vec![
            ep_kind(
                "rt/w",
                "acme_msgs::msg::dds_::Widget_",
                Some("RIHS01_h"),
                Some(gw),
                EndpointKind::Writer,
            ),
            ep_kind(
                "rt/w",
                "acme_msgs::msg::dds_::Widget_",
                Some("RIHS01_h"),
                Some(gr),
                EndpointKind::Reader,
            ),
        ];
        let nodes = vec![node("/", "sub", 0x0b)];
        let parts = vec![
            DiscoveredParticipant {
                guid_prefix: [0x0a; 12],
                vendor_id: VENDOR_CYCLONE,
            },
            DiscoveredParticipant {
                guid_prefix: [0x0b; 12],
                vendor_id: VENDOR_EPROSIMA,
            },
        ];
        let (plans, _skips) =
            plan_service_calls(&["acme_msgs/Widget".to_string()], &eps, &nodes, &parts);
        assert_eq!(
            plans[0].attempts[0].mapping_order,
            vec![WireMapping::Enhanced],
            "swapped vendors must flip the order to the target's Enhanced-only"
        );
    }

    /// TWO writers disagreeing on the RIHS01 hash
    /// (a version-skewed graph) — selection stays DETERMINISTIC (every
    /// (node, hash) pair planned, sorted). This test pins the PLAN only; the
    /// warn EMISSION for this same shape is pinned at the plan site by
    /// `test_plan_skew_warn_emitted_at_the_plan_site` (`#[traced_test]`), and
    /// the message builder's decision table by `test_hash_skew_warning_oracle`.
    #[test]
    fn test_plan_two_writer_hashes_both_planned_deterministically() {
        let g1 = guid(0x01, 1);
        let g2 = guid(0x02, 1);
        let eps = vec![
            ep_kind(
                "rt/a",
                "acme_msgs::msg::dds_::Widget_",
                Some("RIHS01_h1"),
                Some(g1),
                EndpointKind::Writer,
            ),
            ep_kind(
                "rt/a",
                "acme_msgs::msg::dds_::Widget_",
                Some("RIHS01_h2"),
                Some(g2),
                EndpointKind::Writer,
            ),
        ];
        let nodes = vec![node("/", "beta", 0x01), node("/", "alpha", 0x02)];
        let (plans, skips) =
            plan_service_calls(&["acme_msgs/Widget".to_string()], &eps, &nodes, &[]);
        assert!(skips.is_empty());
        assert_eq!(plans.len(), 1);
        // All four (node, hash) pairs, in the deterministic sorted order.
        let pairs: Vec<(&str, &str)> = plans[0]
            .attempts
            .iter()
            .map(|a| (a.node_fqn.as_str(), a.type_hash.as_str()))
            .collect();
        assert_eq!(
            pairs,
            vec![
                ("/alpha", "RIHS01_h1"),
                ("/alpha", "RIHS01_h2"),
                ("/beta", "RIHS01_h1"),
                ("/beta", "RIHS01_h2"),
            ]
        );
    }

    /// The skew warn's EMIT
    /// SITE — the `tracing::warn!` in `plan_service_calls`, not just the pure
    /// message builder — fires for the two-writer-hash skew shape, naming both
    /// hashes. `#[traced_test]` captures the emission; deleting the emit block
    /// (while `hash_skew_warning` stays green in its own oracle) fails here.
    #[test]
    #[traced_test]
    fn test_plan_skew_warn_emitted_at_the_plan_site() {
        let g1 = guid(0x01, 1);
        let g2 = guid(0x02, 1);
        let eps = vec![
            ep_kind(
                "rt/a",
                "acme_msgs::msg::dds_::Widget_",
                Some("RIHS01_h1"),
                Some(g1),
                EndpointKind::Writer,
            ),
            ep_kind(
                "rt/a",
                "acme_msgs::msg::dds_::Widget_",
                Some("RIHS01_h2"),
                Some(g2),
                EndpointKind::Writer,
            ),
        ];
        let nodes = vec![node("/", "beta", 0x01), node("/", "alpha", 0x02)];
        let (plans, skips) =
            plan_service_calls(&["acme_msgs/Widget".to_string()], &eps, &nodes, &[]);
        assert!(skips.is_empty());
        assert_eq!(plans.len(), 1, "the skewed type still plans");
        assert!(
            logs_contain("DISAGREE on the RIHS01 type hash"),
            "the plan site must EMIT the skew warn"
        );
        assert!(
            logs_contain("RIHS01_h1") && logs_contain("RIHS01_h2"),
            "the emitted warn names BOTH hashes"
        );
    }

    /// The non-skew CONTROL for the emit pin: a healthy single-hash plan emits
    /// NO skew warn (anti-tautology for the capture + the no-flood contract on
    /// the steady state).
    #[test]
    #[traced_test]
    fn test_plan_no_skew_warn_on_single_hash() {
        let g = guid(0x0a, 1);
        let eps = vec![ep(
            "rt/w",
            "acme_msgs::msg::dds_::Widget_",
            Some("RIHS01_ab"),
            Some(g),
        )];
        let nodes = vec![node("/", "pub", 0x0a)];
        let (plans, skips) =
            plan_service_calls(&["acme_msgs/Widget".to_string()], &eps, &nodes, &[]);
        assert!(skips.is_empty());
        assert_eq!(plans.len(), 1);
        assert!(
            !logs_contain("DISAGREE on the RIHS01 type hash"),
            "a healthy single-hash source must not warn"
        );
    }

    /// The skew-warn decision + wording oracle —
    /// two distinct hashes in the selected source yield the loud line naming
    /// EVERY hash + the source kind; a single-hash source is silent; the
    /// reader-fallback source is named as such.
    #[test]
    fn test_hash_skew_warning_oracle() {
        let h1 = "RIHS01_h1".to_string();
        let h2 = "RIHS01_h2".to_string();
        // Single hash → no warn (the healthy graph).
        assert_eq!(hash_skew_warning("acme/W", &[&h1], true), None);
        assert_eq!(hash_skew_warning("acme/W", &[], true), None);
        // Two hashes from WRITERS → warn naming both + the source.
        let w = hash_skew_warning("acme/W", &[&h1, &h2], true).expect("skew warns");
        assert!(w.contains("RIHS01_h1"), "{w}");
        assert!(w.contains("RIHS01_h2"), "{w}");
        assert!(w.contains("PUBLISHERS"), "{w}");
        assert!(w.contains("acme/W"), "{w}");
        assert!(w.contains("DISAGREE"), "{w}");
        // Reader-fallback source is named as such.
        let r = hash_skew_warning("acme/W", &[&h1, &h2], false).expect("skew warns");
        assert!(r.contains("SUBSCRIBERS"), "{r}");
    }

    // ─────────────── call-phase orchestration helpers (pure) ───────────────

    /// `apply_attempt_outcome` folds each result — Resolved
    /// returns the sources (retry stops); CallFailed retains the reason (no
    /// cache mutation); WaitTimedOut retains the reason AND caches the node (the
    /// failure-cache POPULATE side).
    #[test]
    fn test_apply_attempt_outcome_folds_each_variant() {
        let sources = vec![WireTypeSource {
            type_name: "acme/msg/Widget".to_string(),
            encoding: "msg".to_string(),
            raw_file_contents: "int32 id\n".to_string(),
        }];

        // Resolved → Some(sources), no state change.
        let mut cache: BTreeSet<String> = BTreeSet::new();
        let mut last: Option<String> = None;
        let got = apply_attempt_outcome(
            AttemptResult::Resolved(sources.clone()),
            "/pub",
            &mut cache,
            &mut last,
        );
        assert_eq!(got, Some(sources));
        assert!(cache.is_empty());
        assert!(last.is_none());

        // CallFailed → None, reason retained, node NOT cached (transient retry).
        let mut cache: BTreeSet<String> = BTreeSet::new();
        let mut last: Option<String> = None;
        let got = apply_attempt_outcome(
            AttemptResult::CallFailed("boom".to_string()),
            "/pub",
            &mut cache,
            &mut last,
        );
        assert_eq!(got, None);
        assert_eq!(last.as_deref(), Some("boom"));
        assert!(cache.is_empty(), "a call failure must NOT cache the node");

        // WaitTimedOut → None, reason retained, node CACHED (the populate side).
        let mut cache: BTreeSet<String> = BTreeSet::new();
        let mut last: Option<String> = None;
        let got = apply_attempt_outcome(
            AttemptResult::WaitTimedOut("no server".to_string()),
            "/dead_node",
            &mut cache,
            &mut last,
        );
        assert_eq!(got, None);
        assert_eq!(last.as_deref(), Some("no server"));
        assert!(
            cache.contains("/dead_node"),
            "wait-timeout must cache the node"
        );
    }

    /// The wait-failure cache READ side — a cached node
    /// yields a loud skip reason naming it; an uncached node yields `None`.
    #[test]
    fn test_wait_cache_skip_reason() {
        let mut cache: BTreeSet<String> = BTreeSet::new();
        assert_eq!(wait_cache_skip_reason(&cache, "/talker"), None);
        cache.insert("/talker".to_string());
        let reason = wait_cache_skip_reason(&cache, "/talker").expect("cached → skip reason");
        assert!(reason.contains("/talker"), "{reason}");
        assert!(reason.contains("not re-waited"), "{reason}");
        // A DIFFERENT node is still not cached.
        assert_eq!(wait_cache_skip_reason(&cache, "/listener"), None);
    }

    /// The inverted-successful classification — a
    /// `successful == false` response is an Err carrying the node + the server's
    /// failure_reason (so the shell tries the next candidate); success is Ok.
    #[test]
    fn test_classify_call_response_inverted_successful() {
        assert_eq!(classify_call_response(true, "", "/talker"), Ok(()));
        let err =
            classify_call_response(false, "Type not currently in use by this node", "/talker")
                .unwrap_err();
        assert!(err.contains("/talker"), "{err}");
        assert!(
            err.contains("Type not currently in use by this node"),
            "{err}"
        );
        assert!(err.contains("reported failure"), "{err}");
    }

    /// The call-phase wall-cap predicate (oracle vector, no
    /// real time — the `Instant` pair mirrors `collect_deadline_reached`).
    #[test]
    fn test_call_phase_budget_exceeded_oracle() {
        let t0 = Instant::now();
        let later = t0 + std::time::Duration::from_millis(10);
        assert!(
            !call_phase_budget_exceeded(t0, later),
            "before the deadline"
        );
        assert!(call_phase_budget_exceeded(later, later), "at the deadline");
        assert!(
            call_phase_budget_exceeded(later + std::time::Duration::from_millis(1), later),
            "past the deadline"
        );
    }

    /// The node-table-LOSS warn
    /// predicate is PER-PARTICIPANT — true iff some hash-bearing endpoint's
    /// participant prefix has no node-table entry. The PARTIAL-drop shape (the
    /// common bounded(8) tail loss: some OTHER participant's node survived
    /// while the hashed endpoint's PEI dropped) must fire — a
    /// global-emptiness predicate false-negatives exactly there.
    #[test]
    fn test_should_warn_node_table_miss() {
        let hashed = ep(
            "rt/w",
            "acme_msgs::msg::dds_::Widget_",
            Some("RIHS01_ab"),
            Some([1u8; 16]),
        );
        let hashless = ep(
            "rt/w",
            "acme_msgs::msg::dds_::Widget_",
            None,
            Some([1u8; 16]),
        );
        let hashed_no_guid = ep(
            "rt/w",
            "acme_msgs::msg::dds_::Widget_",
            Some("RIHS01_ab"),
            None,
        );
        let matching_node = vec![node("/", "talker", 1)]; // prefix [1; 12]
        let other_node = vec![node("/", "bystander", 9)]; // prefix [9; 12]

        // Hashed endpoint + EMPTY node table → warn (kept shape).
        assert!(should_warn_node_table_miss(
            std::slice::from_ref(&hashed),
            &[]
        ));
        // Hashed endpoint + ITS participant's node present → no warn.
        assert!(!should_warn_node_table_miss(
            std::slice::from_ref(&hashed),
            &matching_node
        ));
        // THE partial-drop shape: hashed endpoint's participant MISSING while a
        // DIFFERENT participant's node survived → warn (a global-emptiness
        // predicate stays silent here).
        assert!(
            should_warn_node_table_miss(std::slice::from_ref(&hashed), &other_node),
            "a per-participant miss must fire even when the table is non-empty"
        );
        // No hash anywhere + empty nodes → no warn (nothing callable regardless).
        assert!(!should_warn_node_table_miss(
            std::slice::from_ref(&hashless),
            &[]
        ));
        // Hash but NO GUID → no warn (no participant to miss; skip reason (iii)
        // covers the un-joinable shape).
        assert!(!should_warn_node_table_miss(
            std::slice::from_ref(&hashed_no_guid),
            &[]
        ));
        // Empty everything → no warn.
        assert!(!should_warn_node_table_miss(&[], &[]));
    }

    /// The loss-warn WORDING, pinned against
    /// hand-pasted literals for BOTH build arms — each names BOTH cause classes
    /// (the bounded(8) channel drop AND the GID-width mismatch) and gives the
    /// remediation that can actually work for that build (re-run/lengthen on an
    /// Iron+ build; rebuild-with-jazzy on the pre-Iron opt-in, where re-running
    /// is structurally futile).
    #[test]
    fn test_node_table_miss_warning_wording_literals() {
        assert_eq!(
            node_table_miss_warning(false),
            "wire acquirer: discovered endpoints carry RIHS01 type hashes but their participants \
             have NO ros_discovery_info node-table entry — those publishers cannot be mapped to \
             their get_type_description service. Two cause classes: (a) the DDS status channel \
             (async_channel::bounded(8), try_send DROP) dropped part of the node-announcement \
             burst, or the announcement landed after the discovery window — re-run `cerulion ros2 \
             attach` (a fresh window often catches it) or lengthen --timeout; (b) a GID-width \
             mismatch between builds — not in play here, since hash-bearing peers are Iron+ and \
             share this build's 16-byte ros_discovery_info encoding."
        );
        assert_eq!(
            node_table_miss_warning(true),
            "wire acquirer: discovered endpoints carry RIHS01 type hashes but their participants \
             have NO ros_discovery_info node-table entry — those publishers cannot be mapped to \
             their get_type_description service. This build is the PRE-IRON (24-byte-GID) \
             `humble` opt-in and structurally CANNOT decode Iron+ peers' 16-byte \
             ros_discovery_info — for hash-bearing (Iron+) publishers that is the certain cause, \
             and re-running cannot help. Rebuild cerulion_dds with the default `jazzy` feature \
             to attach to Iron+ robots. (The bounded(8) status-channel drop is the other cause \
             class, but the GID width blocks first here.)"
        );
        // Both arms name BOTH cause classes.
        for arm in [
            node_table_miss_warning(false),
            node_table_miss_warning(true),
        ] {
            assert!(arm.contains("bounded(8)"), "{arm}");
            assert!(arm.contains("GID"), "{arm}");
        }
    }

    /// The wait-cache classification's FULL 4-cell
    /// truth table — a node is cached (WaitTimedOut) ONLY on
    /// (wait_ok=false, wait_timeout=true); every other cell is the transient
    /// CallFailed. Inverting the predicate (that is,
    /// caching a healthy node that answered wait_for_service but failed one
    /// call, making later types skip it as dead) fails 3 of the 4 cells.
    #[test]
    fn test_classify_attempt_failure_truth_table() {
        let r = || "why".to_string();
        // (ok=false, timeout=true) → WaitTimedOut: no server ever appeared.
        assert_eq!(
            classify_attempt_failure(false, true, r()),
            AttemptResult::WaitTimedOut("why".to_string())
        );
        // (ok=true, timeout=true) → CallFailed: SOME mapping connected — the
        // node has a server; the timeout was the other mapping's.
        assert_eq!(
            classify_attempt_failure(true, true, r()),
            AttemptResult::CallFailed("why".to_string())
        );
        // (ok=true, timeout=false) → CallFailed: server reached, call failed.
        assert_eq!(
            classify_attempt_failure(true, false, r()),
            AttemptResult::CallFailed("why".to_string())
        );
        // (ok=false, timeout=false) → CallFailed: nothing even waited (pure
        // client-create failure) — transient, never poisons the cache.
        assert_eq!(
            classify_attempt_failure(false, false, r()),
            AttemptResult::CallFailed("why".to_string())
        );
    }

    /// Bookkeeping reasons (wall-cap, wait-cache)
    /// FILL silence but never OVERWRITE a real answer — the live server's
    /// `successful == false` diagnosis survives a later budget expiry.
    #[test]
    fn test_retain_bookkeeping_reason_never_overwrites_a_real_answer() {
        // THE overwrite case: a real server diagnosis is
        // retained, then the wall cap expires — the diagnosis must survive.
        let server_answer =
            "/nodeA/get_type_description reported failure: Type not currently in use by this node";
        let mut last = Some(server_answer.to_string());
        retain_bookkeeping_reason(
            &mut last,
            "the wire rung's call-phase budget elapsed mid-plan".to_string(),
        );
        assert_eq!(
            last.as_deref(),
            Some(server_answer),
            "budget bookkeeping must not clobber a real server diagnosis"
        );
        // A second bookkeeping arm (wait-cache) also cannot clobber.
        retain_bookkeeping_reason(&mut last, "node /b already failed wait_for_service".into());
        assert_eq!(last.as_deref(), Some(server_answer));

        // Silence IS filled: with nothing better, bookkeeping lands.
        let mut empty: Option<String> = None;
        retain_bookkeeping_reason(&mut empty, "budget elapsed".to_string());
        assert_eq!(empty.as_deref(), Some("budget elapsed"));
    }
}
