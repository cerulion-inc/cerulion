// SPDX-License-Identifier: AGPL-3.0-only
//! The per-robot, verb-dispatched zenoh QUERY SURFACE — key space
//! `cerulion_q/{robot}/{verb}[/...]`, served by the robot's network gateway.
//!
//! ONE gateway (a listening zenoh ACCEPTER) declares ONE wildcard QUERYABLE at
//! `queryable_key` (`cerulion_q/{robot}/**`) and dispatches inbound GETs by
//! verb; remote consumers (dialers) send EXPLICIT-robot GETs. This module owns
//! ALL `cerulion_q` key construction + parsing (no scattered format strings
//! elsewhere) plus the wire types + encoding for the `catalog` and `schema`
//! verbs' replies.
//!
//! # Why a queryable + dialer GETs (the pinned zenoh 1.8 wire asymmetry)
//!
//! On a strict connect-only, listen-less peer link (the real robot↔laptop case)
//! a DIALER's declarations (liveliness tokens, subscribers, interests) never
//! cross to the accepter; only ACCEPTER→dialer declarations (at connect time)
//! and DIALER→accepter GET queries route. So the ROBOT gateway (accepter)
//! declares this queryable and the LAPTOP (dialer) GETs it. A GET with a MID-KEY
//! single-chunk wildcard (`cerulion_q/*/catalog`) computes an EMPTY route on real
//! links even when the intersecting `**` queryable declaration arrived — so the
//! laptop harvests robot identities from the announce space (a proven direction)
//! and always builds EXPLICIT `cerulion_q/{robot}/...` selectors. The one
//! wildcard we retain — the demand-GET fallback when NO identity was harvested —
//! is a best-effort intersect that does not route on such a link (see
//! `demand_selector`); it never regresses a link where an explicit selector is
//! available.
//!
//! # Verb grammar
//!
//! - `cerulion_q/{robot}/demand{canonical topic}` — the demand mechanism
//!   (migrated verbatim from the retired `cerulion_demand` space; behavior is
//!   preserved, only the key moved). `{canonical topic}` keeps its leading `/`.
//! - `cerulion_q/{robot}/catalog` — return the robot's full topic
//!   catalog (every produced/announced topic + its schema hash AND qualified
//!   schema name). Takes no tail.
//! - `cerulion_q/{robot}/runs` — return the runs LIVE on the serving
//!   machine right now (one entry per run: its identity, its graph name, and the
//!   run directory's `graph.yaml` + `run.json` VERBATIM), so a desk can render
//!   the robot's running graph. Takes no tail, exactly like `catalog`. The
//!   answer carries a [`RunsCompleteness`] verdict, so an EMPTY answer is never
//!   mistaken for "this robot is running nothing".
//! - `cerulion_q/{robot}/schema/{pkg}/{Type}` (qualified) OR
//!   `cerulion_q/{robot}/schema/{Name}` (a package-less workspace type):
//!   return the `.msg`/YAML TEXT of the
//!   requested type plus its full nested-CUSTOM-type closure, so a desk with ZERO
//!   local knowledge of the robot's custom types can decode its frames + run
//!   `schema info` against it. The selector is ALWAYS explicit (robot AND the
//!   full type name) — a mid-key wildcard would compute an empty route on a
//!   strict connect-only link (the same asymmetry the `catalog` verb rides).
//!   Built-in types every desk already has (`std_msgs`, `geometry_msgs`, …) are
//!   OMITTED from the served closure — the robot serves only what the desk lacks
//!   (mirrors the schema acquirer's "served from the corpus, never
//!   re-materialized" rule). An unknown type returns an explicit structured error
//!   (never silence, never an empty-200).
//!
//! The `cerulion_q` top-level chunk is the protocol version boundary — an
//! incompatible key-grammar change bumps the chunk. It is DISTINCT from the
//! `cerulion_lv` (demand liveliness) and `cerulion_ann` (announce presence)
//! chunks, both of which STAY (they carry zero periodic traffic — pure routing
//! state that gives free push + death detection — and remain the presence /
//! discovery surface).
//!
//! # Security (pairing integration point)
//!
//! All three verbs are currently UNAUTHENTICATED LAN reads, and the `catalog` +
//! `schema` verbs expose STRICTLY MORE than the announce presence space — so this
//! surface MUST be gated behind install-time PAIRING when that lands.
//! The `schema` verb is the loudest of the three: it hands out the
//! robot's verbatim custom `.msg`/YAML type definitions. Three facts:
//!
//! - The `catalog` verb's source is the gateway's bridge-REGISTERED topic set
//!   (`registered_topics()` joined with the runtime schema-hash table), NOT the
//!   announce space. A topic that is registered but whose announce token has not
//!   (yet) been declared — e.g. a runtime registration whose announce hit a
//!   transient failure (loudly warned in the gateway) — IS served by the catalog
//!   while the announce listing omits it.
//! - The catalog serves a `schema_hash` (a u64 type fingerprint) + qualified
//!   schema NAME per egress topic; the announce tokens are HASHLESS + nameless. So
//!   one `cerulion_q/{robot}/catalog` GET reads type fingerprints + names the
//!   announce space never exposes.
//! - The `schema` verb serves the robot's verbatim custom `.msg`/YAML type
//!   definitions — the richest exposure of the three. It is served ONLY from the
//!   pre-built [`SchemaSource`](super::network::SchemaSource) (workspace custom
//!   types; built-ins omitted), never from arbitrary filesystem paths, but an
//!   unpaired peer must still not read a robot's private type schemas.
//!
//! - The `runs` verb serves a run's effective `graph.yaml` +
//!   `run.json` verbatim — the robot's whole node topology, its topic wiring and
//!   its run-directory path. That is a SUPERSET of the catalog's exposure, so it
//!   is gated on the same [`DemandAuthorizer`](super::demand_authorizer::DemandAuthorizer)
//!   decision and answers a denied party with [`RunsReply::refused`].
//!
//! Therefore, when the presence-only-until-paired gate lands, the
//! `catalog` + `schema` + `runs` verbs (and `demand`) MUST refuse an UNPAIRED
//! peer — an unpaired GET must not enumerate a robot's registered topics + type
//! fingerprints + schema text + graph topology or drive its egress. This is a
//! doc marker only; no gate exists in this module.

use std::collections::BTreeSet;

use serde::{Deserialize, Serialize};

use super::liveness::TopicLiveness;
use super::mirror_registry::GatherCompleteness;
use super::run_registry::RunState;

/// The verb-dispatched query-surface key-space prefix. A DISTINCT top-level chunk
/// from `cerulion_lv` / `cerulion_ann` so a bridge watch or presence gather can
/// never confuse a query for a liveliness/announce token.
pub(crate) const CERULION_Q_PREFIX: &str = "cerulion_q";

/// The `demand` verb chunk — the demand mechanism's key.
const VERB_DEMAND: &str = "demand";

/// The `catalog` verb chunk — the full-topic-catalog request.
const VERB_CATALOG: &str = "catalog";

/// The `schema` verb chunk: the `.msg`/YAML text request. Its
/// key carries a `{pkg}/{Type}` tail (two chunks), e.g.
/// `cerulion_q/go2/schema/vendor_msgs/Widget`, OR a package-less `{Name}` tail
/// (one chunk), e.g. `cerulion_q/go2/schema/MyState`.
const VERB_SCHEMA: &str = "schema";

/// The `runs` verb chunk — the live-runs request. Takes NO tail (an
/// exact match, exactly like [`VERB_CATALOG`]), so a future tailed form
/// (`runs/{run_id}`) is an UNKNOWN verb to this binary rather than something it
/// answers wrongly with the whole list.
const VERB_RUNS: &str = "runs";

/// A parsed query verb recovered from a `cerulion_q/{robot}/{verb}[/...]` key.
/// The gateway's single queryable callback dispatches on this.
#[derive(Debug, Clone, PartialEq, Eq)]
pub(crate) enum QueryVerb {
    /// `demand{canonical topic}` — pull a topic's network egress ON.
    /// `topic` keeps its leading `/` (canonical form).
    Demand { topic: String },
    /// `catalog` — return the robot's full topic catalog.
    Catalog,
    /// `runs` — return the runs LIVE on the serving machine.
    /// Takes no tail.
    Runs,
    /// `schema/{pkg}/{Type}` (qualified) OR `schema/{Name}` (a package-less
    /// workspace type): return the requested type's
    /// `.msg`/YAML text + its nested-custom closure.
    /// `requested` is the full qualified-or-bare type name (all chunks
    /// non-empty), fed directly to [`collect_schema_closure`].
    Schema { requested: String },
}

/// The wildcard key expression a gateway declares its query surface on:
/// `cerulion_q/{robot}/**`. `{robot}` is the gateway's own identity; the `**`
/// tail matches any verb + argument (a canonical topic may be many chunks). An
/// accepter DECLARATION → reaches a connect-only dialer.
pub(crate) fn queryable_key(robot: &str) -> String {
    // hot-path-alloc-ok: cold — the gateway builds this key ONCE at boot to
    // declare its query surface, never on the per-frame data plane.
    format!("{CERULION_Q_PREFIX}/{robot}/**")
}

/// The CONCRETE key a producer replies a demand GET on:
/// `cerulion_q/{robot}/demand{canonical}` (`canonical` keeps its leading `/`).
/// Carries the producer's real identity (so a wildcard GET's replier is
/// attributable) and intersects BOTH the wildcard [`demand_selector`]`("*", ..)`
/// and the explicit [`demand_selector`]`("{robot}", ..)`, so zenoh accepts it
/// against either.
pub(crate) fn demand_reply_key(robot: &str, canonical_topic: &str) -> String {
    // hot-path-alloc-ok: cold — the demand-queryable callback builds this reply
    // key once per remote demand GET (a control-plane request), not per frame.
    format!("{CERULION_Q_PREFIX}/{robot}/{VERB_DEMAND}{canonical_topic}")
}

/// A demand GET selector for ONE canonical ingress topic.
/// `robot_or_wildcard` is a harvested producer identity (EXPLICIT — precise, no
/// fan-out, the direction that routes) or `"*"` (the best-effort mid-key wildcard
/// fallback used only when NO identity was harvested — it intersects every
/// producer's `cerulion_q/{robot}/**` queryable but does not route on a strict
/// connect-only link; see the module docs). `canonical_topic` keeps its leading
/// `/`.
pub(crate) fn demand_selector(robot_or_wildcard: &str, canonical_topic: &str) -> String {
    // hot-path-alloc-ok: cold — the demander builds this GET selector on its ~2 s
    // demand-GET loop (control plane), never on the per-frame data plane.
    format!("{CERULION_Q_PREFIX}/{robot_or_wildcard}/{VERB_DEMAND}{canonical_topic}")
}

/// The catalog GET selector `cerulion_q/{robot}/catalog` — an EXPLICIT robot only
/// (the laptop always knows the identity from the announce harvest; a mid-key
/// wildcard would compute an empty route — see the module docs).
pub fn catalog_selector(robot: &str) -> String {
    // hot-path-alloc-ok: cold — the laptop builds this catalog GET selector during
    // a `topic list` gather (control plane), never on the per-frame data plane.
    format!("{CERULION_Q_PREFIX}/{robot}/{VERB_CATALOG}")
}

/// The runs GET selector `cerulion_q/{robot}/runs` — an EXPLICIT robot only, for
/// the same reason [`catalog_selector`] is: the desk always knows the identity
/// from the announce harvest, and a mid-key wildcard computes an EMPTY route on a
/// strict connect-only link (see the module docs). Attribution is therefore a
/// property of the KEY, which is what makes a phantom "this machine" row
/// unconstructible desk-side.
pub fn runs_selector(robot: &str) -> String {
    // hot-path-alloc-ok: cold — the desk builds this runs GET selector once per
    // graph-panel refresh (control plane), never on the per-frame data plane.
    format!("{CERULION_Q_PREFIX}/{robot}/{VERB_RUNS}")
}

/// The schema GET selector `cerulion_q/{robot}/schema/{requested}` — an EXPLICIT
/// robot AND type. `requested` is the qualified `pkg/Type` (two chunks)
/// OR a package-less bare `Name` (one chunk); the `/`
/// inside a qualified name naturally lands as the chunk separator, so ONE format
/// covers both. The desk always knows the robot identity (announce harvest) and
/// the type name (from the enriched catalog or the user's `schema info <name>`
/// argument); a mid-key wildcard would compute an empty route on a strict
/// connect-only link — see the module docs.
pub fn schema_selector(robot: &str, requested: &str) -> String {
    // hot-path-alloc-ok: cold — the desk builds this schema GET selector during a
    // `schema info` / `topic echo` decode-seed (control plane), never per frame.
    format!("{CERULION_Q_PREFIX}/{robot}/{VERB_SCHEMA}/{requested}")
}

/// Parse a query key `cerulion_q/{robot_or_*}/{verb}[/...]` into its [`QueryVerb`].
/// The robot chunk is IGNORED (a wildcard GET carries `*`; the producer replies
/// with its own identity), so this skips exactly the two leading chunks
/// (`cerulion_q` + the robot/wildcard chunk) and dispatches on the verb chunk.
/// Returns `None` for a malformed key (no verb) or an UNRECOGNIZED verb (a future
/// verb from a newer peer) — the caller logs at `debug` and ignores it.
pub(crate) fn parse_query_verb(key: &str) -> Option<QueryVerb> {
    // hot-path-alloc-ok: cold — this parses an inbound zenoh query key in the
    // gateway's serve-time callback (control plane), never on the data plane.
    let namespace = format!("{CERULION_Q_PREFIX}/");
    let rest = key.strip_prefix(&namespace)?; // "{robot_or_*}/{verb}[/...]"
    let slash = rest.find('/')?; // end of the robot chunk
    let after_robot = &rest[slash + 1..]; // "{verb}[/...]"
    if after_robot == VERB_CATALOG {
        return Some(QueryVerb::Catalog);
    }
    // `runs` takes no tail — an EXACT match, so `runsx` (a foreign verb
    // whose name merely starts with "runs") and a future tailed `runs/{run_id}`
    // both fall through to the `None` no-op a newer peer relies on.
    if after_robot == VERB_RUNS {
        return Some(QueryVerb::Runs);
    }
    // Schema carries a `{pkg}/{Type}` (two-chunk) OR a package-less `{Name}`
    // (one-chunk) tail after `schema/`, every chunk
    // non-empty. Rejects `schema` bare (no `/` tail — falls through), `schema/`
    // (empty), `schema//Type` / `schema/pkg/` (empty chunk), `schema/x/y/z`
    // (three chunks), plus a foreign verb whose name merely starts with "schema"
    // (e.g. `schemax`). The recovered `requested` is fed to `collect_schema_closure`.
    if let Some(tail) = after_robot.strip_prefix(&format!("{VERB_SCHEMA}/")) {
        let parts: Vec<&str> = tail.split('/').collect();
        match parts.as_slice() {
            [name] if !name.is_empty() => {
                return Some(QueryVerb::Schema {
                    requested: (*name).to_string(),
                });
            }
            [pkg, ty] if !pkg.is_empty() && !ty.is_empty() => {
                return Some(QueryVerb::Schema {
                    requested: format!("{pkg}/{ty}"),
                });
            }
            _ => {}
        }
        return None;
    }
    // Demand carries a canonical-topic tail that keeps its leading `/`, so the
    // chunk after `demand` must start with `/` (rejects a foreign verb whose name
    // merely starts with "demand", e.g. `demandx`).
    if let Some(tail) = after_robot.strip_prefix(VERB_DEMAND) {
        if tail.starts_with('/') {
            return Some(QueryVerb::Demand {
                topic: tail.to_string(),
            });
        }
    }
    None
}

// ---------------------------------------------------------------------------
// `catalog` verb — wire types + encoding.
//
// The catalog is a CONTROL-PLANE surface (a one-shot GET reply per `topic
// list`), NOT the zero-copy data path, so it is encoded as versioned JSON:
// self-describing, evolvable (unknown fields ignored on decode), and trivially
// hand-oracle-testable. A catalog entry ALSO carries the
// qualified `schema_name` (`pkg/Type`) alongside the wire `schema_hash`, so the
// desk knows WHICH type to fetch (via the `schema` verb) per topic. The field is
// additive + `#[serde(default)]` (an old desk ignores it; a new desk reading an
// old robot's reply sees `None`). The `.msg`/YAML TEXT rides the separate
// `schema` verb ([`SchemaReply`]).
// ---------------------------------------------------------------------------

/// The wire version of the [`CatalogReply`] payload. A decoder rejects a version
/// it does not understand (an older/newer robot), and the caller falls back to
/// the announce-derived listing. Bump this on any incompatible catalog-payload
/// change.
pub const CATALOG_WIRE_VERSION: u32 = 1;

/// How the gateway learned a catalogued topic — the provenance the gateway
/// already tracks (boot-plan vs runtime-registered). Purely informational.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum CatalogProvenance {
    /// Declared in the boot `GatewayPlan.announce` (a YAML `network: egress:`
    /// topic). The announce is hashless, so such entries typically carry no
    /// `schema_hash`.
    Boot,
    /// Registered at runtime (a `ros2 attach` dds_bridge's raw route arriving over
    /// the `__cerulion/gateway_topics` control channel, which carries the hash).
    Runtime,
}

/// One topic in a robot's catalog.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct CatalogEntry {
    /// The canonical absolute topic name (leading `/`).
    pub topic: String,
    /// The wire `schema_hash` (recipe-3) the gateway advertises for the topic, or
    /// `None` when the gateway holds no hash (a boot-plan hashless announce). A
    /// decoder tolerates the field being absent.
    #[serde(default)]
    pub schema_hash: Option<u64>,
    /// The topic's qualified schema NAME (`pkg/Type`), or `None`
    /// when the gateway holds no name for it (an old robot, or a runtime topic
    /// whose name was never resolved). Additive + `#[serde(default)]`: an old
    /// desk ignores it; a new desk reading an old robot's reply sees `None` and
    /// simply cannot name-fetch that topic's schema. The desk feeds it to
    /// [`schema_selector`] to fetch the type's `.msg`/YAML closure.
    #[serde(default)]
    pub schema_name: Option<String>,
    /// How the gateway learned the topic.
    pub provenance: CatalogProvenance,
    /// The number of LIVE iceoryx2 producers the serving gateway sees for
    /// this topic RIGHT NOW (`dynamic_config().number_of_publishers()`), or `None`
    /// when the gateway did not probe it (an older robot that predates this field, or
    /// a serve path with no transport handle). The desk uses it as a per-row LIVENESS
    /// affordance: `Some(0)` is a registered-but-DEAD route — the topic exists in the
    /// catalog (a DDS route / boot announce) but no producer is currently publishing,
    /// so attaching it shows nothing (`/uslam/cloud_map`, for example) — and the
    /// sidebar dims it + labels it "no data yet"; `Some(n)` (n>0) has a live producer;
    /// `None` degrades to the plain rendering (no liveness affordance). This is a
    /// CHEAP no-wait probe (a `.open()` + a dynamic-config read), NOT a frame observation,
    /// so it reports producer PRESENCE, not a publish rate (a rate for an un-tapped topic
    /// would need a per-topic frame wait — deliberately not paid here). Additive +
    /// `#[serde(default)]`: an old robot omits it ⇒ `None`; an old desk ignores it. No
    /// [`CATALOG_WIRE_VERSION`] bump (the `schema_name` additive-field precedent).
    #[serde(default)]
    pub producer_count: Option<u32>,
    /// What the robot OBSERVED about this topic's DATA FLOW, or `None`
    /// when it observed nothing (an older robot with no observer, a topic whose
    /// liveness tap could not attach, or `CERULION_TOPIC_LIVENESS=off`).
    ///
    /// This is the field a liveness affordance must read — [`Self::producer_count`]
    /// measures publisher EXISTENCE, and on a `cerulion ros2 attach` robot the
    /// bridge graph creates a publisher for every discovered DDS topic at
    /// graph-build time, so it reads `Some(1)` for a dead route exactly as it
    /// does for a streaming one (the measured defect: all 75 Go2 topics
    /// reported `1`). `liveness` instead reports frames the robot actually saw
    /// cross the topic — see
    /// [`TopicLiveness`] and
    /// [`LivenessState`](super::liveness::LivenessState) for the
    /// classification, including why "observed nothing yet" is kept distinct
    /// from "watched a while and nothing ever came".
    ///
    /// `producer_count` is deliberately RETAINED and unchanged: the two answer
    /// different questions (is a port registered / are frames flowing), and a
    /// reader that has both can say "registered, but nothing publishing".
    /// Additive + `#[serde(default, skip_serializing_if)]`: an old robot omits it
    /// ⇒ `None`; an old desk ignores it; a catalog with no liveness serializes
    /// BYTE-IDENTICALLY to the pre-liveness wire. No [`CATALOG_WIRE_VERSION`] bump
    /// (the `schema_name`/`producer_count` additive-field precedent).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub liveness: Option<TopicLiveness>,
}

/// The reply to a `cerulion_q/{robot}/catalog` GET — a robot's full topic
/// catalog. The reply is EXPLICIT-keyed, so it self-attributes via `robot`.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct CatalogReply {
    /// The catalog-payload wire version ([`CATALOG_WIRE_VERSION`]).
    pub version: u32,
    /// The answering robot's identity. The SERVE side self-attributes here, but
    /// the desk-side [`query_robot_catalog`](crate::transport::discovery::query_robot_catalog)
    /// NORMALIZES it to the identity the desk queried (the ANNOUNCE identity in
    /// the explicit `cerulion_q/{robot}/catalog` key) so a divergent
    /// self-attribution never reaches the display.
    pub robot: String,
    /// The robot's produced/announced topics, SORTED + deduped by canonical name.
    pub entries: Vec<CatalogEntry>,
    /// `Some(reason)` iff the robot REFUSED to serve its catalog to
    /// the demanding party (the [`DemandAuthorizer`](crate::transport::demand_authorizer::DemandAuthorizer)
    /// gate denied it — an account/pairing miss). Mutually exclusive with
    /// a non-empty `entries` (a refusal serves NOTHING). The desk-side gather warns
    /// on it (the explicit refusal, never a silent empty catalog). `#[serde(default,
    /// skip_serializing_if)]`: an authorized catalog OMITS the field entirely, so
    /// the AllowAll (deny-nothing) wire is BYTE-IDENTICAL to the older catalog
    /// JSON — the anti-regression contract. An old desk ignores the unknown field.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub error: Option<String>,
}

/// Assemble a [`CatalogReply`] from raw entries — pure (oracle-testable). Sorts +
/// dedups by canonical topic and stamps the current wire version + robot. The
/// gateway-side reader (which reads the live bridge manager + the runtime-hash
/// map) supplies the entries.
pub fn build_catalog_reply(
    robot: &str,
    entries: impl IntoIterator<Item = CatalogEntry>,
) -> CatalogReply {
    let mut entries: Vec<CatalogEntry> = entries.into_iter().collect();
    entries.sort_by(|a, b| a.topic.cmp(&b.topic));
    entries.dedup_by(|a, b| a.topic == b.topic);
    CatalogReply {
        version: CATALOG_WIRE_VERSION,
        // hot-path-alloc-ok: cold — catalog reply construction, built once per
        // remote catalog GET on the control plane, never per frame.
        robot: robot.to_string(),
        entries,
        // A served catalog is never a refusal (mutually exclusive with entries).
        error: None,
    }
}

impl CatalogReply {
    /// An explicit REFUSAL reply — the robot's
    /// [`DemandAuthorizer`](crate::transport::demand_authorizer::DemandAuthorizer)
    /// gate denied this party's catalog GET (an account/pairing miss). Carries NO
    /// entries and an `error` reason, so the desk gets an EXPLICIT refusal (never a
    /// silent empty catalog it cannot distinguish from a topicless robot). Stamps
    /// the current wire version.
    pub fn refused(robot: &str, reason: impl Into<String>) -> Self {
        CatalogReply {
            version: CATALOG_WIRE_VERSION,
            // hot-path-alloc-ok: cold — one refusal reply per denied catalog GET on
            // the control plane, never per frame.
            robot: robot.to_string(),
            entries: Vec::new(),
            error: Some(reason.into()),
        }
    }
}

/// Encode a [`CatalogReply`] to its wire bytes (versioned JSON). Infallible for
/// the reply's plain types; a (theoretically impossible) serialization failure is
/// logged and yields empty bytes, which the demander decodes as "no catalog" and
/// falls back to the announce listing — never a panic on the gateway.
pub fn encode_catalog_reply(reply: &CatalogReply) -> Vec<u8> {
    match serde_json::to_vec(reply) {
        Ok(bytes) => bytes,
        Err(e) => {
            tracing::error!(error = %e, "catalog reply serialization failed");
            // hot-path-alloc-ok: cold — catalog encode error path (a serialization
            // failure of the control-plane reply), never on the data plane.
            Vec::new()
        }
    }
}

/// Why a catalog reply could not be used — the actionable reason the discovery
/// caller warns about ONCE per robot. Both variants map to the same downstream
/// action (ignore this robot's catalog, fall back to its announce listing); they
/// differ only in the log detail.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum CatalogDecodeError {
    /// Parsed as a catalog but carries a wire version this binary does not
    /// understand (an older/newer robot — a `cerulion_q` protocol skew).
    UnknownVersion { got: u32, supported: u32 },
    /// The bytes did not parse as a catalog payload at all (corruption, or a
    /// foreign reply on the key). Carries the serde error class for the log.
    Malformed { detail: String },
}

impl std::fmt::Display for CatalogDecodeError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            CatalogDecodeError::UnknownVersion { got, supported } => write!(
                f,
                "unknown catalog wire version {got} (this binary supports {supported})"
            ),
            CatalogDecodeError::Malformed { detail } => {
                write!(f, "catalog reply did not parse as JSON ({detail})")
            }
        }
    }
}

/// Decode wire bytes into a [`CatalogReply`], or classify WHY they could not be
/// used ([`CatalogDecodeError`]). A caller that gathers a robot's replies warns
/// ONCE per robot on the decode failure (see [`gather_catalog_outcome`]) and
/// falls back to the announce-derived listing — never silent, never per-reply.
pub fn decode_catalog_reply(bytes: &[u8]) -> Result<CatalogReply, CatalogDecodeError> {
    let reply: CatalogReply =
        serde_json::from_slice(bytes).map_err(|e| CatalogDecodeError::Malformed {
            // hot-path-alloc-ok: cold — catalog decode error path (a JSON parse
            // failure of a control-plane reply on `topic list`), never per frame.
            detail: e.to_string(),
        })?;
    if reply.version != CATALOG_WIRE_VERSION {
        return Err(CatalogDecodeError::UnknownVersion {
            got: reply.version,
            supported: CATALOG_WIRE_VERSION,
        });
    }
    Ok(reply)
}

/// The outcome of gathering ONE robot's catalog replies (in arrival order) —
/// what the discovery caller does with them. Pure ([`gather_catalog_outcome`]);
/// the caller maps `Ignored` to a single loud warn-per-robot and the other two
/// to silence.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum CatalogGatherOutcome {
    /// The FIRST decodable reply — use its topics.
    Decoded(CatalogReply),
    /// At least one reply arrived but NONE decoded — warn ONCE with the reason,
    /// then fall back to the announce listing (a real problem: a protocol skew or
    /// corruption).
    Ignored(CatalogDecodeError),
    /// No reply arrived — the robot did not answer (an older binary with no query
    /// surface). The SILENT back-compat fallback path (not a warn).
    NoReply,
}

/// PURE: classify a robot's catalog replies (arrival order) into a
/// [`CatalogGatherOutcome`] — the FIRST decodable reply wins; else, if any reply
/// arrived but none decoded, the reason to warn ONCE (the last decode error);
/// else `NoReply`. Oracle-tested. The discovery caller emits at most one
/// `warn!` per robot from the `Ignored` arm — never per reply, never silent on a
/// genuine skew.
pub fn gather_catalog_outcome<'a>(
    replies: impl IntoIterator<Item = &'a [u8]>,
) -> CatalogGatherOutcome {
    let mut last_err: Option<CatalogDecodeError> = None;
    for bytes in replies {
        match decode_catalog_reply(bytes) {
            Ok(reply) => return CatalogGatherOutcome::Decoded(reply),
            Err(e) => last_err = Some(e),
        }
    }
    match last_err {
        Some(reason) => CatalogGatherOutcome::Ignored(reason),
        None => CatalogGatherOutcome::NoReply,
    }
}

// ---------------------------------------------------------------------------
// `schema` verb — wire types + closure walk.
//
// The `schema` verb serves the `.msg`/YAML TEXT of a requested type plus its
// full nested-CUSTOM-type closure, so a desk with ZERO local knowledge can seed
// a decoder IN MEMORY (no disk write). Encoded as versioned JSON for the same
// reasons as the catalog (control-plane, evolvable, hand-oracle-testable).
// ---------------------------------------------------------------------------

/// The wire version of the [`SchemaReply`] payload. A decoder rejects a version
/// it does not understand (an older/newer robot). Bump on any incompatible
/// schema-payload change.
pub const SCHEMA_WIRE_VERSION: u32 = 1;

/// How a served [`SchemaDoc`]'s `text` is encoded — the desk seeds its decoder
/// from the right parser (`Msg` ⇒ `parse_rosmsg`, `Yaml` ⇒ the Cerulion-native
/// YAML schema parser). Serde snake_case for stable JSON.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum SchemaEncoding {
    /// A ROS 2 `.msg` file's verbatim text (the common case — workspace `.msg`
    /// store + built-in corpus share this encoding).
    Msg,
    /// A Cerulion-native YAML schema's verbatim text (a `schemas/*.yaml` entry).
    Yaml,
}

/// One served schema document — the verbatim text of ONE type plus the qualified
/// names of the OTHER served (custom) types it directly references. The desk
/// walks `deps` transitively to gather a type's full closure.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct SchemaDoc {
    /// The qualified `pkg/Type` name this doc defines.
    pub qualified: String,
    /// How `text` is encoded ([`SchemaEncoding`]).
    pub encoding: SchemaEncoding,
    /// The verbatim source text (comments + whitespace preserved) — byte-copied
    /// so a desk decodes bit-for-bit against a native compiled consumer.
    pub text: String,
    /// The qualified names of the OTHER served (custom) types this doc directly
    /// references. Built-in references (the desk already has them) are OMITTED,
    /// so `deps` only ever names types that ARE in the same served set — the
    /// closure walk never dangles. Additive `#[serde(default)]` (an old robot
    /// omits it ⇒ a leaf).
    #[serde(default)]
    pub deps: Vec<String>,
}

/// The reply to a `cerulion_q/{robot}/schema/{pkg}/{Type}` GET. Either `docs` is
/// non-empty (the requested type FIRST, then its nested-custom closure, deduped)
/// with `error == None`, OR `docs` is empty with `error == Some(reason)` — the
/// explicit structured "unknown type" answer (never silence, never an empty-200).
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct SchemaReply {
    /// The schema-payload wire version ([`SCHEMA_WIRE_VERSION`]).
    pub version: u32,
    /// The answering robot's identity. The SERVE side self-attributes here, but
    /// the desk-side [`query_robot_schema`](crate::transport::discovery::query_robot_schema)
    /// NORMALIZES it to the identity the desk queried (the ANNOUNCE identity in
    /// the explicit `cerulion_q/{robot}/schema/...` key) so the displayed
    /// provenance can never diverge from the query key.
    pub robot: String,
    /// The requested qualified `pkg/Type` (echoed for the desk's correlation).
    pub requested: String,
    /// The requested type FIRST, then its nested-custom closure (deduped). Empty
    /// iff `error` is set.
    #[serde(default)]
    pub docs: Vec<SchemaDoc>,
    /// `Some(reason)` iff the robot could not serve the requested type (unknown
    /// to it). Mutually exclusive with a non-empty `docs`.
    #[serde(default)]
    pub error: Option<String>,
}

impl SchemaReply {
    /// A SUCCESS reply — the requested type + its closure. Stamps the current
    /// wire version. `docs` MUST lead with the requested type.
    pub fn found(robot: &str, requested: &str, docs: Vec<SchemaDoc>) -> Self {
        SchemaReply {
            version: SCHEMA_WIRE_VERSION,
            // hot-path-alloc-ok: cold — one schema reply per remote `schema` GET
            // on the control plane, never per frame.
            robot: robot.to_string(),
            requested: requested.to_string(),
            docs,
            error: None,
        }
    }

    /// An explicit structured NOT-FOUND reply — the robot does not have the
    /// requested type. Carries a human-readable `reason` (never silence).
    pub fn not_found(robot: &str, requested: &str, reason: impl Into<String>) -> Self {
        SchemaReply {
            version: SCHEMA_WIRE_VERSION,
            // hot-path-alloc-ok: cold — one schema reply per remote `schema` GET.
            robot: robot.to_string(),
            requested: requested.to_string(),
            docs: Vec::new(),
            error: Some(reason.into()),
        }
    }

    /// An explicit REFUSAL reply — the robot's
    /// [`DemandAuthorizer`](crate::transport::demand_authorizer::DemandAuthorizer)
    /// gate denied this party's schema GET (an account/pairing miss, distinct from
    /// [`Self::not_found`]'s "the robot has no such type"). Serves NO `docs` and
    /// carries the refusal `reason` in `error` — the existing "structured refusal,
    /// never silence" carrier, reused for authorization. Wire-shape-identical to a
    /// `not_found` reply (both are docs-empty + `error: Some`), so no wire change.
    pub fn refused(robot: &str, requested: &str, reason: impl Into<String>) -> Self {
        SchemaReply {
            version: SCHEMA_WIRE_VERSION,
            // hot-path-alloc-ok: cold — one refusal reply per denied `schema` GET.
            robot: robot.to_string(),
            requested: requested.to_string(),
            docs: Vec::new(),
            error: Some(reason.into()),
        }
    }
}

/// PURE: gather the transitive nested-custom closure of `requested` over a served
/// `docs` map (qualified name → doc). Returns the requested doc FIRST, then every
/// reachable dependency (deduped, deterministic BFS order), or `None` when
/// `requested` is not in the map (the caller replies [`SchemaReply::not_found`]).
/// Cycle-safe (a visited set), so a self- or mutually-recursive schema never
/// loops. Oracle-tested.
pub fn collect_schema_closure(
    requested: &str,
    docs: &std::collections::BTreeMap<String, SchemaDoc>,
) -> Option<Vec<SchemaDoc>> {
    let root = docs.get(requested)?;
    // hot-path-alloc-ok: cold — one closure walk per remote `schema` GET.
    let mut out: Vec<SchemaDoc> = Vec::new();
    let mut seen: std::collections::HashSet<String> = std::collections::HashSet::new();
    let mut queue: std::collections::VecDeque<&SchemaDoc> = std::collections::VecDeque::new();
    seen.insert(requested.to_string());
    queue.push_back(root);
    while let Some(doc) = queue.pop_front() {
        out.push(doc.clone());
        for dep in &doc.deps {
            // A dep naming a type NOT in the served set is skipped (never
            // dangles): the served set contains every custom type, so an absent
            // dep is a built-in the desk already has (or a genuinely-missing type
            // — either way there is nothing to serve).
            if seen.insert(dep.clone()) {
                if let Some(d) = docs.get(dep) {
                    queue.push_back(d);
                }
            }
        }
    }
    Some(out)
}

/// Why a schema reply could not be used — the reason the desk warns ONCE about
/// (mirrors [`CatalogDecodeError`]). Both variants fall back to the desk's
/// hash-only behavior.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum SchemaDecodeError {
    /// Parsed but carries a wire version this binary does not understand.
    UnknownVersion { got: u32, supported: u32 },
    /// The bytes did not parse as a schema payload at all.
    Malformed { detail: String },
}

impl std::fmt::Display for SchemaDecodeError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            SchemaDecodeError::UnknownVersion { got, supported } => write!(
                f,
                "unknown schema wire version {got} (this binary supports {supported})"
            ),
            SchemaDecodeError::Malformed { detail } => {
                write!(f, "schema reply did not parse as JSON ({detail})")
            }
        }
    }
}

/// Encode a [`SchemaReply`] to its wire bytes (versioned JSON). A serialization
/// failure is logged + yields empty bytes (the desk decodes that as "no schema"
/// and falls back to hash-only) — never a panic on the gateway.
pub fn encode_schema_reply(reply: &SchemaReply) -> Vec<u8> {
    match serde_json::to_vec(reply) {
        Ok(bytes) => bytes,
        Err(e) => {
            tracing::error!(error = %e, "schema reply serialization failed");
            // hot-path-alloc-ok: cold — schema encode error path (control plane).
            Vec::new()
        }
    }
}

/// Decode wire bytes into a [`SchemaReply`], or classify WHY they could not be
/// used ([`SchemaDecodeError`]).
pub fn decode_schema_reply(bytes: &[u8]) -> Result<SchemaReply, SchemaDecodeError> {
    let reply: SchemaReply =
        serde_json::from_slice(bytes).map_err(|e| SchemaDecodeError::Malformed {
            // hot-path-alloc-ok: cold — schema decode error path (control plane).
            detail: e.to_string(),
        })?;
    if reply.version != SCHEMA_WIRE_VERSION {
        return Err(SchemaDecodeError::UnknownVersion {
            got: reply.version,
            supported: SCHEMA_WIRE_VERSION,
        });
    }
    Ok(reply)
}

/// The outcome of gathering ONE robot's schema replies (arrival order) — the
/// FIRST decodable reply wins; else, if any reply arrived but none decoded, the
/// reason to warn ONCE (the last decode error); else `NoReply`. Mirrors
/// [`gather_catalog_outcome`]. Oracle-tested.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum SchemaGatherOutcome {
    /// The FIRST decodable reply — use it (it may itself be a NOT-FOUND reply,
    /// which the caller surfaces as "this robot does not have the type").
    Decoded(SchemaReply),
    /// At least one reply arrived but NONE decoded — warn ONCE (a protocol skew
    /// or corruption), then fall back to hash-only.
    Ignored(SchemaDecodeError),
    /// No reply arrived — the robot did not answer (an older binary with no
    /// query surface). The SILENT back-compat fallback.
    NoReply,
}

/// PURE: classify a robot's schema replies into a [`SchemaGatherOutcome`] — the
/// first decodable reply wins; else the last decode error; else `NoReply`.
pub fn gather_schema_outcome<'a>(
    replies: impl IntoIterator<Item = &'a [u8]>,
) -> SchemaGatherOutcome {
    let mut last_err: Option<SchemaDecodeError> = None;
    for bytes in replies {
        match decode_schema_reply(bytes) {
            Ok(reply) => return SchemaGatherOutcome::Decoded(reply),
            Err(e) => last_err = Some(e),
        }
    }
    match last_err {
        Some(reason) => SchemaGatherOutcome::Ignored(reason),
        None => SchemaGatherOutcome::NoReply,
    }
}

// ---------------------------------------------------------------------------
// `runs` verb — wire types + encoding.
//
// The `runs` verb answers "which runs are live on this machine right now, and
// what is each one's graph?". Its payload is the run registry's own gather
// (`run_registry::gather_runs_on_config`) joined with each run's DIRECTORY
// artifacts, served VERBATIM: the effective `graph.yaml` a run writes at boot IS
// the run's authoritative self-description, so the wire carries a
// document that already exists and is already versioned by the graph schema
// itself — no second source of truth, and a desk re-parses it with the SAME
// `GraphConfig` parser the robot used.
//
// Encoded as versioned JSON for the same reasons as `catalog`/`schema`: it is a
// control-plane one-shot GET reply (the DAG is immutable for the life of a
// `run_id`, so a desk fetches it ONCE per run), self-describing, and trivially
// hand-oracle-testable.
// ---------------------------------------------------------------------------

/// The wire version of the [`RunsReply`] payload. A decoder rejects a version it
/// does not understand (an older/newer robot). Bump on any incompatible
/// runs-payload change.
pub const RUNS_WIRE_VERSION: u32 = 1;

/// The maximum size (bytes) of ONE served run-directory artifact — the effective
/// `graph.yaml` or `run.json`.
///
/// A real `graph.yaml` is KILOBYTES; this is ~100× headroom. It is deliberately
/// not larger, because the reply must compose: netd's control line cap is 8 MiB
/// (`cerulion_netd::protocol`), and a machine running the registry's whole
/// reader budget of concurrent runs serves at most `runs × 2 × 256 KiB`, which
/// stays inside that cap with room for the rest of the envelope. The serve side reads at most
/// this many bytes per artifact; [`RunEntry::new`] refuses anything longer, so a
/// pathological run directory cannot wedge or bloat a serve.
pub const MAX_RUN_ARTIFACT_LEN: usize = 256 * 1024;

/// The CANONICAL text form of a `run_id`: `0x` + 32 lowercase hex digits,
/// zero-padded — byte-identical to the `run_id` field a run writes into its own
/// `run.json`, to what `cerulion bag record --run` accepts, and to what the
/// `graph run` breadcrumb logs.
///
/// The wire carries this STRING rather than the 128-bit integer, and that is
/// load-bearing rather than cosmetic: `run_id` is IDENTITY, and a JSON number of
/// that magnitude cannot survive a JavaScript consumer — `JSON.parse` rounds
/// anything past 2^53 to the nearest representable double, so two DIFFERENT runs
/// could compare EQUAL desk-side (Studio's panel keys its fetch-once cache on
/// exactly this value). The zero padding also makes lexicographic order match
/// numeric order, which is what lets [`build_runs_reply`] sort deterministically
/// on the text.
pub fn format_run_id(run_id: u128) -> String {
    // hot-path-alloc-ok: cold — one per served run entry on the control plane.
    format!("0x{run_id:032x}")
}

/// Parse a `run_id` back out of its [`format_run_id`] text — the exact inverse,
/// tolerating a missing `0x` prefix and either case (the tolerance
/// `cerulion bag record --run`'s own selector already grants an operator typing
/// one). Returns `None` for anything that is not 1..=32 ASCII hex digits, so a
/// sign, whitespace or a stray `_` is refused rather than silently accepted by
/// `from_str_radix`.
pub fn parse_run_id(text: &str) -> Option<u128> {
    let digits = text
        .strip_prefix("0x")
        .or_else(|| text.strip_prefix("0X"))
        .unwrap_or(text);
    if digits.is_empty() || digits.len() > 32 || !digits.bytes().all(|b| b.is_ascii_hexdigit()) {
        return None;
    }
    u128::from_str_radix(digits, 16).ok()
}

/// What a served run says about itself — the WIRE mirror of
/// [`RunState`].
///
/// A mirror rather than a serde derive on `RunState` itself, because a wire
/// vocabulary and an internal one must be free to be renamed independently: the
/// registry's enum is an SHM record's byte encoding, and binding this verb's JSON
/// to it would make an internal rename a silent wire break. The two are tied by a
/// TOTAL [`From`] conversion, so adding a third `RunState` variant fails to
/// compile here rather than degrading into a guess.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum RunEntryState {
    /// The run is executing.
    Live,
    /// The run is shutting down gracefully (the registry's LAST WORD — see
    /// [`RunState::Ending`]).
    Ending,
}

impl From<RunState> for RunEntryState {
    fn from(state: RunState) -> Self {
        match state {
            RunState::Live => RunEntryState::Live,
            RunState::Ending => RunEntryState::Ending,
        }
    }
}

impl std::fmt::Display for RunEntryState {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            RunEntryState::Live => write!(f, "live"),
            RunEntryState::Ending => write!(f, "ending"),
        }
    }
}

/// What a `runs` answer is EVIDENCE of — the WIRE mirror of
/// [`GatherCompleteness`], and the
/// reason an empty [`RunsReply::runs`] is readable.
///
/// A gather is a windowed LISTEN, so "I heard no runs" and "there are no runs"
/// are different statements. Carrying which one the desk got is what lets the
/// graph panel say *"could not establish the running graph"* instead of rendering
/// a confident "nothing is running" over a robot that is running something.
///
/// A mirror rather than serde on the shared internal enum, for the same reason
/// [`RunEntryState`] is one — plus the counts are `u32` here, since a wire field
/// must not be platform-sized `usize`. The [`From`] conversion is total and
/// saturating, so the semantics cannot drift from the gather that produced them.
///
/// INTERNALLY tagged (`kind`), so both arms are the same JSON SHAPE — an object.
/// Serde's default external tagging would put a bare string on the wire for
/// `Settled` and an object for `Incomplete`, and the desk-side JS consumer would
/// have to branch on the payload's type before it could read its meaning.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(tag = "kind", rename_all = "snake_case")]
pub enum RunsCompleteness {
    /// The answer is EVIDENCE: every live registry writer was heard from (or
    /// none was live at all). An empty `runs` here genuinely means "this machine
    /// is running nothing".
    Settled,
    /// The gather window expired with live writers it never heard from, OR no
    /// gather established anything at all (see
    /// [`RunsCompleteness::not_established`]). The run set is whatever arrived —
    /// possibly empty, possibly a strict subset — and is NOT an absence claim.
    Incomplete {
        /// Writers the registry reported live on the final pass (0 when nothing
        /// was measured).
        live_writers: u32,
        /// Distinct writer identities the gather actually heard from.
        writers_heard: u32,
    },
}

impl From<GatherCompleteness> for RunsCompleteness {
    fn from(completeness: GatherCompleteness) -> Self {
        match completeness {
            GatherCompleteness::Settled => RunsCompleteness::Settled,
            GatherCompleteness::Incomplete {
                live_writers,
                writers_heard,
            } => RunsCompleteness::Incomplete {
                // Saturating: a count above u32::MAX is not reachable (the
                // registry's reader budget is single digits) and a wrap would be
                // the one failure mode worse than a clamp.
                live_writers: live_writers.min(u32::MAX as usize) as u32,
                writers_heard: writers_heard.min(u32::MAX as usize) as u32,
            },
        }
    }
}

impl RunsCompleteness {
    /// Whether an EMPTY `runs` list from this reply may be read as "no runs".
    /// The one question a consumer actually asks — mirrors
    /// [`GatherCompleteness::is_settled`].
    #[must_use]
    pub fn is_settled(&self) -> bool {
        matches!(self, Self::Settled)
    }

    /// The verdict for a reply where NO gather ran at all — the refusal path
    /// ([`RunsReply::refused`]).
    ///
    /// Deliberately NOT `Settled`: a refused party learned nothing about the
    /// robot's runs, and an empty list beside a `Settled` verdict is a positive
    /// absence claim. The counts are true zeros ("nothing was measured"); WHICH
    /// non-claim it is (refused vs. an expired window) is carried by
    /// [`RunsReply::error`], exactly as [`CatalogReply::refused`] discriminates a
    /// refusal from a topicless robot.
    #[must_use]
    pub fn not_established() -> Self {
        RunsCompleteness::Incomplete {
            live_writers: 0,
            writers_heard: 0,
        }
    }
}

/// One live run in a robot's answer — its identity plus its run directory's two
/// self-describing artifacts, VERBATIM.
///
/// Both artifacts are REQUIRED, not optional, and that is the type doing a job: a
/// run whose `graph.yaml` could not be read is SKIPPED by the serve side with a
/// named reason, never served as an entry carrying an empty document that the
/// desk would render as a graph with no nodes. [`RunEntry::new`] is the only way
/// to mint one from a gathered record, and it refuses the empty and oversized
/// shapes so that skip cannot be forgotten.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct RunEntry {
    /// The run's 128-bit identity in its [`format_run_id`] canonical text form
    /// (`0x` + 32 lowercase hex digits). A restart mints a new one by
    /// construction, which is what makes the desk's fetch-once-per-run cache
    /// correct.
    pub run_id: String,
    /// The run's graph name (its identity: the file stem).
    pub graph_name: String,
    /// Wall-clock ns since the Unix epoch at which the run started, for rendering
    /// and for ordering concurrent runs — never for identity.
    ///
    /// A raw `u64` (the `schema_hash` precedent on this same wire). A JavaScript
    /// consumer's `JSON.parse` quantises a value of this magnitude to ~256 ns,
    /// which is immaterial for a start time — runs begin seconds apart — and is
    /// why the field is NOT given the string treatment `run_id` needs.
    pub run_started_at_ns: u64,
    /// What the run said about itself at gather time.
    pub state: RunEntryState,
    /// The run's effective `graph.yaml`, VERBATIM — the whole in-memory
    /// `GraphConfig` the run is executing, rendered by the same pure function
    /// that produces a `--record` bag's attachment. Never empty (see
    /// the type docs).
    pub graph_yaml: String,
    /// The run's `run.json`, VERBATIM — the run manifest (`run_id`, graph name,
    /// pid, start time, process-group shape). Never empty (see the type docs).
    pub run_json: String,
}

/// Why a [`RunEntry`] could not be minted from a gathered record + its run
/// directory. Every variant is a distinct, testable reason the serve side names
/// in its skip log — mirroring
/// [`RunRecordError`](super::run_registry::RunRecordError)'s vocabulary.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum RunEntryError {
    /// The graph name is empty (a record the registry itself would have refused).
    EmptyGraphName,
    /// The run directory's `graph.yaml` is empty or could not be read — the run
    /// is SKIPPED; an empty document is never served.
    EmptyGraphYaml,
    /// The run directory's `run.json` is empty or could not be read.
    EmptyRunJson,
    /// The `graph.yaml` exceeds [`MAX_RUN_ARTIFACT_LEN`] bytes.
    GraphYamlTooLong { len: usize },
    /// The `run.json` exceeds [`MAX_RUN_ARTIFACT_LEN`] bytes.
    RunJsonTooLong { len: usize },
}

impl std::fmt::Display for RunEntryError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            RunEntryError::EmptyGraphName => write!(f, "run record carries an empty graph name"),
            RunEntryError::EmptyGraphYaml => {
                write!(f, "run directory holds no readable graph.yaml")
            }
            RunEntryError::EmptyRunJson => write!(f, "run directory holds no readable run.json"),
            RunEntryError::GraphYamlTooLong { len } => write!(
                f,
                "graph.yaml is {len} bytes, above the {MAX_RUN_ARTIFACT_LEN}-byte served cap"
            ),
            RunEntryError::RunJsonTooLong { len } => write!(
                f,
                "run.json is {len} bytes, above the {MAX_RUN_ARTIFACT_LEN}-byte served cap"
            ),
        }
    }
}

/// The field invariants a served [`RunEntry`] must satisfy, in ONE place because
/// TWO paths must agree on them: [`RunEntry::new`] (the serve side's only minting
/// path) and [`RunsReply::validate`] (the DECODE side, which must accept exactly
/// what `new` can mint — see that function for why). Sharing the body is what
/// makes the two unable to drift: an invariant added here is enforced on both
/// sides at once, rather than being remembered in the second place.
fn check_run_entry_fields(
    graph_name: &str,
    graph_yaml: &str,
    run_json: &str,
) -> Result<(), RunEntryError> {
    if graph_name.is_empty() {
        return Err(RunEntryError::EmptyGraphName);
    }
    if graph_yaml.is_empty() {
        return Err(RunEntryError::EmptyGraphYaml);
    }
    if run_json.is_empty() {
        return Err(RunEntryError::EmptyRunJson);
    }
    if graph_yaml.len() > MAX_RUN_ARTIFACT_LEN {
        return Err(RunEntryError::GraphYamlTooLong {
            len: graph_yaml.len(),
        });
    }
    if run_json.len() > MAX_RUN_ARTIFACT_LEN {
        return Err(RunEntryError::RunJsonTooLong {
            len: run_json.len(),
        });
    }
    Ok(())
}

/// Whether `text` is the EXACT canonical `run_id` form [`format_run_id`] mints —
/// not merely something [`parse_run_id`] can read.
///
/// The distinction is the whole point. `parse_run_id` is deliberately TOLERANT (a
/// missing `0x`, either case, fewer than 32 digits) because an operator types
/// `--run` by hand; the WIRE has no such excuse, and tolerating it there would be
/// an identity hazard rather than a kindness. `run_id` is compared desk-side AS
/// TEXT — Studio's fetch-once-per-run cache keys on this string — so `0xABC` and
/// `0x0…abc` would be two different runs that are in fact one, which is precisely
/// the confusion the canonical zero-padded form exists to prevent.
///
/// Expressed as a ROUND TRIP through the pair rather than as a hand-written
/// character predicate, so it cannot drift from `format_run_id`'s actual output:
/// whatever that function starts emitting is, by construction, what this accepts.
fn is_canonical_run_id(text: &str) -> bool {
    parse_run_id(text).is_some_and(|id| format_run_id(id) == text)
}

impl RunEntry {
    /// Mint a served entry from a gathered run record plus the two artifacts read
    /// out of its run directory — the ONLY constructor the serve side uses, so
    /// the canonical `run_id` text is minted in exactly one place and an
    /// unreadable artifact becomes a NAMED skip rather than an empty document on
    /// the wire.
    ///
    /// Takes the registry's own [`RunState`] and
    /// converts it, so the served vocabulary is tied to the gathered one.
    ///
    /// # Errors
    ///
    /// [`RunEntryError`] when the graph name is empty, either artifact is empty
    /// (unreadable / missing), or either artifact exceeds
    /// [`MAX_RUN_ARTIFACT_LEN`].
    pub fn new(
        run_id: u128,
        graph_name: impl Into<String>,
        run_started_at_ns: u64,
        state: RunState,
        graph_yaml: impl Into<String>,
        run_json: impl Into<String>,
    ) -> Result<Self, RunEntryError> {
        let graph_name = graph_name.into();
        let graph_yaml = graph_yaml.into();
        let run_json = run_json.into();
        check_run_entry_fields(&graph_name, &graph_yaml, &run_json)?;
        Ok(RunEntry {
            run_id: format_run_id(run_id),
            graph_name,
            run_started_at_ns,
            state: state.into(),
            graph_yaml,
            run_json,
        })
    }
}

/// The reply to a `cerulion_q/{robot}/runs` GET — the runs live on the answering
/// machine, plus what that answer is evidence of. The reply is EXPLICIT-keyed, so
/// it self-attributes via `robot`.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct RunsReply {
    /// The runs-payload wire version ([`RUNS_WIRE_VERSION`]).
    pub version: u32,
    /// The answering robot's identity. The SERVE side self-attributes here, but
    /// the desk-side gather NORMALIZES it to the identity the desk queried (the
    /// ANNOUNCE identity in the explicit `cerulion_q/{robot}/runs` key) so a
    /// divergent self-attribution never reaches the display.
    pub robot: String,
    /// The live runs, sorted by start time then `run_id` and deduped by `run_id`.
    ///
    /// Deliberately NOT `#[serde(default)]`, unlike [`SchemaReply::docs`]: this is
    /// the payload the verb exists for, there is no pre-v1 peer that could omit
    /// it, and a document lacking it is corrupt rather than lean. Tolerating the
    /// absence would buy nothing and would let a truncated document decode as a
    /// well-formed answer.
    pub runs: Vec<RunEntry>,
    /// What an EMPTY [`Self::runs`] means — see [`RunsCompleteness`].
    ///
    /// REQUIRED on the wire, with no `#[serde(default)]`, and that is deliberate:
    /// the only sane default would be `Settled`, which is a POSITIVE absence
    /// claim ("this robot is running nothing") that a peer omitting the field
    /// never made. The discovery marker on netd's query replies hit exactly
    /// that class (a defaulted `Settled` from a daemon that predates the
    /// field), and the remedy there was to refuse to believe it. Here the
    /// field is v1-mandatory, so a document without it fails to decode loudly
    /// instead.
    pub completeness: RunsCompleteness,
    /// Runs the robot KNOWS are live but could not describe — the reason
    /// [`Self::runs`] is a strict subset of what the gather found.
    ///
    /// A serve-side skip (an unreadable `graph.yaml`, an oversized `run.json`)
    /// removes a run from the answer, and without this the removal is INVISIBLE:
    /// the desk renders one fewer run than the robot is executing, and — the
    /// sharp case — a machine whose only run is undescribable serves an EMPTY
    /// list that a settled verdict would license as "this robot is running
    /// nothing". That is the false-absence class this payload's `completeness` exists
    /// to prevent, arriving through the other door.
    ///
    /// So it is not merely reported, it is BINDING: a reply carrying any of these
    /// can never be `Settled` (enforced by [`build_runs_reply`] on the way out and
    /// by [`RunsReply::validate`] on the way in). It exists as well as the
    /// demotion because a bare `Incomplete` cannot be acted on — "the gather
    /// window expired" is retried, "this run's directory is unreadable" is a
    /// person going to look at it — exactly the discrimination
    /// [`Self::error`] provides for a refusal.
    ///
    /// `#[serde(default, skip_serializing_if)]`: a healthy reply OMITS the key.
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub undescribable: Vec<UndescribableRun>,
    /// `Some(reason)` iff the robot REFUSED to serve its runs to the demanding
    /// party (the [`DemandAuthorizer`](super::demand_authorizer::DemandAuthorizer)
    /// gate denied it). Mutually exclusive with a non-empty `runs` (a refusal
    /// serves NOTHING), and paired with a NON-settled `completeness` so the empty
    /// list can never be read as an absence claim. `#[serde(default,
    /// skip_serializing_if)]`: an authorized reply OMITS the key entirely.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub error: Option<String>,
}

/// A run the robot gathered but could not turn into a [`RunEntry`], and why.
///
/// The WIRE half of the serve side's skip: `run_artifacts` builds these from its
/// own richer local type, so the reason an operator reads on the desk is the one
/// the robot logged.
///
/// `reason` is free TEXT rather than a code, deliberately: the conditions are
/// filesystem outcomes (an OS error string, a length against a cap) whose value
/// is in the detail, and a desk that pattern-matched on codes would be making
/// decisions this payload does not support. It renders; it does not dispatch.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct UndescribableRun {
    /// The run's identity in the canonical [`format_run_id`] text form — the same
    /// spelling a served [`RunEntry`] carries, so a desk can tell "this is the run
    /// I cached" from "this is a new one".
    pub run_id: String,
    /// Why it could not be described, in operator-readable text.
    pub reason: String,
}

/// Assemble a [`RunsReply`] from raw entries — pure (oracle-testable). Dedups by
/// `run_id`, serves the survivors in `(run_started_at_ns, run_id)` order, and
/// stamps the current wire version, robot and completeness verdict.
///
/// The order is keyed on the CANONICAL zero-padded `run_id` text, whose
/// lexicographic order is its numeric order, so the served order is deterministic
/// across machines and runs (Principle #7) without decoding anything.
///
/// # The dedup is GLOBAL, and the identity order is what makes it so
///
/// [`Vec::dedup_by`] only ever compares NEIGHBOURS, so a dedup performed under the
/// DISPLAY order is a silent no-op on exactly the shape that matters: two records
/// of ONE identity carrying different start instants with another run sorting
/// BETWEEN them. `(id-2, t30), (id-1, t20), (id-2, t10)` orders as
/// `id-2, id-1, id-2` and BOTH `id-2` entries survive — one run served twice,
/// breaking the one-entry-per-run contract the desk's fetch-once-per-run cache is
/// built on. The list is therefore put in IDENTITY order first, where equal
/// `run_id`s are adjacent by construction, and only then re-ordered for display.
///
/// # The survivor rule: the greatest `run_started_at_ns` wins
///
/// Equivalently: **of the entries claiming one identity, the one that would sort
/// LAST in the served order is the one served**, so the rule needs no second key
/// and cannot disagree with the order this reply promises.
///
/// Note first what the rule is NOT deciding. `run_started_at_ns` is IMMUTABLE per
/// run — it is half of the identity a
/// [`RunWatcher`](super::run_registry::RunWatcher) learns for its own run — so two
/// entries sharing a `run_id` and differing in it are not two valid observations
/// of one run taken at different moments; one of them is stale or corrupt. Neither
/// is authoritative in a truth sense, and what this function owes its caller is
/// exactly one entry per identity, chosen the same way every time. The choice is
/// documented so a reader knows WHICH, rather than inheriting whatever a sort
/// happened to leave adjacent.
///
/// It is keyed on CONTENT rather than on the caller's iteration order, and that is
/// the load-bearing half: the serve side feeds entries derived from
/// [`RunGather::records`](super::run_registry::RunGather), which come out of a
/// `BTreeMap<u128, _>`, so arrival order is already erased there and an
/// order-keyed rule would be silently arbitrary on the one path that actually
/// runs. A content key makes the reply a function of the entry SET, which is the
/// stronger Principle #7 guarantee and cannot be perturbed by an upstream
/// collection's iteration accident.
///
/// The direction matches the rest of the module, where a LATER start is
/// consistently the more current picture of the machine: the registry's own gather
/// dedups last-write-wins so a `Live` → `Ending` flip converges, and
/// `is_successor_record` tests "started no EARLIER than ours". Keeping the
/// greatest start keeps that bias.
///
/// MERGING two records of one run is deliberately NOT done here — that is the
/// gather's job (its last-write-wins union plus
/// [`fold_batch_state`](super::run_registry::fold_batch_state)'s "`Ending` wins
/// within a batch"), already paid upstream. Re-deciding it here would be a second
/// copy of one policy, free to disagree with the first.
///
/// Residue, stated rather than hidden: entries tying on BOTH `run_id` and
/// `run_started_at_ns` are indistinguishable to the ordering key, so the first one
/// supplied survives (the sort is stable). Ordering two contradictory records on
/// their artifacts to break that tie would be inventing a truth this function does
/// not have.
/// # A withheld run DEMOTES the verdict, and that is done HERE
///
/// `completeness` is the GATHER's verdict, and it is not the only thing that can
/// make the served list short: a run the fold could not describe is withheld
/// after the gather settled. Left alone, the sharp case is a machine whose only
/// run is undescribable serving an empty list beside `Settled` — a confident
/// "this robot is running nothing" about a robot running something.
///
/// So the demotion is applied by the ASSEMBLER rather than by its callers: the
/// invariant is "a reply that withheld runs is not an absence claim", and a rule
/// enforced at each call site is one a later call site can forget.
/// [`RunsReply::validate`] enforces the same pairing on DECODE, so it is a
/// property of the wire and not a habit of this serve.
///
/// An ALREADY-`Incomplete` verdict keeps its counts untouched — they are facts
/// about which writers were heard and this function measured nothing new. A
/// demotion FROM `Settled` reports [`RunsCompleteness::not_established`]'s true
/// zeros, because `Settled` is a unit variant carrying no counts to preserve and
/// inventing some would be the one thing worse than reporting none: the zeros
/// mean "no writer arithmetic backs THIS verdict", which is exactly true — the
/// verdict is about a run the fold withheld, not about a writer it failed to
/// hear. WHICH kind of non-claim the reader is holding is carried by
/// `undescribable`, exactly as `not_established` leaves that discrimination to
/// [`RunsReply::error`].
pub fn build_runs_reply(
    robot: &str,
    entries: impl IntoIterator<Item = RunEntry>,
    undescribable: Vec<UndescribableRun>,
    completeness: RunsCompleteness,
) -> RunsReply {
    let mut runs: Vec<RunEntry> = entries.into_iter().collect();
    // IDENTITY order, so equal `run_id`s are ADJACENT and the neighbour-only
    // `dedup_by` can see them at all. Within one identity the GREATEST start sorts
    // FIRST (descending), because `dedup_by` retains the first of each adjacent
    // group — that is the survivor rule, expressed as an ordering.
    runs.sort_by(|a, b| {
        a.run_id
            .cmp(&b.run_id)
            .then_with(|| b.run_started_at_ns.cmp(&a.run_started_at_ns))
    });
    runs.dedup_by(|a, b| a.run_id == b.run_id);
    // DISPLAY order, over a list now holding exactly one entry per identity.
    runs.sort_by(|a, b| {
        a.run_started_at_ns
            .cmp(&b.run_started_at_ns)
            .then_with(|| a.run_id.cmp(&b.run_id))
    });
    // A withheld run makes the answer a strict subset, which is precisely what
    // `Incomplete` means — so the gather's verdict is DOWNGRADED, never upgraded.
    let completeness = if undescribable.is_empty() {
        completeness
    } else {
        match completeness {
            // The gather's own counts survive; only the VERDICT moves.
            RunsCompleteness::Settled => RunsCompleteness::not_established(),
            already_incomplete => already_incomplete,
        }
    };
    RunsReply {
        version: RUNS_WIRE_VERSION,
        // hot-path-alloc-ok: cold — runs reply construction, built once per remote
        // runs GET on the control plane, never per frame.
        robot: robot.to_string(),
        runs,
        completeness,
        undescribable,
        // A served answer is never a refusal (mutually exclusive with runs).
        error: None,
    }
}

impl RunsReply {
    /// An explicit REFUSAL reply — the robot's
    /// [`DemandAuthorizer`](super::demand_authorizer::DemandAuthorizer) gate
    /// denied this party's runs GET. Carries NO runs, an `error` reason, and a
    /// [`RunsCompleteness::not_established`] verdict, so a refused desk gets an
    /// EXPLICIT refusal it cannot confuse with "this robot is running nothing".
    pub fn refused(robot: &str, reason: impl Into<String>) -> Self {
        RunsReply {
            version: RUNS_WIRE_VERSION,
            // hot-path-alloc-ok: cold — one refusal reply per denied runs GET on
            // the control plane, never per frame.
            robot: robot.to_string(),
            runs: Vec::new(),
            completeness: RunsCompleteness::not_established(),
            // A refusal withholds EVERYTHING, and this field is the one that could
            // leak it: an `UndescribableRun` carries a run ID and a reason, so a
            // refusal listing them would tell a DENIED party which runs the machine
            // is executing — strictly more than the topic enumeration `catalog`'s
            // refusal withholds. There is no path from here to a non-empty vector
            // (the constructor takes no such argument), and `RunsReply::validate`
            // refuses the pairing on DECODE so a peer cannot mint one either.
            undescribable: Vec::new(),
            error: Some(reason.into()),
        }
    }

    /// Check every invariant the serve-side constructors enforce — the gate
    /// [`decode_runs_reply`] applies so WIRE data cannot carry a state the local
    /// constructors forbid.
    ///
    /// # Why decoding must validate at all
    ///
    /// [`RunEntry::new`] is documented as the ONLY minting path precisely so a run
    /// whose `graph.yaml` could not be read is SKIPPED rather than served as an
    /// entry carrying an empty document the desk would render as a graph with no
    /// nodes. That guarantee holds for values this binary CONSTRUCTS and says
    /// nothing about values it RECEIVES: a peer is not bound by our constructors,
    /// and `serde` enforces only the shape. Without this gate every invariant the
    /// type advertises is decorative the moment the value arrives over a network —
    /// which is the whole class the mandatory `completeness` field was made
    /// mandatory to avoid (a defaulted `Settled` from a peer that never
    /// made the claim).
    ///
    /// # What it rejects
    ///
    /// - a `run_id` that is not the EXACT canonical [`format_run_id`] text (a
    ///   text-compared identity must have one spelling — `parse_run_id`'s
    ///   tolerance is for an operator typing `--run`, not for the wire);
    /// - any entry [`RunEntry::new`] would refuse (empty graph name, empty or
    ///   oversized artifact) — through the same shared check that constructor
    ///   uses, so the two sides cannot drift;
    /// - one identity appearing TWICE, the contract
    ///   [`build_runs_reply`] guarantees and the desk's fetch-once-per-run cache
    ///   rests on;
    /// - a refusal (`error: Some`) paired with rows, with a `Settled` verdict, or
    ///   with a non-empty `undescribable` — the three exclusions
    ///   [`RunsReply::refused`] exists to uphold. The settled pairing is the sharp
    ///   one for CORRECTNESS: it would tell a refused desk, with confidence, that the
    ///   robot is idle. The `undescribable` pairing is the sharp one for
    ///   DISCLOSURE: those entries carry run IDS and reasons, so a refusal that
    ///   included them would enumerate to a denied party exactly what the refusal
    ///   exists to withhold — a `runs` GET is a whole-machine question, and its
    ///   refusal must serve nothing enumerable at all;
    /// - a `Settled` verdict paired with a non-empty `undescribable` — the same
    ///   hazard through the other door, and the reason [`build_runs_reply`]
    ///   demotes. A robot whose ONE run could not be described would otherwise
    ///   serve an empty list a settled verdict licenses as "running nothing";
    /// - a run named in BOTH `runs` and `undescribable`, or named twice in
    ///   `undescribable` — one identity, one verdict. A run cannot be
    ///   simultaneously served and withheld, and a desk keying its cache on
    ///   `run_id` would have to pick one arbitrarily;
    /// - a non-canonical `run_id` or an empty `reason` on an `undescribable`
    ///   entry — a withheld run must be as identifiable as a served one, and a
    ///   reason-less skip is the unexplained hole this field exists to close.
    ///
    /// # What it deliberately does NOT reject
    ///
    /// ORDER. [`build_runs_reply`] serves `(run_started_at_ns, run_id)` order, but
    /// a document that is correct and merely ordered differently carries no
    /// forbidden state — the desk can order rows however it renders them. A
    /// decoder's job is to refuse states that would mislead, not to enforce a
    /// presentation choice, and refusing here would reject semantically perfect
    /// answers for nothing.
    ///
    /// # Errors
    ///
    /// [`RunsDecodeError::Invalid`], naming the offending entry and reason.
    pub fn validate(&self) -> Result<(), RunsDecodeError> {
        // hot-path-alloc-ok: cold — validation of one control-plane reply per
        // remote runs GET, never per frame. (Applies to every `format!` below.)
        if let Some(reason) = &self.error {
            if !self.runs.is_empty() {
                return Err(RunsDecodeError::Invalid {
                    detail: format!(
                        "a refusal serves NO runs, but this reply pairs error {reason:?} with \
                         {} run(s)",
                        self.runs.len()
                    ),
                });
            }
            if self.completeness.is_settled() {
                return Err(RunsDecodeError::Invalid {
                    detail: format!(
                        "a refusal establishes NOTHING, but this reply pairs error {reason:?} \
                         with a settled verdict — an empty list beside it would read as \
                         'this robot is running nothing'"
                    ),
                });
            }
            if !self.undescribable.is_empty() {
                return Err(RunsDecodeError::Invalid {
                    detail: format!(
                        "a refusal serves NOTHING ENUMERABLE, but this reply pairs error \
                         {reason:?} with {} withheld run(s) — their ids and reasons would \
                         enumerate to a party the robot just denied exactly what the refusal \
                         exists to withhold",
                        self.undescribable.len()
                    ),
                });
            }
        }
        let mut seen: BTreeSet<&str> = BTreeSet::new();
        for entry in &self.runs {
            if !is_canonical_run_id(&entry.run_id) {
                return Err(RunsDecodeError::Invalid {
                    detail: format!(
                        "run_id {:?} is not the canonical `0x` + 32 lowercase hex form",
                        entry.run_id
                    ),
                });
            }
            if let Err(e) =
                check_run_entry_fields(&entry.graph_name, &entry.graph_yaml, &entry.run_json)
            {
                return Err(RunsDecodeError::Invalid {
                    detail: format!("run {}: {e}", entry.run_id),
                });
            }
            if !seen.insert(entry.run_id.as_str()) {
                return Err(RunsDecodeError::Invalid {
                    detail: format!(
                        "run {} appears more than once — one entry per run is the contract \
                         the desk's fetch-once-per-run cache rests on",
                        entry.run_id
                    ),
                });
            }
        }
        if !self.undescribable.is_empty() && self.completeness.is_settled() {
            return Err(RunsDecodeError::Invalid {
                detail: format!(
                    "{} run(s) were withheld as undescribable, but this reply is SETTLED — a \
                     short list beside a settled verdict reads as 'this robot is running \
                     nothing'",
                    self.undescribable.len()
                ),
            });
        }
        for withheld in &self.undescribable {
            if !is_canonical_run_id(&withheld.run_id) {
                return Err(RunsDecodeError::Invalid {
                    detail: format!(
                        "undescribable run_id {:?} is not the canonical `0x` + 32 lowercase \
                         hex form",
                        withheld.run_id
                    ),
                });
            }
            if withheld.reason.is_empty() {
                return Err(RunsDecodeError::Invalid {
                    detail: format!(
                        "run {} is reported undescribable with no reason — an unexplained \
                         hole is what this field exists to close",
                        withheld.run_id
                    ),
                });
            }
            if !seen.insert(withheld.run_id.as_str()) {
                return Err(RunsDecodeError::Invalid {
                    detail: format!(
                        "run {} is both served and withheld (or withheld twice) — one \
                         identity carries one verdict",
                        withheld.run_id
                    ),
                });
            }
        }
        Ok(())
    }
}

/// Encode a [`RunsReply`] to its wire bytes (versioned JSON). A serialization
/// failure is logged + yields empty bytes, which the desk decodes as "no answer"
/// — never a panic on the gateway.
pub fn encode_runs_reply(reply: &RunsReply) -> Vec<u8> {
    match serde_json::to_vec(reply) {
        Ok(bytes) => bytes,
        Err(e) => {
            tracing::error!(error = %e, "runs reply serialization failed");
            // hot-path-alloc-ok: cold — runs encode error path (control plane).
            Vec::new()
        }
    }
}

/// Why a runs reply could not be used — the reason the desk warns ONCE about
/// (mirrors [`CatalogDecodeError`] / [`SchemaDecodeError`]). Both variants mean
/// the desk learned NOTHING about this robot's runs, so it must render "could not
/// establish", never "no runs".
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum RunsDecodeError {
    /// Parsed but carries a wire version this binary does not understand (an
    /// older/newer robot — a `cerulion_q` protocol skew).
    UnknownVersion { got: u32, supported: u32 },
    /// The bytes did not parse as a runs payload at all — corruption, a foreign
    /// reply on the key, or a document missing a MANDATORY field (`runs` /
    /// `completeness`). Carries the serde error class for the log.
    Malformed { detail: String },
    /// The bytes parsed as a well-formed payload of a KNOWN version, but carry a
    /// state the serve-side constructors cannot produce — a noncanonical
    /// `run_id`, an empty or oversized artifact, one identity twice, or a refusal
    /// paired with rows / a settled verdict.
    ///
    /// A DISTINCT variant rather than a reuse of [`Self::Malformed`], because the
    /// two are different conditions with different remedies: `Malformed` says the
    /// peer sent something that is not this payload at all, while `Invalid` says
    /// it sent this payload and got it wrong — a serve-side bug, a version skew
    /// no version field admitted to, or a peer that is not what it claims. An
    /// operator reading the log needs to know which.
    Invalid { detail: String },
}

impl std::fmt::Display for RunsDecodeError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            RunsDecodeError::UnknownVersion { got, supported } => write!(
                f,
                "unknown runs wire version {got} (this binary supports {supported})"
            ),
            RunsDecodeError::Malformed { detail } => {
                write!(f, "runs reply did not parse as JSON ({detail})")
            }
            RunsDecodeError::Invalid { detail } => {
                write!(
                    f,
                    "runs reply carries a state the serve side forbids ({detail})"
                )
            }
        }
    }
}

/// Decode wire bytes into a [`RunsReply`], or classify WHY they could not be used
/// ([`RunsDecodeError`]).
///
/// Three gates, in this order: the bytes must PARSE, they must carry a version
/// this binary understands, and — [`RunsReply::validate`] — they must not carry a
/// state the serve-side constructors forbid. The version check precedes
/// validation deliberately: a skewed peer's document should be reported as the
/// protocol skew it is, which is actionable, rather than as whichever invariant
/// the newer shape happened to trip.
pub fn decode_runs_reply(bytes: &[u8]) -> Result<RunsReply, RunsDecodeError> {
    let reply: RunsReply =
        serde_json::from_slice(bytes).map_err(|e| RunsDecodeError::Malformed {
            // hot-path-alloc-ok: cold — runs decode error path (control plane).
            detail: e.to_string(),
        })?;
    if reply.version != RUNS_WIRE_VERSION {
        return Err(RunsDecodeError::UnknownVersion {
            got: reply.version,
            supported: RUNS_WIRE_VERSION,
        });
    }
    reply.validate()?;
    Ok(reply)
}

/// A robot that ANSWERED the `runs` verb with something this binary could not
/// use — a wire-version skew, corruption, or a document its own serve side
/// forbids.
///
/// It exists because the three [`RunsGatherOutcome`] arms have three DIFFERENT
/// remedies and only one of them is "wait": a robot that did not answer may
/// answer later, while a robot that answered UNUSABLY will answer identically
/// forever — the fix is a redeploy, and nothing a desk does can bring the reply
/// closer. Reporting the two as one silent absence sends an operator to wait out
/// a condition that has no clock.
///
/// `reason` is free TEXT rather than a code, for the same reason
/// [`UndescribableRun::reason`] is: it renders; it does not dispatch.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct UnusableRunsAnswer {
    /// The robot whose reply could not be used, as the desk QUERIED it (the
    /// announce identity in the explicit key — the same spelling a served
    /// [`RunsReply::robot`] carries after normalization).
    pub robot: String,
    /// Why it could not be used, in operator-readable text.
    pub reason: String,
}

/// The outcome of gathering ONE robot's runs replies (arrival order) — the FIRST
/// decodable reply wins; else, if any reply arrived but none decoded, the reason
/// to warn ONCE (the last decode error); else `NoReply`. Mirrors
/// [`CatalogGatherOutcome`] / [`SchemaGatherOutcome`]. Oracle-tested.
///
/// # `NoReply` is NOT an absence claim, and that separation is the whole point
///
/// The three arms answer three different questions, and only one of them says
/// anything about what the robot is running:
///
/// - `Decoded` — the robot ANSWERED. Whether an empty `runs` list means "nothing
///   is running" is then carried by [`RunsCompleteness`] INSIDE the reply, which
///   is where that judgement belongs.
/// - `Ignored` — the robot answered with something unusable (a wire skew,
///   corruption, or a document its own serve side forbids). The desk learned
///   nothing; a loud warn, once per robot.
/// - `NoReply` — the robot did not answer at all. Reachable on any robot whose
///   binary predates this verb, AND on a robot that HAS the verb but declares no
///   query surface (a Strict ingress-only gateway — see
///   `NetworkManager::has_query_surface`), which is a robot that can be running
///   something and cannot be asked about it.
///
/// So a caller must never fabricate a reply for a `NoReply` robot: a synthesized
/// `Settled`-empty answer would assert "this robot is running nothing" on exactly
/// the two shapes that cannot support the claim. The absence of a reply IS the
/// non-claim, and the netd/desk layers key on reply PRESENCE per robot.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum RunsGatherOutcome {
    /// The FIRST decodable reply — use it. It may itself carry a non-settled
    /// [`RunsCompleteness`] or a refusal `error`; both are valid answers the
    /// caller surfaces rather than second-guesses.
    Decoded(RunsReply),
    /// At least one reply arrived but NONE decoded — warn ONCE with the reason (a
    /// protocol skew, corruption, or a serve side that broke its own invariants).
    Ignored(RunsDecodeError),
    /// No reply arrived. The SILENT back-compat path — an older robot with no
    /// `runs` verb, or one with no query surface at all. Contributes NOTHING to
    /// the gather (never an empty reply).
    NoReply,
}

/// PURE: classify a robot's runs replies (arrival order) into a
/// [`RunsGatherOutcome`] — the first decodable reply wins; else the last decode
/// error; else `NoReply`. Oracle-tested. The discovery caller emits at most one
/// `warn!` per robot from the `Ignored` arm — never per reply, never silent on a
/// genuine skew.
pub fn gather_runs_outcome<'a>(replies: impl IntoIterator<Item = &'a [u8]>) -> RunsGatherOutcome {
    let mut last_err: Option<RunsDecodeError> = None;
    for bytes in replies {
        match decode_runs_reply(bytes) {
            Ok(reply) => return RunsGatherOutcome::Decoded(reply),
            Err(e) => last_err = Some(e),
        }
    }
    match last_err {
        Some(reason) => RunsGatherOutcome::Ignored(reason),
        None => RunsGatherOutcome::NoReply,
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::collections::BTreeMap;

    /// The `cerulion_q` chunk is DISTINCT from the demand / announce chunks — the
    /// load-bearing property that keeps a query from ever matching a
    /// `cerulion_lv/**` bridge watch or a `cerulion_ann/**` presence gather.
    #[test]
    fn cerulion_q_prefix_is_a_distinct_top_level_chunk() {
        for other in ["cerulion_lv", "cerulion_ann", "cerulion"] {
            assert_ne!(CERULION_Q_PREFIX, other);
            assert!(!CERULION_Q_PREFIX.starts_with(&format!("{other}/")));
            assert!(!other.starts_with(&format!("{CERULION_Q_PREFIX}/")));
        }
    }

    #[test]
    fn queryable_key_oracle() {
        assert_eq!(queryable_key("go2"), "cerulion_q/go2/**");
        assert_eq!(
            queryable_key("robot.alpha_1"),
            "cerulion_q/robot.alpha_1/**"
        );
    }

    #[test]
    fn demand_reply_and_selector_oracle() {
        // Reply key carries the concrete identity + canonical (leading-slash) tail.
        assert_eq!(
            demand_reply_key("go2", "/utlidar/cloud"),
            "cerulion_q/go2/demand/utlidar/cloud"
        );
        assert_eq!(demand_reply_key("go2", "/imu"), "cerulion_q/go2/demand/imu");
        // Explicit selector == reply key (exact route).
        assert_eq!(
            demand_selector("go2", "/utlidar/cloud"),
            "cerulion_q/go2/demand/utlidar/cloud"
        );
        // Wildcard fallback intersects the reply key (routes on a healthy link).
        assert_eq!(
            demand_selector("*", "/utlidar/cloud"),
            "cerulion_q/*/demand/utlidar/cloud"
        );
    }

    #[test]
    fn catalog_selector_oracle() {
        assert_eq!(catalog_selector("go2"), "cerulion_q/go2/catalog");
        assert_eq!(
            catalog_selector("robot.alpha_1"),
            "cerulion_q/robot.alpha_1/catalog"
        );
    }

    /// The schema/catalog FETCH-KEY is built from the
    /// ANNOUNCE identity chunk (`robot`) — a hostname-keyed query would break
    /// multi-robot routing and any robot whose hostname ≠ its declared identity.
    /// Hand oracle: the `robot` argument
    /// (harvested from the announce space by the desk) IS the key's robot chunk.
    #[test]
    fn schema_selector_keys_on_the_announce_identity() {
        assert_eq!(
            schema_selector("go2", "unitree_go/LowState"),
            "cerulion_q/go2/schema/unitree_go/LowState"
        );
        // A package-less bare type is still keyed on
        // the announce identity, one requested chunk.
        assert_eq!(
            schema_selector("go2", "MyState"),
            "cerulion_q/go2/schema/MyState"
        );
        // The announce identity is verbatim the key's robot chunk — never the
        // hostname.
        assert_eq!(
            schema_selector("robot.alpha_1", "pkg/T"),
            "cerulion_q/robot.alpha_1/schema/pkg/T"
        );
    }

    #[test]
    fn parse_query_verb_demand_oracle() {
        // Explicit-robot demand key.
        assert_eq!(
            parse_query_verb("cerulion_q/go2/demand/utlidar/cloud"),
            Some(QueryVerb::Demand {
                topic: "/utlidar/cloud".to_string()
            })
        );
        // Wildcard-robot demand key (the robot chunk is ignored).
        assert_eq!(
            parse_query_verb("cerulion_q/*/demand/imu"),
            Some(QueryVerb::Demand {
                topic: "/imu".to_string()
            })
        );
        // Deep nested canonical.
        assert_eq!(
            parse_query_verb("cerulion_q/robot.alpha_1/demand/deep/nested/name"),
            Some(QueryVerb::Demand {
                topic: "/deep/nested/name".to_string()
            })
        );
        // Round-trip: a reply key built by `demand_reply_key` parses back to the
        // same canonical topic.
        assert_eq!(
            parse_query_verb(&demand_reply_key("go2", "/deep/nested/name")),
            Some(QueryVerb::Demand {
                topic: "/deep/nested/name".to_string()
            })
        );
    }

    #[test]
    fn parse_query_verb_catalog_oracle() {
        assert_eq!(
            parse_query_verb("cerulion_q/go2/catalog"),
            Some(QueryVerb::Catalog)
        );
        assert_eq!(
            parse_query_verb(&catalog_selector("robot.alpha_1")),
            Some(QueryVerb::Catalog)
        );
        // A catalog selector with a stray tail is NOT the bare catalog verb.
        assert_eq!(parse_query_verb("cerulion_q/go2/catalog/extra"), None);
    }

    #[test]
    fn parse_query_verb_rejects_malformed_and_unknown() {
        // No verb chunk.
        assert_eq!(parse_query_verb("cerulion_q/go2"), None);
        // Wrong namespace.
        assert_eq!(parse_query_verb("cerulion_lv/go2/demand/imu"), None);
        assert_eq!(parse_query_verb("cerulion_ann/go2/imu"), None);
        // Unknown verb (a future verb from a newer peer).
        assert_eq!(parse_query_verb("cerulion_q/go2/subscribe/imu"), None);
        // A verb whose name merely starts with `demand` (no leading-slash tail).
        assert_eq!(parse_query_verb("cerulion_q/go2/demandx"), None);
    }

    #[test]
    fn build_catalog_reply_sorts_dedups_and_stamps_version() {
        let reply = build_catalog_reply(
            "go2",
            vec![
                CatalogEntry {
                    topic: "/imu".to_string(),
                    schema_hash: Some(0x1111),
                    schema_name: Some("sensor_msgs/Imu".to_string()),
                    provenance: CatalogProvenance::Runtime,
                    producer_count: None,
                    liveness: None,
                },
                CatalogEntry {
                    topic: "/camera/image".to_string(),
                    schema_hash: None,
                    schema_name: None,
                    provenance: CatalogProvenance::Boot,
                    producer_count: None,
                    liveness: None,
                },
                // Duplicate canonical — the first (sorted-stable) wins.
                CatalogEntry {
                    topic: "/imu".to_string(),
                    schema_hash: Some(0x2222),
                    schema_name: Some("other/Type".to_string()),
                    provenance: CatalogProvenance::Runtime,
                    producer_count: None,
                    liveness: None,
                },
            ],
        );
        assert_eq!(reply.version, CATALOG_WIRE_VERSION);
        assert_eq!(reply.robot, "go2");
        // Sorted canonical, deduped to two entries.
        assert_eq!(reply.entries.len(), 2);
        assert_eq!(reply.entries[0].topic, "/camera/image");
        assert_eq!(reply.entries[0].schema_hash, None);
        assert_eq!(reply.entries[0].schema_name, None);
        assert_eq!(reply.entries[0].provenance, CatalogProvenance::Boot);
        assert_eq!(reply.entries[1].topic, "/imu");
        assert_eq!(reply.entries[1].schema_hash, Some(0x1111));
        // The first (sorted-stable) /imu wins, carrying ITS schema_name.
        assert_eq!(
            reply.entries[1].schema_name.as_deref(),
            Some("sensor_msgs/Imu")
        );
        assert_eq!(reply.entries[1].provenance, CatalogProvenance::Runtime);
    }

    #[test]
    fn catalog_reply_encode_decode_round_trip() {
        let reply = CatalogReply {
            version: CATALOG_WIRE_VERSION,
            robot: "go2".to_string(),
            entries: vec![
                CatalogEntry {
                    topic: "/camera/image".to_string(),
                    schema_hash: None,
                    schema_name: None,
                    provenance: CatalogProvenance::Boot,
                    producer_count: None,
                    liveness: None,
                },
                CatalogEntry {
                    topic: "/utlidar/cloud".to_string(),
                    schema_hash: Some(0xDEAD_BEEF_CAFE_F00D),
                    schema_name: Some("sensor_msgs/PointCloud2".to_string()),
                    provenance: CatalogProvenance::Runtime,
                    producer_count: None,
                    liveness: None,
                },
            ],
            error: None,
        };
        let bytes = encode_catalog_reply(&reply);
        assert!(!bytes.is_empty());
        // Decodes byte-for-byte back to the same value (hand oracle == input).
        assert_eq!(decode_catalog_reply(&bytes), Ok(reply));
    }

    /// The `error` field is `skip_serializing_if = "Option::is_none"`,
    /// so an AUTHORIZED (deny-nothing / AllowAll) catalog OMITS it entirely — the
    /// serialized JSON is BYTE-IDENTICAL to the older catalog (no `"error"` key).
    /// The anti-regression contract: adding the refusal carrier must not perturb the
    /// wire the discovery path (`topic list`) already relies on.
    #[test]
    fn allow_all_catalog_omits_error_field_byte_identical() {
        let served = build_catalog_reply(
            "go2",
            [CatalogEntry {
                topic: "/imu".to_string(),
                schema_hash: Some(7),
                schema_name: None,
                provenance: CatalogProvenance::Runtime,
                producer_count: None,
                liveness: None,
            }],
        );
        assert_eq!(served.error, None);
        let json = String::from_utf8(encode_catalog_reply(&served)).expect("utf8");
        assert!(
            !json.contains("error"),
            "an authorized catalog must NOT carry an `error` key: {json}"
        );
        // Round-trips (the absent field decodes to None via `#[serde(default)]`).
        assert_eq!(decode_catalog_reply(json.as_bytes()), Ok(served));
    }

    /// The ADDITIVE contract for `liveness`, both directions, against
    /// literal-JSON hand oracles (never a self-compare).
    ///
    /// 1. A catalog with NO liveness serializes with NO `liveness` key — the
    ///    pre-liveness wire, byte-for-byte, so a robot with observation disabled
    ///    or a WAN-served catalog perturbs nothing.
    /// 2. A NEW desk decoding an OLD robot's reply (a literal JSON
    ///    document without the key, which is what an older robot really puts on the wire) yields
    ///    `liveness: None` — UNKNOWN, never a fabricated value.
    /// 3. A liveness-carrying entry serializes the nested object and round-trips
    ///    back to the same value.
    #[test]
    fn liveness_is_additive_in_both_directions() {
        use crate::transport::liveness::TopicLiveness;

        // (1) absent ⇒ no key at all.
        let bare = build_catalog_reply(
            "go2",
            [CatalogEntry {
                topic: "/imu".to_string(),
                schema_hash: Some(7),
                schema_name: None,
                provenance: CatalogProvenance::Runtime,
                producer_count: Some(1),
                liveness: None,
            }],
        );
        let json = String::from_utf8(encode_catalog_reply(&bare)).expect("utf8");
        assert!(
            !json.contains("liveness"),
            "a catalog with no observation must not carry a `liveness` key: {json}"
        );
        assert_eq!(
            json,
            r#"{"version":1,"robot":"go2","entries":[{"topic":"/imu","schema_hash":7,"schema_name":null,"provenance":"runtime","producer_count":1}]}"#,
            "the no-liveness wire must be byte-identical to the pre-liveness document"
        );

        // (2) an OLD robot's literal document decodes with liveness UNKNOWN.
        let old_wire = r#"{"version":1,"robot":"go2","entries":[{"topic":"/imu","schema_hash":7,"schema_name":null,"provenance":"runtime","producer_count":1}]}"#;
        let decoded = decode_catalog_reply(old_wire.as_bytes()).expect("old wire decodes");
        assert_eq!(
            decoded.entries[0].liveness, None,
            "an old robot's entry must read UNKNOWN, never a fabricated liveness"
        );
        assert_eq!(decoded, bare, "old wire == the no-liveness reply");

        // (3) a liveness-carrying entry serializes + round-trips.
        let observed = build_catalog_reply(
            "go2",
            [CatalogEntry {
                topic: "/imu".to_string(),
                schema_hash: Some(7),
                schema_name: None,
                provenance: CatalogProvenance::Runtime,
                producer_count: Some(1),
                liveness: Some(TopicLiveness {
                    last_frame_age_ms: Some(42),
                    observed_for_ms: 9_000,
                    frames_observed: 180,
                    rate_estimate: None,
                }),
            }],
        );
        let json = String::from_utf8(encode_catalog_reply(&observed)).expect("utf8");
        assert!(
            json.contains(r#""liveness":{"last_frame_age_ms":42,"observed_for_ms":9000,"frames_observed":180}"#),
            "the liveness object must serialize with its three fields: {json}"
        );
        assert_eq!(decode_catalog_reply(json.as_bytes()), Ok(observed));
    }

    /// The OTHER direction of the additive contract: an OLD DESK
    /// decoding a NEW ROBOT's reply.
    ///
    /// `liveness_is_additive_in_both_directions` covers new-desk/old-robot (a
    /// missing key defaults to `None`). This covers new-robot/old-desk, which
    /// rests on a DIFFERENT serde property: that an UNKNOWN key is IGNORED rather
    /// than rejected. Nothing in the source says so — it is serde's default, and
    /// a later `#[serde(deny_unknown_fields)]` on `CatalogEntry` (a plausible
    /// hardening edit) would silently turn every new-robot catalog into a decode
    /// failure for every desk in the field.
    ///
    /// So the test decodes a real liveness-carrying document into a PRE-889-SHAPED
    /// stand-in — a hand-written struct with exactly the fields that existed
    /// before this change, standing in for the binary an un-upgraded desk runs —
    /// and asserts the known fields survive.
    #[test]
    fn an_old_desk_ignores_the_new_liveness_key_rather_than_failing() {
        // EXACTLY the pre-liveness `CatalogEntry`/`CatalogReply` shape.
        #[derive(Debug, Deserialize, PartialEq)]
        struct PreCer889Entry {
            topic: String,
            schema_hash: Option<u64>,
            schema_name: Option<String>,
            provenance: CatalogProvenance,
            #[serde(default)]
            producer_count: Option<u32>,
        }
        #[derive(Debug, Deserialize)]
        struct PreCer889Reply {
            version: u32,
            robot: String,
            entries: Vec<PreCer889Entry>,
        }

        // A document a liveness-reporting robot really puts on the wire.
        let new_wire = String::from_utf8(encode_catalog_reply(&build_catalog_reply(
            "go2",
            [CatalogEntry {
                topic: "/utlidar/cloud_deskewed".to_string(),
                schema_hash: Some(7),
                schema_name: Some("sensor_msgs/PointCloud2".to_string()),
                provenance: CatalogProvenance::Runtime,
                producer_count: Some(1),
                liveness: Some(TopicLiveness {
                    last_frame_age_ms: Some(48),
                    observed_for_ms: 30_000,
                    frames_observed: 600,
                    rate_estimate: None,
                }),
            }],
        )))
        .expect("utf8");
        assert!(
            new_wire.contains(r#""liveness""#),
            "precondition: the document really carries the new key: {new_wire}"
        );

        let old: PreCer889Reply =
            serde_json::from_str(&new_wire).expect("an old desk must DECODE a new robot's catalog");
        assert_eq!(old.version, CATALOG_WIRE_VERSION);
        assert_eq!(old.robot, "go2");
        assert_eq!(
            old.entries,
            vec![PreCer889Entry {
                topic: "/utlidar/cloud_deskewed".to_string(),
                schema_hash: Some(7),
                schema_name: Some("sensor_msgs/PointCloud2".to_string()),
                provenance: CatalogProvenance::Runtime,
                producer_count: Some(1),
            }],
            "every field the old desk knows survives; the one it does not is IGNORED"
        );
    }

    /// A REFUSED catalog is an EXPLICIT refusal — empty entries + an
    /// `error` reason on the wire (never a silent empty catalog the desk cannot tell
    /// from a topicless robot). Hand oracle: entries empty, error carries the reason.
    #[test]
    fn refused_catalog_carries_explicit_error_and_no_entries() {
        let refused = CatalogReply::refused("go2", "not authorized (account/pairing)");
        assert!(refused.entries.is_empty(), "a refusal serves NO topics");
        assert_eq!(
            refused.error.as_deref(),
            Some("not authorized (account/pairing)")
        );
        // Encodes with the `error` key present, and round-trips.
        let json = String::from_utf8(encode_catalog_reply(&refused)).expect("utf8");
        assert!(
            json.contains("error"),
            "a refusal MUST carry the error key: {json}"
        );
        assert_eq!(decode_catalog_reply(json.as_bytes()), Ok(refused));
    }

    /// A REFUSED schema reply — no `docs`, an `error` reason
    /// (wire-shape-identical to `not_found`, so no wire change). The desk sees an
    /// explicit refusal, not silence.
    #[test]
    fn refused_schema_carries_explicit_error_and_no_docs() {
        let refused = SchemaReply::refused("go2", "pkg/Type", "not authorized (account/pairing)");
        assert!(refused.docs.is_empty(), "a refusal serves NO docs");
        assert_eq!(
            refused.error.as_deref(),
            Some("not authorized (account/pairing)")
        );
        assert_eq!(refused.requested, "pkg/Type");
        let bytes = encode_schema_reply(&refused);
        assert_eq!(decode_schema_reply(&bytes), Ok(refused));
    }

    #[test]
    fn decode_rejects_unknown_version() {
        let mut reply = CatalogReply {
            version: CATALOG_WIRE_VERSION + 1,
            robot: "go2".to_string(),
            entries: vec![],
            error: None,
        };
        let bytes = encode_catalog_reply(&reply);
        // A newer wire version → the ACTIONABLE UnknownVersion error (got vs
        // supported), never a mis-decode.
        assert_eq!(
            decode_catalog_reply(&bytes),
            Err(CatalogDecodeError::UnknownVersion {
                got: CATALOG_WIRE_VERSION + 1,
                supported: CATALOG_WIRE_VERSION,
            })
        );
        // The current version decodes.
        reply.version = CATALOG_WIRE_VERSION;
        let bytes = encode_catalog_reply(&reply);
        assert_eq!(decode_catalog_reply(&bytes), Ok(reply));
    }

    #[test]
    fn decode_rejects_garbage_bytes() {
        assert!(matches!(
            decode_catalog_reply(b"not json at all"),
            Err(CatalogDecodeError::Malformed { .. })
        ));
        assert!(matches!(
            decode_catalog_reply(&[]),
            Err(CatalogDecodeError::Malformed { .. })
        ));
    }

    /// A reply omitting the optional `schema_hash` field still decodes (the
    /// `#[serde(default)]` tolerance — a forward-compat guard for a producer that
    /// serializes a leaner entry).
    #[test]
    fn decode_tolerates_absent_schema_hash() {
        let json = format!(
            r#"{{"version":{CATALOG_WIRE_VERSION},"robot":"go2","entries":[{{"topic":"/imu","provenance":"runtime"}}]}}"#
        );
        let decoded = decode_catalog_reply(json.as_bytes()).expect("decodes without schema_hash");
        assert_eq!(decoded.entries.len(), 1);
        assert_eq!(decoded.entries[0].schema_hash, None);
        // schema_name is `#[serde(default)]` too — an old robot's reply
        // (no `schema_name` key) decodes to `None`, never a parse failure.
        assert_eq!(decoded.entries[0].schema_name, None);
        // `producer_count` is `#[serde(default)]`: an old robot's reply (no
        // `producer_count` key) decodes to `None`, never a parse failure. The desk
        // then renders that row with no liveness affordance.
        assert_eq!(decoded.entries[0].producer_count, None);
        assert_eq!(decoded.entries[0].provenance, CatalogProvenance::Runtime);
    }

    /// The `producer_count` liveness field round-trips through encode/decode
    /// and preserves the DISTINCT states the desk sidebar affordance keys off — a
    /// live producer (`Some(n)`), a registered-but-DEAD route (`Some(0)`, the
    /// `/uslam/cloud_map` row below), and an un-probed topic (`None`, degrades to the plain
    /// rendering). Hand oracle on the exact per-entry values, never a self-compare.
    #[test]
    fn producer_count_liveness_round_trips_all_three_states() {
        let reply = CatalogReply {
            version: CATALOG_WIRE_VERSION,
            robot: "go2".to_string(),
            entries: vec![
                // A live 20 Hz topic — one producer.
                CatalogEntry {
                    topic: "/utlidar/cloud".to_string(),
                    schema_hash: Some(0x1),
                    schema_name: Some("sensor_msgs/PointCloud2".to_string()),
                    provenance: CatalogProvenance::Runtime,
                    producer_count: Some(1),
                    liveness: None,
                },
                // A registered-but-DEAD route (a DDS route / boot announce whose
                // producer is not publishing).
                CatalogEntry {
                    topic: "/uslam/cloud_map".to_string(),
                    schema_hash: None,
                    schema_name: Some("sensor_msgs/PointCloud2".to_string()),
                    provenance: CatalogProvenance::Boot,
                    producer_count: Some(0),
                    liveness: None,
                },
                // An un-probed topic (an older robot, or a probe-less serve path).
                CatalogEntry {
                    topic: "/legacy".to_string(),
                    schema_hash: None,
                    schema_name: None,
                    provenance: CatalogProvenance::Runtime,
                    producer_count: None,
                    liveness: None,
                },
            ],
            error: None,
        };
        let bytes = encode_catalog_reply(&reply);
        let decoded = decode_catalog_reply(&bytes).expect("round-trips");
        // The whole reply round-trips (structural), and each state is preserved.
        assert_eq!(decoded, reply);
        assert_eq!(decoded.entries[0].producer_count, Some(1), "live producer");
        assert_eq!(decoded.entries[1].producer_count, Some(0), "dead route");
        assert_eq!(decoded.entries[2].producer_count, None, "un-probed");
    }

    /// `gather_catalog_outcome` — the pure decision the discovery caller warns
    /// off. Hand oracle vectors covering every arm: decodable-first-wins,
    /// unknown-version → `Ignored` (warn once), garbage → `Ignored` (warn once),
    /// and no-reply → silent `NoReply`.
    #[test]
    fn gather_catalog_outcome_oracle() {
        let good = encode_catalog_reply(&CatalogReply {
            version: CATALOG_WIRE_VERSION,
            robot: "go2".to_string(),
            entries: vec![CatalogEntry {
                topic: "/imu".to_string(),
                schema_hash: Some(7),
                schema_name: Some("sensor_msgs/Imu".to_string()),
                provenance: CatalogProvenance::Runtime,
                producer_count: None,
                liveness: None,
            }],
            error: None,
        });
        let unknown = encode_catalog_reply(&CatalogReply {
            version: CATALOG_WIRE_VERSION + 1,
            robot: "go2".to_string(),
            entries: vec![],
            error: None,
        });
        let garbage = b"not a catalog".to_vec();

        // No replies at all → silent fallback (an older robot / no query surface).
        assert_eq!(
            gather_catalog_outcome(std::iter::empty::<&[u8]>()),
            CatalogGatherOutcome::NoReply
        );

        // A decodable reply → Decoded (its topics are used).
        match gather_catalog_outcome([good.as_slice()]) {
            CatalogGatherOutcome::Decoded(reply) => {
                assert_eq!(reply.entries.len(), 1);
                assert_eq!(reply.entries[0].topic, "/imu");
            }
            other => panic!("expected Decoded, got {other:?}"),
        }

        // First decodable reply wins even after an undecodable one arrived.
        match gather_catalog_outcome([garbage.as_slice(), good.as_slice()]) {
            CatalogGatherOutcome::Decoded(reply) => assert_eq!(reply.robot, "go2"),
            other => panic!("expected Decoded (first decodable wins), got {other:?}"),
        }

        // A reply arrived but only an UNKNOWN VERSION → Ignored (warn once).
        assert_eq!(
            gather_catalog_outcome([unknown.as_slice()]),
            CatalogGatherOutcome::Ignored(CatalogDecodeError::UnknownVersion {
                got: CATALOG_WIRE_VERSION + 1,
                supported: CATALOG_WIRE_VERSION,
            })
        );

        // A reply arrived but only GARBAGE → Ignored (warn once).
        assert!(matches!(
            gather_catalog_outcome([garbage.as_slice()]),
            CatalogGatherOutcome::Ignored(CatalogDecodeError::Malformed { .. })
        ));
    }

    // ------------------------------ schema verb ------------------------------

    #[test]
    fn schema_selector_oracle() {
        // Qualified `pkg/Type`.
        assert_eq!(
            schema_selector("go2", "vendor_msgs/Widget"),
            "cerulion_q/go2/schema/vendor_msgs/Widget"
        );
        assert_eq!(
            schema_selector("robot.alpha_1", "unitree_go/SportModeState"),
            "cerulion_q/robot.alpha_1/schema/unitree_go/SportModeState"
        );
        // A package-less bare `Name` selector.
        assert_eq!(
            schema_selector("go2", "MyState"),
            "cerulion_q/go2/schema/MyState"
        );
    }

    #[test]
    fn parse_query_verb_schema_oracle() {
        // Qualified two-chunk `pkg/Type`.
        assert_eq!(
            parse_query_verb("cerulion_q/go2/schema/vendor_msgs/Widget"),
            Some(QueryVerb::Schema {
                requested: "vendor_msgs/Widget".to_string()
            })
        );
        // A package-less bare `Name` (one chunk) is
        // ACCEPTED (a `cerulion schema create <Name>` workspace type).
        assert_eq!(
            parse_query_verb("cerulion_q/go2/schema/MyState"),
            Some(QueryVerb::Schema {
                requested: "MyState".to_string()
            })
        );
        // Round-trip: selectors built by `schema_selector` (qualified AND bare)
        // parse back to the same `requested`.
        assert_eq!(
            parse_query_verb(&schema_selector("go2", "unitree_go/SportModeState")),
            Some(QueryVerb::Schema {
                requested: "unitree_go/SportModeState".to_string()
            })
        );
        assert_eq!(
            parse_query_verb(&schema_selector("go2", "MyState")),
            Some(QueryVerb::Schema {
                requested: "MyState".to_string()
            })
        );
        // The robot chunk is ignored (a wildcard-robot key parses the same).
        assert_eq!(
            parse_query_verb("cerulion_q/*/schema/pkg/Type"),
            Some(QueryVerb::Schema {
                requested: "pkg/Type".to_string()
            })
        );
    }

    #[test]
    fn parse_query_verb_schema_rejects_malformed() {
        // Bare `schema` (no `/` tail) is NOT a schema verb.
        assert_eq!(parse_query_verb("cerulion_q/go2/schema"), None);
        // Three chunks (a `pkg/msg/Type` 3-segment name is NOT accepted — the
        // desk always sends the 2-segment `pkg/Type` or the bare `Name` form).
        assert_eq!(parse_query_verb("cerulion_q/go2/schema/pkg/msg/Type"), None);
        // Empty chunk (bare empty tail, or an empty pkg/ty half).
        assert_eq!(parse_query_verb("cerulion_q/go2/schema/"), None);
        assert_eq!(parse_query_verb("cerulion_q/go2/schema//Type"), None);
        assert_eq!(parse_query_verb("cerulion_q/go2/schema/pkg/"), None);
        // A verb whose name merely starts with `schema`.
        assert_eq!(parse_query_verb("cerulion_q/go2/schemax/pkg/Type"), None);
        assert_eq!(parse_query_verb("cerulion_q/go2/schemax/Name"), None);
    }

    /// The four verbs never collide — a `schema` key is never a `catalog`, a
    /// `runs` or a `demand`, and vice versa (the dispatch is unambiguous).
    #[test]
    fn verbs_are_mutually_exclusive() {
        assert!(matches!(
            parse_query_verb("cerulion_q/go2/catalog"),
            Some(QueryVerb::Catalog)
        ));
        assert!(matches!(
            parse_query_verb("cerulion_q/go2/schema/pkg/Type"),
            Some(QueryVerb::Schema { .. })
        ));
        assert!(matches!(
            parse_query_verb("cerulion_q/go2/demand/imu"),
            Some(QueryVerb::Demand { .. })
        ));
        assert!(matches!(
            parse_query_verb("cerulion_q/go2/runs"),
            Some(QueryVerb::Runs)
        ));
    }

    fn doc(qualified: &str, deps: &[&str]) -> SchemaDoc {
        SchemaDoc {
            qualified: qualified.to_string(),
            encoding: SchemaEncoding::Msg,
            text: format!("# {qualified}\n"),
            deps: deps.iter().map(|d| d.to_string()).collect(),
        }
    }

    fn doc_map(docs: Vec<SchemaDoc>) -> BTreeMap<String, SchemaDoc> {
        docs.into_iter().map(|d| (d.qualified.clone(), d)).collect()
    }

    /// A type nesting 2 custom types + 1 builtin → the closure is the requested
    /// type FIRST, then the 2 custom deps (the builtin, absent from the served
    /// set, is correctly OMITTED — the desk already has it). Hand oracle.
    #[test]
    fn collect_schema_closure_two_custom_one_builtin_omitted() {
        // Requested `acme/Widget` nests `acme/Gadget`, `acme/Gizmo` (both custom,
        // in the served set) and `std_msgs/Header` (builtin — NOT in the set, so
        // NOT a dep, correctly omitted from the closure).
        let map = doc_map(vec![
            doc("acme/Widget", &["acme/Gadget", "acme/Gizmo"]),
            doc("acme/Gadget", &[]),
            doc("acme/Gizmo", &[]),
        ]);
        let closure = collect_schema_closure("acme/Widget", &map).expect("requested is present");
        let names: Vec<&str> = closure.iter().map(|d| d.qualified.as_str()).collect();
        // Requested FIRST, then its deps (BFS, deterministic).
        assert_eq!(names, vec!["acme/Widget", "acme/Gadget", "acme/Gizmo"]);
    }

    /// A cycle (A → B → A) is walked exactly once each — no infinite loop.
    #[test]
    fn collect_schema_closure_is_cycle_safe() {
        let map = doc_map(vec![doc("p/A", &["p/B"]), doc("p/B", &["p/A"])]);
        let closure = collect_schema_closure("p/A", &map).expect("present");
        let names: Vec<&str> = closure.iter().map(|d| d.qualified.as_str()).collect();
        assert_eq!(names, vec!["p/A", "p/B"]);
        // A self-loop is also bounded.
        let selfloop = doc_map(vec![doc("p/S", &["p/S"])]);
        let c = collect_schema_closure("p/S", &selfloop).expect("present");
        assert_eq!(c.len(), 1);
    }

    /// A shared nested type reachable via two paths appears exactly ONCE (deduped).
    #[test]
    fn collect_schema_closure_dedups_diamond() {
        // A → B, A → C, B → D, C → D. D appears once.
        let map = doc_map(vec![
            doc("p/A", &["p/B", "p/C"]),
            doc("p/B", &["p/D"]),
            doc("p/C", &["p/D"]),
            doc("p/D", &[]),
        ]);
        let closure = collect_schema_closure("p/A", &map).expect("present");
        let names: Vec<&str> = closure.iter().map(|d| d.qualified.as_str()).collect();
        assert_eq!(names, vec!["p/A", "p/B", "p/C", "p/D"]);
    }

    /// An unknown requested type → `None` (the caller replies `not_found`).
    #[test]
    fn collect_schema_closure_unknown_is_none() {
        let map = doc_map(vec![doc("p/A", &[])]);
        assert!(collect_schema_closure("p/Missing", &map).is_none());
        assert!(collect_schema_closure("p/A", &BTreeMap::new()).is_none());
    }

    /// A dep naming a type NOT in the served set is silently skipped (never
    /// dangles) — the walk still returns the reachable members.
    #[test]
    fn collect_schema_closure_dangling_dep_is_skipped() {
        // `p/A` names a builtin-or-missing `absent/Type` that is NOT in the set.
        let map = doc_map(vec![doc("p/A", &["absent/Type", "p/B"]), doc("p/B", &[])]);
        let closure = collect_schema_closure("p/A", &map).expect("present");
        let names: Vec<&str> = closure.iter().map(|d| d.qualified.as_str()).collect();
        assert_eq!(names, vec!["p/A", "p/B"]);
    }

    /// Hand-oracle JSON round-trip for a FOUND reply — encode→decode is the
    /// input value, and the JSON carries the requested-first doc order + no error.
    #[test]
    fn schema_reply_found_encode_decode_round_trip() {
        let reply = SchemaReply::found(
            "go2",
            "acme/Widget",
            vec![
                SchemaDoc {
                    qualified: "acme/Widget".to_string(),
                    encoding: SchemaEncoding::Msg,
                    text: "acme/Gadget gadget\nstd_msgs/Header header\n".to_string(),
                    deps: vec!["acme/Gadget".to_string()],
                },
                SchemaDoc {
                    qualified: "acme/Gadget".to_string(),
                    encoding: SchemaEncoding::Yaml,
                    text: "acme/Gadget:\n  fields:\n    x: float64\n".to_string(),
                    deps: vec![],
                },
            ],
        );
        assert_eq!(reply.version, SCHEMA_WIRE_VERSION);
        assert_eq!(reply.error, None);
        let bytes = encode_schema_reply(&reply);
        assert!(!bytes.is_empty());
        assert_eq!(decode_schema_reply(&bytes), Ok(reply));
    }

    /// A NOT-FOUND reply round-trips with empty docs + a reason (the explicit
    /// structured error the desk surfaces).
    #[test]
    fn schema_reply_not_found_round_trip() {
        let reply = SchemaReply::not_found("go2", "acme/Nope", "robot has no schema 'acme/Nope'");
        assert!(reply.docs.is_empty());
        assert_eq!(
            reply.error.as_deref(),
            Some("robot has no schema 'acme/Nope'")
        );
        let bytes = encode_schema_reply(&reply);
        assert_eq!(decode_schema_reply(&bytes), Ok(reply));
    }

    #[test]
    fn schema_decode_rejects_unknown_version_and_garbage() {
        let bad_version = SchemaReply {
            version: SCHEMA_WIRE_VERSION + 1,
            robot: "go2".to_string(),
            requested: "p/T".to_string(),
            docs: vec![],
            error: Some("x".to_string()),
        };
        let bytes = encode_schema_reply(&bad_version);
        assert_eq!(
            decode_schema_reply(&bytes),
            Err(SchemaDecodeError::UnknownVersion {
                got: SCHEMA_WIRE_VERSION + 1,
                supported: SCHEMA_WIRE_VERSION,
            })
        );
        assert!(matches!(
            decode_schema_reply(b"not json"),
            Err(SchemaDecodeError::Malformed { .. })
        ));
    }

    /// A reply omitting the optional `deps`/`error`/`docs` fields still decodes
    /// (forward-compat: an old robot serializes a leaner doc).
    #[test]
    fn schema_decode_tolerates_absent_optional_fields() {
        let json = format!(
            r#"{{"version":{SCHEMA_WIRE_VERSION},"robot":"go2","requested":"p/T",
               "docs":[{{"qualified":"p/T","encoding":"msg","text":"x\n"}}]}}"#
        );
        let decoded = decode_schema_reply(json.as_bytes()).expect("decodes without deps/error");
        assert_eq!(decoded.docs.len(), 1);
        assert!(decoded.docs[0].deps.is_empty());
        assert_eq!(decoded.error, None);
    }

    /// `gather_schema_outcome` — first decodable wins; unknown-version/garbage →
    /// Ignored (warn once); no reply → silent NoReply. Hand oracle vectors.
    #[test]
    fn gather_schema_outcome_oracle() {
        let good = encode_schema_reply(&SchemaReply::found("go2", "p/T", vec![doc("p/T", &[])]));
        let unknown = encode_schema_reply(&SchemaReply {
            version: SCHEMA_WIRE_VERSION + 1,
            robot: "go2".to_string(),
            requested: "p/T".to_string(),
            docs: vec![],
            error: None,
        });
        let garbage = b"nope".to_vec();

        assert_eq!(
            gather_schema_outcome(std::iter::empty::<&[u8]>()),
            SchemaGatherOutcome::NoReply
        );
        match gather_schema_outcome([good.as_slice()]) {
            SchemaGatherOutcome::Decoded(r) => assert_eq!(r.requested, "p/T"),
            other => panic!("expected Decoded, got {other:?}"),
        }
        // First decodable wins even after garbage.
        match gather_schema_outcome([garbage.as_slice(), good.as_slice()]) {
            SchemaGatherOutcome::Decoded(r) => assert_eq!(r.robot, "go2"),
            other => panic!("expected Decoded, got {other:?}"),
        }
        assert_eq!(
            gather_schema_outcome([unknown.as_slice()]),
            SchemaGatherOutcome::Ignored(SchemaDecodeError::UnknownVersion {
                got: SCHEMA_WIRE_VERSION + 1,
                supported: SCHEMA_WIRE_VERSION,
            })
        );
        assert!(matches!(
            gather_schema_outcome([garbage.as_slice()]),
            SchemaGatherOutcome::Ignored(SchemaDecodeError::Malformed { .. })
        ));
    }

    // ------------------------------- runs verb -------------------------------

    /// The canonical `run_id` text for 42 — spelled ONCE, hand-counted (30 zeros
    /// then `2a`), and used by both the format oracle and the byte oracle so a
    /// miscount fails loudly rather than agreeing with itself.
    const RUN_ID_42_TEXT: &str = "0x0000000000000000000000000000002a";

    /// A `graph.yaml` small enough to spell in a byte oracle but carrying the two
    /// characters that must survive JSON escaping (a newline and a quote).
    const SAMPLE_YAML: &str = "name: perception\nnodes: []\n";
    const SAMPLE_RUN_JSON: &str = r#"{"run_id":"0x0000000000000000000000000000002a"}"#;

    fn sample_entry() -> RunEntry {
        RunEntry::new(
            42,
            "perception",
            1_753_000_000_000_000_000,
            RunState::Live,
            SAMPLE_YAML,
            SAMPLE_RUN_JSON,
        )
        .expect("the sample entry is well-formed")
    }

    /// The fetch-key rule, applied to the new verb: the runs FETCH-KEY is built
    /// from the ANNOUNCE identity chunk, never a hostname — which is what makes
    /// attribution a property of the KEY (a phantom "this machine" row
    /// is not constructible desk-side).
    #[test]
    fn runs_selector_keys_on_the_announce_identity() {
        assert_eq!(runs_selector("go2"), "cerulion_q/go2/runs");
        assert_eq!(
            runs_selector("robot.alpha_1"),
            "cerulion_q/robot.alpha_1/runs"
        );
    }

    #[test]
    fn parse_query_verb_runs_oracle() {
        assert_eq!(
            parse_query_verb("cerulion_q/go2/runs"),
            Some(QueryVerb::Runs)
        );
        // Round-trip: a selector built by `runs_selector` parses back to the verb.
        assert_eq!(
            parse_query_verb(&runs_selector("robot.alpha_1")),
            Some(QueryVerb::Runs)
        );
        // The robot chunk is ignored (a wildcard-robot key parses the same).
        assert_eq!(parse_query_verb("cerulion_q/*/runs"), Some(QueryVerb::Runs));
    }

    /// THE forward-compat pin — the key-grammar table. `runs` takes NO tail, so
    /// every look-alike must stay a `None` NO-OP rather than being answered.
    ///
    /// This is not pedantry about typos: `runs/{run_id}` is the obvious v2 verb
    /// (fetch ONE run), and a peer that answered it with the whole list would
    /// give a newer desk a wrong answer instead of the silence its fallback is
    /// built on. The `None` is the forward-compatibility mechanism (`network.rs`
    /// makes it a `debug!` no-op), so widening this match is a wire break.
    #[test]
    fn parse_query_verb_runs_rejects_tails_and_lookalikes() {
        // A foreign verb whose name merely starts with `runs`.
        assert_eq!(parse_query_verb("cerulion_q/go2/runsx"), None);
        assert_eq!(parse_query_verb("cerulion_q/go2/runs_history"), None);
        // An empty tail.
        assert_eq!(parse_query_verb("cerulion_q/go2/runs/"), None);
        // Any tail at all — including the plausible v2 `runs/{run_id}` form.
        assert_eq!(parse_query_verb("cerulion_q/go2/runs/x"), None);
        assert_eq!(
            parse_query_verb(&format!("cerulion_q/go2/runs/{RUN_ID_42_TEXT}")),
            None
        );
        // A shorter verb that is a PREFIX of ours is not ours either.
        assert_eq!(parse_query_verb("cerulion_q/go2/run"), None);
        // The wrong namespace never dispatches, whatever the verb.
        assert_eq!(parse_query_verb("cerulion_lv/go2/runs"), None);
    }

    /// `format_run_id` is the CANONICAL text form, and it is not free-floating:
    /// the `0x0…abc` literal below is the one a run's own `run.json` carries
    /// (pinned independently in `cerulion_cli_engine::run_dir`), so this is a
    /// cross-artifact oracle rather than a self-compare. `parse_run_id` is its
    /// exact inverse, tolerant of a missing prefix and of case (what an operator
    /// types), strict about everything else.
    #[test]
    fn run_id_text_is_the_run_json_form_and_round_trips() {
        // Hand oracles, including the literal `run.json` writes for 0xabc.
        assert_eq!(format_run_id(0xabc), "0x00000000000000000000000000000abc");
        assert_eq!(format_run_id(42), RUN_ID_42_TEXT);
        assert_eq!(format_run_id(0), "0x00000000000000000000000000000000");
        assert_eq!(
            format_run_id(u128::MAX),
            "0xffffffffffffffffffffffffffffffff"
        );
        // Always 32 digits after the prefix — the property the sort relies on.
        for id in [0u128, 1, 42, 0xabc, u128::MAX, 1 << 100] {
            let text = format_run_id(id);
            assert_eq!(text.len(), 34, "0x + 32 digits: {text}");
            assert_eq!(parse_run_id(&text), Some(id), "exact inverse for {id}");
        }
        // Zero-padding makes lexicographic order == numeric order, which is what
        // lets `build_runs_reply` sort on the TEXT deterministically.
        assert!(format_run_id(2) < format_run_id(10));
        assert!(format_run_id(0xff) < format_run_id(0x100));
        // Operator tolerance: no prefix, and either case.
        assert_eq!(parse_run_id("2a"), Some(42));
        assert_eq!(parse_run_id("0X2A"), Some(42));
        assert_eq!(parse_run_id("0000000000000000000000000000002A"), Some(42));
        // Strict about everything else — `from_str_radix` alone would accept a
        // leading `+`, and a 33-digit value would silently overflow.
        assert_eq!(parse_run_id(""), None);
        assert_eq!(parse_run_id("0x"), None);
        assert_eq!(parse_run_id("+1"), None);
        assert_eq!(parse_run_id(" 2a"), None);
        assert_eq!(parse_run_id("2a "), None);
        assert_eq!(parse_run_id("zz"), None);
        assert_eq!(parse_run_id("0x1_2"), None);
        assert_eq!(parse_run_id(&"f".repeat(33)), None);
    }

    /// The wire state vocabulary MIRRORS the registry's, and the mirror is tied
    /// to it by a total `From` — so a third `RunState` variant is a compile
    /// error here, never a silent guess. The serde spelling is pinned against
    /// `RunState`'s own `Display`, so the two renderings cannot drift apart.
    #[test]
    fn run_state_wire_mirror_matches_the_registry_vocabulary() {
        assert_eq!(RunEntryState::from(RunState::Live), RunEntryState::Live);
        assert_eq!(RunEntryState::from(RunState::Ending), RunEntryState::Ending);
        for (registry, wire) in [
            (RunState::Live, RunEntryState::Live),
            (RunState::Ending, RunEntryState::Ending),
        ] {
            // The JSON spelling IS the registry's own word for the state.
            let json = serde_json::to_string(&wire).expect("serializes");
            assert_eq!(json, format!("\"{registry}\""));
            assert_eq!(wire.to_string(), registry.to_string());
            assert_eq!(
                serde_json::from_str::<RunEntryState>(&json).expect("decodes"),
                wire
            );
        }
    }

    /// The completeness verdict MIRRORS the gather's, both variants, and the
    /// refusal path's verdict is deliberately NOT settled.
    #[test]
    fn completeness_mirrors_the_gather_verdict() {
        assert_eq!(
            RunsCompleteness::from(GatherCompleteness::Settled),
            RunsCompleteness::Settled
        );
        assert_eq!(
            RunsCompleteness::from(GatherCompleteness::Incomplete {
                live_writers: 3,
                writers_heard: 1,
            }),
            RunsCompleteness::Incomplete {
                live_writers: 3,
                writers_heard: 1,
            }
        );
        assert!(RunsCompleteness::Settled.is_settled());
        assert!(!RunsCompleteness::Incomplete {
            live_writers: 3,
            writers_heard: 1,
        }
        .is_settled());
        // A refusal establishes NOTHING — never a settled empty.
        assert!(!RunsCompleteness::not_established().is_settled());
        // Both arms are the same JSON SHAPE (an object with a `kind`), so a JS
        // consumer reads one thing; the default external tagging would put a bare
        // string on the wire for `Settled`.
        assert_eq!(
            serde_json::to_string(&RunsCompleteness::Settled).expect("serializes"),
            r#"{"kind":"settled"}"#
        );
        assert_eq!(
            serde_json::to_string(&RunsCompleteness::Incomplete {
                live_writers: 2,
                writers_heard: 1,
            })
            .expect("serializes"),
            r#"{"kind":"incomplete","live_writers":2,"writers_heard":1}"#
        );
    }

    /// `RunEntry::new` is the serve side's ONLY minting path, and it refuses
    /// exactly the shapes the serve side must SKIP with a named reason — so "a missing
    /// graph.yaml becomes an empty-string entry" is not a mistake that compiles
    /// into an answer.
    #[test]
    fn run_entry_new_refuses_the_shapes_the_serve_side_must_skip() {
        // Happy path: the canonical id is minted here, and the registry's state
        // vocabulary is carried across.
        let entry = sample_entry();
        assert_eq!(entry.run_id, RUN_ID_42_TEXT);
        assert_eq!(entry.graph_name, "perception");
        assert_eq!(entry.run_started_at_ns, 1_753_000_000_000_000_000);
        assert_eq!(entry.state, RunEntryState::Live);
        assert_eq!(entry.graph_yaml, SAMPLE_YAML);
        assert_eq!(entry.run_json, SAMPLE_RUN_JSON);
        assert_eq!(
            RunEntry::new(1, "g", 0, RunState::Ending, "y", "j")
                .expect("well-formed")
                .state,
            RunEntryState::Ending
        );

        // Every refusal, each a distinct named reason.
        assert_eq!(
            RunEntry::new(1, "", 0, RunState::Live, "y", "j"),
            Err(RunEntryError::EmptyGraphName)
        );
        assert_eq!(
            RunEntry::new(1, "g", 0, RunState::Live, "", "j"),
            Err(RunEntryError::EmptyGraphYaml)
        );
        assert_eq!(
            RunEntry::new(1, "g", 0, RunState::Live, "y", ""),
            Err(RunEntryError::EmptyRunJson)
        );
        let over = "y".repeat(MAX_RUN_ARTIFACT_LEN + 1);
        assert_eq!(
            RunEntry::new(1, "g", 0, RunState::Live, over.as_str(), "j"),
            Err(RunEntryError::GraphYamlTooLong {
                len: MAX_RUN_ARTIFACT_LEN + 1
            })
        );
        assert_eq!(
            RunEntry::new(1, "g", 0, RunState::Live, "y", over.as_str()),
            Err(RunEntryError::RunJsonTooLong {
                len: MAX_RUN_ARTIFACT_LEN + 1
            })
        );
        // The cap is inclusive at exactly the limit (both sides pinned).
        let at_cap = "y".repeat(MAX_RUN_ARTIFACT_LEN);
        assert!(RunEntry::new(1, "g", 0, RunState::Live, at_cap.as_str(), "j").is_ok());
        assert!(RunEntry::new(1, "g", 0, RunState::Live, "y", at_cap.as_str()).is_ok());
        // Every reason renders something an operator can act on.
        for err in [
            RunEntryError::EmptyGraphName,
            RunEntryError::EmptyGraphYaml,
            RunEntryError::EmptyRunJson,
            RunEntryError::GraphYamlTooLong { len: 9 },
            RunEntryError::RunJsonTooLong { len: 9 },
        ] {
            assert!(!err.to_string().is_empty(), "{err:?} renders");
        }
    }

    /// THE byte oracle — the exact document a served runs reply puts on the wire,
    /// hand-written (never a self-compare), then decoded back to the same value.
    ///
    /// It pins the FIELD ORDER of both structs (serde serializes in declaration
    /// order, so swapping two fields changes these bytes), the internally-tagged
    /// completeness shape, the canonical `run_id` text, and the JSON escaping of a
    /// verbatim artifact carrying newlines and quotes.
    #[test]
    fn runs_reply_encode_decode_byte_oracle() {
        let reply = build_runs_reply(
            "go2",
            [sample_entry()],
            Vec::new(),
            RunsCompleteness::Settled,
        );
        let json = String::from_utf8(encode_runs_reply(&reply)).expect("utf8");
        assert_eq!(
            json,
            r#"{"version":1,"robot":"go2","runs":[{"run_id":"0x0000000000000000000000000000002a","graph_name":"perception","run_started_at_ns":1753000000000000000,"state":"live","graph_yaml":"name: perception\nnodes: []\n","run_json":"{\"run_id\":\"0x0000000000000000000000000000002a\"}"}],"completeness":{"kind":"settled"}}"#
        );
        // An AUTHORIZED reply omits the `error` key entirely.
        assert!(
            !json.contains("error"),
            "an authorized runs reply must NOT carry an `error` key: {json}"
        );
        assert_eq!(decode_runs_reply(json.as_bytes()), Ok(reply));

        // The INCOMPLETE arm's document, also hand-written.
        let incomplete = build_runs_reply(
            "go2",
            [],
            Vec::new(),
            RunsCompleteness::Incomplete {
                live_writers: 2,
                writers_heard: 1,
            },
        );
        let json = String::from_utf8(encode_runs_reply(&incomplete)).expect("utf8");
        assert_eq!(
            json,
            r#"{"version":1,"robot":"go2","runs":[],"completeness":{"kind":"incomplete","live_writers":2,"writers_heard":1}}"#
        );
        assert_eq!(decode_runs_reply(json.as_bytes()), Ok(incomplete));
    }

    /// `build_runs_reply` sorts by `(run_started_at_ns, run_id)`, dedups by
    /// `run_id`, and stamps the version + robot + verdict. Hand oracle on the
    /// resulting order — a machine serving two concurrent runs must serve them in
    /// the same order every time (Principle #7).
    #[test]
    fn build_runs_reply_sorts_dedups_and_stamps() {
        let entry = |id: u128, started: u64| {
            RunEntry::new(id, "g", started, RunState::Live, "y", "j").expect("well-formed")
        };
        let reply = build_runs_reply(
            "go2",
            vec![
                entry(7, 300),
                entry(2, 100),
                // Same start instant as run 2 — the `run_id` breaks the tie.
                entry(1, 100),
                // A duplicate identity (two replies folded) — deduped to one. The
                // two carry the SAME start, so which survivor rule applies is
                // immaterial here; the rule itself is pinned by
                // `build_runs_reply_dedups_identities_the_display_order_separates`.
                entry(2, 100),
            ],
            Vec::new(),
            RunsCompleteness::Settled,
        );
        assert_eq!(reply.version, RUNS_WIRE_VERSION);
        assert_eq!(reply.robot, "go2");
        assert_eq!(reply.completeness, RunsCompleteness::Settled);
        assert_eq!(reply.error, None);
        let ids: Vec<&str> = reply.runs.iter().map(|r| r.run_id.as_str()).collect();
        assert_eq!(
            ids,
            vec![
                format_run_id(1).as_str(),
                format_run_id(2).as_str(),
                format_run_id(7).as_str(),
            ],
            "sorted by (start, id), deduped by id"
        );
    }

    /// The global-dedup pin.
    ///
    /// [`Vec::dedup_by`] only ever compares NEIGHBOURS, so deduping under the
    /// DISPLAY order was a silent no-op on exactly the shape that matters: two
    /// records of ONE identity carrying different start instants with another run
    /// sorting BETWEEN them. The repro:
    /// `(id-2, t30), (id-1, t20), (id-2, t10)` orders as `id-2, id-1, id-2`, the
    /// two `id-2` entries are non-adjacent, and BOTH would survive — one run
    /// served TWICE, under a wire contract the desk's fetch-once-per-run cache is
    /// built on.
    ///
    /// The oracle is hand-written on all three fields that decide the answer, and
    /// it discriminates the SURVIVOR RULE rather than merely the row count: keeping
    /// the greatest `run_started_at_ns` serves `[id-1@t20, id-2@t30]`, whereas the
    /// other plausible rule — a `retain` over the display order, which keeps the
    /// FIRST in timestamp order — would serve `[id-2@t10, id-1@t20]`: a different
    /// row, a different order, and a different graph name. Both pass a row count of
    /// two, so a count-only assertion would pin nothing.
    ///
    /// The `graph_name` column is what makes the surviving ENTRY pinned rather than
    /// just its timestamp: an implementation that discarded the t30 record but
    /// reported the maximum start would still be caught.
    #[test]
    fn build_runs_reply_dedups_identities_the_display_order_separates() {
        let entry = |id: u128, started: u64, graph: &str| {
            RunEntry::new(id, graph, started, RunState::Live, "y", "j").expect("well-formed")
        };
        // That shape exactly: run 2 twice, run 1's start BETWEEN them.
        let repro = || {
            vec![
                entry(2, 30, "kept"),
                entry(1, 20, "other"),
                entry(2, 10, "dropped"),
            ]
        };

        let reply = build_runs_reply("go2", repro(), Vec::new(), RunsCompleteness::Settled);
        let rows: Vec<(&str, u64, &str)> = reply
            .runs
            .iter()
            .map(|r| {
                (
                    r.run_id.as_str(),
                    r.run_started_at_ns,
                    r.graph_name.as_str(),
                )
            })
            .collect();
        assert_eq!(
            rows,
            vec![
                (format_run_id(1).as_str(), 20, "other"),
                (format_run_id(2).as_str(), 30, "kept"),
            ],
            "one entry per identity — the GREATEST start surviving — in (start, id) \
             display order"
        );

        // The contract, asserted directly rather than inferred from the row list: an
        // identity appears EXACTLY once, and the discarded observation is GONE (not
        // merely re-ordered behind the survivor).
        assert_eq!(
            reply
                .runs
                .iter()
                .filter(|r| r.run_id == format_run_id(2))
                .count(),
            1,
            "run 2 must appear exactly once: {:?}",
            reply.runs
        );
        assert!(
            !reply.runs.iter().any(|r| r.run_started_at_ns == 10),
            "the t10 observation of run 2 must not survive: {:?}",
            reply.runs
        );

        // The survivor rule is keyed on CONTENT, not on the caller's iteration
        // order — the property the serve side rests on, since its entries come out
        // of a `BTreeMap<u128, _>` where arrival order is already erased. Every
        // permutation of the same SET must yield the identical reply.
        let mut shuffled = repro();
        shuffled.reverse();
        assert_eq!(
            build_runs_reply("go2", shuffled, Vec::new(), RunsCompleteness::Settled),
            reply,
            "the reply must be a function of the entry SET, not its order"
        );
        let rotated = {
            let mut v = repro();
            v.swap(0, 1);
            v
        };
        assert_eq!(
            build_runs_reply("go2", rotated, Vec::new(), RunsCompleteness::Settled),
            reply,
            "the reply must be a function of the entry SET, not its order"
        );
    }

    /// THE distinction the verb exists for: an EMPTY answer is an absence claim
    /// ONLY when the gather settled. The two documents differ in nothing but the
    /// verdict, and the desk's rendering rule keys off exactly that bit.
    #[test]
    fn an_empty_settled_answer_is_an_absence_claim_and_an_incomplete_one_is_not() {
        let settled = build_runs_reply("go2", [], Vec::new(), RunsCompleteness::Settled);
        let unsettled = build_runs_reply(
            "go2",
            [],
            Vec::new(),
            RunsCompleteness::Incomplete {
                live_writers: 1,
                writers_heard: 0,
            },
        );
        assert!(settled.runs.is_empty() && unsettled.runs.is_empty());
        assert!(
            settled.completeness.is_settled(),
            "a settled empty means: this machine is running nothing"
        );
        assert!(
            !unsettled.completeness.is_settled(),
            "an unsettled empty means: could not establish — NEVER 'no runs'"
        );
        // The bit survives the wire in both directions.
        for reply in [settled, unsettled] {
            let settled_before = reply.completeness.is_settled();
            let decoded = decode_runs_reply(&encode_runs_reply(&reply)).expect("round-trips");
            assert_eq!(decoded.completeness.is_settled(), settled_before);
            assert_eq!(decoded, reply);
        }
    }

    /// A REFUSED runs reply is an EXPLICIT refusal: no runs, an `error` reason,
    /// and a verdict that forbids reading the empty list as "no runs". A refusal
    /// that came back `Settled` would be strictly worse than silence — it would
    /// tell an unpaired desk, with confidence, that the robot is idle.
    #[test]
    fn refused_runs_carries_error_no_runs_and_a_non_settled_verdict() {
        let refused = RunsReply::refused("go2", "not authorized (account/pairing)");
        assert!(refused.runs.is_empty(), "a refusal serves NO runs");
        assert_eq!(
            refused.error.as_deref(),
            Some("not authorized (account/pairing)")
        );
        assert!(
            !refused.completeness.is_settled(),
            "a refusal establishes NOTHING about the robot's runs"
        );
        assert_eq!(refused.version, RUNS_WIRE_VERSION);
        let json = String::from_utf8(encode_runs_reply(&refused)).expect("utf8");
        assert!(
            json.contains("error"),
            "a refusal MUST carry the error key: {json}"
        );
        assert_eq!(decode_runs_reply(json.as_bytes()), Ok(refused));
    }

    /// Build one served entry with a hand-set identity.
    fn served(run_id: u128) -> RunEntry {
        RunEntry::new(
            run_id,
            "g",
            1_000,
            RunState::Live,
            "name: g\n",
            "{\"k\":1}\n",
        )
        .expect("mint")
    }

    /// Build one withheld entry with a hand-set identity.
    fn withheld(run_id: u128, reason: &str) -> UndescribableRun {
        UndescribableRun {
            run_id: format_run_id(run_id),
            reason: reason.to_string(),
        }
    }

    /// THE headline of the withheld-run class: a SETTLED gather whose runs were
    /// withheld must NOT be served as a settled answer.
    ///
    /// The sharp shape is the one asserted first — a machine whose ONE run could
    /// not be described serves an EMPTY list, and a settled verdict beside it is a
    /// confident "this robot is running nothing" about a robot running something.
    /// The partial shape is the same defect one row wider: a strict subset served
    /// as if it were the whole picture.
    #[test]
    fn a_withheld_run_is_never_served_beside_a_settled_verdict() {
        // TOTAL: every run withheld.
        let total = build_runs_reply(
            "go2",
            [],
            vec![withheld(1, "graph.yaml could not be read (No such file)")],
            RunsCompleteness::Settled,
        );
        assert!(total.runs.is_empty());
        assert!(
            !total.completeness.is_settled(),
            "an empty list whose runs were WITHHELD is not an absence claim: {:?}",
            total.completeness
        );
        assert_eq!(
            total.undescribable.len(),
            1,
            "and the reader is told WHY it is not settled"
        );
        assert_eq!(total.error, None, "a withheld run is not a refusal");
        total.validate().expect("a demoted reply is well formed");

        // PARTIAL: one served, one withheld — the same demotion.
        let partial = build_runs_reply(
            "go2",
            [served(2)],
            vec![withheld(3, "run.json is not a regular file")],
            RunsCompleteness::Settled,
        );
        assert_eq!(partial.runs.len(), 1);
        assert!(
            !partial.completeness.is_settled(),
            "a STRICT SUBSET is not the whole picture either"
        );
        partial.validate().expect("well formed");

        // ANTI-TAUTOLOGY: with nothing withheld, a settled gather stays settled.
        let clean = build_runs_reply("go2", [served(4)], Vec::new(), RunsCompleteness::Settled);
        assert!(
            clean.completeness.is_settled(),
            "the demotion must be caused by the withholding, not applied always"
        );
        assert!(clean.undescribable.is_empty());
        let json = String::from_utf8(encode_runs_reply(&clean)).expect("utf8");
        assert!(
            !json.contains("undescribable"),
            "a healthy reply OMITS the key: {json}"
        );
    }

    /// An ALREADY-`Incomplete` gather keeps its own counts through the demotion —
    /// they are facts about which writers were heard, and the withholding measured
    /// no writers at all.
    #[test]
    fn a_withheld_run_never_rewrites_the_gathers_own_writer_counts() {
        let already = RunsCompleteness::Incomplete {
            live_writers: 3,
            writers_heard: 2,
        };
        let reply = build_runs_reply("go2", [], vec![withheld(5, "boom")], already);
        assert_eq!(
            reply.completeness, already,
            "an incomplete verdict passes through untouched"
        );
    }

    /// A REFUSAL serves nothing ENUMERABLE — not even the ids of runs it could
    /// not describe.
    ///
    /// The withheld list is the one field on this reply that can leak: an
    /// `UndescribableRun` carries a run ID and a reason, so a refusal listing them
    /// tells a DENIED party which runs the machine is executing — strictly more
    /// than the topic enumeration `catalog`'s refusal already withholds. The
    /// constructor cannot mint that shape; this pins the DECODE side, which is
    /// where a buggy or hostile peer's bytes arrive.
    #[test]
    fn decode_refuses_a_refusal_that_enumerates_the_runs_it_withheld() {
        // Hand-built: `refused` has no argument that could produce this.
        let mut leaky = RunsReply::refused("go2", "not authorized (account/pairing)");
        leaky.undescribable = vec![
            withheld(1, "graph.yaml could not be read"),
            withheld(2, "run.json is not a regular file"),
        ];
        // It satisfies BOTH pre-existing refusal exclusions — no rows, not settled
        // — so only the new one can catch it.
        assert!(leaky.runs.is_empty());
        assert!(!leaky.completeness.is_settled());

        match decode_runs_reply(&encode_runs_reply(&leaky)) {
            Err(RunsDecodeError::Invalid { detail }) => assert!(
                detail.contains("NOTHING ENUMERABLE") && detail.contains("withheld"),
                "the refusal must name the disclosure it is refusing: {detail}"
            ),
            other => {
                panic!("a refusal enumerating the runs it withheld must be refused: {other:?}")
            }
        }

        // ANTI-TAUTOLOGY: the constructor's own reply still decodes, so the new
        // exclusion did not simply outlaw every refusal.
        let clean = RunsReply::refused("go2", "not authorized (account/pairing)");
        assert!(
            clean.undescribable.is_empty(),
            "the constructor pins it empty"
        );
        assert_eq!(
            decode_runs_reply(&encode_runs_reply(&clean)).expect("a refusal decodes"),
            clean
        );
    }

    /// The decode side refuses every state the assembler cannot mint — so the
    /// invariant is a property of the WIRE, not a habit of one serve.
    #[test]
    fn decode_refuses_a_withheld_run_paired_with_a_settled_or_duplicate_verdict() {
        // A hand-built SETTLED reply carrying a withheld run (what a buggy or
        // hostile peer would send).
        let mut settled_with_withheld =
            build_runs_reply("go2", [], Vec::new(), RunsCompleteness::Settled);
        settled_with_withheld.undescribable = vec![withheld(1, "unreadable")];
        let bytes = encode_runs_reply(&settled_with_withheld);
        match decode_runs_reply(&bytes) {
            Err(RunsDecodeError::Invalid { detail }) => assert!(
                detail.contains("SETTLED") && detail.contains("withheld"),
                "the refusal must name the pairing: {detail}"
            ),
            other => panic!("a settled reply carrying a withheld run must be refused: {other:?}"),
        }

        // One identity cannot be both served and withheld.
        let mut both = build_runs_reply("go2", [served(2)], Vec::new(), RunsCompleteness::Settled);
        both.completeness = RunsCompleteness::not_established();
        both.undescribable = vec![withheld(2, "unreadable")];
        match decode_runs_reply(&encode_runs_reply(&both)) {
            Err(RunsDecodeError::Invalid { detail }) => {
                assert!(detail.contains("served and withheld"), "detail: {detail}")
            }
            other => panic!("one identity carries one verdict: {other:?}"),
        }

        // A withheld run must be as identifiable as a served one…
        let mut noncanonical =
            build_runs_reply("go2", [], Vec::new(), RunsCompleteness::not_established());
        noncanonical.undescribable = vec![UndescribableRun {
            run_id: "0xABC".to_string(),
            reason: "unreadable".to_string(),
        }];
        assert!(matches!(
            decode_runs_reply(&encode_runs_reply(&noncanonical)),
            Err(RunsDecodeError::Invalid { .. })
        ));

        // …and must carry the reason that is the whole point of the field.
        let mut reasonless =
            build_runs_reply("go2", [], Vec::new(), RunsCompleteness::not_established());
        reasonless.undescribable = vec![withheld(3, "")];
        match decode_runs_reply(&encode_runs_reply(&reasonless)) {
            Err(RunsDecodeError::Invalid { detail }) => {
                assert!(detail.contains("no reason"), "detail: {detail}")
            }
            other => panic!("a reason-less skip is an unexplained hole: {other:?}"),
        }
    }

    #[test]
    fn decode_rejects_unknown_runs_version_and_garbage() {
        let mut reply = build_runs_reply("go2", [], Vec::new(), RunsCompleteness::Settled);
        reply.version = RUNS_WIRE_VERSION + 1;
        let bytes = encode_runs_reply(&reply);
        assert_eq!(
            decode_runs_reply(&bytes),
            Err(RunsDecodeError::UnknownVersion {
                got: RUNS_WIRE_VERSION + 1,
                supported: RUNS_WIRE_VERSION,
            })
        );
        // The current version decodes.
        reply.version = RUNS_WIRE_VERSION;
        assert_eq!(decode_runs_reply(&encode_runs_reply(&reply)), Ok(reply));
        // Non-JSON and empty bytes are classified, never mis-decoded.
        assert!(matches!(
            decode_runs_reply(b"not json at all"),
            Err(RunsDecodeError::Malformed { .. })
        ));
        assert!(matches!(
            decode_runs_reply(&[]),
            Err(RunsDecodeError::Malformed { .. })
        ));
        // Both reasons render (the desk logs one of them, once per robot).
        assert!(RunsDecodeError::UnknownVersion {
            got: 2,
            supported: 1
        }
        .to_string()
        .contains("unknown runs wire version 2"));
        assert!(RunsDecodeError::Malformed {
            detail: "x".to_string()
        }
        .to_string()
        .contains("did not parse"));
    }

    /// `gather_runs_outcome`: the pure decision the discovery caller
    /// warns on, mirroring `gather_catalog_outcome`.
    ///
    /// The load-bearing arm is the LAST one: a robot that did not answer yields
    /// `NoReply`, which contributes NOTHING — never a synthesized empty reply. An
    /// empty `Settled` reply is a positive claim ("this robot is running
    /// nothing"), and the two shapes that reach `NoReply` — a robot predating the
    /// verb, and a Strict ingress-only gateway that declares no query surface —
    /// are exactly the ones that cannot support it.
    #[test]
    fn gather_runs_outcome_oracle() {
        let good = encode_runs_reply(&build_runs_reply(
            "go2",
            [served(0x11)],
            Vec::new(),
            RunsCompleteness::Settled,
        ));
        let mut skewed = build_runs_reply("go2", [], Vec::new(), RunsCompleteness::Settled);
        skewed.version = RUNS_WIRE_VERSION + 1;
        let skewed = encode_runs_reply(&skewed);
        let garbage = b"not a runs reply".to_vec();
        // A well-formed document of a KNOWN version that the serve side could
        // never have minted (a refusal is `Incomplete` by construction) — the
        // `Invalid` arm, which catalog/schema have no equivalent of.
        let forged = br#"{"version":1,"robot":"go2","runs":[],"completeness":{"kind":"settled"},"error":"denied"}"#.to_vec();

        // No reply at all → the SILENT non-claim. Nothing is contributed.
        assert_eq!(
            gather_runs_outcome(std::iter::empty::<&[u8]>()),
            RunsGatherOutcome::NoReply
        );

        // A decodable reply wins.
        match gather_runs_outcome([good.as_slice()]) {
            RunsGatherOutcome::Decoded(reply) => {
                assert_eq!(reply.runs.len(), 1);
                assert_eq!(reply.runs[0].run_id, format_run_id(0x11));
            }
            other => panic!("expected Decoded, got {other:?}"),
        }

        // The FIRST decodable reply wins even when undecodable ones arrive first
        // (a mixed-version LAN answering one key).
        match gather_runs_outcome([garbage.as_slice(), skewed.as_slice(), good.as_slice()]) {
            RunsGatherOutcome::Decoded(reply) => assert_eq!(reply.runs.len(), 1),
            other => panic!("expected Decoded, got {other:?}"),
        }

        // Replies arrived but NONE decoded → the LAST reason, warned once.
        assert_eq!(
            gather_runs_outcome([garbage.as_slice(), skewed.as_slice()]),
            RunsGatherOutcome::Ignored(RunsDecodeError::UnknownVersion {
                got: RUNS_WIRE_VERSION + 1,
                supported: RUNS_WIRE_VERSION,
            })
        );
        assert!(matches!(
            gather_runs_outcome([skewed.as_slice(), garbage.as_slice()]),
            RunsGatherOutcome::Ignored(RunsDecodeError::Malformed { .. })
        ));

        // A forged SETTLED refusal is `Ignored`, NOT `Decoded` — the desk must not
        // read a refusal's empty list as an absence claim (the `validate` gate,
        // reached through the classifier).
        assert!(
            matches!(
                gather_runs_outcome([forged.as_slice()]),
                RunsGatherOutcome::Ignored(RunsDecodeError::Invalid { .. })
            ),
            "a settled refusal must be refused by the gather, not surfaced"
        );
    }

    /// `completeness` and `runs` are MANDATORY on the wire — a document omitting
    /// either fails to decode rather than defaulting.
    ///
    /// This is the defaulted-absence class: the only plausible default for `completeness`
    /// is `Settled`, which is a POSITIVE absence claim ("this robot is running
    /// nothing") that a peer omitting the field never made. There is a live
    /// precedent for tolerance in this very file (`schema_hash`, `docs`, `deps`
    /// are all `#[serde(default)]`), so the divergence is deliberate and pinned
    /// here: those fields are ADDITIVE and their absence claims nothing.
    #[test]
    fn decode_refuses_a_document_missing_a_mandatory_field() {
        // A complete document decodes — the anti-tautology control, without which
        // every assertion below would pass on a decoder that rejects everything.
        let full = format!(
            r#"{{"version":{RUNS_WIRE_VERSION},"robot":"go2","runs":[],"completeness":{{"kind":"settled"}}}}"#
        );
        assert!(decode_runs_reply(full.as_bytes()).is_ok());

        // No `completeness` — must NOT default to a settled (absence-claiming) answer.
        let no_verdict = format!(r#"{{"version":{RUNS_WIRE_VERSION},"robot":"go2","runs":[]}}"#);
        assert!(matches!(
            decode_runs_reply(no_verdict.as_bytes()),
            Err(RunsDecodeError::Malformed { .. })
        ));
        // No `runs` — the payload the verb exists for; absence is corruption.
        let no_runs = format!(
            r#"{{"version":{RUNS_WIRE_VERSION},"robot":"go2","completeness":{{"kind":"settled"}}}}"#
        );
        assert!(matches!(
            decode_runs_reply(no_runs.as_bytes()),
            Err(RunsDecodeError::Malformed { .. })
        ));
        // An entry missing an artifact is corruption too — never an empty document.
        let no_yaml = format!(
            r#"{{"version":{RUNS_WIRE_VERSION},"robot":"go2","runs":[{{"run_id":"{RUN_ID_42_TEXT}","graph_name":"g","run_started_at_ns":1,"state":"live","run_json":"{{}}"}}],"completeness":{{"kind":"settled"}}}}"#
        );
        assert!(matches!(
            decode_runs_reply(no_yaml.as_bytes()),
            Err(RunsDecodeError::Malformed { .. })
        ));
        // An unknown completeness `kind` is refused rather than guessed.
        let bad_kind = format!(
            r#"{{"version":{RUNS_WIRE_VERSION},"robot":"go2","runs":[],"completeness":{{"kind":"probably"}}}}"#
        );
        assert!(matches!(
            decode_runs_reply(bad_kind.as_bytes()),
            Err(RunsDecodeError::Malformed { .. })
        ));
    }

    /// An UNKNOWN additive field from a NEWER robot is IGNORED, not rejected —
    /// serde's default, and the property every future `runs` field rests on. It
    /// is asserted here because nothing in the source says so: a later
    /// `#[serde(deny_unknown_fields)]` (a plausible hardening edit) would turn
    /// every newer robot's reply into a decode failure for every desk in the
    /// field.
    #[test]
    fn an_old_desk_ignores_an_unknown_future_field() {
        let future = format!(
            r#"{{"version":{RUNS_WIRE_VERSION},"robot":"go2","runs":[],"completeness":{{"kind":"settled"}},"topology_json":"{{}}"}}"#
        );
        let decoded = decode_runs_reply(future.as_bytes()).expect("a newer robot's reply decodes");
        assert_eq!(decoded.robot, "go2");
        assert!(decoded.runs.is_empty());
        assert!(decoded.completeness.is_settled());
    }

    /// Decoding validates.
    ///
    /// [`RunEntry::new`] is documented as the ONLY minting path, so a run whose
    /// `graph.yaml` could not be read is SKIPPED rather than served as an empty
    /// document the desk would render as a graph with no nodes — and
    /// [`RunsReply::refused`] exists so an empty list can never be read as "this
    /// robot is running nothing". Both guarantees covered only values this binary
    /// CONSTRUCTS: a peer is not bound by our constructors and `serde` enforces
    /// only the shape, so before this gate a version-1 document could carry every
    /// state those constructors forbid and flow downstream unchallenged. That is
    /// the same class the mandatory `completeness` field was made mandatory to
    /// avoid, one layer out.
    ///
    /// Hand-built bytes per hostile class (the module's decode-test idiom), each
    /// asserted on the REASON as well as the rejection, so no case can pass by
    /// tripping a different invariant than the one it is named for.
    #[test]
    fn decode_refuses_wire_data_carrying_a_constructor_forbidden_state() {
        let entry = |run_id: &str, graph_yaml: &str| {
            format!(
                r#"{{"run_id":"{run_id}","graph_name":"perception","run_started_at_ns":7,"state":"live","graph_yaml":"{graph_yaml}","run_json":"{{}}"}}"#
            )
        };
        let doc = |runs_body: &str, tail: &str| {
            format!(
                r#"{{"version":{RUNS_WIRE_VERSION},"robot":"go2","runs":[{runs_body}],{tail}}}"#
            )
        };
        const SETTLED: &str = r#""completeness":{"kind":"settled"}"#;
        const NOT_ESTABLISHED: &str =
            r#""completeness":{"kind":"incomplete","live_writers":0,"writers_heard":0}"#;

        // THE ANTI-TAUTOLOGY CONTROL, first: a well-formed document still decodes.
        // Without it every assertion below would pass on a decoder that rejects
        // everything — the failure mode a validation gate is most likely to have.
        let ok = doc(&entry(RUN_ID_42_TEXT, "name: perception\\n"), SETTLED);
        let decoded = decode_runs_reply(ok.as_bytes()).expect("a well-formed reply still decodes");
        assert_eq!(decoded.runs.len(), 1);
        assert_eq!(decoded.runs[0].run_id, RUN_ID_42_TEXT);

        let reason = |bytes: &[u8]| -> String {
            match decode_runs_reply(bytes) {
                Err(RunsDecodeError::Invalid { detail }) => detail,
                other => panic!("expected Invalid, got {other:?}"),
            }
        };

        // 1. A NONCANONICAL `run_id`. Both spellings PARSE to 42 via the
        //    deliberately-tolerant `parse_run_id`, and both are a different STRING
        //    from the canonical form — which is the hazard, since the desk's
        //    fetch-once cache compares this field as text.
        for hostile in ["0x2a", "0x0000000000000000000000000000002A"] {
            let detail = reason(doc(&entry(hostile, "y"), SETTLED).as_bytes());
            assert!(
                detail.contains("canonical"),
                "a noncanonical run_id must be refused AS SUCH, got: {detail}"
            );
        }

        // 2. An EMPTY artifact — arriving over the wire instead of being
        //    compiled. `RunEntry::new` refuses it; so must decode.
        let detail = reason(doc(&entry(RUN_ID_42_TEXT, ""), SETTLED).as_bytes());
        assert!(
            detail.contains("graph.yaml"),
            "an empty graph.yaml must be refused AS SUCH, got: {detail}"
        );

        // 3. An OVERSIZED artifact — the served cap, which exists so a pathological
        //    run directory cannot bloat a reply, and which a peer could ignore.
        let too_long = "a".repeat(MAX_RUN_ARTIFACT_LEN + 1);
        let detail = reason(doc(&entry(RUN_ID_42_TEXT, &too_long), SETTLED).as_bytes());
        assert!(
            detail.contains(&format!("{}", MAX_RUN_ARTIFACT_LEN + 1)),
            "an oversized graph.yaml must be refused naming its length, got: {detail}"
        );

        // 4. ONE IDENTITY TWICE — the contract `build_runs_reply` guarantees, which
        //    a peer's reply is under no obligation to have honoured.
        let twice = format!(
            "{},{}",
            entry(RUN_ID_42_TEXT, "y"),
            entry(RUN_ID_42_TEXT, "z")
        );
        let detail = reason(doc(&twice, SETTLED).as_bytes());
        assert!(
            detail.contains("more than once"),
            "a repeated identity must be refused AS SUCH, got: {detail}"
        );

        // 5. A REFUSAL SERVING ROWS — mutually exclusive by construction.
        let refusal_with_rows = doc(
            &entry(RUN_ID_42_TEXT, "y"),
            &format!(r#"{NOT_ESTABLISHED},"error":"not authorized""#),
        );
        let detail = reason(refusal_with_rows.as_bytes());
        assert!(
            detail.contains("serves NO runs"),
            "a refusal carrying rows must be refused AS SUCH, got: {detail}"
        );

        // 6. THE HEADLINE — a refusal paired with a SETTLED verdict. `RunsReply::
        //    refused` carries `not_established()` precisely so a refused desk is
        //    never told, with confidence, that the robot is idle. Nothing stopped a
        //    peer from sending exactly that pairing.
        let refusal_settled = doc("", &format!(r#"{SETTLED},"error":"not authorized""#));
        let detail = reason(refusal_settled.as_bytes());
        assert!(
            detail.contains("establishes NOTHING"),
            "a settled refusal must be refused AS SUCH, got: {detail}"
        );

        // The new reason RENDERS — the desk logs it once per robot, and a variant
        // whose Display says nothing useful is a variant that teaches nobody.
        assert!(
            RunsDecodeError::Invalid {
                detail: "x".to_string(),
            }
            .to_string()
            .contains("forbids"),
            "Invalid must render as a forbidden-state reason, not a parse failure"
        );
    }

    /// The two sides AGREE: everything the serve-side constructors can produce
    /// passes the decode-side gate.
    ///
    /// This is the pin that keeps the gate correct in the OTHER direction. A
    /// validator is only correct if it is neither too loose (the classes above)
    /// nor too tight — and too-tight is the more dangerous failure here, because it
    /// would refuse a robot's genuine answer and render "could not establish" over
    /// a machine that answered perfectly. Asserted as a property of the
    /// constructors' own output rather than of hand-written documents, so a future
    /// invariant added to one side and forgotten on the other fails here.
    ///
    /// It also pins the ORDER decision explicitly: `validate` does NOT require the
    /// served sort, so a correct-but-differently-ordered document is accepted.
    #[test]
    fn every_reply_the_constructors_produce_passes_the_decode_gate() {
        let built = build_runs_reply(
            "go2",
            vec![
                sample_entry(),
                RunEntry::new(9, "g", 1, RunState::Ending, "y", "j").expect("well-formed"),
            ],
            Vec::new(),
            RunsCompleteness::Settled,
        );
        assert_eq!(built.validate(), Ok(()));
        assert_eq!(
            decode_runs_reply(&encode_runs_reply(&built)),
            Ok(built.clone())
        );

        let empty = build_runs_reply("go2", [], Vec::new(), RunsCompleteness::Settled);
        assert_eq!(empty.validate(), Ok(()));

        let refused = RunsReply::refused("go2", "not authorized (account/pairing)");
        assert_eq!(refused.validate(), Ok(()));
        assert_eq!(decode_runs_reply(&encode_runs_reply(&refused)), Ok(refused));

        // ORDER is deliberately NOT an invariant: the same runs in the reverse of
        // the served order still decode, because a differently-ordered document
        // carries no forbidden state (the desk orders rows as it renders them).
        let mut reordered = built;
        reordered.runs.reverse();
        assert_eq!(
            reordered.validate(),
            Ok(()),
            "validation must refuse forbidden STATES, not a presentation choice"
        );
    }
}
