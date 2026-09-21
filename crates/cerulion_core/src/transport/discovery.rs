// SPDX-License-Identifier: AGPL-3.0-only
//! Liveliness-based topic discovery over zenoh.
//!
//! Publishers declare liveliness tokens when they start publishing a topic
//! over the network. Remote nodes can subscribe to liveliness changes to
//! discover which topics are available on the network.
//!
//! # Key Expression Convention
//!
//! There are TWO disjoint token key-spaces, kept in SEPARATE top-level chunks
//! so a peer's bridge watch can never confuse one for another:
//!
//! - **DEMAND** tokens live under `cerulion_lv{canonical topic}` — declared by
//!   an INGRESS bridge (`register_ingress`) to say "a live subscriber here wants
//!   this topic". The network bridge watch subscribes to `cerulion_lv/**` and
//!   flips a REMOTE machine's egress bridge flag on these (`Put` →
//!   `enable_bridge`, `Delete` → `disable_bridge`).
//! - **ANNOUNCE** tokens live under `cerulion_ann/{robot}{canonical topic}`
//!   and are declared by the gateway to advertise "robot PRODUCES
//!   this topic" so `topic list` can list produced topics AND surface the
//!   producing robot. The `{robot}` chunk is the robot identity (a single clean
//!   key chunk — never containing `/`), so [`parse_announce_key`] recovers the
//!   exact `(robot, canonical topic)` pair: absolute mirror topics group by
//!   PRODUCER, not by their leading topic segment. Every gateway also declares
//!   ONE bare identity token `cerulion_ann/{robot}` (no topic suffix) at network
//!   boot, so an ingress-only / zero-egress gateway still surfaces a ROBOTS row.
//!   A robot is a ROBOTS row iff its announce tokens actually arrive in the
//!   gather (live presence IS the verification) or an mDNS browse answers.
//!   Announces are pure presence/discovery: the bridge watch's `cerulion_lv/**`
//!   subscription does NOT match `cerulion_ann/**`, so an announce NEVER flips
//!   any egress flag. This is load-bearing — otherwise two default-permissive
//!   robots that both announce would spuriously flip each other's egress flags.
//!
//! The announce KEY FORMAT is a version boundary: an incompatible change to the
//! `cerulion_ann/{robot}{topic}` shape must bump the `cerulion_ann` chunk (the
//! parser rejects what it cannot split, never mis-decodes). There is no longer a
//! third `cerulion_meta` beacon key-space: robot identity now rides the announce
//! keys' robot chunk + the mDNS `_cerulion._tcp` SRV/TXT record (the one gateway
//! beacon), not a version-keyed liveliness beacon.
//!
//! Canonical names carry a leading slash; `cerulion_lv` and
//! `cerulion_ann` are both DISTINCT top-level chunks from the `cerulion` data
//! namespace, so no user topic — e.g. a data topic named `/topics/x` — can
//! collide with either token space. For example, robot `go2` publishing topic
//! `camera/image` gets a DEMAND token at `cerulion_lv/camera/image` (demand is
//! topic-keyed — no robot chunk) and an ANNOUNCE token at
//! `cerulion_ann/go2/camera/image` (canonical: `/p/n/o` →
//! `cerulion_lv/p/n/o` / `cerulion_ann/go2/p/n/o`).
//!
//! # Lifecycle
//!
//! - Token is created when a publisher advertises a topic for network transport
//! - Token is automatically dropped when the publisher is dropped or the session closes
//! - Remote nodes detect the drop via liveliness subscription (failure detection for free)

use std::time::{Duration, Instant};

use zenoh::Wait;

use super::cerulion_q::{
    self, CatalogGatherOutcome, CatalogReply, RunsGatherOutcome, RunsReply, SchemaGatherOutcome,
    SchemaReply, UnusableRunsAnswer,
};
use crate::error::{TransportError, TransportResult};

/// Prefix for Cerulion DEMAND (ingress-interest) liveliness tokens — the
/// key-space the network bridge watch flips egress flags on.
const LIVELINESS_PREFIX: &str = "cerulion_lv";

/// Prefix for Cerulion ANNOUNCE (egress-presence) liveliness tokens —
/// a DISTINCT top-level chunk from [`LIVELINESS_PREFIX`], so the bridge watch's
/// `cerulion_lv/**` subscription never matches an announce (an announce is pure
/// discovery, never a demand that flips an egress flag).
const ANNOUNCE_PREFIX: &str = "cerulion_ann";

// The DEMAND request/reply key-space moved OUT of discovery's
// `cerulion_demand` chunk into the unified verb-dispatched `cerulion_q` query
// surface (`cerulion_q/{robot}/demand{topic}`) — key construction + parsing now
// live in [`crate::transport::cerulion_q`]. The wire-proven accepter-declares /
// dialer-GETs asymmetry that motivates the queryable inversion is documented
// there.

/// A liveliness token representing a topic advertised on the network.
///
/// Holds a zenoh liveliness token. When dropped, the token is undeclared
/// and remote subscribers are notified of the topic going away.
pub struct TopicToken {
    topic: String,
    _token: zenoh::liveliness::LivelinessToken,
}

impl TopicToken {
    /// Declare a DEMAND liveliness token for the given topic on the zenoh
    /// session — "a live subscriber here wants this topic".
    ///
    /// The token advertises `cerulion_lv{canonical topic}` on the network. A
    /// remote machine's bridge watch (subscribed to `cerulion_lv/**`) flips its
    /// egress bridge flag on for this topic. Declared by the ingress bridge.
    ///
    /// # Errors
    ///
    /// Returns `TransportError::Publish` if the liveliness declaration fails.
    pub fn declare(session: &zenoh::Session, topic: &str) -> TransportResult<Self> {
        Self::declare_at(session, LIVELINESS_PREFIX, topic, "liveliness")
    }

    /// Declare an ANNOUNCE liveliness token for `topic`,
    /// carrying the producing `robot` identity as the FIRST chunk — "robot
    /// PRODUCES this topic".
    ///
    /// The token advertises `cerulion_ann/{robot}{canonical topic}` (e.g. robot
    /// `go2` + topic `/utlidar/cloud` → `cerulion_ann/go2/utlidar/cloud`). The
    /// robot chunk carries robot identity EXACTLY (`is_valid_robot_identity`
    /// forbids `/` in it, so [`parse_announce_key`] splits it back off cleanly) —
    /// this is what makes ABSOLUTE mirror topics (never prefixed) group by their
    /// PRODUCER rather than by their leading topic segment. It is a key-space the
    /// bridge watch's `cerulion_lv/**` subscription does NOT match, so an announce
    /// is pure discovery surface (`topic list`) and NEVER flips any egress flag.
    /// Declared by the gateway on behalf of an egress producer.
    ///
    /// # Errors
    ///
    /// Returns `TransportError::Publish` if `robot` is empty or contains `/` (it
    /// must be a single clean key chunk), or if the liveliness declaration fails.
    pub fn announce(session: &zenoh::Session, robot: &str, topic: &str) -> TransportResult<Self> {
        let canonical = crate::transport::network::canonical_topic(topic);
        let key = Self::announce_key_for(robot, &canonical)?;
        // The token's identity label is its full canonical topic (for the Drop
        // breadcrumb / error field); the key carries the robot chunk.
        Self::declare_key(session, &key, "announce", &canonical)
    }

    /// Declare the BARE identity ANNOUNCE token `cerulion_ann/{robot}`
    /// (no topic suffix) — "robot is HERE, live gateway present". Every gateway
    /// declares exactly one at network boot so an ingress-only / zero-egress
    /// gateway still surfaces a `ROBOTS` row (presence with zero topics), even on
    /// an mDNS-unreachable network. [`parse_announce_key`] returns `(robot, None)`
    /// for it.
    ///
    /// # Errors
    ///
    /// Returns `TransportError::Publish` if `robot` is empty or contains `/`, or
    /// if the liveliness declaration fails.
    pub fn announce_identity(session: &zenoh::Session, robot: &str) -> TransportResult<Self> {
        let key = Self::announce_key_for(robot, "")?;
        Self::declare_key(session, &key, "announce-identity", robot)
    }

    /// Build an announce key `cerulion_ann/{robot}{canonical}` (`canonical` is
    /// `""` for the bare identity token), rejecting a robot chunk that is empty
    /// or carries a `/` (which would corrupt the exact robot↔topic split in
    /// [`parse_announce_key`]).
    fn announce_key_for(robot: &str, canonical: &str) -> TransportResult<String> {
        if robot.is_empty() || robot.contains('/') {
            return Err(TransportError::Publish {
                topic: format!("{ANNOUNCE_PREFIX}/{robot}"),
                reason: format!(
                    "announce robot identity '{robot}' must be a single non-empty key chunk \
                     with no '/' (it is the first chunk of the announce key)"
                ),
            });
        }
        Ok(format!("{ANNOUNCE_PREFIX}/{robot}{canonical}"))
    }

    /// Shared token declaration under an explicit key-space `prefix`. `kind`
    /// labels the breadcrumb so the key-spaces are distinguishable in logs.
    /// (The DEMAND side; the ANNOUNCE side builds its key via
    /// [`announce_key_for`](Self::announce_key_for).)
    fn declare_at(
        session: &zenoh::Session,
        prefix: &str,
        topic: &str,
        kind: &str,
    ) -> TransportResult<Self> {
        let key = format!(
            "{prefix}{}",
            crate::transport::network::canonical_topic(topic)
        );
        Self::declare_key(session, &key, kind, topic)
    }

    /// Declare a liveliness token at an explicit `key` and wrap it in a
    /// [`TopicToken`]. `kind` labels the breadcrumb (DEMAND vs ANNOUNCE);
    /// `topic_label` is the token's identity string stored in the `topic` field
    /// — the canonical topic for [`declare_at`](Self::declare_at) — and reused as
    /// the error `topic` field. Shared by both callers so the declaration + error
    /// mapping + debug breadcrumb live in exactly one place.
    fn declare_key(
        session: &zenoh::Session,
        key: &str,
        kind: &str,
        topic_label: &str,
    ) -> TransportResult<Self> {
        let token = session
            .liveliness()
            .declare_token(key)
            .wait()
            .map_err(|e| TransportError::Publish {
                topic: topic_label.to_string(),
                reason: format!("{kind} token declaration failed: {e}"),
            })?;

        tracing::debug!(topic = %topic_label, key = %key, kind, "liveliness token declared");

        Ok(Self {
            topic: topic_label.to_string(),
            _token: token,
        })
    }

    /// Returns the topic name this token advertises.
    pub fn topic(&self) -> &str {
        &self.topic
    }
}

impl Drop for TopicToken {
    fn drop(&mut self) {
        tracing::debug!(topic = %self.topic, "liveliness token dropped");
    }
}

/// Recover the canonical topic from a liveliness reply key under `prefix`.
///
/// Strips only the namespace chunk: the
/// remainder keeps its leading slash (canonical form). The `starts_with('/')`
/// filter rejects the BARE namespace key (`**` matches zero chunks, so a
/// foreign token at exactly the namespace name DOES arrive) instead of yielding
/// a phantom empty-name topic — such keys warn loudly and return `None`.
fn recover_topic_from_key(prefix: &str, key: &str) -> Option<String> {
    match key.strip_prefix(prefix).filter(|t| t.starts_with('/')) {
        Some(topic) => Some(topic.to_string()),
        None => {
            tracing::warn!(
                key = %key,
                "liveliness key matched the namespace but is not a \
                 canonical topic — ignored"
            );
            None
        }
    }
}

/// Query currently live topics from the network.
///
/// Performs a one-shot liveliness query on the DEMAND key-space
/// ([`liveliness_prefix`]) and returns a list of topic names that are
/// currently advertised by remote subscribers.
///
/// UNBOUNDED drain (back-compat): replies are drained until zenoh closes the
/// reply channel (zenoh's own query timeout). For a CLI-friendly bounded
/// gather use [`query_live_topics_by_prefix`]; for the ANNOUNCE key-space
/// (whose keys carry a robot chunk) use [`query_announce_entries`].
///
/// # Errors
///
/// Returns `TransportError::Receive` if the liveliness query fails.
pub fn query_live_topics(session: &zenoh::Session) -> TransportResult<Vec<String>> {
    let key = format!("{LIVELINESS_PREFIX}/**");
    let replies = session
        .liveliness()
        .get(&key)
        .wait()
        .map_err(|e| TransportError::Receive {
            topic: "*".to_string(),
            reason: format!("liveliness query failed: {e}"),
        })?;

    let mut topics = Vec::new();

    while let Ok(reply) = replies.recv() {
        if let Ok(sample) = reply.into_result() {
            if let Some(topic) =
                recover_topic_from_key(LIVELINESS_PREFIX, sample.key_expr().as_str())
            {
                topics.push(topic);
            }
        }
    }

    Ok(topics)
}

/// Query currently live topics under an EXPLICIT liveliness key-space
/// `prefix` — the DEMAND space ([`liveliness_prefix`]), whose keys are the bare
/// namespace + canonical topic — with a BOUNDED gather window. (The ANNOUNCE
/// space's keys carry a robot chunk and are queried via
/// [`query_announce_entries`], not this fn.)
///
/// Reply parsing is identical to [`query_live_topics`] (strip the namespace
/// chunk, keep the canonical leading-`/` filter, warn-and-ignore malformed
/// keys), but the reply drain is bounded by `gather_window`: a deadline loop
/// over `recv_timeout` returns whatever arrived within the window instead of
/// blocking until zenoh's (slow) default query timeout closes the channel.
/// The gather also ends early when the reply channel closes (query complete).
///
/// Callers: the CLI's `topic list` remote-topics gather, and the
/// gateway's demand reconciler
/// ([`crate::transport::network::NetworkManager::start_demand_reconciler`]),
/// which polls this over `cerulion_lv/**` as the belt for links where the demand
/// token crosses. NOTE: this gather is an ACCEPTER→DIALER
/// GET, one of the two directions that do NOT route on a strict connect-only,
/// listen-less peer link — on that link the real demand path is the
/// queryable inversion, not this query.
///
/// # Errors
///
/// Returns `TransportError::Receive` if the liveliness query fails. A window
/// that elapses with no replies is NOT an error — it returns an empty list
/// (no live tokens gathered).
pub fn query_live_topics_by_prefix(
    session: &zenoh::Session,
    prefix: &str,
    gather_window: Duration,
) -> TransportResult<Vec<String>> {
    let key = format!("{prefix}/**");
    let replies = session
        .liveliness()
        .get(&key)
        .wait()
        .map_err(|e| TransportError::Receive {
            topic: "*".to_string(),
            reason: format!("liveliness query failed: {e}"),
        })?;

    let mut topics = Vec::new();
    let deadline = Instant::now() + gather_window;
    loop {
        let remaining = deadline.saturating_duration_since(Instant::now());
        if remaining.is_zero() {
            break;
        }
        match replies.recv_timeout(remaining) {
            Ok(Some(reply)) => {
                if let Ok(sample) = reply.into_result() {
                    if let Some(topic) = recover_topic_from_key(prefix, sample.key_expr().as_str())
                    {
                        topics.push(topic);
                    }
                }
            }
            // Channel closed (query complete) or the window elapsed — either
            // way the bounded gather ends with whatever was collected.
            _ => break,
        }
    }

    Ok(topics)
}

/// Parse an ANNOUNCE reply key back into `(robot, Option<canonical
/// topic>)` — the inverse of the `cerulion_ann/{robot}{canonical}` format
/// [`TopicToken::announce`] declares and the bare `cerulion_ann/{robot}`
/// identity token [`TopicToken::announce_identity`] declares.
///
/// Strips the `cerulion_ann/` namespace, then splits at the FIRST `/`: the
/// leading chunk is the robot (it can never contain `/` — the declare side
/// rejects that), the remainder — which keeps its leading `/` — is the
/// canonical topic. No remainder ⇒ the bare identity token ⇒ `(robot, None)`
/// (robot presence, zero topics). This is a pure PARSER, not a policy gate —
/// it only rejects the STRUCTURALLY malformed.
///
/// Returns `None` (and warns loudly, mirroring `recover_topic_from_key`) for:
/// the bare namespace key (`**` matches zero chunks, so a foreign token at
/// exactly `cerulion_ann` DOES arrive), a missing prefix, or an empty robot
/// chunk (`cerulion_ann//x`). Callers only feed it keys that matched
/// `cerulion_ann/**`, so a warn on malformed is correct and loud.
pub fn parse_announce_key(key: &str) -> Option<(String, Option<String>)> {
    fn inner(key: &str) -> Option<(String, Option<String>)> {
        let namespace = format!("{ANNOUNCE_PREFIX}/");
        let rest = key.strip_prefix(&namespace)?;
        match rest.find('/') {
            // No topic remainder ⇒ the bare identity token (robot presence).
            None => {
                if rest.is_empty() {
                    return None;
                }
                Some((rest.to_string(), None))
            }
            Some(idx) => {
                let robot = &rest[..idx];
                if robot.is_empty() {
                    return None;
                }
                // The remainder keeps its leading '/' — canonical form.
                Some((robot.to_string(), Some(rest[idx..].to_string())))
            }
        }
    }
    match inner(key) {
        Some(parsed) => Some(parsed),
        None => {
            tracing::warn!(
                key = %key,
                "announce key is malformed (expected cerulion_ann/<robot>[<canonical topic>]) \
                 — ignored"
            );
            None
        }
    }
}

/// Query the live ANNOUNCE tokens on the network — the
/// `(robot, Option<canonical topic>)` presence entries `topic list` builds its
/// ROBOTS rows and REMOTE TOPICS announce half from.
///
/// Performs a BOUNDED liveliness gather over `cerulion_ann/**` (the drain logic
/// is identical to [`query_live_topics_by_prefix`]: a deadline loop over
/// `recv_timeout`, ending early when the reply channel closes), parses each
/// reply key via [`parse_announce_key`], and returns the entries SORTED and
/// DEDUPED for deterministic presentation. A `(robot, None)` entry is a bare
/// identity token — a live gateway that announced no topic.
///
/// # Errors
///
/// Returns [`TransportError::Receive`] if the liveliness query fails. A window
/// that elapses with no replies is NOT an error — it returns an empty list.
pub fn query_announce_entries(
    session: &zenoh::Session,
    gather_window: Duration,
) -> TransportResult<Vec<(String, Option<String>)>> {
    let key = format!("{ANNOUNCE_PREFIX}/**");
    let replies = session
        .liveliness()
        .get(&key)
        .wait()
        .map_err(|e| TransportError::Receive {
            topic: "*".to_string(),
            reason: format!("announce liveliness query failed: {e}"),
        })?;

    let mut entries: Vec<(String, Option<String>)> = Vec::new();
    let deadline = Instant::now() + gather_window;
    loop {
        let remaining = deadline.saturating_duration_since(Instant::now());
        if remaining.is_zero() {
            break;
        }
        match replies.recv_timeout(remaining) {
            Ok(Some(reply)) => {
                if let Ok(sample) = reply.into_result() {
                    if let Some(entry) = parse_announce_key(sample.key_expr().as_str()) {
                        entries.push(entry);
                    }
                }
            }
            // Channel closed (query complete) or the window elapsed — either
            // way the bounded gather ends with whatever was collected.
            _ => break,
        }
    }

    // Deterministic presentation: sort then dedupe the (robot, topic) entries.
    entries.sort();
    entries.dedup();
    Ok(entries)
}

/// GET a robot's full topic catalog over the `cerulion_q` query surface
/// — an EXPLICIT-robot `cerulion_q/{robot}/catalog` GET (never a mid-key wildcard,
/// which computes an empty route on a real link; the laptop always knows the
/// identity from the announce harvest). This is a regular queryable GET (a
/// dialer→accepter query, the working direction on a strict connect-only link).
///
/// Returns the decoded [`CatalogReply`] on the FIRST decodable answer within
/// `gather_window`, or `None` when the robot does not answer (an older binary
/// with no query surface — SILENT back-compat fallback) OR answers only with a
/// reply that could not be used (an unknown wire version / corruption — a LOUD
/// `warn!` EXACTLY ONCE for this robot, naming the reason, then fall back). A
/// query that cannot even be issued (session error) is a `debug!` + `None` — the
/// catalog is a best-effort ENRICHMENT over the presence-derived listing, never a
/// hard dependency.
/// Attribute a decoded [`CatalogReply`] to the identity the
/// desk QUERIED (the ANNOUNCE identity carried in the explicit
/// `cerulion_q/{robot}/catalog` key), overriding the payload's self-attributed
/// `robot`. On a strict connect-only link the GET is EXPLICIT-robot, so the
/// reply DID come from `robot`; pinning the display to the queried announce
/// identity keeps `topic list` provenance correct even when a robot
/// self-attributes a divergent name in its payload (differing from the queried
/// announce identity — for example a robot announcing under its graph prefix
/// 'go2' while its hostname is 'ubuntu'; identity is the hostname, which
/// closes that case) and matches the identity multi-robot routing uses. Pure —
/// oracle-tested.
fn attribute_catalog_to_queried(mut catalog: CatalogReply, robot: &str) -> CatalogReply {
    catalog.robot = robot.to_string();
    catalog
}

/// The [`SchemaReply`] twin of [`attribute_catalog_to_queried`]
/// — attribute a decoded schema reply to the identity the desk QUERIED (the
/// explicit-key `robot`), so `schema info` provenance + the `topic echo`
/// breadcrumb show the announce identity the desk routed to, never a divergent
/// self-attribution. Pure — oracle-tested.
fn attribute_schema_to_queried(mut reply: SchemaReply, robot: &str) -> SchemaReply {
    reply.robot = robot.to_string();
    reply
}

pub fn query_robot_catalog(
    session: &zenoh::Session,
    robot: &str,
    gather_window: Duration,
) -> Option<CatalogReply> {
    let selector = cerulion_q::catalog_selector(robot);
    let replies = match session.get(&selector).timeout(gather_window).wait() {
        Ok(replies) => replies,
        Err(e) => {
            tracing::debug!(
                robot = %robot,
                error = %e,
                "catalog GET could not be issued — falling back to the announce listing"
            );
            return None;
        }
    };
    // Collect the (tiny, control-plane) reply payloads within the window — the
    // channel closes when the GET completes or the timeout elapses — then classify
    // them once (`gather_catalog_outcome`): the first decodable reply wins; a reply
    // that arrived but never decoded warns ONCE for this robot; no reply is the
    // silent back-compat fallback.
    let mut payloads: Vec<Vec<u8>> = Vec::new();
    while let Ok(reply) = replies.recv() {
        if let Ok(sample) = reply.into_result() {
            payloads.push(sample.payload().to_bytes().to_vec());
        }
    }
    match cerulion_q::gather_catalog_outcome(payloads.iter().map(|p| p.as_slice())) {
        CatalogGatherOutcome::Decoded(catalog) => {
            Some(attribute_catalog_to_queried(catalog, robot))
        }
        CatalogGatherOutcome::Ignored(reason) => {
            tracing::warn!(
                robot = %robot,
                reason = %reason,
                "catalog reply from this robot could not be used — ignoring its \
                 catalog and falling back to the announce-derived listing"
            );
            None
        }
        // No reply at all — the robot did not answer (an older binary with no
        // query surface). Silent: this is the expected back-compat path.
        CatalogGatherOutcome::NoReply => None,
    }
}

/// What ONE catalog/schema fan-out gathered, plus the robots whose GET
/// worker PANICKED.
///
/// # This is NOT the [`RunsHarvest`] partition, deliberately
///
/// `RunsHarvest` classifies EVERY asked robot onto exactly one of three lists,
/// because there a missing robot is a COVERAGE claim. Here it is not: a robot that
/// does not answer, or answers with bytes this binary cannot read, still
/// "contributes nothing (the caller falls back to its announce-derived listing)" —
/// that meaning is byte-unchanged, and widening these gathers to per-robot
/// coverage would touch every catalog/schema consumer AND the
/// `DiscoveryState` contract (which lives in `cerulion-netd`, one layer up), so it
/// is a separate, larger decision.
///
/// [`Self::panicked`] carries ONE thing those lists cannot: a robot this process
/// never actually asked, because the thread doing the asking unwound. That is a
/// BUG IN THIS BINARY rather than a fact about the robot, and a plain
/// `filter_map(|h| h.join().ok())` would destroy it — so the robot would land on no
/// list, be named in no log, and the pass that lost it would be indistinguishable from
/// a pass that asked everyone.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct GatherReplies<T> {
    /// The decodable replies, in no particular order — a subset of the robots
    /// asked, possibly all of them and possibly none.
    pub replies: Vec<T>,
    /// The robots whose GET worker PANICKED — asked, and unknown for a reason that
    /// is this binary's fault. EMPTY on every healthy pass, which is what makes a
    /// caller that reads it pay nothing when nothing is wrong.
    pub panicked: Vec<String>,
}

impl<T> Default for GatherReplies<T> {
    fn default() -> Self {
        Self {
            replies: Vec::new(),
            panicked: Vec::new(),
        }
    }
}

impl<T> GatherReplies<T> {
    /// Whether every robot this pass meant to ask was actually asked.
    ///
    /// `false` means the returned `replies` are a PARTIAL account of what the
    /// network would have said, so their emptiness — about the panicked robot or
    /// about anything the panicked robot might have served — is evidence of
    /// nothing.
    pub fn is_complete(&self) -> bool {
        self.panicked.is_empty()
    }
}

/// How ONE joined fan-out worker ended.
///
/// `NoReply` and `Panicked` both yield no reply and are deliberately DISTINCT: the
/// first is a fact about the robot (an older binary with no query surface, or a
/// window it missed) and the second is a bug in this process. Collapsing them is
/// exactly what `join().ok()` did.
enum JoinedReply<T> {
    /// The worker returned a decodable reply.
    Answered(T),
    /// The worker ran to completion and the robot said nothing usable.
    NoReply,
    /// The worker UNWOUND — this robot was never really asked.
    Panicked,
}

/// Classify ONE joined fan-out worker, logging a panic LOUDLY.
///
/// `error!` and not `warn!`, and the message says whose fault it is: a GET worker
/// unwinding is a defect in this binary, not something the robot did, so an
/// operator reading it must not be steered at the robot. The `robot` is available
/// because the caller pairs the identity with the handle OUTSIDE the closure —
/// a closure returning `(robot, reply)` loses BOTH halves when it unwinds.
fn joined_reply_outcome<T>(robot: &str, joined: std::thread::Result<Option<T>>) -> JoinedReply<T> {
    match joined {
        Ok(Some(reply)) => JoinedReply::Answered(reply),
        Ok(None) => JoinedReply::NoReply,
        Err(_) => {
            tracing::error!(
                robot = %robot,
                "the catalog/schema GET worker for this robot PANICKED — this \
                 robot was never really asked, so the gather is reported INCOMPLETE and \
                 its answer cannot support an absence claim. This is a bug in THIS \
                 binary, not a fact about the robot."
            );
            JoinedReply::Panicked
        }
    }
}

/// The CONCURRENCY + JOIN + ACCOUNTING half shared by
/// [`query_robot_catalogs`] and [`query_robot_schemas`], with the per-robot GET
/// taken as a parameter.
///
/// ONE copy for both verbs (a second copy of a rule is free to disagree
/// with the first), and split out for the same reason `gather_runs_over` was:
/// the panic arm is otherwise UNDRIVABLE. Reaching it through the public functions
/// needs a live zenoh session AND a worker that unwinds, so with the query inlined
/// the only thing a test could reach was the classifier — and a pure arm over a
/// classifier cannot see the CALL SITE reverting to a closure that pairs the
/// identity INSIDE the worker, which is precisely the shape that loses it.
///
/// `query` is `Sync` because each worker borrows it; it is called EXACTLY once per
/// robot, on its own thread.
fn gather_replies_over<T: Send>(
    robots: &[String],
    query: impl Fn(&str) -> Option<T> + Sync,
) -> GatherReplies<T> {
    if robots.is_empty() {
        return GatherReplies::default();
    }
    let query = &query;
    let joined: Vec<(&String, JoinedReply<T>)> = std::thread::scope(|s| {
        // The robot's IDENTITY is paired with its handle OUT HERE, not returned
        // from inside the closure — the whole of the panic fix. A closure returning
        // `(robot, reply)` loses BOTH halves when it unwinds, so `join().ok()`
        // could only drop the worker entirely, and a dropped worker is a robot that
        // was ASKED, is UNKNOWN, and is named nowhere. Zipping the original slice
        // keeps the name available no matter how the worker ended.
        let handles: Vec<_> = robots
            .iter()
            .map(|robot| s.spawn(move || query(robot)))
            .collect();
        robots
            .iter()
            .zip(handles)
            .map(|(robot, handle)| (robot, joined_reply_outcome(robot, handle.join())))
            .collect()
    });
    let mut out = GatherReplies::default();
    for (robot, outcome) in joined {
        match outcome {
            JoinedReply::Answered(reply) => out.replies.push(reply),
            JoinedReply::NoReply => {}
            // hot-path-alloc-ok: cold — at most one per robot per control-plane GET,
            // and only on a pass that already lost a thread.
            JoinedReply::Panicked => out.panicked.push(robot.clone()),
        }
    }
    out
}

/// GET the catalog of EACH robot in `robots` over `session`,
/// CONCURRENTLY (scoped threads, like the announce/demand gathers), returning the
/// successfully-decoded replies. Each GET is an EXPLICIT `cerulion_q/{robot}/catalog`
/// selector (see [`query_robot_catalog`]). A robot that does not answer OR answers
/// with an unknown wire version contributes nothing (the caller falls back to its
/// announce-derived listing). The added remote wait stays ~ONE `gather_window`
/// regardless of robot count. `zenoh::Session` is `Send + Sync`, so the concurrent
/// GETs are safe (the scoped borrow proves it).
///
/// A PANICKED GET worker is no longer swallowed. It is named in
/// [`GatherReplies::panicked`] and logged LOUDLY, so a pass that lost a thread
/// cannot be read as a pass that asked everyone — see [`GatherReplies`] for why
/// that one class is carried while "did not answer" still is not.
pub fn query_robot_catalogs(
    session: &zenoh::Session,
    robots: &[String],
    gather_window: Duration,
) -> GatherReplies<CatalogReply> {
    gather_replies_over(robots, |robot| {
        query_robot_catalog(session, robot, gather_window)
    })
}

/// GET the `.msg`/YAML closure of ONE type `requested` from
/// `robot` over `session` — an EXPLICIT `cerulion_q/{robot}/schema/{requested}`
/// selector (the dialer→accepter direction that routes on a strict connect-only
/// link). `requested` is the qualified `pkg/Type` OR a package-less bare `Name`.
/// Returns the decoded [`SchemaReply`] on the FIRST
/// decodable answer within `gather_window` — which MAY be a NOT-FOUND reply (the
/// robot answered but does not have the type; the caller surfaces that as-is).
/// `None` means the robot did not answer (an older binary with no query surface,
/// OR — for a bare-name GET — an OLD robot whose queryable rejects the 1-chunk
/// key, so the bounded gather returns empty; silent back-compat) OR answered only
/// with an undecodable reply (unknown wire version / corruption — a LOUD `warn!`
/// ONCE for this robot). A query that cannot even be issued (session error) is a
/// `debug!` + `None`. Best-effort: never a hard dependency, never a hang.
pub fn query_robot_schema(
    session: &zenoh::Session,
    robot: &str,
    requested: &str,
    gather_window: Duration,
) -> Option<SchemaReply> {
    let selector = cerulion_q::schema_selector(robot, requested);
    let replies = match session.get(&selector).timeout(gather_window).wait() {
        Ok(replies) => replies,
        Err(e) => {
            tracing::debug!(
                robot = %robot, requested = %requested, error = %e,
                "schema GET could not be issued — falling back to hash-only decoding"
            );
            return None;
        }
    };
    let mut payloads: Vec<Vec<u8>> = Vec::new();
    while let Ok(reply) = replies.recv() {
        if let Ok(sample) = reply.into_result() {
            payloads.push(sample.payload().to_bytes().to_vec());
        }
    }
    match cerulion_q::gather_schema_outcome(payloads.iter().map(|p| p.as_slice())) {
        SchemaGatherOutcome::Decoded(reply) => Some(attribute_schema_to_queried(reply, robot)),
        SchemaGatherOutcome::Ignored(reason) => {
            tracing::warn!(
                robot = %robot, requested = %requested, reason = %reason,
                "schema reply from this robot could not be used — falling back to \
                 hash-only decoding"
            );
            None
        }
        SchemaGatherOutcome::NoReply => None,
    }
}

/// GET `requested` (a qualified `pkg/Type` OR a package-less
/// bare `Name`) from EACH robot in `robots`
/// CONCURRENTLY (scoped threads, like the catalog gather), returning every
/// DECODABLE reply (found OR not-found — each carries its answering `robot`). The
/// caller picks the FIRST reply with non-empty `docs` (a robot that HAS the type)
/// and reports its provenance. A robot that does not answer / answers undecodably
/// contributes nothing. The added remote wait stays ~ONE `gather_window`
/// regardless of robot count.
///
/// A PANICKED GET worker is no longer swallowed — see
/// [`query_robot_catalogs`] and [`GatherReplies`]. It matters most here: the
/// caller's "no robot serves this type" is an ABSENCE claim, and before that fix a
/// robot whose worker unwound was missing from the replies in a way nothing could
/// tell apart from a robot that answered "I do not have it".
pub fn query_robot_schemas(
    session: &zenoh::Session,
    robots: &[String],
    requested: &str,
    gather_window: Duration,
) -> GatherReplies<SchemaReply> {
    gather_replies_over(robots, |robot| {
        query_robot_schema(session, robot, requested, gather_window)
    })
}

/// The [`RunsReply`] twin of [`attribute_catalog_to_queried`]
/// — attribute a decoded runs reply to the identity the desk QUERIED (the
/// explicit-key `robot`), so the desk's run rows carry the announce identity it
/// routed to, never a divergent self-attribution.
///
/// This is what the `RunsReply::robot` doc promises the desk side does, and it
/// is the mechanism robot attribution rests on: a run's `robot` is a property of the KEY
/// the reply came back on, not of anything the answering machine wrote about
/// itself. A robot whose hostname drifted from its announce identity (the live
/// Go2 `ubuntu`-vs-`go2` gap) therefore cannot mint a second row, and no fold
/// downstream has to reconcile two spellings of one machine. Pure —
/// oracle-tested.
fn attribute_runs_to_queried(mut reply: RunsReply, robot: &str) -> RunsReply {
    reply.robot = robot.to_string();
    reply
}

/// What a gather over a SET of robots learned: the replies that
/// arrived, and the robots that answered UNUSABLY.
///
/// Two lists rather than one, because the robots missing from `replies` are
/// missing for reasons with OPPOSITE remedies. A robot that did not answer may
/// answer later (wait); a robot in `unusable` answered and will answer
/// identically until somebody redeploys (do not wait). Collapsing them into one
/// silent absence is what made a skewed robot look like a slow one.
#[derive(Debug, Clone, Default, PartialEq, Eq)]
pub struct RunsHarvest {
    /// One reply per robot that answered USABLY — a possibly-STRICT SUBSET of the
    /// robots asked. Which robots are MISSING is carried by [`Self::silent`].
    pub replies: Vec<RunsReply>,
    /// Robots that answered with something this binary could not use.
    pub unusable: Vec<UnusableRunsAnswer>,
    /// Robots that were ASKED and did not answer AT ALL.
    ///
    /// # Why absence is a NAMED list
    ///
    /// The three lists are the whole taxonomy, and this one cannot be left
    /// implicit — "its absence from `replies` IS the report". That is true only
    /// for a reader holding the ASKED set, and the desk-side fold does not: it
    /// receives replies and unusable answers and cannot reconstruct who was
    /// approached. So an announced robot that stayed silent would be invisible, and a
    /// consumer folding a complete-looking harvest into "these are the runs"
    /// would turn it into a SETTLED ABSENCE — precisely the confident-empty class
    /// the discovery latch exists to kill, one layer up from where that latch operates.
    ///
    /// Naming it here rather than making each caller diff `robots` against
    /// `replies` keeps the taxonomy in the one place that already owns it
    /// ([`Self::absorb`]), so the three arms cannot drift apart.
    ///
    /// Its REMEDY is distinct from both siblings: such a robot may answer later
    /// (a slow serve), or may never answer (a binary predating the verb, or a
    /// Strict ingress-only gateway that declares no query surface). Either way a
    /// consumer must not claim to know what it is running.
    pub silent: Vec<String>,
}

impl RunsHarvest {
    /// PURE: fold ONE robot's gather outcome in, routing it to the list whose
    /// REMEDY it belongs to. Oracle-tested.
    ///
    /// The routing is the whole taxonomy in three lines, which is why it lives
    /// here rather than being open-coded at each caller: `query_robot_runs_all`
    /// and netd's single-robot plane arm must agree about what an unusable answer
    /// IS, and two copies of that judgement are how they stop agreeing.
    pub fn absorb(&mut self, robot: &str, outcome: RunsGatherOutcome) {
        match outcome {
            RunsGatherOutcome::Decoded(reply) => self.replies.push(reply),
            RunsGatherOutcome::Ignored(reason) => self.unusable.push(UnusableRunsAnswer {
                // hot-path-alloc-ok: cold — one entry per skewed robot per query
                // on the control plane, never per frame.
                robot: robot.to_string(),
                reason: reason.to_string(),
            }),
            // Nothing came back. NAMED rather than left to the
            // caller's arithmetic — see `silent`'s docs for why an implicit
            // absence became a settled-absence bug one layer up.
            RunsGatherOutcome::NoReply => self.silent.push(robot.to_string()),
        }
    }

    /// Whether a retrying caller should STOP after this harvest.
    ///
    /// True iff some robot ANSWERED unusably. Such a peer has converged — its
    /// reply is unusable because of a wire skew, so a retry re-issues the GET and
    /// reads the same bytes, at the robot's full serve cost. A harvest that is
    /// merely EMPTY is not terminal: nobody answered, and somebody still might.
    #[must_use]
    pub fn is_terminal(&self) -> bool {
        !self.unusable.is_empty()
    }
}

/// GET the runs LIVE on ONE robot over `session`, through an EXPLICIT
/// `cerulion_q/{robot}/runs` selector (the dialer→accepter direction that routes
/// on a strict connect-only link), and report WHICH of the three outcomes it
/// was, re-attributing a decoded reply to the QUERIED identity.
///
/// # It returns the taxonomy, not an `Option`, and that is the point
///
/// The three arms are three different facts with three different remedies:
///
/// - `Decoded` — the robot answered. What an empty `runs` list inside that reply
///   means is carried by its own `RunsCompleteness`.
/// - `Ignored` — the robot ANSWERED with something unusable (a wire skew,
///   corruption, or a document its own serve side forbids). LOUD, once per robot.
///   **Retrying cannot help**: the same peer will re-serve the same bytes, and on
///   this verb each attempt costs the robot a fresh iceoryx2 reader node (the
///   ~620 ms serve). A caller running under a retry budget must treat this as
///   TERMINAL for the query.
/// - `NoReply` — nothing came back. Reachable on any robot whose binary predates
///   the verb, AND on one with no query surface at all (a Strict ingress-only
///   gateway declares none — see `NetworkManager::has_query_surface`), so it is
///   the SILENT back-compat path and the one arm on which waiting is sensible.
///
/// Collapsing the last two into `None` would read as "nothing
/// came back" and put a skewed robot on the cold-start retry ladder — paying its
/// serve cost repeatedly to re-read bytes that cannot change, and reporting a
/// redeploy condition as though it might converge.
///
/// A caller must still never substitute an empty reply for a robot that did not
/// answer: an empty `runs` beside
/// [`RunsCompleteness::Settled`](cerulion_q::RunsCompleteness) is a positive claim
/// that the machine is running nothing, which no arm here supports.
///
/// Best-effort: never a hard dependency, never a hang.
pub fn query_robot_runs(
    session: &zenoh::Session,
    robot: &str,
    gather_window: Duration,
) -> RunsGatherOutcome {
    let selector = cerulion_q::runs_selector(robot);
    let replies = match session.get(&selector).timeout(gather_window).wait() {
        Ok(replies) => replies,
        Err(e) => {
            tracing::debug!(
                robot = %robot,
                error = %e,
                "runs GET could not be issued — this robot's runs stay UNKNOWN"
            );
            return RunsGatherOutcome::NoReply;
        }
    };
    let mut payloads: Vec<Vec<u8>> = Vec::new();
    while let Ok(reply) = replies.recv() {
        if let Ok(sample) = reply.into_result() {
            payloads.push(sample.payload().to_bytes().to_vec());
        }
    }
    classify_robot_runs(
        robot,
        cerulion_q::gather_runs_outcome(payloads.iter().map(|p| p.as_slice())),
    )
}

/// The POST-GET half of [`query_robot_runs`]: attribute a decoded
/// reply to the queried identity, report an unusable one LOUDLY, and PRESERVE
/// which of the three arms it was.
///
/// Extracted so the arm that matters most is reachable without a peer: the
/// difference between `Ignored` and `NoReply` is invisible in the returned data
/// (both contribute no reply) and visible only in what the CALLER then does —
/// stop asking, or keep retrying at the robot's ~620 ms serve. A collapse to
/// `NoReply` here is therefore a silent behavioural revert, and this is where an
/// oracle can catch it.
///
/// Pure apart from the `warn!`, which is deliberately on this side: it fires ONCE
/// per robot per query, at the one place that knows both the identity and the
/// reason, and it names the remedy (a redeploy) rather than implying a wait.
fn classify_robot_runs(robot: &str, outcome: RunsGatherOutcome) -> RunsGatherOutcome {
    match outcome {
        RunsGatherOutcome::Decoded(reply) => {
            RunsGatherOutcome::Decoded(attribute_runs_to_queried(reply, robot))
        }
        RunsGatherOutcome::Ignored(reason) => {
            tracing::warn!(
                robot = %robot,
                reason = %reason,
                "runs reply from this robot could not be used — its runs stay \
                 UNKNOWN and RETRYING WILL NOT HELP (a wire skew or corruption: rebuild \
                 and redeploy the robot's cerulion binaries)"
            );
            RunsGatherOutcome::Ignored(reason)
        }
        // No reply at all — an older robot, or one with no query surface. Silent:
        // this is the expected back-compat path, and the ONE arm on which a retry
        // is sensible.
        RunsGatherOutcome::NoReply => RunsGatherOutcome::NoReply,
    }
}

/// Pair one runs worker's JOIN RESULT with the robot it was asked
/// about, classifying a PANIC as the silence it is.
///
/// Extracted so the panic arm is drivable at all: reaching it through
/// [`query_robot_runs_all`] needs a live zenoh session AND a worker that unwinds,
/// which no hermetic arm can arrange. Taking `robot` and returning the PAIR (rather
/// than just the outcome) also puts the identity INSIDE the tested unit, so the
/// regression this closes — a name lost with the thread that carried it — is
/// covered rather than merely the mapping.
///
/// A panic is a bug in THIS binary, not a fact about the robot, so it is LOUD. But
/// it is still classified as [`RunsGatherOutcome::NoReply`], which is exactly true
/// downstream: nothing came back, the desk learned nothing about that machine, and
/// [`RunsHarvest::absorb`] routes it onto `silent` where it forbids the absence
/// claim. Dropping it instead would leave the robot on NO list, which is the one
/// state the coverage contract does not allow.
fn joined_runs_outcome(
    robot: &str,
    joined: std::thread::Result<RunsGatherOutcome>,
) -> (String, RunsGatherOutcome) {
    let outcome = joined.unwrap_or_else(|_| {
        tracing::error!(
            robot = %robot,
            "the runs GET worker for this robot PANICKED — reporting it as SILENT \
             (it was asked and nothing came back), so the answer can never be read as 'this \
             robot is running nothing'"
        );
        RunsGatherOutcome::NoReply
    });
    // hot-path-alloc-ok: cold — one per robot per runs GET on the control plane.
    (robot.to_string(), outcome)
}

/// GET the live runs of EACH robot in `robots` CONCURRENTLY (scoped
/// threads, like the catalog/schema gathers), folding the per-robot outcomes into
/// a [`RunsHarvest`]. The added remote wait stays ~ONE `gather_window` regardless
/// of robot count.
///
/// `replies` is a possibly-STRICT SUBSET of `robots` and the caller reads it that
/// way: a robot present in `robots` and absent from the result is UNKNOWN, not
/// idle. A robot in `unusable` is a DIFFERENT kind of absent — it answered, and a
/// retry will fetch the same unusable bytes at the same serve cost.
///
/// # EVERY robot in `robots` comes back on exactly one list
///
/// That is the contract `silent` bought, and a panicking worker was the one thing
/// that could still break it. The catalog and schema siblings drop a panicked
/// thread — correct for THEM, because their return type is a bare `Vec` of replies
/// with no vocabulary for "asked, and unknown", and their callers treat a missing
/// robot as an enrichment that did not arrive. Here a missing robot is a COVERAGE
/// claim, so a dropped worker would let the fold report `Settled` over a machine
/// nobody heard from. A panic is therefore classified rather than swallowed: it
/// logs LOUDLY (it is a bug in this binary, not a fact about the robot) and lands
/// on `silent`, which forbids the absence claim.
pub fn query_robot_runs_all(
    session: &zenoh::Session,
    robots: &[String],
    gather_window: Duration,
) -> RunsHarvest {
    gather_runs_over(robots, |robot| {
        query_robot_runs(session, robot, gather_window)
    })
}

/// The CONCURRENCY + JOIN + ACCOUNTING half of
/// [`query_robot_runs_all`], with the per-robot GET taken as a parameter.
///
/// Split out because the panic arm is otherwise undrivable: reaching it through
/// the public function needs a live zenoh session AND a worker that unwinds, so
/// with the query inlined the ONE thing a test could reach was
/// [`joined_runs_outcome`] — and a pure arm over that classifier cannot see the
/// CALL SITE reverting to a closure that pairs the identity INSIDE the worker,
/// which is precisely the shape that loses it. MEASURED: with the query inlined,
/// restoring `filter_map(|h| h.join().ok())` left every arm in this module green.
///
/// `query` is `Sync` because each worker borrows it; it is called EXACTLY once
/// per robot, on its own thread.
fn gather_runs_over(
    robots: &[String],
    query: impl Fn(&str) -> RunsGatherOutcome + Sync,
) -> RunsHarvest {
    if robots.is_empty() {
        return RunsHarvest::default();
    }
    let query = &query;
    let outcomes: Vec<(String, RunsGatherOutcome)> = std::thread::scope(|s| {
        // The robot's IDENTITY is paired with its handle OUT HERE, not returned
        // from inside the closure. That is the whole of the panic fix: a closure
        // that returns `(robot, outcome)` loses BOTH halves when it unwinds, so a
        // `join().ok()` could only drop the worker entirely — and a dropped worker
        // is a robot that was ASKED, is UNKNOWN, and is named on no list, which is
        // exactly the coverage hole `RunsHarvest::silent` exists to close. Zipping
        // the original slice keeps the name available no matter how the worker
        // ended.
        let handles: Vec<_> = robots
            .iter()
            .map(|robot| s.spawn(move || query(robot)))
            .collect();
        robots
            .iter()
            .zip(handles)
            .map(|(robot, handle)| joined_runs_outcome(robot, handle.join()))
            .collect()
    });
    let mut harvest = RunsHarvest::default();
    for (robot, outcome) in outcomes {
        harvest.absorb(&robot, outcome);
    }
    harvest
}

/// Returns the DEMAND (ingress-interest) liveliness key expression prefix used
/// by Cerulion — the key-space the network bridge watch flips egress flags on.
pub fn liveliness_prefix() -> &'static str {
    LIVELINESS_PREFIX
}

/// Returns the ANNOUNCE (egress-presence) liveliness key expression
/// prefix — the discovery key-space `topic list` lists, distinct from
/// [`liveliness_prefix`] so an announce never flips an egress flag.
/// Announce keys carry the robot identity as their first chunk
/// (`cerulion_ann/{robot}{canonical topic}`) — parse via [`parse_announce_key`].
pub fn announce_prefix() -> &'static str {
    ANNOUNCE_PREFIX
}

// `demand_queryable_prefix` / `demand_queryable_key` /
// `demand_queryable_reply_key` / `recover_demand_topic` moved to
// [`crate::transport::cerulion_q`] (now `queryable_key` / `demand_reply_key` /
// `demand_selector` / `parse_query_verb`) when the demand mechanism folded into
// the unified `cerulion_q` verb-dispatched surface.

/// ONE observed transition in the ANNOUNCE key-space — a robot (or one of
/// its topics) appearing or disappearing, as the network reports it.
///
/// This is the EVENT half of the announce space. [`query_announce_entries`] POLLS it
/// (a bounded liveliness GET returning a snapshot); an [`AnnounceWatch`] SUBSCRIBES
/// to it and is told the moment a token appears or its declaring session dies — which
/// is what lets the desk sidebar fill itself in when a robot's `ros2 attach` comes up,
/// instead of waiting for somebody to click refresh.
///
/// `topic` is `None` for the BARE identity token (`cerulion_ann/{robot}` — robot
/// presence with no topic), exactly as [`parse_announce_key`] reports it.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum AnnounceEvent {
    /// A token became live: `Some(topic)` = that robot now announces that canonical
    /// topic; `None` = the robot's bare identity token (it is present).
    Alive {
        /// The announcing robot identity (the key's first chunk).
        robot: String,
        /// The canonical topic, or `None` for the bare identity token.
        topic: Option<String>,
    },
    /// A token went away — the declaring session dropped it (an undeclare, OR the
    /// whole gateway process dying, which drops EVERY token it held at once).
    Lost {
        /// The announcing robot identity.
        robot: String,
        /// The canonical topic, or `None` for the bare identity token.
        topic: Option<String>,
    },
}

impl AnnounceEvent {
    /// The announcing robot this event is about.
    pub fn robot(&self) -> &str {
        match self {
            AnnounceEvent::Alive { robot, .. } | AnnounceEvent::Lost { robot, .. } => robot,
        }
    }

    /// The canonical topic, or `None` for the bare identity token.
    pub fn topic(&self) -> Option<&str> {
        match self {
            AnnounceEvent::Alive { topic, .. } | AnnounceEvent::Lost { topic, .. } => {
                topic.as_deref()
            }
        }
    }
}

/// Classify ONE announce liveliness sample into an [`AnnounceEvent`].
///
/// PURE (no zenoh types) so the mapping is oracle-testable: `is_put` is the sample
/// kind (`Put` = the token became live / was replayed from history; `Delete` = it went
/// away). A key that [`parse_announce_key`] rejects yields `None` — the caller skips
/// it, and `parse_announce_key` has already warned loudly about the malformed key.
pub fn classify_announce_sample(key: &str, is_put: bool) -> Option<AnnounceEvent> {
    let (robot, topic) = parse_announce_key(key)?;
    Some(if is_put {
        AnnounceEvent::Alive { robot, topic }
    } else {
        AnnounceEvent::Lost { robot, topic }
    })
}

/// A live SUBSCRIPTION to the ANNOUNCE key-space — the event source behind
/// the desk's self-updating topic sidebar.
///
/// Announce tokens ARE zenoh liveliness tokens, so a liveliness subscriber is told
/// the moment one appears (`Put`) or its declaring session dies (`Delete`) — no
/// polling anywhere. `declare` sets `.history(true)`, so the tokens ALREADY live when
/// the watch starts are replayed as ordinary `Put` samples: a watch that starts after
/// a robot is up still learns the whole current state, and a consumer therefore never
/// needs a separate snapshot GET to bootstrap. (This mirrors the DEMAND-space watch
/// in `network.rs`, which sets `.history(true)` for exactly the same reason.)
///
/// SYNCHRONOUS by construction (zenoh's `Wait` resolver + a `recv_timeout` drain), so
/// a caller drives it from a plain thread with no tokio runtime — `cerulion-netd`
/// does precisely that, which is why this type's surface names no zenoh type.
pub struct AnnounceWatch {
    /// The declared liveliness subscriber. Dropping it undeclares the subscription.
    subscriber:
        zenoh::pubsub::Subscriber<zenoh::handlers::FifoChannelHandler<zenoh::sample::Sample>>,
}

impl AnnounceWatch {
    /// Declare a liveliness subscriber over `cerulion_ann/**` with history replay.
    ///
    /// # Errors
    ///
    /// Returns [`TransportError::Receive`] if the subscription cannot be declared.
    pub fn declare(session: &zenoh::Session) -> TransportResult<Self> {
        let key = format!("{ANNOUNCE_PREFIX}/**");
        let subscriber = session
            .liveliness()
            .declare_subscriber(&key)
            // Replay tokens already live at declaration as ordinary `Put` samples —
            // without it, a watch started after a robot booted would never learn about
            // that robot until it re-announced (i.e. effectively never).
            .history(true)
            .wait()
            .map_err(|e| TransportError::Receive {
                topic: "*".to_string(),
                reason: format!("announce liveliness subscribe failed: {e}"),
            })?;
        tracing::debug!(key = %key, "announce liveliness watch active");
        Ok(Self { subscriber })
    }

    /// Block up to `timeout` for the next announce transition.
    ///
    /// `None` means the window elapsed with nothing to report (NOT an error, and NOT
    /// "nothing is out there"), or the sample carried a malformed key that
    /// [`parse_announce_key`] already warned about. The caller loops.
    pub fn next_event(&self, timeout: Duration) -> Option<AnnounceEvent> {
        // `Err` = the window elapsed; `Ok(None)` = the subscription closed. Neither is
        // an event, and neither is an error the caller can act on — it loops.
        let sample = self.subscriber.recv_timeout(timeout).ok().flatten()?;
        classify_announce_sample(
            sample.key_expr().as_str(),
            matches!(sample.kind(), zenoh::sample::SampleKind::Put),
        )
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// RESTORE the process-global panic hook on drop.
    ///
    /// Two problems with the bare `take_hook` / `set_hook` pair these tests used
    /// to write inline, and only the second is cosmetic.
    ///
    /// * **The hook is PROCESS-GLOBAL and libtest runs this binary's tests in
    ///   PARALLEL.** Two tests swapping concurrently can interleave so that one
    ///   `take_hook` captures the OTHER's silencing hook and then "restores" it —
    ///   after which the default hook is gone for the rest of the process and
    ///   every later panic in this binary prints nothing. That is a corruption
    ///   with an unbounded blast radius, not a lost line.
    /// * A panic between the swap and the restore leaks the silencing hook the
    ///   same way. `Drop` runs on the unwind, so the RAII form cannot.
    ///
    /// Paired with `#[serial(panic_hook)]` on every test that silences, which is
    /// what makes the first problem impossible rather than unlikely.
    ///
    /// SCOPE: the key serializes hook SWAPPERS against each other, which
    /// is the corruption. It does NOT stop an unrelated test panicking inside
    /// somebody's window and losing its message — a cosmetic loss that changes no
    /// verdict (a `#[should_panic]` arm and an `assert!` both fail on the unwind,
    /// not on the printing). Silencing at all is deliberate: these arms panic a
    /// worker ON PURPOSE, and its backtrace is noise a reader would have to learn
    /// to ignore.
    /// The boxed hook `std::panic::take_hook` hands back. Named because
    /// `clippy::type_complexity` refuses it inline (and it reads better twice).
    type BoxedPanicHook = Box<dyn Fn(&std::panic::PanicHookInfo<'_>) + Sync + Send + 'static>;

    struct PanicHookGuard(Option<BoxedPanicHook>);

    impl PanicHookGuard {
        /// Silence the panic hook until the returned guard drops.
        fn silence() -> Self {
            let previous = std::panic::take_hook();
            std::panic::set_hook(Box::new(|_| {}));
            Self(Some(previous))
        }
    }

    impl Drop for PanicHookGuard {
        fn drop(&mut self) {
            if let Some(previous) = self.0.take() {
                if std::thread::panicking() {
                    // `set_hook` PANICS when called from a panicking THREAD,
                    // which here would turn a plain test failure into a
                    // process ABORT (panic-in-panic). Skipping the restore is
                    // not an option either: the no-op silencer would stay
                    // process-global, and the NEXT guard's `silence()` would
                    // capture it as "previous" and faithfully restore the leak
                    // — silencing every later failure's diagnostics. Only this
                    // thread is panicking, so restore from a helper thread.
                    let _ = std::thread::spawn(move || std::panic::set_hook(previous)).join();
                } else {
                    std::panic::set_hook(previous);
                }
            }
        }
    }

    /// A failing test under the guard must (a) UNWIND rather than abort — if
    /// the unwind arm regressed to an inline `set_hook`, this whole BINARY
    /// aborts, its own loud signal — and (b) RESTORE the hook it captured, or
    /// the leaked silencer would be re-captured as "previous" by every later
    /// guard and swallow all subsequent failure diagnostics. The marker hook
    /// makes (b) behavioral: a post-guard panic must reach it.
    #[test]
    #[serial_test::serial(panic_hook)]
    fn a_failing_test_under_the_hook_guard_unwinds_and_restores_the_hook() {
        use std::sync::atomic::{AtomicBool, Ordering};
        static MARKER_FIRED: AtomicBool = AtomicBool::new(false);
        MARKER_FIRED.store(false, Ordering::SeqCst);
        std::panic::set_hook(Box::new(|_| MARKER_FIRED.store(true, Ordering::SeqCst)));

        let caught = std::panic::catch_unwind(|| {
            let _hook = PanicHookGuard::silence();
            panic!("a test assertion failing while the guard is live");
        });
        assert!(caught.is_err(), "the panic must unwind normally");
        assert!(
            !MARKER_FIRED.load(Ordering::SeqCst),
            "the guard really was silencing when the failure happened"
        );

        // If Drop leaked the silencer, this panic is swallowed and the marker
        // stays false — the propagation the helper-thread restore prevents.
        let _ = std::panic::catch_unwind(|| panic!("must reach the RESTORED hook"));
        let fired = MARKER_FIRED.load(Ordering::SeqCst);
        let _ = std::panic::take_hook(); // leave the default hook installed
        assert!(
            fired,
            "the guard must restore the captured hook after an unwind"
        );
    }

    #[test]
    fn test_liveliness_prefix() {
        assert_eq!(liveliness_prefix(), "cerulion_lv");
    }

    /// The sample→event mapping, against a HAND-WRITTEN table. Every row
    /// states the key and the sample kind, and names the event it must become —
    /// including the two shapes that are easy to get backwards (a `Delete` on the
    /// BARE identity key is "this whole robot went away", and a malformed key is
    /// skipped rather than turned into an event about a robot named `""`).
    #[test]
    fn announce_samples_classify_into_alive_and_lost_events() {
        // (key, is_put, expected)
        let cases: Vec<(&str, bool, Option<AnnounceEvent>)> = vec![
            (
                "cerulion_ann/go2/utlidar/cloud",
                true,
                Some(AnnounceEvent::Alive {
                    robot: "go2".to_string(),
                    topic: Some("/utlidar/cloud".to_string()),
                }),
            ),
            (
                "cerulion_ann/go2/utlidar/cloud",
                false,
                Some(AnnounceEvent::Lost {
                    robot: "go2".to_string(),
                    topic: Some("/utlidar/cloud".to_string()),
                }),
            ),
            // The BARE identity token — robot presence, no topic.
            (
                "cerulion_ann/go2",
                true,
                Some(AnnounceEvent::Alive {
                    robot: "go2".to_string(),
                    topic: None,
                }),
            ),
            (
                "cerulion_ann/go2",
                false,
                Some(AnnounceEvent::Lost {
                    robot: "go2".to_string(),
                    topic: None,
                }),
            ),
            // Malformed: `**` matches zero chunks, so the bare namespace key really
            // does arrive. It is SKIPPED, never an event about a robot named "".
            ("cerulion_ann", true, None),
            ("cerulion_ann/", true, None),
            ("cerulion_ann//x", true, None),
            ("cerulion_lv/utlidar/cloud", true, None),
        ];
        for (key, is_put, expected) in cases {
            assert_eq!(
                classify_announce_sample(key, is_put),
                expected,
                "key {key:?} (put={is_put})"
            );
        }
    }

    /// The accessors report the event's own robot/topic on BOTH arms — a
    /// `Lost` that reported the robot of an `Alive` (or dropped the topic) would send
    /// the desk chasing the wrong row.
    #[test]
    fn announce_event_accessors_report_each_arm_honestly() {
        let alive = AnnounceEvent::Alive {
            robot: "go2".to_string(),
            topic: Some("/tf".to_string()),
        };
        let lost = AnnounceEvent::Lost {
            robot: "orin".to_string(),
            topic: None,
        };
        assert_eq!(alive.robot(), "go2");
        assert_eq!(alive.topic(), Some("/tf"));
        assert_eq!(lost.robot(), "orin");
        assert_eq!(lost.topic(), None);
    }

    /// A decoded reply/catalog is ATTRIBUTED to the identity
    /// the desk QUERIED (the announce identity in the explicit key), OVERRIDING
    /// a divergent self-attribution in the payload — which closes the
    /// gap where a robot announcing as `go2` would display as its OS hostname
    /// `ubuntu`. Hand oracle: feed a payload self-stamped `ubuntu`, queried
    /// `go2` ⇒ the returned `robot` is `go2` (and the rest is untouched); the
    /// already-matching case is a no-op (anti-tautology on the payload fields).
    #[test]
    fn decoded_reply_is_attributed_to_the_queried_announce_identity() {
        use cerulion_q::{SchemaDoc, SchemaEncoding};

        // SCHEMA reply: divergent self-attribution is overridden with the query
        // key; docs/requested/error are preserved verbatim.
        let served = SchemaReply::found(
            "ubuntu",
            "acme/Widget",
            vec![SchemaDoc {
                qualified: "acme/Widget".to_string(),
                encoding: SchemaEncoding::Msg,
                text: "float64 x\n".to_string(),
                deps: vec![],
            }],
        );
        let out = attribute_schema_to_queried(served.clone(), "go2");
        assert_eq!(
            out.robot, "go2",
            "the DISPLAYED robot is the queried identity"
        );
        assert_eq!(out.requested, served.requested);
        assert_eq!(out.docs, served.docs);
        assert_eq!(out.version, served.version);
        // Already-matching ⇒ no-op (the value is the queried key either way).
        let same = attribute_schema_to_queried(SchemaReply::found("go2", "p/T", vec![]), "go2");
        assert_eq!(same.robot, "go2");

        // CATALOG reply: same override, entries/version preserved.
        let served_cat = CatalogReply {
            version: cerulion_q::CATALOG_WIRE_VERSION,
            robot: "ubuntu".to_string(),
            entries: Vec::new(),
            error: None,
        };
        let out_cat = attribute_catalog_to_queried(served_cat.clone(), "go2");
        assert_eq!(out_cat.robot, "go2");
        assert_eq!(out_cat.version, served_cat.version);
        assert_eq!(out_cat.entries, served_cat.entries);

        // `classify_robot_runs` PRESERVES which arm it was: the
        // difference between "answered unusably" and "did not answer" is invisible
        // in the data (neither yields a reply) and decides only what the CALLER
        // does next: stop asking, or keep retrying at the robot's ~620 ms serve.
        let decoded = classify_robot_runs(
            "go2",
            cerulion_q::RunsGatherOutcome::Decoded(cerulion_q::build_runs_reply(
                "ubuntu",
                [],
                Vec::new(),
                cerulion_q::RunsCompleteness::Settled,
            )),
        );
        match decoded {
            cerulion_q::RunsGatherOutcome::Decoded(reply) => assert_eq!(
                reply.robot, "go2",
                "a decoded reply is attributed to the QUERIED identity"
            ),
            other => panic!("expected Decoded, got {other:?}"),
        }
        let skew = cerulion_q::RunsDecodeError::UnknownVersion {
            got: 2,
            supported: 1,
        };
        assert_eq!(
            classify_robot_runs("go2", cerulion_q::RunsGatherOutcome::Ignored(skew.clone())),
            cerulion_q::RunsGatherOutcome::Ignored(skew),
            "an UNUSABLE answer must stay distinguishable from silence — collapsing \
             it to NoReply puts a skewed robot back on the retry ladder, re-reading \
             bytes that cannot change"
        );
        assert_eq!(
            classify_robot_runs("go2", cerulion_q::RunsGatherOutcome::NoReply),
            cerulion_q::RunsGatherOutcome::NoReply,
            "silence stays silence — the one arm on which a retry is sensible"
        );

        // RUNS reply: the same override, and here it is what robot attribution
        // rests on. A run row's `robot` is a property of the KEY it came back on,
        // so a machine whose hostname drifted from its announce identity cannot
        // mint a second row in the desk's fold.
        let served_runs = cerulion_q::build_runs_reply(
            "ubuntu",
            [cerulion_q::RunEntry::new(
                0x2A,
                "perception",
                7_000,
                crate::transport::run_registry::RunState::Live,
                "name: perception\n",
                "{\"run_id\":\"0x0000000000000000000000000000002a\"}\n",
            )
            .expect("mint entry")],
            Vec::new(),
            cerulion_q::RunsCompleteness::Settled,
        );
        let out_runs = attribute_runs_to_queried(served_runs.clone(), "go2");
        assert_eq!(
            out_runs.robot, "go2",
            "the DISPLAYED robot is the queried identity, never the payload's"
        );
        // Everything the run row IS survives the re-attribution untouched.
        assert_eq!(out_runs.version, served_runs.version);
        assert_eq!(out_runs.runs, served_runs.runs);
        assert_eq!(out_runs.completeness, served_runs.completeness);
        assert_eq!(out_runs.undescribable, served_runs.undescribable);
        assert_eq!(out_runs.error, served_runs.error);
        // Already-matching ⇒ no-op (anti-tautology: the override is not a rename).
        let same_runs = attribute_runs_to_queried(
            cerulion_q::build_runs_reply(
                "go2",
                [],
                Vec::new(),
                cerulion_q::RunsCompleteness::Settled,
            ),
            "go2",
        );
        assert_eq!(same_runs.robot, "go2");
    }

    /// `RunsHarvest::absorb` routes each outcome to the list whose
    /// REMEDY it belongs to, and `is_terminal` reports only the one that cannot
    /// converge.
    ///
    /// It is the ONE fold both the fan-out gather and netd's single-robot plane
    /// arm use, so this oracle covers both: a robot that answered unusably is
    /// NAMED (with its reason, which is what an operator acts on), a robot that
    /// did not answer is NAMED TOO — on its own list — and an empty harvest is
    /// NOT terminal, since nobody answered and somebody still might.
    ///
    /// # Why a `NoReply` lands on a list of its own
    ///
    /// Letting a `NoReply` land in NEITHER list, on the stated
    /// grounds that "its absence from `replies` IS the report", holds only
    /// for a reader holding the ASKED set, and the desk-side fold does not: it
    /// receives replies and unusable answers and cannot reconstruct who was
    /// approached. So a half-answered LAN would arrive looking exactly like a
    /// fully-answered one, and the consumer would settle — a confident absence about
    /// machines nobody heard from. The three arms are three remedies (read it,
    /// redeploy it, wait or upgrade it), and the third needs a list of its own.
    #[test]
    fn a_runs_harvest_routes_each_outcome_to_the_remedy_it_belongs_to() {
        let reply = cerulion_q::build_runs_reply(
            "go2",
            [],
            Vec::new(),
            cerulion_q::RunsCompleteness::Settled,
        );
        let skew = cerulion_q::RunsDecodeError::UnknownVersion {
            got: 2,
            supported: 1,
        };

        let mut harvest = RunsHarvest::default();
        assert!(
            !harvest.is_terminal(),
            "an EMPTY harvest is not terminal — nobody answered, so a retry may help"
        );

        harvest.absorb("go2", cerulion_q::RunsGatherOutcome::Decoded(reply));
        harvest.absorb("orin", cerulion_q::RunsGatherOutcome::NoReply);
        assert_eq!(harvest.replies.len(), 1);
        assert_eq!(harvest.replies[0].robot, "go2");
        assert_eq!(
            harvest.silent,
            vec!["orin".to_string()],
            "a robot that did not answer must be NAMED — a reader holding only \
             `replies` cannot reconstruct who was asked, so an unnamed silence makes \
             a half-answered LAN indistinguishable from a fully-answered one and \
             licenses a settled absence about a machine nobody heard from"
        );
        assert!(
            harvest.unusable.is_empty(),
            "…on its OWN list, never the unusable one: naming it there would send an \
             operator to redeploy a robot that is merely older"
        );
        assert!(
            !harvest.is_terminal(),
            "and a silence is not terminal — somebody may still answer"
        );

        harvest.absorb("spot", cerulion_q::RunsGatherOutcome::Ignored(skew));
        assert_eq!(
            harvest.replies.len(),
            1,
            "one skew suppresses no good reply"
        );
        assert_eq!(harvest.unusable.len(), 1);
        assert_eq!(harvest.unusable[0].robot, "spot");
        assert!(
            harvest.unusable[0]
                .reason
                .contains("unknown runs wire version 2"),
            "the reason must reach the operator: {}",
            harvest.unusable[0].reason
        );
        assert!(
            harvest.is_terminal(),
            "an answered-but-unusable peer has CONVERGED — a retry re-reads the same \
             bytes at the robot's full serve cost"
        );
        assert_eq!(
            harvest.silent,
            vec!["orin".to_string()],
            "…and the skew does not move the silent robot: the three lists are three \
             remedies and each stays on its own"
        );
    }

    /// A PANICKING runs worker keeps its robot's NAME and is
    /// classified as SILENT — never dropped.
    ///
    /// A site that returns `(robot, outcome)` from INSIDE the worker and
    /// joins with `filter_map(|h| h.join().ok())` loses BOTH halves on an unwind,
    /// and the robot comes back on NO list at all. That is the one state the
    /// coverage contract forbids: `silent` exists so every asked robot is
    /// accounted for, and a robot on no list lets the fold report `Settled` over a
    /// machine nobody heard from — the confident-empty defect `silent` was added
    /// to close, re-entering through the panic door.
    ///
    /// The `Err` is a REAL join payload from a REAL panicking thread, not a
    /// hand-built one, so the arm exercises the shape `join()` actually produces.
    #[serial_test::serial(panic_hook)]
    #[test]
    fn a_panicking_runs_worker_is_named_and_classified_as_silent() {
        // A genuine panic payload. The hook is silenced so an EXPECTED unwind does
        // not print a backtrace into an otherwise-passing run.
        let _hook = PanicHookGuard::silence();
        let joined: std::thread::Result<cerulion_q::RunsGatherOutcome> =
            std::thread::spawn(|| panic!("the GET worker fell over")).join();
        assert!(joined.is_err(), "the fixture really did panic");

        let (robot, outcome) = joined_runs_outcome("orin", joined);
        assert_eq!(
            robot, "orin",
            "the robot's NAME survives its worker — it is paired OUTSIDE the closure \
             precisely so an unwind cannot take it with it"
        );
        assert!(
            matches!(outcome, cerulion_q::RunsGatherOutcome::NoReply),
            "a panic is classified as the silence it is: nothing came back, so the \
             desk learned nothing about this machine"
        );

        // …and it lands on `silent`, which is what forbids the absence claim.
        let mut harvest = RunsHarvest::default();
        harvest.absorb(&robot, outcome);
        assert_eq!(harvest.silent, vec!["orin".to_string()]);
        assert!(
            harvest.replies.is_empty() && harvest.unusable.is_empty(),
            "and on NEITHER of the other two: it did not answer, and it did not \
             answer unusably"
        );

        // ANTI-TAUTOLOGY: a healthy join passes through untouched, so the arm above
        // pins the PANIC arm rather than a function that answers `NoReply` to
        // everything.
        let reply = cerulion_q::build_runs_reply(
            "go2",
            [],
            Vec::new(),
            cerulion_q::RunsCompleteness::Settled,
        );
        let (robot, outcome) =
            joined_runs_outcome("go2", Ok(cerulion_q::RunsGatherOutcome::Decoded(reply)));
        assert_eq!(robot, "go2");
        assert!(matches!(outcome, cerulion_q::RunsGatherOutcome::Decoded(_)));
    }

    /// The FAN-OUT keeps a panicked worker's robot, driven
    /// through the REAL concurrency, the REAL join and the REAL accounting.
    ///
    /// The sibling arm above pins the CLASSIFIER; this one pins the CALL SITE, and
    /// the difference is the whole point. The broken shape — pair the
    /// identity INSIDE the worker, `filter_map(|h| h.join().ok())` — leaves the
    /// classifier untouched and correct, so a pure arm over it stays green while
    /// the robot vanishes from every list: a variant with that shape passes all
    /// 12 other arms in this module.
    ///
    /// Every robot in `robots` must come back on EXACTLY one list, so the oracle is
    /// a PARTITION over three shapes at once rather than three separate arms — a
    /// per-shape test cannot see one robot's outcome being attributed to another's
    /// name, which is the failure mode a zip has and a per-worker return does not.
    #[serial_test::serial(panic_hook)]
    #[test]
    fn the_fan_out_names_a_panicked_worker_rather_than_dropping_it() {
        let robots = vec![
            "go2".to_string(),
            "orin".to_string(),
            "spot".to_string(),
            "quiet".to_string(),
        ];

        let _hook = PanicHookGuard::silence();
        let harvest = gather_runs_over(&robots, |robot| match robot {
            "go2" => cerulion_q::RunsGatherOutcome::Decoded(cerulion_q::build_runs_reply(
                "go2",
                [],
                Vec::new(),
                cerulion_q::RunsCompleteness::Settled,
            )),
            "orin" => panic!("the GET worker for orin fell over"),
            "spot" => cerulion_q::RunsGatherOutcome::Ignored(
                cerulion_q::RunsDecodeError::UnknownVersion {
                    got: 2,
                    supported: 1,
                },
            ),
            _ => cerulion_q::RunsGatherOutcome::NoReply,
        });

        // The PARTITION: every asked robot on exactly one list, none invented.
        assert_eq!(harvest.replies.len(), 1);
        assert_eq!(harvest.replies[0].robot, "go2");
        assert_eq!(harvest.unusable.len(), 1);
        assert_eq!(harvest.unusable[0].robot, "spot");
        let mut silent = harvest.silent.clone();
        silent.sort();
        assert_eq!(
            silent,
            vec!["orin".to_string(), "quiet".to_string()],
            "the PANICKED robot is named beside the merely-silent one — a dropped worker \
             would put it on no list at all, so the fold could report `Settled` over a machine \
             nobody heard from"
        );

        // A panic must not be mistaken for a skew: its remedy is a bug report, and
        // `unusable` means REDEPLOY THAT ROBOT.
        assert!(
            !harvest.unusable.iter().any(|u| u.robot == "orin"),
            "a panic is OUR fault, not a wire skew on the robot: {:?}",
            harvest.unusable
        );

        // ANTI-TAUTOLOGY: a fan-out with NO panic partitions the same way, so the
        // arm above pins the panic arm rather than a gather that names everything
        // silent.
        let clean = gather_runs_over(&robots, |robot| match robot {
            "go2" | "orin" => cerulion_q::RunsGatherOutcome::Decoded(cerulion_q::build_runs_reply(
                robot,
                [],
                Vec::new(),
                cerulion_q::RunsCompleteness::Settled,
            )),
            _ => cerulion_q::RunsGatherOutcome::NoReply,
        });
        assert_eq!(clean.replies.len(), 2);
        let mut clean_silent = clean.silent.clone();
        clean_silent.sort();
        assert_eq!(clean_silent, vec!["quiet".to_string(), "spot".to_string()]);
    }

    /// The guard RESTORES the hook it replaced.
    ///
    /// Without an arm the RAII fix is unobservable — no other test in this
    /// binary asserts anything about the process-global hook, so a guard whose
    /// `Drop` merely dropped the old hook on the floor would leave every later
    /// panic in the process silent and the whole suite green. That is the
    /// failure mode the guard exists for, so it needs its own oracle.
    ///
    /// The oracle is a MARKER hook: it records that it ran, so "the hook came
    /// back" is a fact this test observes rather than a claim about pointers
    /// (boxed closures cannot be compared). Both readings are taken BEFORE the
    /// marker is uninstalled, so an assertion failure cannot leave it behind for
    /// the rest of the binary.
    #[serial_test::serial(panic_hook)]
    #[test]
    fn the_panic_hook_guard_restores_the_hook_it_replaced() {
        use std::sync::atomic::{AtomicBool, Ordering};
        static SAW_PANIC: AtomicBool = AtomicBool::new(false);

        let outer = std::panic::take_hook();
        std::panic::set_hook(Box::new(|_| SAW_PANIC.store(true, Ordering::SeqCst)));

        let silenced_while_held = {
            let _hook = PanicHookGuard::silence();
            let _ = std::panic::catch_unwind(|| panic!("deliberate, and silenced"));
            !SAW_PANIC.load(Ordering::SeqCst)
        };
        // The guard has dropped. If it restored, the marker hook runs again.
        let _ = std::panic::catch_unwind(|| panic!("deliberate, and audible"));
        let audible_after_drop = SAW_PANIC.load(Ordering::SeqCst);

        std::panic::set_hook(outer);

        assert!(
            silenced_while_held,
            "the guard must SILENCE while it lives — otherwise every arm that \
             panics a worker on purpose prints a backtrace a reader must learn \
             to ignore"
        );
        assert!(
            audible_after_drop,
            "the guard must RESTORE the hook it replaced. Dropping the old hook \
             instead leaves this binary's panic hook silenced for the rest of \
             the process — every later panic, in every unrelated test, prints \
             nothing"
        );
    }

    /// The catalog/schema FAN-OUT names a panicked worker's robot
    /// instead of dropping it — driven through the REAL concurrency, the REAL join
    /// and the REAL accounting.
    ///
    /// This is the runs fan-out's shape applied to the two gathers that kept `join().ok()`.
    /// The oracle is a PARTITION over three shapes at once, because a per-shape
    /// test cannot see one robot's outcome being attributed to another's name —
    /// the failure mode a zip has and a per-worker return does not.
    ///
    /// The `quiet`/`orin` pair is the discrimination this test is about: both
    /// contribute NO reply, and only one of them was actually asked. A join
    /// that drops a panicked worker makes them the same value.
    #[serial_test::serial(panic_hook)]
    #[test]
    fn the_reply_fan_out_names_a_panicked_worker_rather_than_dropping_it() {
        let robots = vec![
            "go2".to_string(),
            "orin".to_string(),
            "spot".to_string(),
            "quiet".to_string(),
        ];

        let _hook = PanicHookGuard::silence();
        let gathered: GatherReplies<String> = gather_replies_over(&robots, |robot| match robot {
            "go2" => Some("go2-catalog".to_string()),
            "spot" => Some("spot-catalog".to_string()),
            "orin" => panic!("the GET worker for orin fell over"),
            _ => None,
        });

        let mut replies = gathered.replies.clone();
        replies.sort();
        assert_eq!(
            replies,
            vec!["go2-catalog".to_string(), "spot-catalog".to_string()],
            "a panicked sibling must not sink the robots that DID answer — this \
             gather is best-effort enrichment and always was"
        );
        assert_eq!(
            gathered.panicked,
            vec!["orin".to_string()],
            "the PANICKED robot is NAMED. A bare `filter_map(|h| h.join().ok())` \
             drops the whole worker, so the robot lands on no list, in no log, \
             and the pass that lost it reads identically to one that asked everyone"
        );
        assert!(
            !gathered.is_complete(),
            "a pass that lost a worker asked fewer robots than it meant to"
        );
        // The DISCRIMINATION: `quiet` answered nothing and is NOT reported as
        // panicked. Collapsing the two is the defect, so an implementation
        // that reported every silent robot would satisfy the assertion above while
        // making the signal meaningless.
        assert!(
            !gathered.panicked.contains(&"quiet".to_string()),
            "a robot that was ASKED and said nothing is not a panicked worker: {:?}",
            gathered.panicked
        );

        // ANTI-TAUTOLOGY: the same robots with NO panic report a COMPLETE pass, so
        // the arm above pins the panic arm rather than a gather that names
        // everything incomplete.
        let clean: GatherReplies<String> = gather_replies_over(&robots, |robot| match robot {
            "go2" | "spot" => Some(format!("{robot}-catalog")),
            _ => None,
        });
        assert_eq!(clean.replies.len(), 2);
        assert!(
            clean.panicked.is_empty() && clean.is_complete(),
            "a healthy pass claims nothing and costs nothing: {:?}",
            clean.panicked
        );
    }

    /// An EMPTY robot set gathers nothing and claims a COMPLETE pass.
    ///
    /// The boundary matters because `is_complete()` is what gates an absence claim
    /// downstream: "we asked nobody" must not read as "we could not ask", which
    /// would demote a marker on every desk with no robot on it. That is the
    /// class where a permanently-unsettleable plane re-paid its grace forever.
    #[test]
    fn an_empty_robot_set_is_a_complete_pass_that_gathered_nothing() {
        let gathered: GatherReplies<String> =
            gather_replies_over(&[], |_| unreachable!("no robot to ask"));
        assert!(gathered.replies.is_empty());
        assert!(gathered.is_complete());
    }

    /// The panic is reported at ERROR and names the robot.
    ///
    /// The log is the operator's only window on WHICH robot was lost — nothing on
    /// the wire carries it — and the LEVEL is part of the claim: a worker unwinding
    /// is a defect in this binary, so it must not be filtered out beside the
    /// ordinary `warn!` a robot's unusable reply earns. The predicate matches the
    /// level TOKEN as well as the message, because a text-only filter passes a
    /// variant that keeps the words and drops the severity.
    #[tracing_test::traced_test]
    #[serial_test::serial(panic_hook)]
    #[test]
    fn a_panicked_worker_is_reported_at_error_naming_the_robot() {
        let robots = vec!["orin".to_string(), "go2".to_string()];
        let _hook = PanicHookGuard::silence();
        let gathered: GatherReplies<String> = gather_replies_over(&robots, |robot| match robot {
            "orin" => panic!("the GET worker for orin fell over"),
            _ => Some("go2-catalog".to_string()),
        });
        assert_eq!(gathered.panicked, vec!["orin".to_string()]);

        logs_assert(|lines: &[&str]| {
            let loud = lines
                .iter()
                .filter(|l| {
                    l.split_whitespace().any(|t| t == "ERROR")
                        && l.contains("PANICKED")
                        && l.contains("orin")
                })
                .count();
            if loud == 1 {
                Ok(())
            } else {
                Err(format!(
                    "expected exactly ONE loud ERROR naming the panicked robot, got \
                     {loud}: {lines:?}"
                ))
            }
        });
        // ANTI-TAUTOLOGY: the healthy sibling earns no panic line of its own, so
        // the count above cannot be satisfied by a reporter that fires on every
        // worker.
        logs_assert(|lines: &[&str]| {
            let about_go2 = lines
                .iter()
                .filter(|l| l.contains("PANICKED") && l.contains("go2"))
                .count();
            if about_go2 == 0 {
                Ok(())
            } else {
                Err(format!(
                    "a healthy robot must earn no panic line: {lines:?}"
                ))
            }
        });
    }

    /// The announce key-space and the
    /// `cerulion_q` query surface AGREE on the identity — a robot's announce
    /// presence (`cerulion_ann/{id}{topic}`) and its query surface
    /// (`cerulion_q/{id}/...`) both key on the SAME identity string, so a desk
    /// that harvests `id` from the announce space routes its GETs to the right
    /// surface by construction (ONE identity resolver feeds both). Hand oracle
    /// for a hostname-shaped identity.
    #[test]
    fn announce_and_query_surfaces_agree_on_the_identity() {
        let id = "orin-lab-3";
        let topic = "/lf/lowstate";

        // Announce presence carries `id` as its robot chunk; parse recovers it.
        let ann_key = format!("{}/{}{}", announce_prefix(), id, topic);
        assert_eq!(
            parse_announce_key(&ann_key),
            Some((id.to_string(), Some(topic.to_string()))),
            "the announce key recovers the identity harvested for the query"
        );

        // The query surface keys on the SAME identity chunk.
        let cat = cerulion_q::catalog_selector(id);
        let sch = cerulion_q::schema_selector(id, "pkg/T");
        assert_eq!(cat, format!("cerulion_q/{id}/catalog"));
        assert_eq!(sch, format!("cerulion_q/{id}/schema/pkg/T"));
    }

    /// The DEMAND (`cerulion_lv`) and ANNOUNCE (`cerulion_ann`)
    /// key-spaces are two DISTINCT top-level chunks — the load-bearing property
    /// that keeps a peer's `cerulion_lv/**` bridge watch from ever matching an
    /// announce (which would spuriously flip egress flags between two permissive
    /// robots). Neither prefix is a string-prefix of the other AT the chunk
    /// boundary: each is followed by a `/`, so a chunk-exact zenoh subscription on
    /// one first chunk never matches a key under the other. (There is no longer a
    /// third `cerulion_meta` beacon chunk — robot identity now rides the announce
    /// keys' robot chunk + mDNS.)
    #[test]
    fn token_key_spaces_are_two_distinct_top_level_chunks() {
        assert_eq!(liveliness_prefix(), "cerulion_lv");
        assert_eq!(announce_prefix(), "cerulion_ann");
        // The two are distinct.
        assert_ne!(announce_prefix(), liveliness_prefix());
        // Pairwise non-prefixing (both directions).
        assert!(!announce_prefix().starts_with(liveliness_prefix()));
        assert!(!liveliness_prefix().starts_with(announce_prefix()));
    }

    /// The DEMAND reply-key parser behind [`query_live_topics`] and
    /// [`query_live_topics_by_prefix`] — oracle vectors. (Announce keys carry a
    /// robot chunk and parse via [`parse_announce_key`], pinned
    /// below. The file's tests are pure/session-less by convention; the LIVE
    /// bounded-gather coverage lands with the CLI engine's `topic list`
    /// loopback e2e in `crates/cerulion_cli_engine/tests/topic_network_live_test.rs`.)
    #[test]
    fn recover_topic_from_key_oracle_demand_space() {
        // Happy path, demand space: strip the namespace, keep the slash.
        assert_eq!(
            recover_topic_from_key(liveliness_prefix(), "cerulion_lv/p/n/o"),
            Some("/p/n/o".to_string())
        );
        // The BARE namespace key (`**` matches zero chunks) is rejected —
        // never a phantom empty-name topic.
        assert_eq!(
            recover_topic_from_key(liveliness_prefix(), "cerulion_lv"),
            None
        );
        // A foreign key that doesn't carry the prefix at all is rejected.
        assert_eq!(
            recover_topic_from_key(liveliness_prefix(), "foreign/key"),
            None
        );
        // Cross-space keys never leak: an announce key under the DEMAND parser
        // fails the strip — the distinct-chunk property.
        assert_eq!(
            recover_topic_from_key(liveliness_prefix(), "cerulion_ann/p/n/o"),
            None
        );
    }

    /// [`parse_announce_key`] against HAND-WRITTEN oracle vectors —
    /// the inverse of the `cerulion_ann/{robot}{canonical}` announce key and
    /// the bare `cerulion_ann/{robot}` identity token.
    #[test]
    fn parse_announce_key_oracle() {
        // Topic announce: robot chunk split off, remainder keeps its slash.
        assert_eq!(
            parse_announce_key("cerulion_ann/go2/utlidar/cloud"),
            Some(("go2".to_string(), Some("/utlidar/cloud".to_string())))
        );
        // Single-segment topic under a robot.
        assert_eq!(
            parse_announce_key("cerulion_ann/go2/imu"),
            Some(("go2".to_string(), Some("/imu".to_string())))
        );
        // Bare identity token → robot presence, no topic.
        assert_eq!(
            parse_announce_key("cerulion_ann/go2"),
            Some(("go2".to_string(), None))
        );
        // The bare namespace is not an announce (`**` matches zero chunks).
        assert_eq!(parse_announce_key("cerulion_ann"), None);
        // Empty robot chunk (double slash) → rejected.
        assert_eq!(parse_announce_key("cerulion_ann//x"), None);
        // Missing prefix / foreign key → rejected.
        assert_eq!(parse_announce_key("foreign/key"), None);
        // Demand-space keys never leak through the announce parser (the
        // distinct-chunk property, parser half).
        assert_eq!(parse_announce_key("cerulion_lv/p/n/o"), None);
        // A hostile robot chunk with control chars is STRUCTURALLY fine here —
        // the parser is not a policy gate; the render seam sanitizes.
        assert_eq!(
            parse_announce_key("cerulion_ann/ev\u{1b}il/t"),
            Some(("ev\u{1b}il".to_string(), Some("/t".to_string())))
        );
    }

    /// Round-trip property — keys built exactly as
    /// [`TopicToken::announce`] / [`TopicToken::announce_identity`] build them
    /// (via the shared `announce_key_for`) parse back to the original
    /// `(robot, topic)` — including absolute mirror topics and the bare
    /// identity token.
    #[test]
    fn announce_key_round_trips_through_parse() {
        let cases: &[(&str, &str)] = &[
            ("go2", "/utlidar/cloud"),
            ("go2", "/lf/lowstate"), // absolute mirror shape (ros2 attach)
            ("e2e", "/a"),
            ("robot.alpha_1", "/deep/nested/topic/name"),
        ];
        for &(robot, canonical) in cases {
            let key = TopicToken::announce_key_for(robot, canonical).expect("valid announce key");
            assert_eq!(
                parse_announce_key(&key),
                Some((robot.to_string(), Some(canonical.to_string()))),
                "round-trip failed for key {key}"
            );
        }
        // The bare identity token round-trips to (robot, None).
        let key = TopicToken::announce_key_for("go2-main", "").expect("valid identity key");
        assert_eq!(
            parse_announce_key(&key),
            Some(("go2-main".to_string(), None))
        );
    }

    // The demand key-space oracles (prefix distinctness, wildcard
    // queryable key, concrete reply key, canonical-tail recovery) moved to
    // `crate::transport::cerulion_q`'s test module along with the functions,
    // rephrased for the unified `cerulion_q/{robot}/demand{topic}` grammar.

    /// The robot-chunk gate on the DECLARE side — an empty robot or
    /// one carrying `/` (which would corrupt the exact robot↔topic split) is
    /// rejected with a loud reason naming the offender.
    #[test]
    fn announce_key_for_rejects_empty_and_slashed_robot() {
        for bad in ["", "lab/go2", "/leading", "trailing/"] {
            let err = TopicToken::announce_key_for(bad, "/t")
                .expect_err("robot chunk with '/' or empty must be rejected");
            assert!(
                format!("{err}").contains("single non-empty key chunk"),
                "rejection must state the single-chunk contract; got: {err}"
            );
        }
        // Control: a clean single chunk is accepted.
        assert_eq!(
            TopicToken::announce_key_for("go2", "/t").expect("clean chunk"),
            "cerulion_ann/go2/t"
        );
    }
}
