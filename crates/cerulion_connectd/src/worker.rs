// SPDX-License-Identifier: AGPL-3.0-only
//! The desk-side re-inject worker: dial the robot's `cerulion/wire/1` ALPN, speak
//! the `cerulion_q` catalog/demand/schema vocabulary over the bidi control
//! stream, and spawn one per-topic re-inject reader per accepted demand
//! ("remote = local").
//!
//! # Frame path discipline
//!
//! One dedicated task per demanded topic OWNS its uni [`RecvStream`] and drives
//! [`read_frame`] to COMPLETION in a loop (never inside a `select!` — the framing
//! is not cancel-safe). Each frame is VALIDATED (`total_size` + `schema_hash`)
//! and re-injected via the zenoh-free
//! [`IngressInjector`](cerulion_core::transport::network::IngressInjector) on the
//! desk's `network:None` [`TransportManager`]. Shutdown is an `abort()` from the
//! parent (which DISCARDS the stream — a mid-frame abort is safe because the
//! stream is never read again). No unbounded buffering: each reader re-injects
//! inline into the bounded SHM queue.
//!
//! # Reader extraction
//!
//! The reader loop ([`run_topic_reader`] + [`TopicReinjectCounters`] /
//! [`TopicReinjectStats`]) was extracted into `cerulion_wireclient::reader` so
//! `cerulion_netd`'s iroh WAN plane drives the SAME re-inject implementation; it
//! is re-exported here so this module's public surface is unchanged. This module
//! keeps the SESSION driver ([`run_connect`]): dial, catalog, demand, and the
//! per-topic reader spawn/teardown.

use std::path::Path;
use std::sync::Arc;

use cerulion_core::transport::cerulion_q::CatalogReply;
use cerulion_core::TransportManager;
use cerulion_link::{
    accept_uni_frame_stream, alpn, build_endpoint, dial, direct_addr, open_frame_stream,
    read_frame, write_frame, EndpointAddr, EndpointConfig, RecvStream, SendStream,
    DEFAULT_MAX_FRAME_LEN,
};
use std::future::Future;

use crate::config::{ConnectConfig, DemandSet};
use crate::error::{ConnectError, ConnectResult};
use crate::protocol::{decode_first_reply, StreamPreamble, WireRequest, WireResponse};
// The SHARED revocation-epoch push substrate — the SAME decisions
// `cerulion-netd`'s WAN plane makes on ITS dial (one implementation, never a fork).
use cerulion_wireclient::epoch::{
    classify_epoch_reply, epoch_cache_path, note_transport_failure, prepare_epoch_push,
    sanitize_peer_text, EpochPushOutcome, EpochPushPlan,
};

// The per-topic re-inject reader (`run_topic_reader` + its counters +
// the snapshot row) moved to the neutral `cerulion_wireclient::reader` crate so
// `cerulion_netd` can reuse the wire vocabulary + dial-config parsers with no
// `netd → connectd` cyclic package edge. This `run_topic_reader` loop is CONNECTD's
// session-driver reader (netd's WAN plane drives its OWN `iroh_plane::run_reinject_reader`
// loop, not this one — the shared element is cerulion_core's `reinject_raw` PRIMITIVE;
// the two loops are not unified). Re-exported here so
// `cerulion_connectd::worker::{run_topic_reader, TopicReinjectCounters,
// TopicReinjectStats}` keeps resolving (the reinject e2e drives them by that path).
pub use cerulion_wireclient::reader::{
    run_topic_reader, TopicReinjectCounters, TopicReinjectStats,
};

/// The outcome of one `cerulion connect` session — the catalog, the topics that
/// were demanded, and the per-topic re-injection tallies.
#[derive(Debug, Clone)]
pub struct ConnectSummary {
    /// The robot's topic catalog (fetched on connect).
    pub catalog: CatalogReply,
    /// The topics the robot accepted a demand for.
    pub demanded: Vec<String>,
    /// The per-topic re-injection tallies at teardown.
    pub per_topic: Vec<TopicReinjectStats>,
    /// What happened to this session's revocation-epoch push. Reported as
    /// STATE (Principle #3) so "did this desk actually deliver the revocation it was
    /// carrying?" is answerable without grepping logs — exactly the observable netd's
    /// `last_push_outcome` exposes for the WAN plane.
    pub epoch_push: EpochPushOutcome,
}

impl ConnectSummary {
    /// The robot's name RENDERED SAFELY for an operator-facing log line or terminal.
    ///
    /// `catalog.robot` is whatever the ROBOT said about itself — fully peer-controlled,
    /// unbounded text that arrives over the wire. The desk is where it becomes visible
    /// output, so every line that names the robot goes through here: control characters
    /// (terminal escapes, CR overwrites, newlines that could forge a second log record,
    /// BEL) are neutered and the text is bounded, exactly as the classifier already
    /// treats the robot's `message`/`reason` text
    /// ([`cerulion_wireclient::epoch::sanitize_peer_text`]) and as `topic list` treats
    /// robot names.
    ///
    /// The raw value stays available on `catalog.robot` for machine consumers (the cache
    /// keying deliberately never uses it — see `push_epoch`); this is the ONE spelling
    /// for putting it in front of a human when a [`ConnectSummary`] is in hand.
    ///
    /// Sites that render the robot name BEFORE a summary exists (the session's
    /// catalog-received line, the binary's catalog print) call the same underlying
    /// [`sanitize_peer_text`] directly — same policy, same bound, no second convention.
    pub fn robot_display(&self) -> String {
        sanitize_peer_text(&self.catalog.robot)
    }
}

/// A TOPIC NAME rendered safely for an operator-facing log line or terminal.
///
/// Topic names are peer-controlled on every path that gets them from the robot's
/// catalog (`--all` demands exactly the names the robot listed) and on the uni-stream
/// preamble path (the robot picks the name outright). They are the same class of
/// untrusted text as the robot's own name, so they get the same treatment — this is a
/// NAMED alias rather than a second policy, so a reader of a log call site can see at a
/// glance that the string was neutered and why.
pub(crate) fn topic_display(topic: &str) -> String {
    sanitize_peer_text(topic)
}

/// Dial a robot, demand its topics, and re-inject their frames into `manager`'s
/// local SHM until `shutdown` resolves (or the robot's connection drops).
///
/// `catalog_sink` is invoked ONCE with the fetched catalog (the binary prints it;
/// tests pass a no-op). `manager` is the desk's `network:None` transport — the
/// re-injected topics become LOCAL SHM topics a `topic echo` / `viz` / subscriber
/// taps with ZERO changes.
///
/// # Errors
///
/// - [`ConnectError::Refused`] if the robot refuses the wire plane (unpaired desk
///   key / unclaimed robot / missing `CAP_OBSERVE`) — exit non-zero, pair first.
/// - [`ConnectError::Link`] / [`ConnectError::Protocol`] on a dial / stream /
///   decode failure.
pub async fn run_connect<S>(
    config: ConnectConfig,
    manager: Arc<TransportManager>,
    mut catalog_sink: S,
    shutdown: impl Future<Output = ()>,
) -> ConnectResult<ConnectSummary>
where
    S: FnMut(&CatalogReply),
{
    // 1. Bind the desk endpoint (wire ALPN only — the desk never accepts).
    let endpoint = build_endpoint(
        EndpointConfig::new(config.desk_seed)
            .with_alpns(vec![alpn::WIRE.to_vec()])
            .with_relay(config.relay.clone()),
    )
    .await?;
    tracing::info!(
        desk_id = %endpoint.id(),
        robot = %config.robot_eid,
        direct_addrs = config.direct_addrs.len(),
        "cerulion connect: desk endpoint bound; dialing robot wire ALPN"
    );

    // 2. Build the peer address (LAN direct-dial when addrs are given, else bare
    //    eid via relay/discovery).
    let peer: EndpointAddr = if config.direct_addrs.is_empty() {
        EndpointAddr::new(config.robot_eid)
    } else {
        direct_addr(config.robot_eid, config.direct_addrs.iter().copied())
    };

    // Bound on every robot control/preamble read.
    let timeout = config.robot_timeout;

    // 3. Dial the wire plane and open the bidi control stream.
    let connection = dial(&endpoint, peer, alpn::WIRE).await?;
    let (mut send, mut recv) = open_frame_stream(&connection).await?;

    // 4. Catalog — the presence answer + the schema-hash source. A refusal (an
    //    AcceptDecision on the skeleton path) surfaces here as ConnectError::Refused.
    let catalog = fetch_catalog(&mut send, &mut recv, timeout).await?;
    tracing::info!(
        // Peer-reported identity → sanitized + bounded before it reaches an
        // operator's terminal (same policy as `ConnectSummary::robot_display`).
        robot = %sanitize_peer_text(&catalog.robot),
        topics = catalog.entries.len(),
        "cerulion connect: catalog received"
    );
    catalog_sink(&catalog);

    // 4b. PUSH this desk's cached revocation epoch — UNCONDITIONALLY.
    //     Revocation propagation is not something a user opts into. This is by
    //     design: a flag here would let someone dial a robot while silently withholding
    //     a revocation they are holding, a footgun we would be creating. It is a no-op
    //     when nothing is cached, and it NEVER blocks or delays the connection — the
    //     same posture (and the same shared substrate) as netd's WAN plane.
    //
    //     Keyed by the name the DESK knows this robot by (`config.robot_name`), NEVER
    //     by the identity the robot reports about itself — see `push_epoch`. Ordered
    //     right after the catalog admission gate so an unpaired/unclaimed refusal keeps
    //     its own precise message and the robot's ACL is current for every demand that
    //     follows.
    let epoch_outcome = push_epoch(
        &mut send,
        &mut recv,
        config.robot_name.as_deref(),
        &catalog.robot,
        config.epoch_dir.as_deref(),
        timeout,
    )
    .await?;

    // 5. Resolve which topics to demand.
    let topics = resolve_demand_topics(&config.demand, &catalog);
    if topics.is_empty() {
        // Catalog-only (or --all over an empty catalog): nothing to stream.
        finish_control(&mut send, &endpoint).await;
        return Ok(ConnectSummary {
            catalog,
            demanded: vec![],
            per_topic: vec![],
            epoch_push: epoch_outcome,
        });
    }

    // 6. Demand each topic (best-effort per topic) + fetch/materialize its schema.
    //    A demand that the robot rejects (unknown/silent topic) is a loud warn,
    //    NOT a session failure — the other topics still stream.
    let mut accepted: Vec<(String, Option<u64>)> = Vec::new();
    for topic in &topics {
        if let Some(dir) = config.schemas_dir.as_deref() {
            fetch_and_materialize_schema(&mut send, &mut recv, topic, dir, timeout).await;
        }
        // A rejected demand (`false`) is already logged in `demand_topic`; the
        // other topics still stream.
        if demand_topic(&mut send, &mut recv, topic, timeout).await? {
            accepted.push((topic.clone(), catalog_hash(&catalog, topic)));
        }
    }
    if accepted.is_empty() {
        finish_control(&mut send, &endpoint).await;
        return Ok(ConnectSummary {
            catalog,
            demanded: vec![],
            per_topic: vec![],
            epoch_push: epoch_outcome,
        });
    }

    // 7. Accept one uni data stream per accepted demand; correlate each to its
    //    topic via the preamble; spawn a dedicated reader task (owns the stream +
    //    re-injects). The robot opens each uni stream + writes its preamble BEFORE
    //    replying DemandAccepted (tap.rs::demand), so every stream is already in
    //    flight — but each accept is bounded so a misbehaving / incompatible robot
    //    that ACKs a demand yet never opens the stream cannot hang the desk
    //    indefinitely (bounded waits everywhere). On timeout we stop accepting; the
    //    streams already accepted keep flowing.
    let mut readers: Vec<Reader> = Vec::with_capacity(accepted.len());
    for _ in 0..accepted.len() {
        let mut ustream =
            match tokio::time::timeout(timeout, accept_uni_frame_stream(&connection)).await {
                Ok(res) => res?,
                Err(_) => {
                    tracing::warn!(
                        expected = accepted.len(),
                        opened = readers.len(),
                        timeout_ms = timeout.as_millis(),
                        "cerulion connect: the robot accepted a demand but did not open its uni \
                         data stream in time — not waiting further (the streams already open keep \
                         flowing)"
                    );
                    break;
                }
            };
        // Bound the preamble read too — an opened-but-silent stream must not
        // hang setup. On timeout drop THIS stream (cancel-safe: it is never reused)
        // and keep going.
        let preamble_bytes =
            match tokio::time::timeout(timeout, read_frame(&mut ustream, DEFAULT_MAX_FRAME_LEN))
                .await
            {
                Ok(res) => res?,
                Err(_) => {
                    tracing::warn!(
                    timeout_ms = timeout.as_millis(),
                    "cerulion connect: a uni data stream opened but sent no preamble in time — \
                     dropping it and continuing"
                );
                    continue;
                }
            };
        let preamble: StreamPreamble = serde_json::from_slice(&preamble_bytes)
            .map_err(|e| ConnectError::Protocol(format!("uni-stream preamble: {e}")))?;
        let topic = preamble.topic;
        // f2: REFUSE a preamble naming a topic we did NOT demand. The robot could
        // otherwise mirror an arbitrary, never-requested topic into desk-local SHM
        // under a robot-chosen name + hash (bypassing the catalog cross-check). The
        // catalog hash for a demanded topic (None ⇒ the reader derives it from the
        // first frame — silent-at-catalog-time topics).
        let Some((_, expected_hash)) = accepted.iter().find(|(t, _)| t == &topic) else {
            tracing::warn!(
                topic = %topic_display(&topic),
                "cerulion connect: robot opened a uni stream for a topic we did NOT demand — \
                 REFUSING to mirror it (dropping the stream, no desk-local publisher created)"
            );
            continue;
        };
        let expected_hash = *expected_hash;
        let counters = Arc::new(TopicReinjectCounters::default());
        let handle = tokio::spawn(run_topic_reader(
            ustream,
            topic.clone(),
            catalog.robot.clone(),
            manager.clone(),
            counters.clone(),
            expected_hash,
        ));
        readers.push(Reader {
            topic,
            counters,
            handle,
        });
    }

    tracing::info!(
        streaming = readers.len(),
        "cerulion connect: re-injecting into desk-local SHM (remote = local); Ctrl-C or robot \
         disconnect to stop"
    );

    // 8. Run until shutdown OR the robot connection drops.
    tokio::pin!(shutdown);
    tokio::select! {
        _ = &mut shutdown => {
            tracing::info!("cerulion connect: shutdown signalled; tearing down");
        }
        reason = connection.closed() => {
            tracing::warn!(%reason, "cerulion connect: robot connection closed; tearing down");
        }
    }

    // 9. Teardown: abort every reader (dropping its stream + injector — releases
    //    the desk SHM publisher), snapshot counters, close the control stream +
    //    endpoint (the robot sees the control read error and releases its taps).
    let mut per_topic = Vec::with_capacity(readers.len());
    for reader in &readers {
        reader.handle.abort();
    }
    for reader in readers {
        let _ = reader.handle.await; // Err on abort — expected
        per_topic.push(reader.counters.snapshot(&reader.topic));
    }
    finish_control(&mut send, &endpoint).await;

    Ok(ConnectSummary {
        catalog,
        demanded: per_topic.iter().map(|s| s.topic.clone()).collect(),
        per_topic,
        epoch_push: epoch_outcome,
    })
}

/// A live per-topic reader: its topic, its shared counters, and its task handle.
struct Reader {
    topic: String,
    counters: Arc<TopicReinjectCounters>,
    handle: tokio::task::JoinHandle<()>,
}

/// The catalog's declared `schema_hash` for `topic`, if the entry carries one.
fn catalog_hash(catalog: &CatalogReply, topic: &str) -> Option<u64> {
    catalog
        .entries
        .iter()
        .find(|e| e.topic == topic)
        .and_then(|e| e.schema_hash)
}

/// Finish the control send stream and close the desk endpoint (best-effort).
async fn finish_control(send: &mut SendStream, endpoint: &cerulion_link::Endpoint) {
    let _ = send.finish();
    endpoint.close().await;
}

/// Resolve the demand set into concrete topic names against the catalog.
fn resolve_demand_topics(demand: &DemandSet, catalog: &CatalogReply) -> Vec<String> {
    match demand {
        DemandSet::CatalogOnly => vec![],
        DemandSet::All => catalog.entries.iter().map(|e| e.topic.clone()).collect(),
        DemandSet::Named(topics) => {
            for t in topics {
                if !catalog.entries.iter().any(|e| &e.topic == t) {
                    tracing::warn!(
                        topic = %topic_display(t),
                        "cerulion connect: requested topic is not in the robot's catalog — \
                         demanding it anyway (the robot will report if it does not exist)"
                    );
                }
            }
            topics.clone()
        }
    }
}

/// Send one control request and read its response (a single round-trip on the
/// bidi control stream). Framing runs to completion (never in a `select!`).
///
/// The reply read is BOUNDED: a robot that admits the connection (or ACKs a
/// verb) but then withholds its reply must not hang desk setup — the whole
/// catalog/demand/schema phase runs BEFORE the shutdown `select!`, so an un-timed
/// read here would wedge, uninterruptible. On timeout we abort the session (the
/// control stream is abandoned, so the mid-frame cancel is harmless).
async fn control_round_trip(
    send: &mut SendStream,
    recv: &mut RecvStream,
    request: &WireRequest,
    timeout: std::time::Duration,
) -> ConnectResult<Vec<u8>> {
    let bytes = serde_json::to_vec(request)
        .map_err(|e| ConnectError::Protocol(format!("encode request: {e}")))?;
    control_round_trip_encoded(send, recv, &bytes, timeout).await
}

/// The same round-trip over a PRE-ENCODED request frame — the seam the epoch
/// push uses, because the shared substrate encodes the frame itself in order to
/// size-check it against the peer's frame cap BEFORE anything is written.
async fn control_round_trip_encoded(
    send: &mut SendStream,
    recv: &mut RecvStream,
    frame: &[u8],
    timeout: std::time::Duration,
) -> ConnectResult<Vec<u8>> {
    // BOTH halves are bounded, exactly as `cerulion-netd`'s WAN plane bounds them.
    // The write can block indefinitely on its own: QUIC
    // stream flow control stalls a large frame once the peer stops reading, and the
    // revocation-epoch push is the one control frame that can approach the 16 MiB cap.
    // An unbounded write there would wedge the whole verb on a peer that admits the
    // connection and then goes quiet — the requirement that "the push must never block or
    // delay the connection", violated by the first control write that can stall.
    match tokio::time::timeout(timeout, write_frame(send, frame)).await {
        Ok(res) => res?,
        Err(_) => {
            return Err(ConnectError::Control(format!(
                "the robot did not read a control request within {}ms (a stalled peer: it \
                 admitted the connection then stopped reading) — aborting",
                timeout.as_millis()
            )))
        }
    }
    match tokio::time::timeout(timeout, read_frame(recv, DEFAULT_MAX_FRAME_LEN)).await {
        Ok(res) => Ok(res?),
        Err(_) => Err(ConnectError::Control(format!(
            "the robot did not answer a control request within {}ms (admit-then-stall) — aborting",
            timeout.as_millis()
        ))),
    }
}

/// Classify the robot's first control reply as a catalog. PURE over the reply bytes —
/// oracle-tested.
///
/// EVERY error variant this produces reaches an operator surface (`main.rs` renders
/// `ConnectError::Refused`'s reason at ERROR and every other variant through the error's
/// `Display`), and BOTH the refusal `reason` and the `Error { message }` are text the
/// ROBOT chose. They are neutered + bounded HERE, at the one boundary where peer bytes
/// become a `ConnectError`, so no present or future render site can leak a raw terminal
/// escape or a megabyte "reason" — the render sites cannot be forgotten because there is
/// nothing left for them to forget.
fn classify_catalog_reply(bytes: &[u8]) -> ConnectResult<CatalogReply> {
    match decode_first_reply(bytes) {
        Ok(WireResponse::Catalog(reply)) => Ok(reply),
        Ok(WireResponse::Error { message, .. }) => {
            Err(ConnectError::Control(sanitize_peer_text(&message)))
        }
        // `{:?}` escapes control chars but is UNBOUNDED — bound the rendered blob.
        Ok(other) => Err(ConnectError::Protocol(format!(
            "expected a catalog reply, got {}",
            unexpected_reply_display(&other)
        ))),
        Err(Some(reason)) => Err(ConnectError::Refused(sanitize_peer_text(&reason))),
        Err(None) => Err(ConnectError::Protocol(
            "the robot's first reply decoded as neither a wire response nor an accept \
             decision (an incompatible robot or a corrupt stream)"
                .to_string(),
        )),
    }
}

/// Fetch + decode the catalog. A robot REFUSAL (an AcceptDecision on the skeleton
/// path) becomes [`ConnectError::Refused`]; a non-catalog reply is a protocol
/// error. Peer text is sanitized by [`classify_catalog_reply`].
async fn fetch_catalog(
    send: &mut SendStream,
    recv: &mut RecvStream,
    timeout: std::time::Duration,
) -> ConnectResult<CatalogReply> {
    let bytes = control_round_trip(send, recv, &WireRequest::Catalog, timeout).await?;
    classify_catalog_reply(&bytes)
}

/// The `robot=` label used on the no-cache-directory path when this desk has no pinned
/// name for the peer either. A FIXED placeholder — deliberately not the peer's
/// self-reported name, so peer-controlled text never reaches a log line on a path that
/// does not key anything by a name.
const NO_DESK_NAME_LABEL: &str = "<unpinned robot>";

/// Push this desk's cached revocation epoch for `robot` — unconditionally
/// (by design). This is the `cerulion connect` half of "desks push; robots
/// never poll": a robot has no outbound cloud connection, so every party that dials it
/// is the delivery mechanism, and a desk that is holding a revocation must never be
/// able to dial while silently withholding it. There is deliberately NO flag, env gate
/// or config knob.
///
/// Every decision (what to send, the frame-size guard, what the answer meant) is the
/// SHARED [`cerulion_wireclient::epoch`] substrate — the exact code `cerulion-netd`'s
/// WAN plane runs — so the two desk paths can never diverge.
///
/// # Which identity keys the cache
///
/// `desk_robot_name` — the name THIS DESK knows the robot by (`~/.cerulion/robots.toml`,
/// pinned by `cerulion pair`, resolved by `cerulion connect`) — and NEVER
/// `peer_reported_name`, the identity the robot puts in its own catalog. The desk dials
/// an eid it chose and iroh authenticates; the cache is filed under the desk's own name
/// for that robot. Keying the lookup by peer-supplied text would let the dialed robot
/// choose WHICH cached artifact it receives — another robot's signed epoch, whose
/// revoked account/device ids are none of its business (the robot would reject an epoch
/// bound to a different robot, but it would still have been handed it).
///
/// With no desk-verified name the push is skipped as
/// [`EpochPushOutcome::UnverifiedRobotIdentity`] with a loud remediation — the desk
/// never falls back to the peer's word. The peer-reported name is used ONLY to warn when
/// it disagrees with the desk's (an operator-visible mismatch, not an input to the
/// lookup).
///
/// # Which reason is reported when there are two
///
/// The cache-LOCATION check comes FIRST. A desk with no epochs directory at all (an
/// ephemeral key and no `CERULION_EPOCH_DIR`) holds nothing for ANY robot, so which name
/// it would have keyed by is moot: reporting `UnverifiedRobotIdentity` there — as the
/// original ordering did — prescribes "pair the robot" to a desk whose actual situation is
/// "there is no cache to push from", sending an operator to fix a problem they do not
/// have. The identity gate only matters once there IS a directory to look in.
///
/// Failure posture, identical to netd's:
///
/// - EVERY epoch outcome is a POLICY result: logged where an operator should care,
///   returned as state, and the session proceeds. No cache / a corrupt cache / an
///   oversized cache / an older robot / a refused epoch NEVER blocks the connection.
/// - ONLY a control round-trip TRANSPORT failure is fatal (`Err`), and not because of
///   the epoch: the framing is cancel-unsafe, so a timeout mid-push leaves the control
///   stream desynced and every later verb on it would fail anyway. That is the same
///   class `fetch_catalog` already fails on.
async fn push_epoch(
    send: &mut SendStream,
    recv: &mut RecvStream,
    desk_robot_name: Option<&str>,
    peer_reported_name: &str,
    epoch_dir: Option<&Path>,
    timeout: std::time::Duration,
) -> ConnectResult<EpochPushOutcome> {
    // The cache LOCATION first: with no epochs directory this desk holds nothing for any
    // robot, so `NoCacheDir` is the correct answer whatever name we would have keyed by
    // (see "Which reason is reported when there are two"). The label is a
    // fixed placeholder, never the peer's text, so nothing peer-controlled can reach the
    // substrate's log line on a path that does not use a name at all.
    let (robot, cache_path) = match (epoch_dir, desk_robot_name) {
        (None, name) => (name.unwrap_or(NO_DESK_NAME_LABEL), None),
        (Some(_), None) => {
            tracing::warn!(
                peer_reported_name = %sanitize_peer_text(peer_reported_name),
                "cerulion connect: this desk has no pinned name for the robot it dialed, so it \
                 cannot tell WHICH cached revocation epoch is meant — pushing NONE. \
                 The robot's own reported name is deliberately NOT used to pick a cache file (a \
                 dialed robot must not choose which artifact it is handed). Pair it \
                 (`cerulion pair <name>`, which pins it) or add a `[robots]` entry to \
                 ~/.cerulion/robots.toml, then dial it by that name."
            );
            return Ok(EpochPushOutcome::UnverifiedRobotIdentity);
        }
        (Some(dir), Some(robot)) => {
            if robot != peer_reported_name {
                // Not an error: the desk's name for a robot need not equal the robot's own
                // hostname. Worth ONE line, because it is also what a mis-filed cache
                // looks like from the operator's chair.
                tracing::info!(
                    desk_name = %robot,
                    peer_reported_name = %sanitize_peer_text(peer_reported_name),
                    "cerulion connect: the robot reports a different name than this desk knows \
                     it by; the revocation-epoch cache is keyed by the DESK's name"
                );
            }
            (robot, Some(epoch_cache_path(dir, robot)))
        }
    };
    // A one-shot session: always log the no-cached-epoch line (netd, which re-dials,
    // dedups it per process instead).
    let frame = match prepare_epoch_push(robot, cache_path.as_deref(), &|| true) {
        EpochPushPlan::Skip(outcome) => return Ok(outcome),
        EpochPushPlan::Send { frame } => frame,
    };
    let bytes = match control_round_trip_encoded(send, recv, &frame, timeout).await {
        Ok(b) => b,
        Err(e) => {
            note_transport_failure(robot, &e.to_string());
            return Err(e);
        }
    };
    Ok(classify_epoch_reply(robot, &bytes))
}

/// What a robot's reply to a `Demand` means. Split out of [`demand_topic`] so the
/// peer-text handling is oracle-testable: a `tracing::warn!` field is not, and a raw
/// `message = %message` field can slip into the very macro call that renders it.
#[derive(Debug, PartialEq, Eq)]
pub(crate) enum DemandReply {
    /// The robot accepted the demand for the topic we asked for.
    Accepted,
    /// The robot answered a verb-level error (non-fatal — other topics still stream).
    /// `message` is fully peer-chosen and is carried ALREADY SANITIZED + BOUNDED, so
    /// every render of it is safe by construction.
    Rejected { message: String },
    /// The reply did not decode, or was not a demand answer at all — fatal. `detail`
    /// becomes a [`ConnectError::Protocol`] message, so any peer text it embeds is
    /// neutered + bounded here.
    Protocol { detail: String },
}

/// Classify a robot's demand reply for `topic`. PURE over the reply bytes — oracle-tested.
pub(crate) fn classify_demand_reply(bytes: &[u8], topic: &str) -> DemandReply {
    let response: WireResponse = match serde_json::from_slice(bytes) {
        Ok(r) => r,
        Err(e) => {
            return DemandReply::Protocol {
                detail: format!("decode demand reply: {e}"),
            }
        }
    };
    match response {
        WireResponse::DemandAccepted { topic: t } if t == topic => DemandReply::Accepted,
        WireResponse::Error { message, .. } => DemandReply::Rejected {
            message: sanitize_peer_text(&message),
        },
        // Both halves are peer-derived: `topic` came from the robot's catalog under
        // `--all`, and `other` is the robot's own reply. `{:?}` escapes control chars but
        // is UNBOUNDED, so the rendered blob is bounded through the same sanitizer.
        other => DemandReply::Protocol {
            detail: format!(
                "expected DemandAccepted for '{}', got {}",
                topic_display(topic),
                unexpected_reply_display(&other)
            ),
        },
    }
}

/// Demand ONE topic. Returns `Ok(true)` on `DemandAccepted`, `Ok(false)` on a
/// verb-level robot error (logged, non-fatal), `Err(..)` on a control-stream
/// failure (fatal).
async fn demand_topic(
    send: &mut SendStream,
    recv: &mut RecvStream,
    topic: &str,
    timeout: std::time::Duration,
) -> ConnectResult<bool> {
    let bytes = control_round_trip(
        send,
        recv,
        &WireRequest::Demand {
            topic: topic.to_string(),
        },
        timeout,
    )
    .await?;
    match classify_demand_reply(&bytes, topic) {
        DemandReply::Accepted => Ok(true),
        // `message` arrives sanitized + bounded from `classify_demand_reply`.
        DemandReply::Rejected { message } => {
            tracing::warn!(
                topic = %topic_display(topic),
                message = %message,
                "cerulion connect: robot rejected the demand — skipping this topic (others \
                 still stream)"
            );
            Ok(false)
        }
        DemandReply::Protocol { detail } => Err(ConnectError::Protocol(detail)),
    }
}

/// Fetch `topic`'s schema closure and materialize it into `schemas_dir`
/// (best-effort — a not-found / materialize error is a debug/warn, never fatal;
/// most topics are built-in types the desk already compiled).
async fn fetch_and_materialize_schema(
    send: &mut SendStream,
    recv: &mut RecvStream,
    topic: &str,
    schemas_dir: &Path,
    timeout: std::time::Duration,
) {
    let bytes = match control_round_trip(
        send,
        recv,
        &WireRequest::Schema {
            topic: topic.to_string(),
        },
        timeout,
    )
    .await
    {
        Ok(b) => b,
        Err(e) => {
            tracing::debug!(topic = %topic_display(topic), error = %e, "cerulion connect: schema fetch failed");
            return;
        }
    };
    let reply: WireResponse = match serde_json::from_slice(&bytes) {
        Ok(r) => r,
        Err(e) => {
            tracing::debug!(topic = %topic_display(topic), error = %e, "cerulion connect: schema reply decode failed");
            return;
        }
    };
    match reply {
        WireResponse::Schema(schema) if schema.error.is_none() && !schema.docs.is_empty() => {
            match crate::schema::materialize_docs(schemas_dir, &schema.docs) {
                Ok(outcomes) => tracing::info!(
                    topic = %topic_display(topic),
                    docs = outcomes.len(),
                    "cerulion connect: materialized robot-served schema closure"
                ),
                Err(e) => tracing::warn!(
                    topic = %topic_display(topic), error = %e,
                    "cerulion connect: schema materialization failed (topic still streams; a \
                     built-in-typed subscriber can still decode)"
                ),
            }
        }
        // `schema.error` is the ROBOT's text — sanitized + bounded like every other
        // peer-chosen string on a desk surface (debug-level, but a log pipeline is a
        // pipeline: a megabyte "error" floods it just as well as a warn would).
        WireResponse::Schema(schema) => tracing::debug!(
            topic = %topic_display(topic),
            reason = %schema
                .error
                .as_deref()
                .map(sanitize_peer_text)
                .unwrap_or_else(|| "no docs".to_string()),
            "cerulion connect: robot served no schema closure (a built-in type the desk already \
             has, or an unknown type)"
        ),
        // The peer chose this whole reply. `{:?}` escapes control characters but is
        // UNBOUNDED, and `read_frame(recv, DEFAULT_MAX_FRAME_LEN)` admits a 16 MiB
        // frame — so a robot answering the Schema verb with
        // `WireResponse::Error { message: <16 MiB> }` lands HERE and floods the log
        // pipeline exactly as the sibling `schema.error` arm above did before it was
        // bounded. Render a bounded summary through the same sanitizer, matching
        // `classify_catalog_reply` / `classify_demand_reply`'s `Ok(other)` arms.
        other => tracing::debug!(
            topic = %topic_display(topic),
            reply = %unexpected_reply_display(&other),
            "cerulion connect: unexpected schema reply"
        ),
    }
}

/// How much of a peer-chosen reply's `{:?}` rendering is ever MATERIALIZED
/// ([`unexpected_reply_display`]).
///
/// Comfortably above [`cerulion_wireclient::epoch::sanitize_peer_text`]'s own 512-char
/// ceiling, so capping the render is BYTE-TRANSPARENT — the sanitizer keeps only the
/// first 512 chars either way. That transparency is pinned (not merely asserted) by
/// `capped_render_is_byte_transparent_versus_the_unbounded_render`, which recomputes the
/// unbounded rendering by hand and demands equality — so if the sanitizer's ceiling ever
/// grew past this cap, the test fails loudly instead of silently under-rendering.
const REPLY_RENDER_CAP_CHARS: usize = 1024;

/// A [`std::fmt::Write`] sink that accepts at most [`REPLY_RENDER_CAP_CHARS`] chars and
/// then ABORTS the formatting (returning `fmt::Error`, which `Debug` impls propagate
/// with `?`), keeping whatever was written so far.
///
/// The abort is the point: it stops `{:?}` from building the FULL rendering of a
/// peer-sized reply before any bound is applied.
struct CappedRender {
    out: String,
    remaining: usize,
}

impl CappedRender {
    fn new() -> Self {
        Self {
            out: String::new(),
            remaining: REPLY_RENDER_CAP_CHARS,
        }
    }
}

impl std::fmt::Write for CappedRender {
    fn write_str(&mut self, s: &str) -> std::fmt::Result {
        for c in s.chars() {
            if self.remaining == 0 {
                // Stop the whole `Debug` render here — the caller ignores this Err.
                return Err(std::fmt::Error);
            }
            self.out.push(c);
            self.remaining -= 1;
        }
        Ok(())
    }
}

/// Render a peer-chosen `Display` value BOUNDED (capped as produced, then
/// [`sanitize_peer_text`]) — the `Display` twin of [`unexpected_reply_display`].
///
/// Used by [`crate::pair`], whose peer-text surfaces interpolate
/// whole `serde_json::Value` replies and foreign error `Display`s — the same
/// materialize-then-bound transient this module's cap exists to kill. `&dyn Display`
/// (not a generic) keeps ONE monomorphization for a purely cold diagnostic path.
pub(crate) fn bounded_peer_display(value: &dyn std::fmt::Display) -> String {
    use std::fmt::Write as _;
    let mut sink = CappedRender::new();
    // An `Err` here is EXPECTED when the cap bites — see `CappedRender`.
    let _ = write!(sink, "{value}");
    sanitize_peer_text(&sink.out)
}

/// Render a peer-chosen [`WireResponse`] for a log line: a `{:?}` rendering (which
/// escapes control characters) CAPPED at [`REPLY_RENDER_CAP_CHARS`] as it is produced,
/// then put through [`sanitize_peer_text`] (which BOUNDS it to 512 chars and neuters
/// anything `{:?}` passed through). PURE — oracle-tested.
///
/// The bound is the point: a `WireResponse` deserialized from a frame as large as
/// `DEFAULT_MAX_FRAME_LEN` (16 MiB) can carry a megabyte-scale `message`, and a debug
/// line is still a log line. The cap sits inside the formatting for a reason:
/// `sanitize_peer_text(&format!("{reply:?}"))` bounded the OUTPUT but materialized the
/// whole input first — and `Debug` for `String` expands each control byte into a 6-char
/// `\u{1b}` escape, so a 16 MiB message of ESC bytes built a ~96 MB `String` on the desk
/// to throw all but 512 chars of it away. Two of the three call sites are unconditional
/// error paths (no level gate), so that transient was paid whether or not anything was
/// logged. Shared by every "unexpected reply" render so the three sites cannot drift.
pub(crate) fn unexpected_reply_display(reply: &WireResponse) -> String {
    use std::fmt::Write as _;
    let mut sink = CappedRender::new();
    // An `Err` here is EXPECTED when the cap bites — it is HOW the render is stopped,
    // and `sink.out` holds the accepted prefix either way.
    let _ = write!(sink, "{reply:?}");
    sanitize_peer_text(&sink.out)
}

#[cfg(test)]
mod tests {
    use super::*;
    use cerulion_core::transport::cerulion_q::{
        CatalogEntry, CatalogProvenance, CATALOG_WIRE_VERSION,
    };

    fn catalog(entries: Vec<(&str, Option<u64>)>) -> CatalogReply {
        CatalogReply {
            version: CATALOG_WIRE_VERSION,
            robot: "robo".to_string(),
            entries: entries
                .into_iter()
                .map(|(t, h)| CatalogEntry {
                    topic: t.to_string(),
                    schema_hash: h,
                    schema_name: None,
                    provenance: CatalogProvenance::Runtime,
                    // The connectd WAN catalog does not stamp the live
                    // producer count (the LAN gateway probes it via
                    // `topic_publisher_count`). `None` degrades the desk to the
                    // plain rendering (no liveness affordance) — correct, exactly like an older robot.
                    producer_count: None,
                    // Same for the data-flow liveness — a WAN catalog
                    // fixture carries none, which is a true UNKNOWN.
                    liveness: None,
                })
                .collect(),
            error: None,
        }
    }

    /// `CatalogOnly` demands nothing; `All` demands every catalog topic; `Named`
    /// demands exactly the requested topics (even one not in the catalog). Hand
    /// oracle.
    #[test]
    fn resolve_demand_topics_oracle() {
        let cat = catalog(vec![("/a", Some(1)), ("/b", None)]);
        assert_eq!(
            resolve_demand_topics(&DemandSet::CatalogOnly, &cat),
            Vec::<String>::new()
        );
        assert_eq!(
            resolve_demand_topics(&DemandSet::All, &cat),
            vec!["/a".to_string(), "/b".to_string()]
        );
        assert_eq!(
            resolve_demand_topics(&DemandSet::Named(vec!["/a".into(), "/z".into()]), &cat),
            vec!["/a".to_string(), "/z".to_string()],
            "a named topic absent from the catalog is still demanded (robot arbitrates)"
        );
    }

    /// The robot's own reported name is fully peer-controlled,
    /// unbounded text, and the session-end summary line is where it becomes an operator's
    /// terminal output — so [`ConnectSummary::robot_display`] neuters it and bounds it.
    ///
    /// Logged RAW by `main.rs`, `summary.catalog.robot` would let a hostile robot
    /// paint the operator's terminal (ESC-based CSI screen-clears), overwrite the line
    /// with CR, forge a SECOND log record with a newline, or dump a megabyte name into the
    /// log pipeline. Hand oracles for each of those shapes; the raw value is deliberately
    /// left intact on `catalog.robot` for machine consumers.
    #[test]
    fn robot_display_neuters_and_bounds_the_peer_reported_name() {
        let summary = |robot: &str| ConnectSummary {
            catalog: CatalogReply {
                robot: robot.to_string(),
                ..catalog(vec![])
            },
            demanded: vec![],
            per_topic: vec![],
            epoch_push: EpochPushOutcome::NoCacheDir,
        };

        // An ordinary name is untouched (including non-ASCII).
        assert_eq!(summary("go2").robot_display(), "go2");
        assert_eq!(summary("naïve-🤖").robot_display(), "naïve-🤖");

        // A terminal-escape screen-clear, a CR line overwrite, a BEL, and a NEWLINE that
        // would forge a second log record are ALL neutered to U+FFFD.
        assert_eq!(
            summary("a\u{1b}[2Jb\rc\u{7}d\ne").robot_display(),
            "a\u{fffd}[2Jb\u{fffd}c\u{fffd}d\u{fffd}e"
        );

        // A megabyte "name" cannot flood the log: bounded, with an EXPLICIT truncation
        // marker (never a silent clip that reads as the robot's whole name).
        let flood = summary(&"z".repeat(1_000_000));
        let shown = flood.robot_display();
        assert!(shown.len() < 600, "bounded, got {} bytes", shown.len());
        assert!(shown.ends_with("…(truncated)"));
        assert_eq!(
            flood.catalog.robot.len(),
            1_000_000,
            "the RAW value is untouched for machine consumers — only the display is bounded"
        );
    }

    /// A topic name is peer-controlled too — under `--all` the
    /// desk demands exactly the names the robot listed, and on the uni-stream path the
    /// robot picks the name outright — so every operator-facing render of one goes
    /// through [`topic_display`], the same policy and bound as the robot name.
    ///
    /// Sanitizing only the summary's robot name is not enough: the sibling sites (the
    /// catalog-received session line, the demand/schema log lines, and the binary's
    /// STDOUT catalog print) would still render peer text RAW, open to the exact attack
    /// the summary line guards against.
    #[test]
    fn topic_display_neuters_and_bounds_peer_chosen_topic_names() {
        // Ordinary topic names (including non-ASCII) are untouched.
        assert_eq!(topic_display("/scan"), "/scan");
        assert_eq!(topic_display("/naïve/🤖"), "/naïve/🤖");

        // Screen-clear, CR overwrite, BEL, and a newline that would forge an extra
        // catalog ROW on STDOUT are ALL neutered.
        assert_eq!(
            topic_display("/a\u{1b}[2Jb\rc\u{7}d\ne"),
            "/a\u{fffd}[2Jb\u{fffd}c\u{fffd}d\u{fffd}e"
        );

        // A megabyte "topic" cannot flood the terminal or the log pipeline.
        let shown = topic_display(&"z".repeat(1_000_000));
        assert!(shown.len() < 600, "bounded, got {} bytes", shown.len());
        assert!(shown.ends_with("…(truncated)"));

        // It is the SAME policy as the robot-name spelling, not a second convention.
        //
        // Scope: both `topic_display` and `robot_display` are
        // one-line delegations to `sanitize_peer_text`, so this equality holds BY
        // CONSTRUCTION and cannot detect a change to the policy (a weakened bound or a
        // narrowed control class moves both sides together). The literal oracles above
        // carry the actual policy coverage; this arm guards exactly ONE mutation —
        // someone inlining a DIFFERENT sanitizer into one of the two wrappers.
        let hostile = "x\u{1b}[2J\r\n\u{7}y";
        let summary = ConnectSummary {
            catalog: CatalogReply {
                robot: hostile.to_string(),
                ..catalog(vec![])
            },
            demanded: vec![],
            per_topic: vec![],
            epoch_push: EpochPushOutcome::NoCacheDir,
        };
        assert_eq!(topic_display(hostile), summary.robot_display());
    }

    // ---------------------------------------------------------------------
    // Peer text that becomes an operator-facing string is
    // sanitized at the ONE boundary where robot bytes are classified — so the
    // downstream `tracing::warn!` fields and `main.rs`'s error renders are safe by
    // construction rather than by remembering. Sweeping the log call sites instead of the
    // classifier can still miss a field: `message` on the very macro call it renders, or `reason`
    // on the `ConnectError::Refused` render. So these pins are over the PURE classifiers: they
    // catch the miss instead of restating the policy.
    // ---------------------------------------------------------------------

    /// A hostile string exercising every neutered class at once: ESC (a CSI
    /// screen-clear), CR (an overwrite), LF (a forged second log record), BEL.
    const HOSTILE: &str = "x\u{1b}[2Jy\rz\nw\u{7}v";
    /// What `sanitize_peer_text` must make of [`HOSTILE`] — hand-written, NOT a call
    /// to the sanitizer (that would be a self-compare).
    const HOSTILE_NEUTERED: &str = "x\u{fffd}[2Jy\u{fffd}z\u{fffd}w\u{fffd}v";

    /// The robot's verb-level demand `message` is peer-chosen: neutered + bounded
    /// before it can reach the warn line. A raw `message = %message` on the line
    /// directly below a sanitized one would reopen the hole.
    #[test]
    fn demand_reply_error_message_is_neutered_and_bounded() {
        let bytes = serde_json::to_vec(&WireResponse::Error {
            message: HOSTILE.to_string(),
            topic: None,
        })
        .unwrap();
        assert_eq!(
            classify_demand_reply(&bytes, "/scan"),
            DemandReply::Rejected {
                message: HOSTILE_NEUTERED.to_string()
            }
        );

        // A megabyte "message" cannot flood the log pipeline.
        let flood = serde_json::to_vec(&WireResponse::Error {
            message: "z".repeat(1_000_000),
            topic: None,
        })
        .unwrap();
        let DemandReply::Rejected { message } = classify_demand_reply(&flood, "/scan") else {
            panic!("a WireResponse::Error must classify as Rejected");
        };
        assert!(message.len() < 600, "bounded, got {} bytes", message.len());
        assert!(message.ends_with("…(truncated)"));

        // Control: an ordinary message is untouched, and the ACCEPT arm still works.
        let ok = serde_json::to_vec(&WireResponse::DemandAccepted {
            topic: "/scan".to_string(),
        })
        .unwrap();
        assert_eq!(classify_demand_reply(&ok, "/scan"), DemandReply::Accepted);
        assert_ne!(
            classify_demand_reply(&ok, "/other"),
            DemandReply::Accepted,
            "a reply for a DIFFERENT topic is not an acceptance"
        );
    }

    /// The unexpected-reply arm embeds BOTH the (peer-chosen) topic name and the
    /// robot's own reply — neither may carry a raw control byte, and the rendered
    /// reply blob is bounded (`{:?}` escapes but does not truncate).
    #[test]
    fn demand_reply_protocol_detail_neuters_topic_and_bounds_the_reply() {
        let bytes = serde_json::to_vec(&WireResponse::DemandAccepted {
            topic: "z".repeat(1_000_000),
        })
        .unwrap();
        let DemandReply::Protocol { detail } = classify_demand_reply(&bytes, HOSTILE) else {
            panic!("a mismatched DemandAccepted must classify as Protocol");
        };
        assert!(
            detail.contains(HOSTILE_NEUTERED),
            "the peer-chosen topic is neutered in the error text; got: {detail}"
        );
        assert!(
            !detail.contains('\u{1b}') && !detail.contains('\r') && !detail.contains('\n'),
            "no raw control byte survives into the error text; got: {detail}"
        );
        assert!(
            detail.len() < 700,
            "the rendered reply is bounded, got {} bytes",
            detail.len()
        );

        // Undecodable bytes are a Protocol error, not a panic or a silent accept.
        assert!(matches!(
            classify_demand_reply(b"not json", "/scan"),
            DemandReply::Protocol { .. }
        ));
    }

    /// The shared "unexpected reply" renderer bounds the peer's
    /// whole reply.
    ///
    /// Three sites render a peer-chosen `WireResponse` with `{:?}`:
    /// `classify_catalog_reply`'s `Ok(other)`, `classify_demand_reply`'s `other`, and
    /// `fetch_and_materialize_schema`'s `other` arm. The third was RAW (`?other`) —
    /// one arm below the bounded `schema.error` arm, reachable with the
    /// same stimulus at the same level: `read_frame(recv, DEFAULT_MAX_FRAME_LEN)`
    /// admits a 16 MiB frame, so a `WireResponse::Error { message: <megabytes> }`
    /// answered to the Schema verb landed there and flooded the log pipeline. All three
    /// now go through [`unexpected_reply_display`], pinned here.
    ///
    /// HAND ORACLES: a megabyte `message` is bounded with the explicit marker; a CSI
    /// escape does not survive raw; an ordinary reply still renders its content (the
    /// anti-tautology arm — a renderer that returned `""` would pass the bound asserts).
    ///
    /// Mutation scope:
    /// reverting THIS function to a bare `format!("{reply:?}")` fails arm **(a)** (the
    /// bound + marker asserts). Arm **(b)** is a DEFENSE-IN-DEPTH assert, not a mutation
    /// killer: `Debug` for `String` already escapes every `char::is_control()` byte into
    /// printable text, so (b) holds for ANY implementation routing through `{:?}`. It
    /// stays because it pins the CONTRACT ("no raw control byte reaches a log line")
    /// independently of which layer delivers it — a future renderer that stopped using
    /// `{:?}` would be caught by (b) and by nothing else here.
    #[test]
    fn unexpected_reply_render_is_bounded_and_neutered() {
        // (a) The flood shape — the exact stimulus the schema arm admitted raw.
        let flood = WireResponse::Error {
            message: "z".repeat(1_000_000),
            topic: None,
        };
        let rendered = unexpected_reply_display(&flood);
        assert!(
            rendered.chars().count() < 600,
            "a megabyte reply is bounded, got {} chars",
            rendered.chars().count()
        );
        assert!(rendered.ends_with("…(truncated)"));

        // (b) Terminal escapes: `{:?}` escapes them into `\u{1b}` text, and the
        //     sanitizer guarantees no RAW control byte can survive either way.
        let hostile = WireResponse::Error {
            message: HOSTILE.to_string(),
            topic: None,
        };
        let rendered = unexpected_reply_display(&hostile);
        assert!(
            !rendered.chars().any(|c| c.is_control()),
            "no raw control character survives: {rendered:?}"
        );

        // (c) Anti-tautology: an ordinary reply still renders its real content.
        let plain = WireResponse::DemandAccepted {
            topic: "/scan".to_string(),
        };
        let rendered = unexpected_reply_display(&plain);
        assert!(
            rendered.contains("DemandAccepted") && rendered.contains("/scan"),
            "the renderer must still carry the reply's content: {rendered}"
        );
        assert!(
            !rendered.contains("…(truncated)"),
            "a short reply is intact"
        );
    }

    /// [`unexpected_reply_display`] caps the `{:?}` rendering as it
    /// IS PRODUCED, so a peer-sized reply is never fully materialized — and doing so
    /// changes NOTHING about the output.
    ///
    /// (a) TRANSPARENCY / DRIFT GUARD — hand-recompute the UNBOUNDED rendering
    ///     (`sanitize_peer_text(&format!("{reply:?}"))`, the previous body) over a
    ///     reply whose full `Debug` far exceeds [`REPLY_RENDER_CAP_CHARS`], and demand
    ///     byte equality. This is the pin that keeps the cap transparent: it holds only while
    ///     the cap stays `>=` the sanitizer's own ceiling, so raising
    ///     `MAX_PEER_TEXT_LEN` past the cap fails HERE instead of silently truncating
    ///     operator-visible diagnostics. (Deliberately sized so the by-hand unbounded
    ///     leg is cheap — the megabyte case is (b).)
    ///
    /// (b) THE BOUND ON THE HOSTILE SHAPE — a multi-megabyte all-ESC `message`, whose
    ///     `Debug` expands each byte into a 6-char `\u{1b}` escape (the ~6× blow-up that
    ///     would make an unbounded transient ~96 MB for a 16 MiB frame). The OUTPUT is asserted
    ///     bounded; the absence of the transient is a property of the `CappedRender`
    ///     sink (documented there), not measured here — this crate has no allocation
    ///     probe.
    #[test]
    fn capped_render_is_byte_transparent_versus_the_unbounded_render() {
        // (a) 4096 chars of payload ⇒ full Debug ≫ REPLY_RENDER_CAP_CHARS (1024).
        let long = WireResponse::Error {
            message: "q".repeat(4096),
            topic: Some("/lidar/points".to_string()),
        };
        let unbounded_by_hand = sanitize_peer_text(&format!("{long:?}"));
        assert_eq!(
            unexpected_reply_display(&long),
            unbounded_by_hand,
            "capping the render must not change one byte of the rendered output"
        );
        // Anti-vacuity: the sanitizer really did bite on this input.
        assert!(unbounded_by_hand.ends_with("…(truncated)"));

        // (b) The hostile shape: 2 MiB of ESC ⇒ ~12 MB of `\u{1b}` text unbounded.
        let hostile_flood = WireResponse::Error {
            message: "\u{1b}".repeat(2 * 1024 * 1024),
            topic: None,
        };
        let rendered = unexpected_reply_display(&hostile_flood);
        assert!(
            rendered.chars().count() < 600,
            "a multi-megabyte hostile reply renders bounded, got {} chars",
            rendered.chars().count()
        );
        assert!(rendered.ends_with("…(truncated)"));
        assert!(
            !rendered.chars().any(|c| c.is_control()),
            "no raw control character survives: {rendered:?}"
        );
    }

    /// The catalog boundary: a robot REFUSAL reason and a verb-level `Error` message
    /// are both peer text, and both become a `ConnectError` `main.rs` renders — the
    /// refusal at ERROR level. Sanitize them where they enter the error, so every
    /// render site is safe.
    #[test]
    fn catalog_reply_neuters_refusal_reason_and_control_message() {
        // (a) The skeleton-path refusal (`AcceptDecision::Refuse`).
        let refusal = serde_json::to_vec(&cerulion_wireclient::protocol::AcceptDecision::Refuse {
            reason: HOSTILE.to_string(),
        })
        .unwrap();
        match classify_catalog_reply(&refusal) {
            Err(ConnectError::Refused(reason)) => assert_eq!(reason, HOSTILE_NEUTERED),
            other => panic!("expected a Refused, got {other:?}"),
        }

        // (b) A verb-level robot error becomes `Control`, rendered through the error's
        //     Display on `main.rs`'s catch-all arm.
        let verb_err = serde_json::to_vec(&WireResponse::Error {
            message: HOSTILE.to_string(),
            topic: None,
        })
        .unwrap();
        match classify_catalog_reply(&verb_err) {
            Err(ConnectError::Control(msg)) => assert_eq!(msg, HOSTILE_NEUTERED),
            other => panic!("expected a Control, got {other:?}"),
        }

        // (c) A megabyte refusal reason cannot flood the terminal.
        let flood = serde_json::to_vec(&cerulion_wireclient::protocol::AcceptDecision::Refuse {
            reason: "z".repeat(1_000_000),
        })
        .unwrap();
        match classify_catalog_reply(&flood) {
            Err(ConnectError::Refused(reason)) => {
                assert!(reason.len() < 600, "bounded, got {} bytes", reason.len());
                assert!(reason.ends_with("…(truncated)"));
            }
            other => panic!("expected a Refused, got {other:?}"),
        }

        // (d) CONTROL — a real catalog still decodes (the sanitizer is not in its path).
        let good =
            serde_json::to_vec(&WireResponse::Catalog(catalog(vec![("/scan", Some(7))]))).unwrap();
        let reply = classify_catalog_reply(&good).expect("a real catalog decodes");
        assert_eq!(reply.entries[0].topic, "/scan");
        assert_eq!(reply.robot, "robo");
    }

    /// `catalog_hash` returns the entry's declared hash, or `None` for a silent /
    /// absent topic.
    #[test]
    fn catalog_hash_oracle() {
        let cat = catalog(vec![("/a", Some(0xABCD)), ("/b", None)]);
        assert_eq!(catalog_hash(&cat, "/a"), Some(0xABCD));
        assert_eq!(catalog_hash(&cat, "/b"), None, "silent topic → no hash");
        assert_eq!(
            catalog_hash(&cat, "/missing"),
            None,
            "absent topic → no hash"
        );
    }
}
