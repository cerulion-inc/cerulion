// SPDX-License-Identifier: AGPL-3.0-only
//! Deterministic request-response (services) over Cerulion pub/sub.
//!
//! ROS 2's rmw layer makes services MANDATORY (rcl init fails on missing
//! symbols) and actions decompose into services + topics at the rcl
//! level — so `rmw_cerulion` cannot exist without a request-response
//! primitive. This module implements
//! **services as correlated topics** — the public contract carries an
//! explicit correlation envelope, and the carrier can later be swapped to
//! iceoryx2's native request-response ports behind the same API.
//!
//! # Topology
//!
//! ```text
//! client                                  server
//!   │  publish  svc/{service}/request       │  (one topic, N clients)
//!   │ ───────────────────────────────────►  │  FIFO drain (non-coalescing)
//!   │                                       │
//!   │  subscribe svc/{service}/reply/{guid} │  (one topic PER CLIENT)
//!   │ ◄───────────────────────────────────  │  lazy publisher per client
//! ```
//!
//! Per-client reply topics (instead of one shared reply topic) mean a
//! client is never woken by — and never drains — another client's
//! replies. The reply-topic name embeds the client GUID in hex (32
//! chars), keeping well within iceoryx2's service-name length limit for
//! realistic service names.
//!
//! # Wire format
//!
//! Each frame is a normal Cerulion wire frame published via
//! `publish_raw`:
//!
//! ```text
//! [WireHeader (32 B)][ServiceEnvelope (24 B)][opaque payload …]
//! ```
//!
//! The envelope carries the rmw correlation contract verbatim
//! (`rmw_request_id_t`: 16-byte writer GUID + i64 sequence number), so
//! the rmw bridge maps 1:1 with no translation table. The payload is
//! opaque bytes — for the rmw bridge that is the CDR-serialized
//! request/response (services are NOT the zero-copy hot path; the
//! design accepts 1 copy per service message, matching
//! `rmw_iceoryx`).
//!
//! # Taking: one message per call, never a drain-everything pass
//!
//! Both take paths —
//! [`ServiceClient::try_take_one_response`] and
//! [`ServiceServer::try_take_one_request`] — deliver AT MOST ONE message
//! per call and return whether they delivered one; call again to take the
//! next. That is the rmw take contract (`rmw_take_request` /
//! `rmw_take_response` hand ONE message to rcl per call), and it is the
//! only shape the sole production consumer of this module ever drives.
//!
//! There is deliberately NO drain-everything counterpart. A caller that
//! takes one at a time loses nothing: the take is FIFO and
//! non-coalescing, so what it does not take stays queued behind it
//! (Principle #6). A drain-everything call, by contrast, forces its
//! caller to buffer or discard the surplus it never asked for — and the
//! one caller that exists must hand back exactly one. The two halves of
//! the pair would only ever be reached from tests; they
//! are absent rather than kept as a symmetric-looking API that
//! production must remember not to call.
//!
//! # Determinism (Principles #2, #7)
//!
//! Nothing in this module reads the wall clock or any RNG:
//!
//! - Client GUIDs are a pure function of `(node_id, service, instance)`
//!   — see [`derive_client_guid`]. Same inputs → same GUID, every run.
//! - Sequence numbers are a per-client monotonic counter starting at 1.
//! - Timestamps come from the transport's [`Clock`] (the `VirtualClock`
//!   under replay).
//! - Requests are taken through the subscriber's FIFO, non-coalescing
//!   `try_receive_one` — NEVER the latest-wins `try_view` path (a
//!   coalesced request would be silently lost; Principle #6).
//!
//! Truth is in the data: correlation lives in the envelope bytes on the
//! wire, not in callback identity (Principle #2).
//!
//! # Loss bounds (read this before relying on services)
//!
//! The carrier is iceoryx2 pub/sub with a BOUNDED safe-overflow queue
//! (`subscriber_buffer_size`, default 16): when more than that many
//! requests (or responses) accumulate between drains, the OLDEST are
//! displaced. The drains themselves are FIFO and non-coalescing — the
//! loss can only happen in the queue behind them. Mitigations:
//!
//! - The server WARNS on per-client sequence gaps (the wire carries the
//!   evidence — contiguous client sequences from 1), so displacement is
//!   never silent.
//! - `send_response` WARNS when a response reaches zero subscribers.
//! - There is NO timeout/retransmit layer here; that lives in the caller
//!   (rcl retries; native callers own their deadline policy).
//!
//! # Known limits
//!
//! - The `svc/` topic prefix is shared namespace: a plain data topic
//!   literally named `svc/x/request` would collide with service `x`'s
//!   request channel. Schema-hash gating degrades this loudly (foreign
//!   frames warn + skip) but treat `svc/` as RESERVED for services.
//! - `ServiceServer` keeps one lazily-created reply publisher per client
//!   GUID forever (deterministic, but unbounded under client churn with
//!   ever-fresh `instance` values). Restarting clients reuse their
//!   derived GUID and cost nothing new.
//! - The 16-byte GUID matches `RMW_GID_STORAGE_SIZE` on ROS 2 Iron and
//!   later (Humble used 24 — not a target).

use std::collections::BTreeMap;
use std::sync::Arc;

use crate::clock::Clock;
use crate::error::TransportResult;
use crate::wire::{fnv1a_hash, MaxSliceLen, WireHeader};

use super::failure_regime_latch::FailureRegimeLatch;
use super::frame_drop_latch::{
    report_envelope_intact, report_schema_hash_match, report_schema_hash_mismatch,
    report_short_envelope, FrameDropSite,
};
use super::publisher::CerulionPublisher;
use super::subscriber::CerulionSubscriber;
use super::{TopicServiceConfig, TransportManager};

/// Correlation envelope carried on every service frame, immediately
/// after the `WireHeader`. Mirrors `rmw_request_id_t` exactly:
/// 16-byte client (writer) GUID + i64 sequence number.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ServiceEnvelope {
    /// The CLIENT's GUID — identifies which client a request came from
    /// (and therefore which reply topic the response goes to). Mint via
    /// [`derive_client_guid`] for replay-stable values.
    pub client_guid: [u8; 16],
    /// Client-minted sequence number; the response echoes it back
    /// verbatim. Starts at 1, monotonic per client.
    pub sequence: i64,
}

impl ServiceEnvelope {
    /// Wire size of the envelope in bytes.
    pub const SIZE: usize = 24;

    /// Serialize into `buf[..Self::SIZE]` (little-endian sequence).
    pub fn write_to_buf(&self, buf: &mut [u8]) {
        buf[..16].copy_from_slice(&self.client_guid);
        buf[16..24].copy_from_slice(&self.sequence.to_le_bytes());
    }

    /// Deserialize from `buf[..Self::SIZE]`. Returns `None` when the
    /// buffer is too short.
    pub fn read_from_buf(buf: &[u8]) -> Option<Self> {
        if buf.len() < Self::SIZE {
            return None;
        }
        let mut client_guid = [0u8; 16];
        client_guid.copy_from_slice(&buf[..16]);
        let mut seq = [0u8; 8];
        seq.copy_from_slice(&buf[16..24]);
        Some(Self {
            client_guid,
            sequence: i64::from_le_bytes(seq),
        })
    }
}

/// Derive a replay-stable client GUID from node identity.
///
/// A pure function of its inputs — NEVER wall-clock or RNG (Principle
/// #7: two runs of the same graph mint identical GUIDs, so recorded
/// service traffic replays bit-identically). Collision resistance is
/// best-effort (two FNV-1a hashes over salt-differentiated compositions
/// — decorrelated in practice, not formally independent); GUIDs only need to be unique among the live clients of one
/// service, where `instance` already disambiguates same-node clients.
pub fn derive_client_guid(node_id: &str, service: &str, instance: u64) -> [u8; 16] {
    // Normalize first so the SAME logical service spelled with or without a
    // leading slash ("/x" vs "x") mints the SAME replay-stable GUID —
    // matching request_topic/reply_topic, which also normalize (otherwise
    // a recording and a replay spelling the service differently
    // would desync the GUID from the reply topic).
    let service = normalize_service(service);
    let service = service.as_str();
    // Length-prefixed composition — a delimiter scheme is injectable
    // (node_id="p|q|p", service="q" collides with node_id="p",
    // service="q|p|q" in both orderings). Length
    // prefixes make the composed byte string a bijection of the inputs.
    let compose = |salt: u8| {
        let mut bytes = Vec::with_capacity(1 + 8 + node_id.len() + 8 + service.len() + 8);
        bytes.push(salt);
        bytes.extend_from_slice(&(node_id.len() as u64).to_le_bytes());
        bytes.extend_from_slice(node_id.as_bytes());
        bytes.extend_from_slice(&(service.len() as u64).to_le_bytes());
        bytes.extend_from_slice(service.as_bytes());
        bytes.extend_from_slice(&instance.to_le_bytes());
        bytes
    };
    let h1 = fnv1a_hash(&compose(1));
    let h2 = fnv1a_hash(&compose(2));
    let mut guid = [0u8; 16];
    guid[..8].copy_from_slice(&h1.to_le_bytes());
    guid[8..].copy_from_slice(&h2.to_le_bytes());
    guid
}

/// Normalize a service name into the middle chunk of an `svc/.../...`
/// topic. Collapses every run of slashes — leading, interior, AND trailing
/// — by dropping empty path segments, so an absolute ROS name
/// (`/compute_ik`) or a sloppily-formed one (`/ns//svc_x/`) never yields a
/// `svc//` double-slash. An empty path chunk is an INVALID zenoh key if the
/// topic is bridged cross-machine, and a standalone-API hazard even today;
/// the rmw path strips upstream (`runtime.rs`), but normalizing here
/// protects every other caller too. Inner namespace
/// boundaries (single slashes) are preserved.
///
/// A name that is empty or all-slashes normalizes to `""` — there is no
/// valid topic for it; `create_service_{client,server}` reject that loudly
/// via [`validate_service_name`] rather than emit a degenerate `svc//` key.
fn normalize_service(service: &str) -> String {
    service
        .split('/')
        .filter(|seg| !seg.is_empty())
        .collect::<Vec<_>>()
        .join("/")
}

/// Reject a service name that normalizes to empty (empty or all-slashes)
/// before it can build a degenerate `svc//request` topic. Returns the
/// failure reason as a `String` (mirroring [`validate_topic_name`]); the
/// caller maps it to the role-appropriate `TransportError` variant. Loud at
/// the API boundary (project rule: prefer a loud error over a silent malformed
/// key).
fn validate_service_name(service: &str) -> Result<(), String> {
    if normalize_service(service).is_empty() {
        return Err(format!(
            "service name '{service}' is empty after slash normalization \
             (empty or all-slashes) — cannot build a valid svc/ topic"
        ));
    }
    Ok(())
}

/// Topic carrying requests for `service`.
pub fn request_topic(service: &str) -> String {
    format!("svc/{}/request", normalize_service(service))
}

/// Per-client topic carrying replies for `service` to `client_guid`.
pub fn reply_topic(service: &str, client_guid: &[u8; 16]) -> String {
    let mut hex = String::with_capacity(32);
    for b in client_guid {
        use std::fmt::Write;
        write!(hex, "{b:02x}").expect("writing to String cannot fail");
    }
    format!("svc/{}/reply/{hex}", normalize_service(service))
}

/// Build one service wire frame: `[WireHeader][ServiceEnvelope][payload]`.
///
/// Fails loudly when the frame exceeds the wire format's u32 size field —
/// the receive path clamps to `total_size`, so an unchecked cast would
/// become silent truncation at the receiver if a future allocation
/// strategy ever allowed >4 GiB loans.
fn build_frame(
    service: &str,
    schema_hash: u64,
    envelope: &ServiceEnvelope,
    payload: &[u8],
    sequence_low: u32,
    timestamp_ns: u64,
) -> TransportResult<Vec<u8>> {
    let total_size = WireHeader::SIZE + ServiceEnvelope::SIZE + payload.len();
    let total_size_u32 =
        u32::try_from(total_size).map_err(|_| crate::error::TransportError::Publish {
            topic: service.to_string(),
            reason: format!("service frame of {total_size} bytes exceeds the u32 wire size field"),
        })?;
    let header = WireHeader {
        schema_hash,
        total_size: total_size_u32,
        offset_table_offset: 0,
        offset_table_count: 0,
        sequence: sequence_low,
        timestamp_ns,
    };
    let mut frame = vec![0u8; total_size];
    header.write_to_buf(&mut frame[..WireHeader::SIZE]);
    envelope.write_to_buf(&mut frame[WireHeader::SIZE..WireHeader::SIZE + ServiceEnvelope::SIZE]);
    frame[WireHeader::SIZE + ServiceEnvelope::SIZE..].copy_from_slice(payload);
    Ok(frame)
}

/// Parse `[ServiceEnvelope][payload]` out of a received frame's payload
/// (the `WireHeader` was already stripped by the subscriber). Returns
/// `None` (with a loud warning at the caller) when the frame is too
/// short to carry an envelope.
fn parse_frame(payload: &[u8]) -> Option<(ServiceEnvelope, &[u8])> {
    let envelope = ServiceEnvelope::read_from_buf(payload)?;
    Some((envelope, &payload[ServiceEnvelope::SIZE..]))
}

/// Client side of a Cerulion service: sends requests, takes correlated
/// responses.
pub struct ServiceClient {
    service: String,
    guid: [u8; 16],
    request_pub: CerulionPublisher,
    reply_sub: CerulionSubscriber,
    transport: Arc<TransportManager>,
    /// Replay-stable monotonic sequence counter (first request = 1,
    /// matching rmw conventions where 0 is "no request").
    next_sequence: i64,
    request_schema_hash: u64,
    response_schema_hash: u64,
    clock: Arc<dyn Clock>,
    /// Flood-suppression + unconditional counter for responses
    /// dropped on the schema-hash gate. A hash mismatch is a type SKEW,
    /// so it fails every frame until somebody redeploys — a bare
    /// per-frame `warn!` here is the disk-fill class. One latch
    /// per client, and a client binds ONE service name for its lifetime,
    /// so this is per-topic keying with no map (see
    /// [`frame_drop_latch`](super::frame_drop_latch)).
    /// Shared by BOTH response take paths — same topic, same condition.
    response_hash_mismatches: FailureRegimeLatch,
    /// The FRAMING-skew twin of `response_hash_mismatches` — a
    /// response whose hash MATCHED but whose payload is too short to hold
    /// the envelope. Same per-frame flood shape, so the same latch
    /// treatment; a SEPARATE latch because the two conditions have
    /// different remedies and neither regime may swallow the other's loud
    /// head. Shared by both response take paths.
    short_response_frames: FailureRegimeLatch,
}

impl ServiceClient {
    /// Send `payload` as a request; returns the minted sequence number
    /// for correlation with
    /// [`try_take_one_response`](Self::try_take_one_response).
    pub fn send_request(&mut self, payload: &[u8]) -> TransportResult<i64> {
        // Mint the sequence but COMMIT it only after a successful
        // publish — a failed send must not burn a number, or the
        // server's gap detector would report a spurious "request LOST"
        // for traffic that never existed.
        let sequence = self.next_sequence;
        let envelope = ServiceEnvelope {
            client_guid: self.guid,
            sequence,
        };
        let frame = build_frame(
            &self.service,
            self.request_schema_hash,
            &envelope,
            payload,
            sequence as u32,
            self.clock.now_ns(),
        )?;
        self.request_pub.publish_raw(&frame)?;
        // Publish succeeded — commit the sequence number.
        self.next_sequence += 1;
        // Drain our own publisher-side listener BEFORE notifying: the
        // notify below is delivered to ALL listeners on the event
        // service INCLUDING our own (iceoryx2 does not skip self).
        // `loan_proxy` drains on every publish for the pub/sub path;
        // services bypass it, and an undrained listener socket fills up
        // and turns every later notify into warn-spam.
        // Also handles SubscriberConnected (history is off
        // for service topics — deliver_history no-ops).
        self.request_pub.check_subscriber_events();
        // Wake any server blocked in wait_for_request — publish_raw does
        // NOT notify (it serves history replay, whose event protocol is
        // SentHistory); without this, a waiting server burns its full
        // timeout before draining (measured, not assumed).
        // Best-effort: the data is already in SHM; a notify failure can
        // only delay a poller, not lose data.
        if let Err(e) = self.request_pub.notify_sent_sample() {
            tracing::debug!(service = %self.service, error = ?e, "request notify failed");
        }
        tracing::debug!(
            service = %self.service,
            sequence,
            size_bytes = payload.len(),
            "service request sent"
        );
        Ok(sequence)
    }

    /// The client's GUID (envelope identity).
    pub fn guid(&self) -> [u8; 16] {
        self.guid
    }

    /// How many responses this client has dropped on the
    /// schema-hash gate, across both take paths and all regimes.
    ///
    /// UNCONDITIONAL and never reset — the Principle #3 signal that
    /// survives the loud head scrolling away and the repeats being
    /// `debug!`-suppressed. A nonzero and GROWING value means this client
    /// and the server disagree on the response type.
    pub fn schema_mismatch_count(&self) -> u64 {
        self.response_hash_mismatches.total_failures()
    }

    /// How many responses this client has dropped because the frame
    /// was too short to hold the envelope, across both take paths and all
    /// regimes. The FRAMING-skew counterpart to
    /// [`schema_mismatch_count`](Self::schema_mismatch_count); same
    /// unconditional, never-reset contract.
    pub fn short_frame_count(&self) -> u64 {
        self.short_response_frames.total_failures()
    }

    /// Non-consuming readiness probe: true when a response is queued
    /// (backs `rmw_wait`).
    pub fn has_pending_response(&self) -> bool {
        self.reply_sub.has_pending_sample()
    }

    /// Take AT MOST ONE response — the ONLY response take path, because
    /// the rmw take contract is one message per call. Call
    /// it again to take the next; it never coalesces, so nothing queued
    /// is lost by taking one at a time (Principle #6).
    ///
    /// Frames failing validation (short envelope, wrong schema hash,
    /// foreign GUID) are skipped LOUDLY — never silently — and the take
    /// continues past them, so a corrupt frame cannot wedge it. The two
    /// per-frame skew classes (wrong hash, short envelope) ride shared
    /// flood latches, so a persistently skewed peer is loud once per
    /// regime and at each decade of the running total rather than once
    /// per frame; the foreign-GUID arm stays a bare `warn!` because
    /// per-client reply topics make it unreachable in practice.
    pub fn try_take_one_response<F>(&mut self, mut f: F) -> TransportResult<bool>
    where
        F: FnMut(i64, &[u8]),
    {
        let service = &self.service;
        let expected_hash = self.response_schema_hash;
        let own_guid = self.guid;
        let hash_latch = &mut self.response_hash_mismatches;
        let short_latch = &mut self.short_response_frames;
        let mut delivered = false;
        // Loop: skip invalid frames (warned) until one valid response or
        // the queue is empty — a corrupt frame must not wedge the take.
        loop {
            let got = self.reply_sub.try_receive_one(|msg| {
                if msg.header().schema_hash != expected_hash {
                    report_schema_hash_mismatch(
                        hash_latch,
                        FrameDropSite::ServiceResponse,
                        service,
                        expected_hash,
                        msg.header().schema_hash,
                    );
                    return;
                }
                report_schema_hash_match(hash_latch, FrameDropSite::ServiceResponse, service);
                match parse_frame(msg.payload()) {
                    Some((envelope, payload)) => {
                        report_envelope_intact(
                            short_latch,
                            FrameDropSite::ServiceResponse,
                            service,
                        );
                        if envelope.client_guid != own_guid {
                            tracing::warn!(
                                service = %service,
                                "service response with foreign client GUID, skipping"
                            );
                            return;
                        }
                        f(envelope.sequence, payload);
                        delivered = true;
                    }
                    None => {
                        report_short_envelope(
                            short_latch,
                            FrameDropSite::ServiceResponse,
                            service,
                            msg.payload().len(),
                        );
                    }
                }
            })?;
            if delivered || !got {
                return Ok(delivered);
            }
        }
    }

    /// True when at least one server is subscribed to this service's
    /// request topic (backs `rmw_service_server_is_available`).
    /// Cross-process: iceoryx2's service registry is host-global.
    pub fn server_available(&self) -> bool {
        self.transport
            .topic_subscriber_count(&request_topic(&self.service))
            > 0
    }

    /// The service name.
    pub fn service(&self) -> &str {
        &self.service
    }

    /// Borrow the reply subscriber (rmw wait-set integration: the
    /// subscriber's event listener is the wakeup source for "client has
    /// a response ready").
    pub fn reply_subscriber(&self) -> &CerulionSubscriber {
        &self.reply_sub
    }
}

/// Server side of a Cerulion service: drains requests FIFO, sends
/// correlated responses back on per-client reply topics.
pub struct ServiceServer {
    service: String,
    request_sub: CerulionSubscriber,
    /// Lazily-created reply publishers, one per client GUID, in
    /// deterministic (BTreeMap) order. Creation is driven purely by
    /// request arrival (data), never by timing.
    reply_pubs: BTreeMap<[u8; 16], CerulionPublisher>,
    transport: Arc<TransportManager>,
    reply_max_slice_len: MaxSliceLen,
    request_schema_hash: u64,
    response_schema_hash: u64,
    clock: Arc<dyn Clock>,
    /// Server-side monotonic counter for response WireHeader sequences
    /// (the authoritative correlation id is the echoed envelope).
    responses_sent: u64,
    /// Highest sequence seen per client — used to WARN on gaps, which
    /// are the on-the-wire signature of request loss (the iceoryx2
    /// subscriber queue is a bounded safe-overflow ring; see the module
    /// docs' "Loss bounds" section). Detection only — no retransmit.
    last_seen_seq: BTreeMap<[u8; 16], i64>,
    /// Flood-suppression + unconditional counter for requests
    /// dropped on the schema-hash gate — the server-side twin of
    /// [`ServiceClient::response_hash_mismatches`]. One latch per server
    /// (which binds ONE service name), shared by both request take paths.
    request_hash_mismatches: FailureRegimeLatch,
    /// The FRAMING-skew twin — the server-side counterpart of
    /// [`ServiceClient::short_response_frames`]. Separate latch, same
    /// reason.
    short_request_frames: FailureRegimeLatch,
}

impl ServiceServer {
    /// Send `payload` as the response to the request identified by
    /// `envelope` (echo the envelope you received from
    /// [`try_take_one_request`](Self::try_take_one_request) verbatim).
    ///
    /// The reply publisher for the client is created lazily on first
    /// response to that client — creation order is driven by request
    /// data, never timing, so it is replay-stable.
    pub fn send_response(
        &mut self,
        envelope: &ServiceEnvelope,
        payload: &[u8],
    ) -> TransportResult<()> {
        let publisher = match self.reply_pubs.entry(envelope.client_guid) {
            std::collections::btree_map::Entry::Vacant(vacant) => {
                let topic = reply_topic(&self.service, &envelope.client_guid);
                // Explicit single-writer reply-topic
                // provisioning (the role is known here, never sniffed from the name).
                let publisher = self.transport.create_publisher_with_topic_config(
                    &topic,
                    self.reply_max_slice_len,
                    0,
                    TopicServiceConfig::for_service_reply(self.transport.subscriber_buffer_size()),
                )?;
                tracing::debug!(
                    service = %self.service,
                    topic = %topic,
                    "service reply publisher created for new client"
                );
                vacant.insert(publisher)
            }
            std::collections::btree_map::Entry::Occupied(occupied) => occupied.into_mut(),
        };

        self.responses_sent += 1;
        let frame = build_frame(
            &self.service,
            self.response_schema_hash,
            envelope,
            payload,
            self.responses_sent as u32,
            self.clock.now_ns(),
        )?;
        // Drain our own listener before notifying — see send_request.
        publisher.check_subscriber_events();
        let recipients = publisher.publish_raw(&frame)?;
        if recipients == 0 {
            // Responding into the void: the client's reply subscriber is
            // gone (or never existed — fabricated envelope). The request
            // was already consumed, so this client will never see an
            // answer — say so loudly.
            tracing::warn!(
                service = %self.service,
                sequence = envelope.sequence,
                "service response delivered to ZERO subscribers — client gone?"
            );
        }
        // Wake a client blocked in a wait — see send_request for why
        // publish_raw itself does not notify.
        if let Err(e) = publisher.notify_sent_sample() {
            tracing::debug!(service = %self.service, error = ?e, "response notify failed");
        }
        tracing::debug!(
            service = %self.service,
            sequence = envelope.sequence,
            size_bytes = payload.len(),
            "service response sent"
        );
        Ok(())
    }

    /// Non-consuming readiness probe: true when a request is queued
    /// (backs `rmw_wait`).
    pub fn has_pending_request(&self) -> bool {
        self.request_sub.has_pending_sample()
    }

    /// Take AT MOST ONE request — the ONLY request take path (rmw
    /// take semantics; see
    /// [`try_take_one_response`](ServiceClient::try_take_one_response)).
    /// Call it again to take the next; it never coalesces, so nothing
    /// queued is lost by taking one at a time (Principle #6).
    ///
    /// Frames failing validation are skipped LOUDLY and the take
    /// continues past them, so a corrupt frame cannot wedge it; the two
    /// per-frame skew classes ride shared flood latches.
    pub fn try_take_one_request<F>(&mut self, mut f: F) -> TransportResult<bool>
    where
        F: FnMut(ServiceEnvelope, &[u8]),
    {
        let service = &self.service;
        let expected_hash = self.request_schema_hash;
        let last_seen = &mut self.last_seen_seq;
        let hash_latch = &mut self.request_hash_mismatches;
        let short_latch = &mut self.short_request_frames;
        let mut delivered = false;
        loop {
            let got = self.request_sub.try_receive_one(|msg| {
                if msg.header().schema_hash != expected_hash {
                    report_schema_hash_mismatch(
                        hash_latch,
                        FrameDropSite::ServiceRequest,
                        service,
                        expected_hash,
                        msg.header().schema_hash,
                    );
                    return;
                }
                report_schema_hash_match(hash_latch, FrameDropSite::ServiceRequest, service);
                match parse_frame(msg.payload()) {
                    Some((envelope, payload)) => {
                        report_envelope_intact(short_latch, FrameDropSite::ServiceRequest, service);
                        // Loss detection: client sequences are contiguous
                        // from 1, so a FORWARD gap means requests were
                        // displaced from the bounded subscriber queue
                        // between takes. The data to warn is already on the
                        // wire — losses are detected on the next surviving
                        // request from that client (a client's TRAILING
                        // losses are inherently undetectable here). A
                        // BACKWARD jump is a different story: the client
                        // restarted (derived GUIDs are reused by design) —
                        // diagnose it as such, never as queue overflow.
                        //
                        // BOTH arms must exist here: this is the ONLY
                        // request take path, so it is the only place either
                        // diagnostic can be emitted, and a client restart
                        // would otherwise pass in SILENCE — the backward
                        // jump satisfies neither `> prev + 1` nor any other
                        // arm, so nothing at all would be reported.
                        let prev = last_seen
                            .insert(envelope.client_guid, envelope.sequence)
                            .unwrap_or(0);
                        if envelope.sequence > prev + 1 {
                            tracing::warn!(
                                service = %service,
                                expected_sequence = prev + 1,
                                actual_sequence = envelope.sequence,
                                "request sequence gap — requests were LOST to queue \
                                 overflow (drain more often; the queue bound is the \
                                 process-wide subscriber_buffer_size fixed at \
                                 transport init)"
                            );
                        } else if envelope.sequence <= prev {
                            tracing::info!(
                                service = %service,
                                previous_sequence = prev,
                                actual_sequence = envelope.sequence,
                                "request sequence went backwards — client restarted \
                                 with a reused GUID"
                            );
                        }
                        f(envelope, payload);
                        delivered = true;
                    }
                    None => {
                        report_short_envelope(
                            short_latch,
                            FrameDropSite::ServiceRequest,
                            service,
                            msg.payload().len(),
                        );
                    }
                }
            })?;
            if delivered || !got {
                return Ok(delivered);
            }
        }
    }

    /// The service name.
    pub fn service(&self) -> &str {
        &self.service
    }

    /// Number of distinct clients this server has replied to.
    pub fn known_client_count(&self) -> usize {
        self.reply_pubs.len()
    }

    /// Borrow the request subscriber (rmw wait-set integration: the
    /// subscriber's event listener is the wakeup source for "service has
    /// a request ready").
    pub fn request_subscriber(&self) -> &CerulionSubscriber {
        &self.request_sub
    }

    /// How many requests this server has dropped on the
    /// schema-hash gate, across both take paths and all regimes.
    ///
    /// UNCONDITIONAL and never reset — see
    /// [`ServiceClient::schema_mismatch_count`]. A nonzero and GROWING
    /// value means a client and this server disagree on the request type.
    pub fn schema_mismatch_count(&self) -> u64 {
        self.request_hash_mismatches.total_failures()
    }

    /// How many requests this server has dropped because the frame
    /// was too short to hold the envelope — the FRAMING-skew counterpart to
    /// [`schema_mismatch_count`](Self::schema_mismatch_count). Same
    /// unconditional, never-reset contract.
    pub fn short_frame_count(&self) -> u64 {
        self.short_request_frames.total_failures()
    }
}

impl TransportManager {
    /// Create the client side of a service.
    ///
    /// `client_guid` should come from [`derive_client_guid`] (or the rmw
    /// layer's gid) — it must be unique among live clients of this
    /// service and replay-stable. `request_schema_hash` /
    /// `response_schema_hash` identify the service's request/response
    /// types on the wire (e.g. FNV-1a of
    /// `"moveit_msgs/srv/GetPlanningScene_Request"`).
    pub fn create_service_client(
        self: &Arc<Self>,
        service: &str,
        client_guid: [u8; 16],
        max_slice_len: MaxSliceLen,
        request_schema_hash: u64,
        response_schema_hash: u64,
    ) -> TransportResult<ServiceClient> {
        validate_service_name(service).map_err(|reason| {
            crate::error::TransportError::PublisherCreation {
                // The raw input — NOT request_topic(service), which for a
                // reject-worthy name is itself the degenerate `svc//request`
                // the validation exists to prevent.
                topic: service.to_string(),
                reason,
            }
        })?;
        // Explicit role-based provisioning. The client's
        // request publisher fans into a multi-writer request topic; its
        // reply subscriber opens a single-writer per-client reply topic.
        let request_pub = self.create_publisher_with_topic_config(
            &request_topic(service),
            max_slice_len,
            0,
            TopicServiceConfig::for_service_request(self.subscriber_buffer_size()),
        )?;
        let reply_sub = self.create_subscriber_with_buffers(
            &reply_topic(service, &client_guid),
            TopicServiceConfig::for_service_reply(self.subscriber_buffer_size()),
            self.subscriber_buffer_size(),
        )?;
        tracing::debug!(service = %service, "service client created");
        Ok(ServiceClient {
            service: service.to_string(),
            guid: client_guid,
            request_pub,
            reply_sub,
            transport: Arc::clone(self),
            next_sequence: 1,
            request_schema_hash,
            response_schema_hash,
            clock: self.clock_arc(),
            response_hash_mismatches: FailureRegimeLatch::new(),
            short_response_frames: FailureRegimeLatch::new(),
        })
    }

    /// Create the server side of a service.
    pub fn create_service_server(
        self: &Arc<Self>,
        service: &str,
        max_slice_len: MaxSliceLen,
        request_schema_hash: u64,
        response_schema_hash: u64,
    ) -> TransportResult<ServiceServer> {
        validate_service_name(service).map_err(|reason| {
            crate::error::TransportError::SubscriberCreation {
                // Raw input, not the degenerate request_topic (see the
                // client side).
                topic: service.to_string(),
                reason,
            }
        })?;
        // The server subscribes to the multi-writer request
        // topic with the same role provisioning the clients publish under
        // (open_or_create requires consistent caps across all participants).
        let request_sub = self.create_subscriber_with_buffers(
            &request_topic(service),
            TopicServiceConfig::for_service_request(self.subscriber_buffer_size()),
            self.subscriber_buffer_size(),
        )?;
        tracing::debug!(service = %service, "service server created");
        Ok(ServiceServer {
            service: service.to_string(),
            request_sub,
            reply_pubs: BTreeMap::new(),
            transport: Arc::clone(self),
            reply_max_slice_len: max_slice_len,
            request_schema_hash,
            response_schema_hash,
            clock: self.clock_arc(),
            responses_sent: 0,
            last_seen_seq: BTreeMap::new(),
            request_hash_mismatches: FailureRegimeLatch::new(),
            short_request_frames: FailureRegimeLatch::new(),
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn envelope_round_trips_bit_identically() {
        let envelope = ServiceEnvelope {
            client_guid: [
                0x00, 0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88, 0x99, 0xAA, 0xBB, 0xCC, 0xDD,
                0xEE, 0xFF,
            ],
            sequence: -42,
        };
        let mut buf = [0u8; ServiceEnvelope::SIZE];
        envelope.write_to_buf(&mut buf);
        let parsed = ServiceEnvelope::read_from_buf(&buf).expect("parse");
        assert_eq!(parsed, envelope);

        // Too-short buffers parse as None, never panic.
        assert_eq!(ServiceEnvelope::read_from_buf(&buf[..23]), None);
        assert_eq!(ServiceEnvelope::read_from_buf(&[]), None);
    }

    #[test]
    fn derive_client_guid_is_deterministic_and_distinct() {
        let a1 = derive_client_guid("move_group", "/get_planning_scene", 0);
        let a2 = derive_client_guid("move_group", "/get_planning_scene", 0);
        assert_eq!(a1, a2, "same inputs must mint the same GUID (replay)");

        // Each input dimension changes the GUID.
        assert_ne!(
            a1,
            derive_client_guid("move_group", "/get_planning_scene", 1)
        );
        assert_ne!(a1, derive_client_guid("rviz", "/get_planning_scene", 0));
        assert_ne!(a1, derive_client_guid("move_group", "/compute_ik", 0));
    }

    #[test]
    fn topic_names_are_per_service_and_per_client() {
        let guid = derive_client_guid("n", "s", 0);
        // Absolute ROS names are normalized — the leading slash is
        // stripped so no `svc//` double-slash (an invalid zenoh key if
        // bridged) is produced.
        assert_eq!(request_topic("/compute_ik"), "svc/compute_ik/request");
        assert_eq!(request_topic("/compute_ik"), request_topic("compute_ik"));
        let reply = reply_topic("/compute_ik", &guid);
        assert!(reply.starts_with("svc/compute_ik/reply/"));
        // 32 hex chars of GUID.
        assert_eq!(reply.len(), "svc/compute_ik/reply/".len() + 32);
        // Inner namespace slashes are preserved.
        assert_eq!(request_topic("/ns/svc_x"), "svc/ns/svc_x/request");

        // Every slash-run pathology collapses — NO `svc//` ever escapes
        // (leading trim alone would leave interior/trailing/all-slash
        // double-slashes). Each must contain no empty `//` chunk.
        for (name, want) in [
            ("//compute_ik", "svc/compute_ik/request"), // multi-leading
            ("/ns//svc_x", "svc/ns/svc_x/request"),     // interior
            ("compute_ik/", "svc/compute_ik/request"),  // trailing
            ("/a//b/", "svc/a/b/request"),              // mixed
        ] {
            assert_eq!(request_topic(name), want, "request_topic({name:?})");
            assert!(!request_topic(name).contains("//"), "{name:?} left a //");
            assert!(
                !reply_topic(name, &guid).contains("//"),
                "{name:?} reply //"
            );
        }

        // Distinct clients → distinct reply topics.
        let other = derive_client_guid("n", "s", 1);
        assert_ne!(reply, reply_topic("/compute_ik", &other));
    }

    #[test]
    fn empty_or_all_slash_service_names_are_rejected_not_silently_degenerate() {
        // normalize_service collapses these to "" (no valid topic exists);
        // validate_service_name must reject them LOUDLY rather than let a
        // degenerate `svc//request` key reach iceoryx2.
        for bad in ["", "/", "//", "///"] {
            assert_eq!(
                normalize_service(bad),
                "",
                "{bad:?} should normalize to empty"
            );
            assert!(
                validate_service_name(bad).is_err(),
                "service name {bad:?} must be rejected loudly",
            );
        }
        // A real name (even sloppily formed) passes.
        assert!(validate_service_name("/ns//svc_x/").is_ok());
    }

    #[test]
    fn derive_client_guid_normalizes_service_like_the_topics() {
        // "/x" and "x" are the same logical service → same replay-stable
        // GUID (matches request_topic/reply_topic normalization).
        assert_eq!(
            derive_client_guid("node", "/get_planning_scene", 0),
            derive_client_guid("node", "get_planning_scene", 0),
        );
        assert_eq!(
            derive_client_guid("node", "/ns//svc", 3),
            derive_client_guid("node", "ns/svc", 3),
        );
    }

    #[test]
    fn normalize_service_is_idempotent() {
        // The rmw path normalizes upstream (ros_topic_to_cerulion) and then
        // request_topic/reply_topic/derive_client_guid each normalize AGAIN —
        // double-normalization must be a no-op or the guid/topic lockstep
        // breaks.
        for name in [
            "compute_ik",
            "/compute_ik",
            "ns/get_planning_scene",
            "/ns//svc_x/",
            "a//b///c",
        ] {
            let once = normalize_service(name);
            let twice = normalize_service(&once);
            assert_eq!(once, twice, "normalize_service not idempotent on {name:?}");
        }
    }

    #[test]
    fn derive_client_guid_is_byte_stable_for_the_common_name() {
        // ABSOLUTE pin (not a self-comparison): a no-slash name's GUID must
        // never change — recorded service bags and the rmw gid depend on it
        // (Principle #7). normalize_service is the identity on a no-slash
        // name, so this value equals the pre-normalization-fix bytes; a
        // future transform that mangled the common case (which the
        // '/x'==' x' equivalence test would NOT catch) breaks this pin.
        assert_eq!(
            derive_client_guid("move_group", "get_planning_scene", 0),
            [
                0x02, 0x93, 0xdb, 0x4a, 0xf7, 0xf0, 0x36, 0x4a, 0x6b, 0x6f, 0xc4, 0xa6, 0x86, 0xfc,
                0x46, 0xaa,
            ],
        );
    }

    #[test]
    fn frame_build_parse_round_trip() {
        let envelope = ServiceEnvelope {
            client_guid: [7u8; 16],
            sequence: 99,
        };
        let payload = b"cdr-bytes-here";
        let frame =
            build_frame("/svc_test", 0xABCD, &envelope, payload, 99, 123_456).expect("frame");

        let header = WireHeader::read_from_buf(&frame).expect("header");
        assert_eq!(header.schema_hash, 0xABCD);
        assert_eq!(header.total_size as usize, frame.len());
        assert_eq!(header.offset_table_count, 0);
        assert_eq!(header.timestamp_ns, 123_456);

        let (parsed, body) = parse_frame(&frame[WireHeader::SIZE..]).expect("frame");
        assert_eq!(parsed, envelope);
        assert_eq!(body, payload);
    }
}
