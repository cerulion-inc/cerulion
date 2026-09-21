// SPDX-License-Identifier: AGPL-3.0-only
//! The live SPDP/SEDP discovery backend — a [`DdsDiscovery`] impl over
//! `ros2-client`/`rustdds`.
//!
//! [`LiveDiscovery::discover`] builds a one-per-process [`DiscoveryParticipant`],
//! starts the node spinner, and runs a bounded discovery window on one
//! `smol::LocalExecutor` (the demos-bridge pump's exact executor shape). The
//! window harvest has TWO halves (the wire rung):
//!
//! * **Endpoints — a loss-proof DB SNAPSHOT.** At window end we snapshot the
//!   participant's internal DiscoveryDB via rustdds 0.14.2's
//!   [`DiscoveryParticipant::discovered_writers`] /
//!   [`discovered_readers`](DiscoveryParticipant::discovered_readers) accessors
//!   and map each `DiscoveredWriterData`/`DiscoveredReaderData` into a DDS-free
//!   [`DiscoveredEndpoint`] (`snapshot_endpoints` → the pure
//!   `map_snapshot_endpoints`). The DB is maintained by rustdds's own discovery
//!   threads, so this is loss-proof (vs the bounded ros2-client status channel)
//!   AND carries each endpoint's SEDP `USER_DATA` — where the REP-2011 RIHS01
//!   type hash rides. The published cerulion-rustdds fork is retained only for
//!   its participant lease duration knob. Endpoints whose GUID prefix
//!   matches OUR participant's (the discovery node's own parameter-service
//!   readers/writers) are filtered STRUCTURALLY and counted (live P0 fix 2; see
//!   [`crate::DiscoveryResult::own_endpoints_hidden`]) — a GUID match, never a
//!   name heuristic.
//! * **Nodes + participants — the event stream.** The `ros_discovery_info`
//!   `ParticipantEntitiesInfo` node table (the endpoint→node service join) and
//!   remote-participant vendor ids (SPDP, the correlation-mapping key) are still
//!   harvested from the `collect_endpoints` `NodeEvent` loop, which also PACES
//!   the window. That channel is `async_channel::bounded(8)`, so a large graph's
//!   join-burst CAN drop node announcements (the loud
//!   [`crate::wire::should_warn_node_table_miss`] warn keys off exactly that,
//!   per hash-bearing participant): the lossy-channel caveat now applies ONLY to
//!   the node/participant side; the endpoint + type-hash harvest above is
//!   loss-proof. Our OWN node's looped-back PEI is filtered from the table by
//!   participant prefix (`split_off_own_nodes` — the node-side own-filter twin).
//!
//! # Live-peer exercise
//!
//! The window needs a live DDS peer, so its behavior is proven against
//! a real ROS 2 / CycloneDDS graph (the `#[ignore]`d `live_discovery_box_test.rs`,
//! run inside the ROS 2 container). This module compiles cross-platform (macOS +
//! Linux) exactly like the demos bridge.

use std::time::Instant;

use ros2_client::entities_info::ParticipantEntitiesInfo;
use ros2_client::ros2::{policy, QosPolicies};
use ros2_client::rustdds::{DomainParticipantStatusEvent, GUID};
use ros2_client::NodeEvent;

use crate::participant::DiscoveryParticipant;
use crate::{
    DdsDiscovery, DdsError, DiscoveredEndpoint, DiscoveredNode, DiscoveredParticipant,
    DiscoveredQos, DiscoveryParams, DiscoveryResult, EndpointKind, QosDurability, QosReliability,
};

/// The production [`DdsDiscovery`] backend. Stateless — a fresh participant is
/// built (and dropped, releasing the one-per-process slot) per `discover` call.
#[derive(Debug, Default, Clone, Copy)]
pub struct LiveDiscovery;

impl DdsDiscovery for LiveDiscovery {
    fn discover(&self, params: &DiscoveryParams) -> Result<DiscoveryResult, DdsError> {
        let participant = DiscoveryParticipant::new(params.domain_id, &params.only_networks)?;
        // Our own GUID — the STRUCTURAL identity for
        // filtering the attach participant's own endpoints (parameter services
        // etc.) out of the SNAPSHOT below. GUID-prefix match, never a name
        // heuristic.
        let own_guid = participant.guid();
        // `create_node` returns an owned Node kept alive for the whole window
        // (its discovery readers back the status stream).
        let mut node = participant.create_node("discovery")?;
        // `spinner()` marks `have_spinner()` (so `status_receiver()` will not
        // panic) BEFORE the spinner is driven.
        let spinner = node.spinner().map_err(|e| DdsError::Spinner {
            cause: format!("{e:?}"),
        })?;
        let status = node.status_receiver();
        let deadline = Instant::now() + params.window;

        // `async move` so `spinner`/`status` are OWNED by this future (the inner
        // spawn moves `spinner`); `node` (and `participant`) stay on the stack,
        // alive for the whole window. The event loop harvests the node table +
        // participant vendors and PACES the window; endpoints come from the DB
        // snapshot AFTER the window (the loss-proof, USER_DATA-bearing
        // harvest).
        let (nodes, participants, closed_early) = smol::block_on(async move {
            let ex = smol::LocalExecutor::new();
            // Spin runs concurrently while we collect: `ex.run(fut)` drives all
            // spawned tasks until `fut` resolves.
            let spin_task = ex.spawn(async move {
                if let Err(e) = spinner.spin().await {
                    tracing::warn!(error = ?e, "cerulion_dds: node spinner exited with error");
                }
            });

            let out = ex.run(collect_endpoints(status, deadline)).await;
            drop(spin_task); // cancel the spinner task
            out
        });
        if closed_early {
            // A spinner death must not read as "no graph". The
            // endpoint snapshot below is independent of the channel, so only the
            // event-fed node/participant harvest can be partial.
            tracing::warn!(
                "cerulion_dds: the discovery status channel closed BEFORE the collection \
                 window elapsed (the node spinner exited early) — the node/participant harvest \
                 may be PARTIAL"
            );
        }

        // OUR OWN discovery node's
        // ParticipantEntitiesInfo loops back on `ros_discovery_info`
        // (TRANSIENT_LOCAL; ros2-client republishes it at node creation and
        // rustdds delivers same-participant samples to local readers), so the
        // raw table contains `/cerulion_attach/discovery` every window —
        // masking node-table-loss diagnostics and offering a join target no
        // foreign endpoint can match. Filter it STRUCTURALLY by participant
        // prefix — the node-side twin of the endpoint own-filter.
        let own_prefix: [u8; 12] = own_guid.to_bytes()[..12].try_into().expect("16 >= 12");
        let (nodes, own_nodes_hidden) = split_off_own_nodes(nodes, own_prefix);
        if own_nodes_hidden > 0 {
            tracing::debug!(
                count = own_nodes_hidden,
                "cerulion_dds: our own discovery node hidden from the ros_discovery_info node \
                 table (own-prefix filter)"
            );
        }

        // WINDOW END: snapshot the loss-proof, USER_DATA-bearing DiscoveryDB for
        // the endpoints + own_hidden (not the lossy event
        // stream). `participant` is still alive here (drops at fn end).
        let (endpoints, own_endpoints_hidden) = snapshot_endpoints(&participant, own_guid);

        tracing::info!(
            count = endpoints.len(),
            nodes = nodes.len(),
            participants = participants.len(),
            own_hidden = own_endpoints_hidden,
            "cerulion_dds: discovery window elapsed"
        );
        // `node` (and `participant`) drop here → the one-per-process slot frees.
        Ok(DiscoveryResult {
            endpoints,
            own_endpoints_hidden,
            // The ROS graph node table + vendor map harvested
            // from the event stream during the window — the wire rung's
            // endpoint→node→service + vendor→correlation-mapping inputs.
            nodes,
            participants,
        })
    }
}

/// The bounded SPDP/SEDP status-event collect loop — extracted from
/// [`LiveDiscovery::discover`] so the in-module tests can drive it over a
/// hand-fed `async_channel` with NO DDS peer.
///
/// It harvests ONLY the event-fed halves — the `ros_discovery_info`
/// node table (`NodeEvent::ROS`) + remote-participant vendor ids
/// (`ParticipantDiscovered`) — and PACES the window. ENDPOINTS are NOT
/// accumulated here: they come from the loss-proof DiscoveryDB snapshot in
/// [`LiveDiscovery::discover`] (via `snapshot_endpoints`), so
/// `WriterDetected`/`ReaderDetected` events (and their `EndpointDescription`) are
/// deliberately ignored on this path. Returns `(nodes, participants,
/// closed_early)` — `closed_early` is true iff the status channel closed before
/// the deadline (a spinner death; the caller warns loudly and
/// still uses the PARTIAL node/participant set).
///
/// The deadline is enforced INDEPENDENTLY of `select`'s poll
/// bias. `futures::future::select` polls its FIRST future (recv) before the
/// timer, so a sustained discovery-event flood (a hostile or churning peer)
/// keeps recv perpetually `Ready` and the timer arm alone can be starved
/// indefinitely — the top-of-loop `Instant` check bounds the window regardless
/// of recv readiness. The timer stays as the WAKE source for the quiet case
/// (no busy loop); the `Instant` check is the guarantee.
pub(crate) async fn collect_endpoints(
    status: async_channel::Receiver<NodeEvent>,
    deadline: Instant,
) -> (Vec<DiscoveredNode>, Vec<DiscoveredParticipant>, bool) {
    // The ROS graph node table (from `ros_discovery_info`) +
    // the remote-participant vendor map (from SPDP) — the wire rung's
    // endpoint→node + vendor→mapping inputs. (Endpoints themselves ride the DB
    // snapshot, not this loop.)
    let mut nodes: Vec<DiscoveredNode> = Vec::new();
    let mut participants: Vec<DiscoveredParticipant> = Vec::new();
    let mut closed_early = false;
    let timer = smol::Timer::at(deadline);
    futures::pin_mut!(timer);
    loop {
        // The select-bias-independent deadline check.
        if collect_deadline_reached(Instant::now(), deadline) {
            break;
        }
        let recv = status.recv();
        futures::pin_mut!(recv);
        match futures::future::select(recv, timer.as_mut()).await {
            futures::future::Either::Left((Ok(NodeEvent::DDS(event)), _)) => {
                // Remote participant's vendor id (SPDP) — the
                // correlation-mapping key. Upsert (a participant may be
                // re-announced); latest wins. Endpoint-detected events on this
                // arm (`WriterDetected`/`ReaderDetected`) map to `None` here and
                // are ignored — the DB snapshot is their loss-proof source.
                if let Some(p) = event_participant(&event) {
                    upsert_participant(&mut participants, p);
                }
            }
            // `ros_discovery_info` (TRANSIENT_LOCAL) —
            // ParticipantEntitiesInfo → the node table for the endpoint→node
            // service join.
            futures::future::Either::Left((Ok(NodeEvent::ROS(pei)), _)) => {
                merge_ros_nodes(&mut nodes, &pei);
            }
            // Status channel closed (spinner exited) before the deadline —
            // stop with the partial set; the caller warns.
            futures::future::Either::Left((Err(_closed), _)) => {
                closed_early = true;
                break;
            }
            // The collection window elapsed (the quiet-case wake path).
            futures::future::Either::Right((_elapsed, _)) => break,
        }
    }
    (nodes, participants, closed_early)
}

/// Extract a remote participant's vendor identity from an SPDP
/// `ParticipantDiscovered` event, keyed by its 12-byte GUID prefix (the first 12
/// bytes of its 16-byte GUID). Returns `None` for any other status event.
/// `DomainParticipantStatusEvent` is `#[non_exhaustive]`, so the catch-all is
/// required.
fn event_participant(event: &DomainParticipantStatusEvent) -> Option<DiscoveredParticipant> {
    match event {
        DomainParticipantStatusEvent::ParticipantDiscovered { dpd } => Some(
            discovered_participant_from_parts(dpd.guid, dpd.vendor_id.vendor_id),
        ),
        _ => None,
    }
}

/// The pure core of [`event_participant`]: build the DDS-free
/// [`DiscoveredParticipant`] from a participant's 16-byte GUID + its RTPS vendor
/// id bytes (the two nameable fields the match arm reads off the un-nameable
/// `ParticipantDescription`). Extracted so the guid-prefix extraction + the
/// vendor-id carry are oracle-testable with hand-built inputs (rustdds's
/// `ParticipantDescription`/`VendorId` cannot be constructed outside rustdds —
/// see the module tests). Pure.
fn discovered_participant_from_parts(guid: GUID, vendor_id: [u8; 2]) -> DiscoveredParticipant {
    let bytes = guid.to_bytes();
    let guid_prefix: [u8; 12] = bytes[..12].try_into().expect("16 >= 12");
    DiscoveredParticipant {
        guid_prefix,
        vendor_id,
    }
}

/// Upsert a discovered participant by GUID prefix (a
/// participant may re-announce; the latest vendor id wins). Prefix is the stable
/// identity, so this dedupes to one entry per participant.
fn upsert_participant(list: &mut Vec<DiscoveredParticipant>, p: DiscoveredParticipant) {
    if let Some(existing) = list.iter_mut().find(|e| e.guid_prefix == p.guid_prefix) {
        existing.vendor_id = p.vendor_id;
    } else {
        list.push(p);
    }
}

/// Fold one `ros_discovery_info` `ParticipantEntitiesInfo`
/// into the node table. Each hosted node maps to a [`DiscoveredNode`] keyed by
/// the participant's 12-byte GUID prefix (the accessible join granularity —
/// `ros2-client` keeps `writer_gid_seq` private).
///
/// The table key is `(participant_prefix, namespace, name)` — same-named nodes
/// on DIFFERENT participants must not collapse into one. A
/// PEI is a FULL per-participant snapshot (`ros_discovery_info` is
/// `TRANSIENT_LOCAL` and re-announced whole on any change), so folding it in
/// REPLACES that participant's entire node set: this participant's prior nodes
/// are pruned first (so a node that dropped out is removed), then the announced
/// set is inserted. Other participants' nodes are untouched.
fn merge_ros_nodes(nodes: &mut Vec<DiscoveredNode>, pei: &ParticipantEntitiesInfo) {
    // `pei.gid()` returns a private `Gid`; convert via the public
    // `From<Gid> for GUID` impl (Gid stays unnamed) and take its 16-byte GUID.
    let guid: GUID = pei.gid().into();
    let bytes = guid.to_bytes();
    let prefix: [u8; 12] = bytes[..12].try_into().expect("16 >= 12");
    // Snapshot semantics: drop this participant's prior nodes (prunes any that
    // dropped out) then re-insert the whole announced set. Other participants'
    // nodes (a different prefix) are retained in place.
    nodes.retain(|n| n.participant_prefix != prefix);
    for n in pei.nodes() {
        nodes.push(DiscoveredNode {
            namespace: n.namespace().to_string(),
            name: n.name().to_string(),
            participant_prefix: prefix,
        });
    }
}

/// The pure half of the deadline check: has the collect window closed? Extracted so
/// the predicate is oracle-testable without time control.
fn collect_deadline_reached(now: Instant, deadline: Instant) -> bool {
    now >= deadline
}

/// Split the attach participant's OWN nodes out of
/// the harvested node table. ros2-client publishes our own discovery node's
/// `ParticipantEntitiesInfo` on `ros_discovery_info` and rustdds loops it back
/// to our own reader, so without this filter EVERY window's table contains
/// `/cerulion_attach/discovery` — which (a) masks a
/// table-emptiness loss diagnostic (the wire rung's per-participant miss
/// predicate is robust to it, but the table should still be accurate), and (b)
/// offers a join target no FOREIGN endpoint can ever match (our own endpoints
/// are filtered out of the endpoint list). Structural GUID-prefix match — the
/// node-side twin of the endpoint own-filter, never a name heuristic. Returns
/// `(foreign nodes in input order, hidden count)`. Pure — oracle-tested.
fn split_off_own_nodes(
    nodes: Vec<DiscoveredNode>,
    own_prefix: [u8; 12],
) -> (Vec<DiscoveredNode>, usize) {
    let before = nodes.len();
    let foreign: Vec<DiscoveredNode> = nodes
        .into_iter()
        .filter(|n| n.participant_prefix != own_prefix)
        .collect();
    let hidden = before - foreign.len();
    (foreign, hidden)
}

// ───────────────────── DiscoveryDB snapshot → endpoints ────────────────────
//
// The wire rung: endpoints are harvested from a window-end snapshot
// of the participant's loss-proof internal DiscoveryDB (provided by rustdds
// 0.14.2; the fork is retained only for the participant lease knob), NOT the
// lossy event stream. The mapping is split so the load-bearing logic — the
// USER_DATA→type_hash parse, the GUID carry, and the own-prefix filter — is a
// PURE function over plain field values (`map_snapshot_endpoints`),
// oracle-tested with hand-built inputs; the live `snapshot_endpoints` adaptor
// is a thin field-extraction shell over the rustdds discovery structs (which
// need crate-internal builders and so are exercised against a live peer).

/// Plain per-endpoint field values extracted from a DiscoveryDB snapshot
/// (`DiscoveredWriterData`/`DiscoveredReaderData`) — the input to the PURE
/// `map_snapshot_endpoints` mapper, so the snapshot→[`DiscoveredEndpoint`]
/// logic is unit-testable without constructing rustdds discovery structs.
#[derive(Debug)]
struct SnapshotEndpointFields {
    kind: EndpointKind,
    /// Raw DDS topic name (`rt/utlidar/cloud`).
    topic_name: String,
    /// Raw DDS type name (`sensor_msgs::msg::dds_::PointCloud2_`).
    type_name: String,
    /// The endpoint's own 16-byte RTPS GUID.
    guid: [u8; 16],
    /// Summarized offered/requested QoS.
    qos: DiscoveredQos,
    /// The endpoint's SEDP `USER_DATA` — the RAW blob as rustdds hands it,
    /// i.e. CDR-ENCAPSULATED (`[u32 length][typehash=RIHS01_…;][padding]`; the
    /// wire.rs parse seam strips the prefix). REAL on the
    /// published rustdds 0.14.2 dependency (the Cerulion fork is retained only
    /// for its participant lease duration knob).
    user_data: Vec<u8>,
}

/// Map a DiscoveryDB snapshot of endpoint fields into `(foreign endpoints,
/// own_hidden count)`: each entry becomes a [`DiscoveredEndpoint`] — the RIHS01
/// hash parsed from its `USER_DATA` via
/// [`crate::wire::parse_type_hash_from_user_data`], its GUID carried as
/// `writer_guid` — EXCEPT entries whose GUID prefix matches `own_prefix` (our
/// participant's own parameter-service endpoints), which are COUNTED, not
/// collected. Deterministic (input order preserved). Pure — oracle-tested.
fn map_snapshot_endpoints(
    snapshot: Vec<SnapshotEndpointFields>,
    own_prefix: [u8; 12],
) -> (Vec<DiscoveredEndpoint>, usize) {
    let mut foreign: Vec<DiscoveredEndpoint> = Vec::new();
    let mut own_hidden: usize = 0;
    for f in snapshot {
        if f.guid[..12] == own_prefix {
            own_hidden += 1;
            continue;
        }
        foreign.push(DiscoveredEndpoint {
            dds_topic: f.topic_name,
            type_name: f.type_name,
            qos: f.qos,
            kind: f.kind,
            type_hash: crate::wire::parse_type_hash_from_user_data(&f.user_data),
            writer_guid: Some(f.guid),
        });
    }
    (foreign, own_hidden)
}

/// Snapshot the participant's loss-proof DiscoveryDB at window end and map it to
/// `(foreign endpoints, own_hidden)`. THIN adaptor: it extracts plain field
/// values from each fork-provided `DiscoveredWriterData`/`DiscoveredReaderData`
/// (topic/type via the topic-data accessors, GUID via the endpoint proxy, the
/// summarized QoS, and the REAL SEDP `USER_DATA`) and defers the map +
/// own-prefix filter to the pure `map_snapshot_endpoints`. The rustdds field
/// access is exercised against a live peer; the mapping logic is unit-tested.
fn snapshot_endpoints(
    participant: &DiscoveryParticipant,
    own_guid: GUID,
) -> (Vec<DiscoveredEndpoint>, usize) {
    let mut snapshot: Vec<SnapshotEndpointFields> = Vec::new();
    for w in participant.discovered_writers() {
        let topic_data = &w.publication_topic_data;
        snapshot.push(SnapshotEndpointFields {
            kind: EndpointKind::Writer,
            topic_name: topic_data.topic_name().clone(),
            // `type_name` is a bare pub field on PublicationBuiltinTopicData
            // (only the subscription side has the accessor method upstream).
            type_name: topic_data.type_name.clone(),
            guid: w.writer_proxy.remote_writer_guid.to_bytes(),
            qos: summarize_qos(&topic_data.qos()),
            user_data: w.user_data.clone(),
        });
    }
    for r in participant.discovered_readers() {
        let topic_data = &r.subscription_topic_data;
        snapshot.push(SnapshotEndpointFields {
            kind: EndpointKind::Reader,
            topic_name: topic_data.topic_name().clone(),
            type_name: topic_data.type_name().clone(),
            guid: r.reader_proxy.remote_reader_guid.to_bytes(),
            qos: summarize_qos(&topic_data.qos()),
            user_data: r.user_data.clone(),
        });
    }
    let own_prefix: [u8; 12] = own_guid.to_bytes()[..12].try_into().expect("16 >= 12");
    map_snapshot_endpoints(snapshot, own_prefix)
}

/// Summarize a rustdds [`QosPolicies`] into the DDS-free [`DiscoveredQos`].
fn summarize_qos(qos: &QosPolicies) -> DiscoveredQos {
    let reliability = match qos.reliability() {
        Some(policy::Reliability::Reliable { .. }) => QosReliability::Reliable,
        Some(policy::Reliability::BestEffort) => QosReliability::BestEffort,
        None => QosReliability::Unknown,
    };
    let durability = match qos.durability() {
        Some(policy::Durability::Volatile) => QosDurability::Volatile,
        Some(policy::Durability::TransientLocal) => QosDurability::TransientLocal,
        Some(policy::Durability::Transient) => QosDurability::Transient,
        Some(policy::Durability::Persistent) => QosDurability::Persistent,
        None => QosDurability::Unknown,
    };
    DiscoveredQos {
        reliability,
        durability,
    }
}

#[cfg(test)]
mod tests {
    use std::collections::BTreeSet;
    use std::time::Duration;

    use ros2_client::rustdds::GUID;

    use super::*;

    /// A distinct "our participant" GUID for the own-prefix filter test — a
    /// random prefix (`new_participant_guid`), so a snapshot endpoint built under
    /// a different prefix is FOREIGN to it.
    fn own_guid() -> GUID {
        GUID::new_participant_guid()
    }

    /// A 16-byte endpoint GUID = participant `prefix` (first 12 bytes) + a
    /// distinct entity `tail` — models a real endpoint GUID under a participant.
    fn endpoint_guid(prefix: [u8; 12], tail: u8) -> [u8; 16] {
        let mut g = [tail; 16];
        g[..12].copy_from_slice(&prefix);
        g
    }

    /// A hand-built [`SnapshotEndpointFields`] — the DiscoveryDB-snapshot payload
    /// shape the pure [`map_snapshot_endpoints`] mapper consumes (the
    /// replacement for the old `EndpointDescription` event oracle). QoS is fixed
    /// to best_effort/volatile; override the field directly where a test needs a
    /// specific profile.
    fn snap(
        kind: EndpointKind,
        topic: &str,
        ty: &str,
        guid: [u8; 16],
        user_data: &[u8],
    ) -> SnapshotEndpointFields {
        SnapshotEndpointFields {
            kind,
            topic_name: topic.to_string(),
            type_name: ty.to_string(),
            guid,
            qos: DiscoveredQos {
                reliability: QosReliability::BestEffort,
                durability: QosDurability::Volatile,
            },
            user_data: user_data.to_vec(),
        }
    }

    /// A flood-filler event that maps to NEITHER a node NOR a participant
    /// (`TopicDetected` is not `ParticipantDiscovered`/`ROS`) — keeps the channel
    /// perpetually Ready without polluting the collected node/participant sets.
    fn flood_event() -> NodeEvent {
        NodeEvent::DDS(DomainParticipantStatusEvent::TopicDetected {
            name: "rt/flood".to_string(),
            type_name: "std_msgs::msg::dds_::String_".to_string(),
        })
    }

    /// The deadline check under a sustained event flood: the flood must NOT
    /// starve the deadline. `futures::future::select` polls recv first, so
    /// without the top-of-loop `Instant` check a perpetually-Ready channel keeps
    /// the loop consuming events past the window — this test's flooder runs 3 s
    /// past the 200 ms deadline, and `collect_endpoints` would only return when
    /// the flooder stopped (elapsed >= ~3.2 s, failing the 1.5 s bound). The
    /// `Instant` check breaks at the deadline regardless of recv readiness.
    /// Bounds are generous (200 ms window, 1.5 s allowance) so a stalled CI VM
    /// does not flake the pin.
    #[test]
    fn sustained_event_flood_cannot_starve_the_deadline() {
        let (tx, rx) = async_channel::bounded::<NodeEvent>(8);
        let window = Duration::from_millis(200);
        let deadline = Instant::now() + window;
        let flood_stop = deadline + Duration::from_millis(3000);

        let (nodes, participants, closed_early, elapsed) = smol::block_on(async move {
            let ex = smol::LocalExecutor::new();
            // The flooder keeps the channel perpetually Ready until well past
            // the deadline. When the consumer stops and drops `rx`, the
            // pending send errs and the flooder exits (no deadlock).
            let flood = ex.spawn(async move {
                while Instant::now() < flood_stop {
                    if tx.send(flood_event()).await.is_err() {
                        return;
                    }
                }
            });
            let started = Instant::now();
            let (nodes, parts, closed) = ex.run(collect_endpoints(rx, deadline)).await;
            let elapsed = started.elapsed();
            drop(flood);
            (nodes, parts, closed, elapsed)
        });

        assert!(
            elapsed < Duration::from_millis(1500),
            "the deadline must bound the collect regardless of recv readiness \
             (elapsed {elapsed:?}; without the deadline check this runs until the flooder stops at ~3.2 s)"
        );
        assert!(!closed_early, "the channel never closed before the break");
        // `TopicDetected` is neither a node (ROS PEI) nor a participant
        // (SPDP ParticipantDiscovered), so the flood fabricates neither.
        assert!(nodes.is_empty(), "the flood must not fabricate a node");
        assert!(
            participants.is_empty(),
            "the flood must not fabricate a participant"
        );
    }

    /// A status channel that closes BEFORE the deadline (spinner
    /// death) returns the PARTIAL node/participant set collected so far and flags
    /// `closed_early` — the caller's loud warn keys off the flag. The
    /// partial set is the EVENT-fed half (nodes/participants); endpoints ride
    /// the DB snapshot, independent of the channel. A `ros_discovery_info` PEI is
    /// delivered, then the channel closes — the node survives, the flag is set.
    #[test]
    fn closed_channel_returns_partial_set_and_flags_closed_early() {
        use ros2_client::entities_info::NodeEntitiesInfo;
        use ros2_client::NodeName;

        let part_guid = GUID::new_participant_guid();
        let node_info = NodeEntitiesInfo::new(NodeName::new("/robot", "talker").expect("name"));
        let pei = ParticipantEntitiesInfo::new(part_guid.into(), vec![node_info]);

        let (tx, rx) = async_channel::unbounded::<NodeEvent>();
        tx.send_blocking(NodeEvent::ROS(pei)).expect("send");
        tx.send_blocking(flood_event()).expect("send"); // maps to no node/participant
        drop(tx); // spinner death: the channel closes with the deadline far away

        let deadline = Instant::now() + Duration::from_secs(30);
        let (nodes, participants, closed_early) = smol::block_on(collect_endpoints(rx, deadline));

        assert!(closed_early, "a pre-deadline close must be flagged");
        assert!(participants.is_empty(), "no ParticipantDiscovered was sent");
        // The partial node set collected before the close survives (hand oracle).
        assert_eq!(nodes.len(), 1);
        assert_eq!(nodes[0].name, "talker");
        assert_eq!(nodes[0].fully_qualified_name(), "/robot/talker");
    }

    /// The deadline predicate's oracle vector (pure half).
    #[test]
    fn collect_deadline_predicate_oracle() {
        let t0 = Instant::now();
        let later = t0 + Duration::from_millis(10);
        assert!(!collect_deadline_reached(t0, later), "before the deadline");
        assert!(collect_deadline_reached(later, later), "at the deadline");
        assert!(
            collect_deadline_reached(later + Duration::from_millis(1), later),
            "past the deadline"
        );
    }

    // ─────────────────── snapshot → DiscoveredEndpoint mapper ───────────────
    //
    // The endpoint mapping + own-prefix filter moved off the (lossy)
    // event stream onto the pure `map_snapshot_endpoints` over a DiscoveryDB
    // snapshot. These tests pin that pure fn with hand-built field values (the
    // live `snapshot_endpoints` adaptor's rustdds field extraction is
    // exercised against a live peer).

    /// The own-endpoint filter over the pure snapshot mapper: endpoints whose
    /// GUID prefix matches OUR participant (the discovery node's own
    /// parameter-service endpoints) are HIDDEN and COUNTED — foreign endpoints
    /// pass through untouched, in input order. Structural GUID match: the own
    /// endpoints carry a robot-looking topic name, proving no name heuristic. A
    /// writer AND a reader of ours are both filtered (both kinds).
    #[test]
    fn map_snapshot_own_prefix_endpoints_are_hidden_and_counted() {
        let ours = own_guid();
        let own_prefix: [u8; 12] = ours.to_bytes()[..12].try_into().unwrap();
        // One of OURS (writer, robot-looking topic), one FOREIGN (distinct
        // prefix), and a second of OURS as a Reader (both kinds filtered).
        let snapshot = vec![
            snap(
                EndpointKind::Writer,
                "rt/utlidar/cloud",
                "sensor_msgs::msg::dds_::PointCloud2_",
                endpoint_guid(own_prefix, 0x07),
                b"",
            ),
            snap(
                EndpointKind::Writer,
                "rt/foreign/topic",
                "acme_msgs::msg::dds_::Widget_",
                endpoint_guid([0xAB; 12], 0x01),
                b"typehash=RIHS01_ab;",
            ),
            snap(
                EndpointKind::Reader,
                "rt/utlidar/cloud",
                "sensor_msgs::msg::dds_::PointCloud2_",
                endpoint_guid(own_prefix, 0x09),
                b"",
            ),
        ];
        let (endpoints, own_hidden) = map_snapshot_endpoints(snapshot, own_prefix);
        assert_eq!(own_hidden, 2, "both of our own endpoints must be counted");
        assert_eq!(endpoints.len(), 1, "only the foreign endpoint survives");
        assert_eq!(endpoints[0].kind, EndpointKind::Writer);
        assert_eq!(endpoints[0].dds_topic, "rt/foreign/topic");
        assert_eq!(endpoints[0].type_hash.as_deref(), Some("RIHS01_ab"));
    }

    /// Retargeted from `enriched_endpoint_carries_type_hash_and_writer_guid`:
    /// a writer endpoint's SEDP `USER_DATA` `typehash=` is parsed into
    /// [`DiscoveredEndpoint::type_hash`], its GUID carried into `writer_guid`, and
    /// its kind/topic/type pass through verbatim (hand oracle).
    #[test]
    fn map_snapshot_writer_carries_type_hash_and_guid() {
        let g = endpoint_guid([0x0a; 12], 0x01);
        let snapshot = vec![snap(
            EndpointKind::Writer,
            "rt/widget",
            "acme_msgs::msg::dds_::Widget_",
            g,
            b"enclave=/robot;typehash=RIHS01_ab;",
        )];
        // own_prefix is 0xff.. → distinct from g's 0x0a.. prefix (nothing hidden).
        let (endpoints, own_hidden) = map_snapshot_endpoints(snapshot, [0xff; 12]);
        assert_eq!(own_hidden, 0);
        assert_eq!(endpoints.len(), 1);
        assert_eq!(endpoints[0].kind, EndpointKind::Writer);
        assert_eq!(endpoints[0].dds_topic, "rt/widget");
        assert_eq!(endpoints[0].type_name, "acme_msgs::msg::dds_::Widget_");
        assert_eq!(endpoints[0].type_hash.as_deref(), Some("RIHS01_ab"));
        assert_eq!(endpoints[0].writer_guid, Some(g));
    }

    /// Production-path parity: the LIVE blob is
    /// CDR-ENCAPSULATED (`[u32 len][content][padding]` — the shape captured
    /// from a live Jazzy peer), and the snapshot mapper feeds it to the wire.rs parse
    /// seam RAW — the strip must compose through `map_snapshot_endpoints`, not
    /// only in the wire.rs unit oracles (whose naked twins pin the fall-through).
    #[test]
    fn map_snapshot_encapsulated_user_data_parses_hash() {
        let g = endpoint_guid([0x0b; 12], 0x01);
        // [u32 LE len=19]["typehash=RIHS01_ab;"][1 pad byte → 20, 4-aligned].
        let mut ud: Vec<u8> = vec![19, 0, 0, 0];
        ud.extend_from_slice(b"typehash=RIHS01_ab;");
        ud.push(0);
        let snapshot = vec![snap(
            EndpointKind::Writer,
            "rt/widget",
            "acme_msgs::msg::dds_::Widget_",
            g,
            &ud,
        )];
        let (endpoints, _own) = map_snapshot_endpoints(snapshot, [0xff; 12]);
        assert_eq!(endpoints.len(), 1);
        assert_eq!(endpoints[0].type_hash.as_deref(), Some("RIHS01_ab"));
    }

    /// A READER endpoint with EMPTY `USER_DATA` maps to kind Reader with
    /// NO type hash (pre-Iron / non-ROS peer), the GUID still carried.
    #[test]
    fn map_snapshot_reader_empty_user_data_has_no_hash() {
        let g = endpoint_guid([0x0c; 12], 0x02);
        let snapshot = vec![snap(
            EndpointKind::Reader,
            "rt/cmd_vel",
            "geometry_msgs::msg::dds_::Twist_",
            g,
            b"",
        )];
        let (endpoints, _own) = map_snapshot_endpoints(snapshot, [0xff; 12]);
        assert_eq!(endpoints.len(), 1);
        assert_eq!(endpoints[0].kind, EndpointKind::Reader);
        assert_eq!(endpoints[0].type_hash, None, "empty USER_DATA → no hash");
        assert_eq!(endpoints[0].writer_guid, Some(g));
    }

    /// Malformed `USER_DATA` (non-UTF-8 bytes, or a segment with no
    /// `=`) yields NO hash — never a panic (the `parse_user_data` tolerance,
    /// exercised through the mapper).
    #[test]
    fn map_snapshot_malformed_user_data_has_no_hash() {
        // Non-UTF-8 bytes.
        let g1 = endpoint_guid([0x0d; 12], 0x03);
        let (eps1, _) = map_snapshot_endpoints(
            vec![snap(
                EndpointKind::Writer,
                "rt/x",
                "acme::msg::dds_::X_",
                g1,
                &[0xff, 0xfe, 0x00],
            )],
            [0xff; 12],
        );
        assert_eq!(eps1[0].type_hash, None);
        // A `typehash` segment with no `=` (and no other typehash) → no hash.
        let g2 = endpoint_guid([0x0d; 12], 0x04);
        let (eps2, _) = map_snapshot_endpoints(
            vec![snap(
                EndpointKind::Writer,
                "rt/y",
                "acme::msg::dds_::Y_",
                g2,
                b"typehash;enclave=/",
            )],
            [0xff; 12],
        );
        assert_eq!(eps2[0].type_hash, None);
    }

    /// The summarized QoS carried on the snapshot field passes through
    /// the mapper untouched (the report's QoS cell survives the harvest change).
    #[test]
    fn map_snapshot_qos_passes_through() {
        let g = endpoint_guid([0x0e; 12], 0x05);
        let mut f = snap(EndpointKind::Writer, "rt/z", "acme::msg::dds_::Z_", g, b"");
        f.qos = DiscoveredQos {
            reliability: QosReliability::Reliable,
            durability: QosDurability::TransientLocal,
        };
        let (endpoints, _own) = map_snapshot_endpoints(vec![f], [0xff; 12]);
        assert_eq!(
            endpoints[0].qos,
            DiscoveredQos {
                reliability: QosReliability::Reliable,
                durability: QosDurability::TransientLocal,
            }
        );
    }

    /// A `ros_discovery_info` `ParticipantEntitiesInfo`
    /// (delivered as `NodeEvent::ROS`) folds into the node table, keyed by the
    /// participant's 12-byte GUID prefix (hand oracle). This is the endpoint→node
    /// service-join input.
    #[test]
    fn ros_discovery_info_builds_node_table() {
        // `ParticipantEntitiesInfo` is already in scope from the module import.
        use ros2_client::entities_info::NodeEntitiesInfo;
        use ros2_client::NodeName;

        let part_guid = GUID::new_participant_guid();
        let node_info = NodeEntitiesInfo::new(NodeName::new("/robot", "talker").expect("name"));
        // `ParticipantEntitiesInfo::new` takes a private `Gid`; `.into()` infers
        // it from the param type (the `From<GUID>` impl is public).
        let pei = ParticipantEntitiesInfo::new(part_guid.into(), vec![node_info]);

        let (tx, rx) = async_channel::unbounded::<NodeEvent>();
        tx.send_blocking(NodeEvent::ROS(pei)).expect("send");
        drop(tx);

        let deadline = Instant::now() + Duration::from_secs(30);
        let (nodes, _parts, _closed) = smol::block_on(collect_endpoints(rx, deadline));

        let expected_prefix: [u8; 12] = part_guid.to_bytes()[..12].try_into().unwrap();
        assert_eq!(nodes.len(), 1);
        assert_eq!(nodes[0].name, "talker");
        assert_eq!(nodes[0].participant_prefix, expected_prefix);
        // The fully-qualified name is the namespace under which the node's
        // get_type_description service is served (robust to the namespace form).
        assert_eq!(nodes[0].fully_qualified_name(), "/robot/talker");
    }

    // ─────────────────── merge_ros_nodes rekey + snapshot (d.2) ─────────────

    /// A `ParticipantEntitiesInfo` over `nodes` (namespace, base name pairs)
    /// keyed to `part_guid`'s prefix — the `merge_ros_nodes` input oracle.
    fn pei_for(part_guid: GUID, node_pairs: &[(&str, &str)]) -> ParticipantEntitiesInfo {
        use ros2_client::entities_info::NodeEntitiesInfo;
        use ros2_client::NodeName;
        let infos: Vec<NodeEntitiesInfo> = node_pairs
            .iter()
            .map(|(ns, name)| NodeEntitiesInfo::new(NodeName::new(ns, name).expect("name")))
            .collect();
        ParticipantEntitiesInfo::new(part_guid.into(), infos)
    }

    fn prefix_of(g: GUID) -> [u8; 12] {
        g.to_bytes()[..12].try_into().unwrap()
    }

    /// Two SAME-named nodes on DIFFERENT participants must
    /// both survive (a `(namespace, name)` key would collapse them).
    #[test]
    fn merge_ros_nodes_keeps_same_named_nodes_on_different_participants() {
        let a = GUID::new_participant_guid();
        let b = GUID::new_participant_guid();
        let mut nodes: Vec<DiscoveredNode> = Vec::new();
        merge_ros_nodes(&mut nodes, &pei_for(a, &[("/robot", "talker")]));
        merge_ros_nodes(&mut nodes, &pei_for(b, &[("/robot", "talker")]));
        assert_eq!(
            nodes.len(),
            2,
            "same-named nodes on distinct participants both survive"
        );
        let prefixes: BTreeSet<[u8; 12]> = nodes.iter().map(|n| n.participant_prefix).collect();
        assert!(prefixes.contains(&prefix_of(a)));
        assert!(prefixes.contains(&prefix_of(b)));
        assert!(nodes
            .iter()
            .all(|n| n.name == "talker" && n.namespace == "/robot"));
    }

    /// A participant re-announcing with a CHANGED node list
    /// REPLACES its whole set — the dropped node is pruned, others unaffected.
    #[test]
    fn merge_ros_nodes_replaces_participant_snapshot_and_prunes() {
        let a = GUID::new_participant_guid();
        let b = GUID::new_participant_guid();
        let mut nodes: Vec<DiscoveredNode> = Vec::new();
        // A hosts alpha+beta; B hosts gamma (a bystander whose set must survive).
        merge_ros_nodes(&mut nodes, &pei_for(a, &[("/", "alpha"), ("/", "beta")]));
        merge_ros_nodes(&mut nodes, &pei_for(b, &[("/", "gamma")]));
        assert_eq!(nodes.len(), 3);

        // A re-announces with ONLY alpha → beta pruned; gamma (participant B)
        // untouched.
        merge_ros_nodes(&mut nodes, &pei_for(a, &[("/", "alpha")]));
        let names: BTreeSet<&str> = nodes.iter().map(|n| n.name.as_str()).collect();
        assert_eq!(nodes.len(), 2, "beta pruned on the snapshot replace");
        assert!(names.contains("alpha"));
        assert!(
            names.contains("gamma"),
            "the other participant's set survives"
        );
        assert!(!names.contains("beta"), "the dropped node is pruned");
    }

    /// A multi-node PEI folds every hosted node under one
    /// participant prefix.
    #[test]
    fn merge_ros_nodes_multi_node_pei() {
        let a = GUID::new_participant_guid();
        let mut nodes: Vec<DiscoveredNode> = Vec::new();
        merge_ros_nodes(
            &mut nodes,
            &pei_for(a, &[("/", "n1"), ("/robot", "n2"), ("/robot/sub", "n3")]),
        );
        assert_eq!(nodes.len(), 3);
        assert!(nodes.iter().all(|n| n.participant_prefix == prefix_of(a)));
        let fqns: BTreeSet<String> = nodes.iter().map(|n| n.fully_qualified_name()).collect();
        assert!(fqns.contains("/n1"));
        assert!(fqns.contains("/robot/n2"));
        assert!(fqns.contains("/robot/sub/n3"));
    }

    // ─────────────── own-node filter ────────────────────────────────────────

    /// The headline pin: an OWN-NODE-ONLY table —
    /// exactly what a window harvests when every REMOTE PEI dropped in the
    /// bounded(8) join burst but our own node's looped-back PEI survived — is
    /// EMPTY after the filter, so table-keyed loss diagnostics can actually
    /// fire (without the filter, `/cerulion_attach/discovery` sits in the table forever and
    /// an emptiness check is structurally unreachable on the live path).
    #[test]
    fn split_off_own_nodes_own_only_input_empties_the_table() {
        let ours = own_guid();
        let own_prefix: [u8; 12] = ours.to_bytes()[..12].try_into().unwrap();
        let nodes = vec![DiscoveredNode {
            namespace: "/cerulion_attach".to_string(),
            name: "discovery".to_string(),
            participant_prefix: own_prefix,
        }];
        let (foreign, hidden) = split_off_own_nodes(nodes, own_prefix);
        assert!(
            foreign.is_empty(),
            "an own-node-only table must filter to EMPTY"
        );
        assert_eq!(hidden, 1);
    }

    /// Foreign nodes pass through in input order;
    /// only own-prefix entries are hidden (structural match — an own node with
    /// a robot-looking name is still filtered, a foreign node named
    /// `discovery` is still kept: no name heuristic).
    #[test]
    fn split_off_own_nodes_keeps_foreign_in_order() {
        let ours = own_guid();
        let own_prefix: [u8; 12] = ours.to_bytes()[..12].try_into().unwrap();
        let nodes = vec![
            DiscoveredNode {
                namespace: "/robot".to_string(),
                name: "talker".to_string(),
                participant_prefix: [0xAB; 12],
            },
            DiscoveredNode {
                namespace: "/robot".to_string(),
                name: "camera_driver".to_string(), // robot-looking, but OURS
                participant_prefix: own_prefix,
            },
            DiscoveredNode {
                namespace: "/".to_string(),
                name: "discovery".to_string(), // ours-looking, but FOREIGN
                participant_prefix: [0xCD; 12],
            },
        ];
        let (foreign, hidden) = split_off_own_nodes(nodes, own_prefix);
        assert_eq!(hidden, 1, "only the own-prefix node is hidden");
        let names: Vec<&str> = foreign.iter().map(|n| n.name.as_str()).collect();
        assert_eq!(names, vec!["talker", "discovery"], "input order preserved");
    }

    // ───────────── GID width pins ───────────────────────────────────────────

    /// The feature-WIRING pin: the `jazzy` feature
    /// must select ros2-client's 16-byte-GID (Iron+) world — the width
    /// `ros_discovery_info` PEIs are encoded with across the wire rung's whole
    /// functional domain. ros2-client keys the width on its `iron` feature
    /// (`gid.rs`: `GID_LENGTH` 16 from `iron` on, 24 below) and our `jazzy`
    /// feature maps to `ros2-client/jazzy` ⊇ `iron`; a fat-fingered mapping
    /// (e.g. `jazzy = ["live", "ros2-client/humble"]`) compiles a 24-byte `Gid`
    /// and fails BOTH arms here. `Gid` (and `GID_LENGTH`) live in a PRIVATE
    /// ros2-client module, so the width pin rides `size_of_val` on a real
    /// `pei.gid()` value (`Gid` is a bare `[u8; GID_LENGTH]` newtype — its size
    /// IS the wire width).
    #[cfg(feature = "jazzy")]
    #[test]
    fn jazzy_feature_selects_the_16_byte_gid_world() {
        let pei = pei_for(GUID::new_participant_guid(), &[]);
        assert_eq!(
            std::mem::size_of_val(&pei.gid()),
            16,
            "the jazzy build must carry the 16-byte (Iron+) Gid"
        );
        assert!(
            ros2_client::COMPILED_ROS_DISTRO >= ros2_client::RosDistro::Iron,
            "the jazzy build must compile an Iron+ distro (got {})",
            ros2_client::COMPILED_ROS_DISTRO
        );
    }

    /// The DEFAULT-flip tripwire: the crate's
    /// default features must stay on `jazzy`. The failure it guards against is the
    /// `humble` default compiling the 24-byte-GID world — every Iron+ network's
    /// `ros_discovery_info` failed to decode, the node table stayed empty, and
    /// the wire rung was structurally inert with a fully GREEN suite. This test
    /// is deliberately UNCONDITIONAL in the live module (a `#[cfg(feature =
    /// "jazzy")]` gate would VANISH with the flip and stay green): reverting
    /// `default = ["jazzy"]` fails it in the default `cargo test` run. NB: a
    /// DELIBERATE humble-only test run (`--no-default-features --features
    /// humble` + `cargo test`) fails it by design — that build is structurally
    /// inert on the rung's functional domain, and the CI gate only ever
    /// cargo-CHECKs the humble opt-in.
    #[test]
    fn default_build_stays_on_the_16_byte_gid_jazzy_world() {
        // Bound first so the assert sees a variable, not a cfg!-expanded
        // boolean literal (clippy::assertions_on_constants safety across
        // toolchains).
        let jazzy_enabled = cfg!(feature = "jazzy");
        assert!(
            jazzy_enabled,
            "cerulion_dds must default to the `jazzy` (16-byte-GID, Iron+) feature — a `humble` \
             default re-inerts the wire rung on every Iron+ network"
        );
    }

    // ──────────────── participant vendor mapping + upsert (d.7) ─────────────

    /// The pure core of the `ParticipantDiscovered` arm —
    /// `discovered_participant_from_parts` extracts the 12-byte prefix from the
    /// GUID and carries the vendor id verbatim (hand oracle). NB: the match arm
    /// itself cannot be exercised hermetically because rustdds's
    /// `ParticipantDescription`/`VendorId` are not constructible outside rustdds
    /// (`mod messages` is private); this pins the load-bearing extraction.
    #[test]
    fn discovered_participant_from_parts_extracts_prefix_and_vendor() {
        let g = GUID::new_participant_guid();
        let dp = discovered_participant_from_parts(g, [0x01, 0x10]); // Cyclone
        assert_eq!(dp.guid_prefix, prefix_of(g));
        assert_eq!(dp.vendor_id, [0x01, 0x10]);
    }

    /// `upsert_participant` dedups by GUID prefix and the
    /// LATEST vendor id wins on re-announce (the SPDP re-announcement path).
    #[test]
    fn upsert_participant_dedups_by_prefix_latest_vendor_wins() {
        let a = GUID::new_participant_guid();
        let b = GUID::new_participant_guid();
        let mut parts: Vec<DiscoveredParticipant> = Vec::new();
        upsert_participant(
            &mut parts,
            discovered_participant_from_parts(a, [0x01, 0x10]), // Cyclone
        );
        upsert_participant(
            &mut parts,
            discovered_participant_from_parts(b, [0x01, 0x0F]), // Fast DDS
        );
        assert_eq!(parts.len(), 2);
        // A re-announces with a DIFFERENT vendor id → dedup, latest wins.
        upsert_participant(
            &mut parts,
            discovered_participant_from_parts(a, [0x01, 0x0F]),
        );
        assert_eq!(parts.len(), 2, "re-announce dedups by prefix");
        let a_entry = parts
            .iter()
            .find(|p| p.guid_prefix == prefix_of(a))
            .expect("A present");
        assert_eq!(a_entry.vendor_id, [0x01, 0x0F], "latest vendor id wins");
    }
}
