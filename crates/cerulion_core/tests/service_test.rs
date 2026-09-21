// SPDX-License-Identifier: AGPL-3.0-only
//! End-to-end tests for the deterministic request-response (services)
//! layer over real iceoryx2 transport.
//!
//! ⚠️ iceoryx2 shared memory is a process singleton — run with
//! `--test-threads=1`:
//!
//! ```bash
//! cargo test -p cerulion_core --test service_test -- --test-threads=1
//! ```

use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::Arc;
use std::time::SystemTime;

use cerulion_core::error::TransportResult;
use cerulion_core::testing::{count_at, count_at_exclusively, debug_lines_expected, never_loud};
use cerulion_core::transport::service::{
    derive_client_guid, ServiceClient, ServiceEnvelope, ServiceServer,
};
use cerulion_core::transport::TransportManager;
use cerulion_core::wire::{fnv1a_hash, MaxSliceLen};
// No-env-filter so the capture reaches the `cerulion_core` target
// (the schema-mismatch reporter's crate), not just this test crate.
use tracing_test::traced_test;

const SLICE: MaxSliceLen = MaxSliceLen::const_new(64 * 1024);

static SERVICE_COUNTER: AtomicU64 = AtomicU64::new(0);

/// Global iceoryx2 singleton (same pattern as transport_test.rs) +
/// unique-per-run service names so reruns and the singleton don't
/// interfere.
fn test_transport(_label: &str) -> Arc<TransportManager> {
    TransportManager::get_or_init().expect("transport init")
}

fn unique_service(base: &str) -> String {
    let nanos = SystemTime::now()
        .duration_since(SystemTime::UNIX_EPOCH)
        .unwrap()
        .as_nanos();
    let id = SERVICE_COUNTER.fetch_add(1, Ordering::Relaxed);
    format!("{base}/{nanos}/{id}")
}

fn req_hash(service: &str) -> u64 {
    fnv1a_hash(format!("{service}_Request").as_bytes())
}

fn resp_hash(service: &str) -> u64 {
    fnv1a_hash(format!("{service}_Response").as_bytes())
}

/// Take every queued request through the PRODUCTION take path, returning the
/// number delivered to `f`.
///
/// `try_take_one_request` is the only request take path the crate offers (see
/// the `service` module's "Taking" section): it delivers at most one VALID
/// request per call, skipping — and LOUDLY reporting — invalid frames on its
/// way, and returns `false` once the queue holds no further valid request. So
/// looping it to `false` visits exactly the frames one pass over the queue
/// holds, in the same FIFO order, and emits exactly the same per-frame
/// reports. This is a TEST convenience for arms whose oracle is a multi-frame
/// sequence; production (rmw) hands back one message per take and calls again.
fn drain_requests<F>(server: &mut ServiceServer, mut f: F) -> TransportResult<usize>
where
    F: FnMut(ServiceEnvelope, &[u8]),
{
    let mut delivered = 0usize;
    while server.try_take_one_request(&mut f)? {
        delivered += 1;
    }
    Ok(delivered)
}

/// The client-side twin of [`drain_requests`] — same contract, same reason.
fn drain_responses<F>(client: &mut ServiceClient, mut f: F) -> TransportResult<usize>
where
    F: FnMut(i64, &[u8]),
{
    let mut delivered = 0usize;
    while client.try_take_one_response(&mut f)? {
        delivered += 1;
    }
    Ok(delivered)
}

/// Full round-trip: two requests drained FIFO by the server, responses
/// correlated back by sequence, payloads byte-exact.
#[test]
fn request_response_round_trip_fifo() {
    let transport = test_transport("rt");
    let service = &unique_service("/compute_ik_rt");

    let mut server = transport
        .create_service_server(service, SLICE, req_hash(service), resp_hash(service))
        .expect("server");
    let guid = derive_client_guid("planner_node", service, 0);
    let mut client = transport
        .create_service_client(service, guid, SLICE, req_hash(service), resp_hash(service))
        .expect("client");

    let seq1 = client.send_request(b"request-one").expect("send 1");
    let seq2 = client.send_request(b"request-two").expect("send 2");
    assert_eq!(seq1, 1, "sequences start at 1 (rmw convention)");
    assert_eq!(seq2, 2, "sequences are monotonic");

    // Server drains BOTH requests in FIFO order (non-coalescing —
    // Principle #6: latest-wins would silently lose request one).
    let mut seen: Vec<(ServiceEnvelope, Vec<u8>)> = Vec::new();
    let drained = drain_requests(&mut server, |envelope, payload| {
        seen.push((envelope, payload.to_vec()));
    })
    .expect("take requests");
    assert_eq!(drained, 2);
    assert_eq!(seen[0].0.sequence, 1);
    assert_eq!(seen[0].1, b"request-one");
    assert_eq!(seen[1].0.sequence, 2);
    assert_eq!(seen[1].1, b"request-two");
    assert_eq!(seen[0].0.client_guid, guid);

    // Respond out of order — correlation is by envelope, not arrival.
    server
        .send_response(&seen[1].0, b"answer-two")
        .expect("respond 2");
    server
        .send_response(&seen[0].0, b"answer-one")
        .expect("respond 1");

    let mut responses: Vec<(i64, Vec<u8>)> = Vec::new();
    let delivered = drain_responses(&mut client, |seq, payload| {
        responses.push((seq, payload.to_vec()));
    })
    .expect("take responses");
    assert_eq!(delivered, 2);
    assert_eq!(responses[0], (2, b"answer-two".to_vec()));
    assert_eq!(responses[1], (1, b"answer-one".to_vec()));
}

/// Two clients of the same service: replies are isolated per client
/// (per-client reply topics — client A never sees client B's response).
#[test]
fn responses_are_isolated_per_client() {
    let transport = test_transport("iso");
    let service = &unique_service("/get_planning_scene_iso");

    let mut server = transport
        .create_service_server(service, SLICE, req_hash(service), resp_hash(service))
        .expect("server");

    let guid_a = derive_client_guid("node_a", service, 0);
    let guid_b = derive_client_guid("node_b", service, 0);
    assert_ne!(guid_a, guid_b);

    let mut client_a = transport
        .create_service_client(
            service,
            guid_a,
            SLICE,
            req_hash(service),
            resp_hash(service),
        )
        .expect("client a");
    let mut client_b = transport
        .create_service_client(
            service,
            guid_b,
            SLICE,
            req_hash(service),
            resp_hash(service),
        )
        .expect("client b");

    client_a.send_request(b"from-a").expect("send a");
    client_b.send_request(b"from-b").expect("send b");

    let mut requests: Vec<(ServiceEnvelope, Vec<u8>)> = Vec::new();
    drain_requests(&mut server, |e, p| requests.push((e, p.to_vec()))).expect("take");
    assert_eq!(requests.len(), 2);

    // Reply to each with a payload tagged by origin.
    for (envelope, payload) in &requests {
        let mut reply = b"reply-to-".to_vec();
        reply.extend_from_slice(payload);
        server.send_response(envelope, &reply).expect("respond");
    }
    assert_eq!(server.known_client_count(), 2);

    let mut got_a: Vec<Vec<u8>> = Vec::new();
    drain_responses(&mut client_a, |_, p| got_a.push(p.to_vec())).expect("take a");
    let mut got_b: Vec<Vec<u8>> = Vec::new();
    drain_responses(&mut client_b, |_, p| got_b.push(p.to_vec())).expect("take b");

    assert_eq!(got_a, vec![b"reply-to-from-a".to_vec()]);
    assert_eq!(got_b, vec![b"reply-to-from-b".to_vec()]);
}

/// Determinism (Principle #7): two identical runs produce bit-identical
/// envelopes and sequences — GUIDs are derived, sequences are counters,
/// timestamps come from the VirtualClock.
#[test]
fn service_traffic_is_replay_stable() {
    let run = |label: &str| -> (String, Vec<(ServiceEnvelope, Vec<u8>)>) {
        let transport = test_transport(label);
        let service = unique_service(&format!("/replay_{label}"));

        let mut server = transport
            .create_service_server(&service, SLICE, req_hash(&service), resp_hash(&service))
            .expect("server");
        let guid = derive_client_guid("replay_node", &service, 7);
        let mut client = transport
            .create_service_client(
                &service,
                guid,
                SLICE,
                req_hash(&service),
                resp_hash(&service),
            )
            .expect("client");

        for i in 0..3u8 {
            client.send_request(&[i, i, i]).expect("send");
        }
        let mut seen = Vec::new();
        drain_requests(&mut server, |e, p| seen.push((e, p.to_vec()))).expect("take");
        (service, seen)
    };

    // Envelopes are a pure function of the derivation inputs and the
    // request order — assert against independently recomputed values
    // (an oracle, never a self-comparison; the project forbids self-compares).
    let (service_name, a) = run("a1");
    assert_eq!(a.len(), 3);
    let expected_guid = derive_client_guid("replay_node", &service_name, 7);
    for (i, (envelope, payload)) in a.iter().enumerate() {
        assert_eq!(envelope.client_guid, expected_guid);
        assert_eq!(envelope.sequence, (i + 1) as i64);
        assert_eq!(payload, &vec![i as u8; 3]);
    }
}

/// Empty payloads are legal (many ROS service requests are empty, e.g.
/// std_srvs/Trigger).
#[test]
fn empty_payload_round_trips() {
    let transport = test_transport("empty");
    let service = &unique_service("/trigger_empty");

    let mut server = transport
        .create_service_server(service, SLICE, req_hash(service), resp_hash(service))
        .expect("server");
    let guid = derive_client_guid("n", service, 0);
    let mut client = transport
        .create_service_client(service, guid, SLICE, req_hash(service), resp_hash(service))
        .expect("client");

    client.send_request(b"").expect("send");
    let mut got = Vec::new();
    drain_requests(&mut server, |e, p| got.push((e, p.to_vec()))).expect("take");
    assert_eq!(got.len(), 1);
    assert!(got[0].1.is_empty());

    server.send_response(&got[0].0, b"").expect("respond");
    let mut replies = 0usize;
    drain_responses(&mut client, |seq, p| {
        assert_eq!(seq, 1);
        assert!(p.is_empty());
        replies += 1;
    })
    .expect("take response");
    assert_eq!(replies, 1);
}

/// A frame with the WRONG schema hash on the request topic is skipped
/// loudly, never delivered as a request (cross-type frames must not
/// alias — same contract as typed pub/sub).
#[test]
fn wrong_schema_hash_requests_are_skipped() {
    let transport = test_transport("hash");
    let service = &unique_service("/typed_hash");

    let mut server = transport
        .create_service_server(service, SLICE, req_hash(service), resp_hash(service))
        .expect("server");
    let guid = derive_client_guid("n", service, 0);

    // A client created with a DIFFERENT request hash (e.g. version-skewed
    // peer): its requests must not reach the server callback.
    let mut skewed_client = transport
        .create_service_client(
            service,
            guid,
            SLICE,
            fnv1a_hash(b"SomeOther_Request"),
            resp_hash(service),
        )
        .expect("client");
    skewed_client.send_request(b"bad").expect("send");

    let mut delivered = 0usize;
    let count = drain_requests(&mut server, |_, _| delivered += 1).expect("take");
    assert_eq!(count, 0, "wrong-hash request must be skipped");
    assert_eq!(delivered, 0);
}

/// Malformed frames (too short for the envelope) are skipped loudly,
/// never panicking the drain loop.
#[test]
fn malformed_request_frames_are_skipped() {
    use cerulion_core::transport::service::request_topic;
    use cerulion_core::wire::WireHeader;

    let transport = test_transport("malformed");
    let service = &unique_service("/malformed_svc");

    let mut server = transport
        .create_service_server(service, SLICE, req_hash(service), resp_hash(service))
        .expect("server");

    // Publish a frame with a valid WireHeader + correct hash but a body
    // SHORTER than the 24-byte envelope.
    let mut raw_pub = transport
        .create_publisher_simple(&request_topic(service), SLICE)
        .expect("raw pub");
    let body = [0u8; 10]; // < ServiceEnvelope::SIZE
    let total = WireHeader::SIZE + body.len();
    let header = WireHeader {
        schema_hash: req_hash(service),
        total_size: total as u32,
        offset_table_offset: 0,
        offset_table_count: 0,
        sequence: 1,
        timestamp_ns: 0,
    };
    let mut frame = vec![0u8; total];
    header.write_to_buf(&mut frame[..WireHeader::SIZE]);
    frame[WireHeader::SIZE..].copy_from_slice(&body);
    raw_pub.publish_raw(&frame).expect("publish raw");

    let mut delivered = 0usize;
    let count = drain_requests(&mut server, |_, _| delivered += 1).expect("take");
    assert_eq!(count, 0, "short frame must be skipped, not delivered");
    assert_eq!(delivered, 0);
}

/// Oversized payloads surface as loud Loan errors (the configured
/// max_slice_len bounds the SHM slot), never silent truncation.
#[test]
fn oversized_request_errors_loudly() {
    let transport = test_transport("oversize");
    let service = &unique_service("/oversize_svc");

    let guid = derive_client_guid("n", service, 0);
    let mut client = transport
        .create_service_client(service, guid, SLICE, req_hash(service), resp_hash(service))
        .expect("client");

    let huge = vec![0u8; SLICE.get() as usize + 1];
    let result = client.send_request(&huge);
    assert!(
        result.is_err(),
        "oversized request must error, not truncate"
    );
}

/// Three concurrent clients of one service — pins the iceoryx2
/// max_publishers provisioning (the default of 2 would make the THIRD client's
/// request publisher fail).
#[test]
fn three_clients_can_share_one_service() {
    let transport = test_transport("three");
    let service = &unique_service("/three_clients");

    let mut server = transport
        .create_service_server(service, SLICE, req_hash(service), resp_hash(service))
        .expect("server");

    let mut clients: Vec<_> = (0..3)
        .map(|i| {
            let guid = derive_client_guid("n", service, i);
            transport
                .create_service_client(service, guid, SLICE, req_hash(service), resp_hash(service))
                .unwrap_or_else(|e| panic!("client {i} creation failed: {e}"))
        })
        .collect();

    for (i, c) in clients.iter_mut().enumerate() {
        c.send_request(&[i as u8]).expect("send");
    }

    let mut seen = Vec::new();
    drain_requests(&mut server, |e, p| seen.push((e, p.to_vec()))).expect("take");
    assert_eq!(seen.len(), 3);
    // Cross-client drain order: iceoryx2 holds PER-PUBLISHER connection
    // queues and `receive()` scans them in connection-establishment
    // order — which here equals client-creation order, which equals
    // send order (one request each). Deterministic and replay-relevant,
    // so pin it; but it is NOT a general send-order guarantee across
    // interleaved multi-request clients (per-client FIFO is).
    assert_eq!(seen[0].1, vec![0u8]);
    assert_eq!(seen[1].1, vec![1u8]);
    assert_eq!(seen[2].1, vec![2u8]);
}

/// SentSample notification: a server (or client) blocked in
/// wait_for_message must be WOKEN by service traffic — publish_raw alone
/// does not notify (without the notify the wait stalls for the
/// full timeout and a response-loss window widens).
#[test]
fn service_sends_notify_listeners() {
    let transport = test_transport("notify");
    let service = &unique_service("/notify_svc");

    let mut server = transport
        .create_service_server(service, SLICE, req_hash(service), resp_hash(service))
        .expect("server");
    let guid = derive_client_guid("n", service, 0);
    let mut client = transport
        .create_service_client(service, guid, SLICE, req_hash(service), resp_hash(service))
        .expect("client");

    // Drain construction-time events (SubscriberConnected etc.).
    let _ = server.request_subscriber().test_drain_events();
    let _ = client.reply_subscriber().test_drain_events();

    client.send_request(b"ping").expect("send");
    let events = server.request_subscriber().test_drain_events();
    assert!(
        events
            .iter()
            .any(|e| matches!(e, cerulion_core::transport::events::PubSubEvent::SentSample)),
        "request must notify SentSample so wait_for_message wakes; got {events:?}"
    );

    let mut envelope = None;
    drain_requests(&mut server, |e, _| envelope = Some(e)).expect("take");
    server
        .send_response(&envelope.expect("envelope"), b"pong")
        .expect("respond");
    let events = client.reply_subscriber().test_drain_events();
    assert!(
        events
            .iter()
            .any(|e| matches!(e, cerulion_core::transport::events::PubSubEvent::SentSample)),
        "response must notify SentSample; got {events:?}"
    );
}

/// Reply-topic defensive guards: wrong schema hash AND foreign GUID
/// frames on a client's reply topic are skipped (warned), never
/// delivered. These branches guard the future carrier swap to native
/// iceoryx2 request-response.
#[test]
fn reply_topic_guards_skip_foreign_and_wrong_hash_frames() {
    use cerulion_core::transport::service::{reply_topic, ServiceEnvelope};
    use cerulion_core::wire::WireHeader;

    let transport = test_transport("guards");
    let service = &unique_service("/guarded");

    let guid = derive_client_guid("n", service, 0);
    let mut client = transport
        .create_service_client(service, guid, SLICE, req_hash(service), resp_hash(service))
        .expect("client");

    let mut raw_pub = transport
        .create_publisher_simple(&reply_topic(service, &guid), SLICE)
        .expect("raw pub");

    let build = |hash: u64, env_guid: [u8; 16]| {
        let envelope = ServiceEnvelope {
            client_guid: env_guid,
            sequence: 1,
        };
        let total = WireHeader::SIZE + ServiceEnvelope::SIZE + 4;
        let header = WireHeader {
            schema_hash: hash,
            total_size: total as u32,
            offset_table_offset: 0,
            offset_table_count: 0,
            sequence: 1,
            timestamp_ns: 0,
        };
        let mut frame = vec![0u8; total];
        header.write_to_buf(&mut frame[..WireHeader::SIZE]);
        envelope.write_to_buf(&mut frame[WireHeader::SIZE..WireHeader::SIZE + 24]);
        frame[WireHeader::SIZE + 24..].copy_from_slice(b"evil");
        frame
    };

    // Wrong hash, correct guid.
    raw_pub
        .publish_raw(&build(fnv1a_hash(b"Wrong_Response"), guid))
        .expect("publish");
    // Correct hash, FOREIGN guid.
    let foreign = derive_client_guid("other", service, 9);
    raw_pub
        .publish_raw(&build(resp_hash(service), foreign))
        .expect("publish");

    let mut delivered = 0usize;
    let count = drain_responses(&mut client, |_, _| delivered += 1).expect("take");
    assert_eq!(count, 0, "both frames must be skipped");
    assert_eq!(delivered, 0);
}

/// Drains are idempotent when empty, and a response to a dead client
/// (no subscriber) succeeds — with the zero-recipient warning — without
/// corrupting server state.
#[test]
fn double_drain_and_dead_client_response() {
    let transport = test_transport("edge");
    let service = &unique_service("/edge_svc");

    let mut server = transport
        .create_service_server(service, SLICE, req_hash(service), resp_hash(service))
        .expect("server");

    // Dead client: fabricate an envelope for a GUID with no subscriber.
    let ghost = ServiceEnvelope {
        client_guid: derive_client_guid("ghost", service, 0),
        sequence: 1,
    };
    server
        .send_response(&ghost, b"into-the-void")
        .expect("response to dead client is Ok (warned, not an error)");
    assert_eq!(server.known_client_count(), 1);

    // Empty double-drain.
    let mut calls = 0usize;
    assert_eq!(
        drain_requests(&mut server, |_, _| calls += 1).expect("d1"),
        0
    );
    assert_eq!(
        drain_requests(&mut server, |_, _| calls += 1).expect("d2"),
        0
    );
    assert_eq!(calls, 0);
}

/// A failed send must not burn a sequence number —
/// otherwise the server's gap detector reports a spurious "request
/// LOST" for traffic that never existed.
#[test]
fn failed_send_does_not_burn_sequence() {
    let transport = test_transport("seqburn");
    let service = &unique_service("/seq_burn");

    let mut server = transport
        .create_service_server(service, SLICE, req_hash(service), resp_hash(service))
        .expect("server");
    let guid = derive_client_guid("n", service, 0);
    let mut client = transport
        .create_service_client(service, guid, SLICE, req_hash(service), resp_hash(service))
        .expect("client");

    // Oversized → publish fails → sequence NOT consumed.
    let huge = vec![0u8; SLICE.get() as usize + 1];
    assert!(client.send_request(&huge).is_err());

    // The next successful request must still be sequence 1 — the server
    // sees a contiguous stream with no gap.
    let seq = client.send_request(b"ok").expect("send");
    assert_eq!(seq, 1, "failed send must not burn a sequence number");

    let mut seen = Vec::new();
    drain_requests(&mut server, |e, p| seen.push((e.sequence, p.to_vec()))).expect("take");
    assert_eq!(seen, vec![(1, b"ok".to_vec())]);
}

// =====================================================================
// The schema-hash-mismatch flood latch, at the PRODUCTION sites
// =====================================================================

/// Substring unique to the LOUD (`warn!`) arm of the mismatch report.
const HASH_LOUD: &str = "dropping a frame whose wire schema hash does not match";
/// Substring unique to the SUPPRESSED (`debug!`) arm.
const HASH_SUPPRESSED: &str = "schema-hash mismatch suppressed";
/// Substring unique to the RECOVERY (`info!`) arm.
const HASH_RECOVERY: &str = "schema hashes match again";
/// Substring unique to the LOUD arm of the SHORT-ENVELOPE (framing) report.
const SHORT_LOUD: &str = "dropping a frame too short to hold the service envelope";
/// Substring unique to the SUPPRESSED arm of the short-envelope report.
const SHORT_SUPPRESSED: &str = "short-envelope drop suppressed";
/// Substring unique to the RECOVERY arm of the short-envelope report.
const SHORT_RECOVERY: &str = "service envelopes parse again";

/// Build a well-formed service frame carrying `hash` and `env_guid` — the
/// same `[WireHeader][ServiceEnvelope][payload]` shape both directions use.
fn service_frame(hash: u64, env_guid: [u8; 16], sequence: i64, body: &[u8]) -> Vec<u8> {
    use cerulion_core::wire::WireHeader;
    let envelope = ServiceEnvelope {
        client_guid: env_guid,
        sequence,
    };
    let total = WireHeader::SIZE + ServiceEnvelope::SIZE + body.len();
    let header = WireHeader {
        schema_hash: hash,
        total_size: total as u32,
        offset_table_offset: 0,
        offset_table_count: 0,
        sequence: sequence as u32,
        timestamp_ns: 0,
    };
    let mut frame = vec![0u8; total];
    header.write_to_buf(&mut frame[..WireHeader::SIZE]);
    envelope.write_to_buf(&mut frame[WireHeader::SIZE..WireHeader::SIZE + ServiceEnvelope::SIZE]);
    frame[WireHeader::SIZE + ServiceEnvelope::SIZE..].copy_from_slice(body);
    frame
}

/// The flood latch, CLIENT half, at the production call sites.
///
/// A hash mismatch is a type SKEW: it fails EVERY response until somebody
/// redeploys, so a bare per-frame `warn!` would be ~100 lines/s on a
/// 100 Hz service — the disk-fill class. Drive 6 mismatched responses
/// through the take path and assert the whole contract against hand oracles:
/// exactly ONE loud head, 5 suppressed repeats, an UNCONDITIONAL counter of 6,
/// one recovery line carrying the SUPPRESSED count, and a re-armed loud head
/// afterwards.
///
/// The 6 frames arrive over 6 SEPARATE takes, so the pin is that the latch is
/// held on the CLIENT and spans calls — a per-call latch would emit six loud
/// heads, and one reset per call would report recovery six times.
#[test]
#[traced_test]
fn wrong_hash_responses_are_loud_once_counted_always_and_recover() {
    use cerulion_core::transport::service::reply_topic;

    let transport = test_transport("client");
    let service = &unique_service("/client");
    let guid = derive_client_guid("n", service, 0);
    let mut client = transport
        .create_service_client(service, guid, SLICE, req_hash(service), resp_hash(service))
        .expect("client");
    let mut raw_pub = transport
        .create_publisher_simple(&reply_topic(service, &guid), SLICE)
        .expect("raw pub");

    let wrong = fnv1a_hash(b"SkewedPeer_Response");
    assert_ne!(wrong, resp_hash(service), "the skew must be a real skew");

    // Phase A+B — six skewed frames, one PUBLISH per TAKE so the
    // subscriber queue depth is irrelevant, all through
    // `try_take_one_response` (the path the rmw service client's
    // `rmw_take_response` drives, and the only one there is).
    for i in 0..6i64 {
        raw_pub
            .publish_raw(&service_frame(wrong, guid, i + 1, b"skew"))
            .expect("publish");
        let mut delivered = 0usize;
        assert!(
            !client
                .try_take_one_response(|_, _| delivered += 1)
                .expect("take one"),
            "a wrong-hash response must never be delivered"
        );
        assert_eq!(delivered, 0);
    }

    assert_eq!(
        client.schema_mismatch_count(),
        6,
        "the counter is UNCONDITIONAL and shared by BOTH take paths — it must \
         count every dropped response, including the debug-suppressed ones"
    );

    // Phase C — recovery: a response carrying the RIGHT hash.
    raw_pub
        .publish_raw(&service_frame(resp_hash(service), guid, 7, b"ok"))
        .expect("publish");
    let mut good = Vec::new();
    client
        .try_take_one_response(|seq, p| good.push((seq, p.to_vec())))
        .expect("take one");
    assert_eq!(
        good,
        vec![(7i64, b"ok".to_vec())],
        "the healed frame must land"
    );
    assert_eq!(
        client.schema_mismatch_count(),
        6,
        "recovery must NEVER reset the running total"
    );

    // Phase D — re-armed: a fresh skew is loud again.
    raw_pub
        .publish_raw(&service_frame(wrong, guid, 8, b"skew"))
        .expect("publish");
    client.try_take_one_response(|_, _| {}).expect("take one");
    assert_eq!(client.schema_mismatch_count(), 7);

    logs_assert(|lines: &[&str]| {
        let warns = count_at_exclusively(lines, "WARN", &[HASH_LOUD])?;
        never_loud(lines, HASH_SUPPRESSED)?;
        let debugs = count_at_exclusively(lines, "DEBUG", &[HASH_SUPPRESSED])?;
        let recoveries = count_at_exclusively(lines, "INFO", &[HASH_RECOVERY])?;
        if warns != 2 {
            return Err(format!(
                "expected exactly 2 WARN loud heads (one per regime: the 6-frame skew, \
                 then the re-armed one), got {warns}"
            ));
        }
        let want_debugs = debug_lines_expected(5);
        if debugs != want_debugs {
            return Err(format!(
                "expected {want_debugs} DEBUG suppressed repeats (6 mismatches minus the loud head), \
                 got {debugs}"
            ));
        }
        if recoveries != 1 {
            return Err(format!(
                "expected exactly 1 INFO recovery line, got {recoveries}"
            ));
        }
        // No arm may leak upward: a suppressed repeat at WARN is the headline
        // regression (suppression ineffective while every count still holds).
        let leaked = count_at(lines, "WARN", HASH_SUPPRESSED);
        if leaked != 0 {
            return Err(format!(
                "the suppressed arm must be DEBUG, found {leaked} at WARN"
            ));
        }
        let rec = lines
            .iter()
            .find(|l| l.contains(HASH_RECOVERY))
            .ok_or("no recovery line")?;
        if !rec.contains("suppressed_count=5") {
            return Err(format!("recovery must report the 5 suppressed: {rec}"));
        }
        if !rec.contains("service response") {
            return Err(format!("recovery must name the site: {rec}"));
        }
        // Operators grep by `service=` — a generic `name=` breaks the query.
        let head = lines
            .iter()
            .find(|l| l.contains(HASH_LOUD))
            .ok_or("no loud head")?;
        if !head.contains(&format!("service={service}")) {
            return Err(format!("loud head must log under `service=`: {head}"));
        }
        Ok(())
    });
}

/// Flood latch: recovery + re-arm reached from INSIDE one batch of queued
/// responses.
///
/// The test above publishes one frame per take, so its regime opens, recovers
/// and re-arms across SEPARATE calls. This arm queues the frames FIRST and then
/// takes, so a single sequence of takes over one backlog walks
/// skew→skew→skew, then good (recovery), then skew again (re-armed head) —
/// the shape a real skewed peer produces when the consumer falls behind, and
/// a different oracle from its sibling: 3 mismatches (1 loud + 2 suppressed)
/// and a recovery that must report `suppressed_count=2`, not 5.
///
/// Deleting the match report from
/// `ServiceClient::try_take_one_response` fails this test twice over — no
/// recovery line, and the later mismatch is a `debug!` repeat of a regime that
/// was never closed instead of a fresh WARN.
#[test]
#[traced_test]
fn recovery_within_one_batch_of_responses_reports_and_rearms() {
    use cerulion_core::transport::service::reply_topic;

    let transport = test_transport("client_batch");
    let service = &unique_service("/client_batch");
    let guid = derive_client_guid("n", service, 0);
    let mut client = transport
        .create_service_client(service, guid, SLICE, req_hash(service), resp_hash(service))
        .expect("client");
    let mut raw_pub = transport
        .create_publisher_simple(&reply_topic(service, &guid), SLICE)
        .expect("raw pub");

    let wrong = fnv1a_hash(b"SkewedPeer_Response_Batch");
    assert_ne!(wrong, resp_hash(service));

    // Open a regime with a real suppressed repeat — three skewed frames
    // QUEUED, then taken, so all three are visited by one run of takes.
    for i in 0..3i64 {
        raw_pub
            .publish_raw(&service_frame(wrong, guid, i + 1, b"skew"))
            .expect("publish");
    }
    assert_eq!(
        drain_responses(&mut client, |_, _| {}).expect("take"),
        0,
        "a wrong-hash response must never be delivered"
    );
    assert_eq!(client.schema_mismatch_count(), 3);

    // Recovery AND re-arm inside ONE backlog: the healed frame closes the
    // regime, the skew queued behind it opens a fresh one.
    raw_pub
        .publish_raw(&service_frame(resp_hash(service), guid, 7, b"ok"))
        .expect("publish");
    raw_pub
        .publish_raw(&service_frame(wrong, guid, 8, b"skew"))
        .expect("publish");
    let mut good = Vec::new();
    assert_eq!(
        drain_responses(&mut client, |seq, p| good.push((seq, p.to_vec()))).expect("take"),
        1
    );
    assert_eq!(good, vec![(7i64, b"ok".to_vec())]);
    assert_eq!(
        client.schema_mismatch_count(),
        4,
        "the batch adds exactly the ONE trailing skew to the running total — a \
         recovery that reset the counter would read 1"
    );

    logs_assert(|lines: &[&str]| {
        let warns = count_at_exclusively(lines, "WARN", &[HASH_LOUD])?;
        never_loud(lines, HASH_SUPPRESSED)?;
        let debugs = count_at_exclusively(lines, "DEBUG", &[HASH_SUPPRESSED])?;
        let recoveries = count_at_exclusively(lines, "INFO", &[HASH_RECOVERY])?;
        if recoveries != 1 {
            return Err(format!(
                "a recovery reached mid-backlog must report exactly once, got {recoveries}"
            ));
        }
        if warns != 2 {
            return Err(format!(
                "the mid-backlog recovery must RE-ARM the loud arm for the skew \
                 queued behind it: expected 2 WARN heads, got {warns}"
            ));
        }
        let want_debugs = debug_lines_expected(2);
        if debugs != want_debugs {
            return Err(format!(
                "expected {want_debugs} DEBUG repeats, got {debugs}"
            ));
        }
        let rec = lines
            .iter()
            .find(|l| l.contains(HASH_RECOVERY))
            .ok_or("no recovery line")?;
        if !rec.contains("suppressed_count=2") {
            return Err(format!("recovery must report the 2 suppressed: {rec}"));
        }
        Ok(())
    });
}

/// The flood latch, SERVER half, at the production call site: the exact twin of
/// the client test above, over `try_take_one_request` (the path the rmw
/// service server's `rmw_take_request` drives, and the only request take path
/// there is).
#[test]
#[traced_test]
fn wrong_hash_requests_are_loud_once_counted_always_and_recover() {
    use cerulion_core::transport::service::request_topic;

    let transport = test_transport("server");
    let service = &unique_service("/server");
    let mut server = transport
        .create_service_server(service, SLICE, req_hash(service), resp_hash(service))
        .expect("server");
    let mut raw_pub = transport
        .create_publisher_simple(&request_topic(service), SLICE)
        .expect("raw pub");
    let client_guid = derive_client_guid("skewed", service, 0);

    let wrong = fnv1a_hash(b"SkewedPeer_Request");
    assert_ne!(wrong, req_hash(service), "the skew must be a real skew");

    // Phase A+B — six skewed frames, one PUBLISH per TAKE so the
    // subscriber queue depth is irrelevant.
    for i in 0..6i64 {
        raw_pub
            .publish_raw(&service_frame(wrong, client_guid, i + 1, b"skew"))
            .expect("publish");
        let mut delivered = 0usize;
        assert!(
            !server
                .try_take_one_request(|_, _| delivered += 1)
                .expect("take one"),
            "a wrong-hash request must never reach the server callback"
        );
        assert_eq!(delivered, 0);
    }

    assert_eq!(
        server.schema_mismatch_count(),
        6,
        "the counter is UNCONDITIONAL and the latch is held on the SERVER, so \
         it spans the six separate takes"
    );

    // Phase C — recovery. Sequence 1 is this client's first, so the loss
    // detector sees no gap and adds no unrelated warning.
    raw_pub
        .publish_raw(&service_frame(req_hash(service), client_guid, 1, b"ok"))
        .expect("publish");
    let mut good = Vec::new();
    server
        .try_take_one_request(|e, p| good.push((e.sequence, p.to_vec())))
        .expect("take one");
    assert_eq!(good, vec![(1i64, b"ok".to_vec())]);
    assert_eq!(server.schema_mismatch_count(), 6);

    // Phase D — re-armed.
    raw_pub
        .publish_raw(&service_frame(wrong, client_guid, 2, b"skew"))
        .expect("publish");
    server.try_take_one_request(|_, _| {}).expect("take one");
    assert_eq!(server.schema_mismatch_count(), 7);

    logs_assert(|lines: &[&str]| {
        let warns = count_at_exclusively(lines, "WARN", &[HASH_LOUD])?;
        never_loud(lines, HASH_SUPPRESSED)?;
        let debugs = count_at_exclusively(lines, "DEBUG", &[HASH_SUPPRESSED])?;
        let recoveries = count_at_exclusively(lines, "INFO", &[HASH_RECOVERY])?;
        if warns != 2 {
            return Err(format!("expected exactly 2 WARN loud heads, got {warns}"));
        }
        let want_debugs = debug_lines_expected(5);
        if debugs != want_debugs {
            return Err(format!(
                "expected {want_debugs} DEBUG suppressed repeats, got {debugs}"
            ));
        }
        if recoveries != 1 {
            return Err(format!(
                "expected exactly 1 INFO recovery line, got {recoveries}"
            ));
        }
        let leaked = count_at(lines, "WARN", HASH_SUPPRESSED);
        if leaked != 0 {
            return Err(format!(
                "the suppressed arm must be DEBUG, found {leaked} at WARN"
            ));
        }
        let rec = lines
            .iter()
            .find(|l| l.contains(HASH_RECOVERY))
            .ok_or("no recovery line")?;
        if !rec.contains("suppressed_count=5") {
            return Err(format!("recovery must report the 5 suppressed: {rec}"));
        }
        if !rec.contains("service request") {
            return Err(format!("recovery must name the site: {rec}"));
        }
        let head = lines
            .iter()
            .find(|l| l.contains(HASH_LOUD))
            .ok_or("no loud head")?;
        if !head.contains(&format!("service={service}")) {
            return Err(format!("loud head must log under `service=`: {head}"));
        }
        Ok(())
    });
}

/// The two request-sequence diagnostics, both emitted by the one and only
/// request take path.
const SEQ_GAP_WARN: &str = "request sequence gap";
const SEQ_RESTART_INFO: &str = "request sequence went backwards";

/// The request take path (the path `rmw_take_request` drives,
/// and the only one there is) must diagnose a client restart, not just a
/// forward gap.
///
/// Derived GUIDs are reused BY DESIGN (`derive_client_guid` is deterministic
/// in node/service/instance), so a restarted client comes back under the very
/// same identity while its own sequence counter starts over at 1 — and the
/// server's `last_seen` entry still holds the dead epoch's high-water mark.
///
/// Hand oracle: three requests taken one-at-a-time leave `last_seen[guid] ==
/// 3`; the client is then DROPPED and replaced by one with the SAME derived
/// GUID, whose first request carries sequence 1.
///
/// Without the backward-jump arm this produces NO diagnostic at all. It does
/// NOT produce a false overflow warning either — `sequence <= prev` cannot
/// satisfy `sequence > prev + 1`, and the `insert` re-bases the tracker to the
/// new epoch so the request after it is contiguous — which is exactly why the
/// WARN-ABSENCE arm is asserted beside the INFO-presence one: the defect is
/// pure SILENCE, and reporting the restart as queue-overflow LOSS would be
/// worse than the silence.
#[test]
#[traced_test]
fn a_restarted_client_is_diagnosed_on_the_one_shot_request_path() {
    let transport = test_transport("seq_restart");
    let service = &unique_service("/seq_restart");
    let mut server = transport
        .create_service_server(service, SLICE, req_hash(service), resp_hash(service))
        .expect("server");
    let guid = derive_client_guid("restarter", service, 0);

    // Epoch 1 — three requests, EACH taken through the one-shot path.
    let mut client = transport
        .create_service_client(service, guid, SLICE, req_hash(service), resp_hash(service))
        .expect("client");
    let mut epoch1 = Vec::new();
    for i in 0..3i64 {
        let seq = client.send_request(b"epoch-1").expect("send");
        assert_eq!(seq, i + 1, "client sequences are contiguous from 1");
        server
            .try_take_one_request(|e, _| epoch1.push(e.sequence))
            .expect("take one");
    }
    assert_eq!(epoch1, vec![1, 2, 3], "hand oracle: the first epoch");

    // THE RESTART: the process dies and comes back under the same identity.
    drop(client);
    let mut restarted = transport
        .create_service_client(service, guid, SLICE, req_hash(service), resp_hash(service))
        .expect("restarted client");
    let seq = restarted.send_request(b"epoch-2").expect("send");
    assert_eq!(seq, 1, "a restarted client's sequences begin again at 1");

    let mut epoch2 = Vec::new();
    server
        .try_take_one_request(|e, _| epoch2.push((e.client_guid, e.sequence)))
        .expect("take one");
    assert_eq!(
        epoch2,
        vec![(guid, 1i64)],
        "the restarted client's request is still DELIVERED — the backward \
         jump is a diagnostic, never a drop"
    );

    // The request AFTER the restart is contiguous against the re-based
    // tracker, so it must add no further diagnostic of either kind.
    let seq = restarted.send_request(b"epoch-2").expect("send");
    assert_eq!(seq, 2);
    server.try_take_one_request(|_, _| {}).expect("take one");

    logs_assert(|lines: &[&str]| {
        let restarts = count_at_exclusively(lines, "INFO", &[SEQ_RESTART_INFO])?;
        if restarts != 1 {
            return Err(format!(
                "expected exactly 1 INFO restart diagnostic, got {restarts} \
                 (a one-shot path with no backward-jump arm reads 0)"
            ));
        }
        // An ABSENCE guard, so the NON-exclusive count: the exclusive form would
        // refuse — not read 0 — the day the gap warn grows a suppressed twin at
        // another level, i.e. fail for the opposite reason to the one it asserts.
        let gaps = count_at(lines, "WARN", SEQ_GAP_WARN);
        if gaps != 0 {
            return Err(format!(
                "a backward jump must NEVER be reported as queue-overflow \
                 LOSS, got {gaps} gap warning(s)"
            ));
        }
        let line = lines
            .iter()
            .find(|l| l.contains(SEQ_RESTART_INFO))
            .ok_or("no restart line")?;
        let required = [
            format!("service={service}"),
            "previous_sequence=3".to_string(),
            "actual_sequence=1".to_string(),
        ];
        for field in &required {
            if !line.contains(field.as_str()) {
                return Err(format!("restart line must carry `{field}`: {line}"));
            }
        }
        Ok(())
    });
}

/// The FORWARD-gap half of the request-sequence bookkeeping, on the one and
/// only request take path.
///
/// `last_seen_seq` is the whole of the service layer's loss detection: the
/// bounded subscriber queue is safe-overflow, so a displaced request leaves no
/// trace except the hole it makes in the client's contiguous-from-1 sequence
/// (see the module docs' "Loss bounds"). Nothing counts it — the WARN IS the
/// detection — so if the take path stops maintaining the map, request loss
/// goes silent.
///
/// Hand oracle over TWO clients on one server, so the arm is not satisfied by
/// a reporter that simply warns on everything: a CONTROL client sends a
/// contiguous 1,2,3 and must produce NO gap warning at all, while the gapped
/// client sends 1,2,4 and must produce EXACTLY ONE naming
/// `expected_sequence=3 actual_sequence=4`. Every frame is still DELIVERED —
/// a gap is a diagnostic, never a drop.
///
/// MUTATIONS, both run: replacing the `insert` with a non-updating read (so
/// `prev` is stuck at 0) fails this test with 4 gap warnings instead of 1 —
/// the control client's own contiguous frames each read as a gap; deleting the
/// `> prev + 1` arm outright fails it with 0.
#[test]
#[traced_test]
fn a_forward_sequence_gap_is_diagnosed_on_the_one_shot_request_path() {
    use cerulion_core::transport::service::request_topic;

    let transport = test_transport("seq_gap");
    let service = &unique_service("/seq_gap");
    let mut server = transport
        .create_service_server(service, SLICE, req_hash(service), resp_hash(service))
        .expect("server");
    // A raw publisher on the request topic so the gap can be MADE rather than
    // waited for — a real displacement needs the queue to overflow, which is
    // timing, and this file is deterministic.
    let mut raw_pub = transport
        .create_publisher_simple(&request_topic(service), SLICE)
        .expect("raw pub");

    let control = derive_client_guid("contiguous", service, 0);
    let gapped = derive_client_guid("lossy", service, 0);
    assert_ne!(control, gapped);

    // CONTROL is contiguous from 1 and must produce no gap warning; GAPPED
    // sends 1, 2, then 4 — request 3 was displaced from the queue.
    let script: [([u8; 16], i64); 6] = [
        (control, 1),
        (control, 2),
        (control, 3),
        (gapped, 1),
        (gapped, 2),
        (gapped, 4),
    ];
    let mut seen: Vec<([u8; 16], i64)> = Vec::new();
    for (guid, seq) in script {
        raw_pub
            .publish_raw(&service_frame(req_hash(service), guid, seq, b"body"))
            .expect("publish");
        assert!(
            server
                .try_take_one_request(|e, _| seen.push((e.client_guid, e.sequence)))
                .expect("take one"),
            "every frame is DELIVERED — a sequence gap is a diagnostic, never a drop"
        );
    }
    assert_eq!(
        seen,
        script.to_vec(),
        "hand oracle: both clients' frames, in publish order"
    );

    logs_assert(|lines: &[&str]| {
        // An exact count over BOTH clients: the control contributes 0, so a
        // reporter that warned on every request would read 5, and one that
        // lost the per-client high-water mark would read 4.
        let gaps = count_at_exclusively(lines, "WARN", &[SEQ_GAP_WARN])?;
        if gaps != 1 {
            return Err(format!(
                "expected exactly 1 WARN gap diagnostic (the 3 that never \
                 arrived), got {gaps}"
            ));
        }
        // Absence guard, non-exclusive for the same reason as the restart arm.
        let restarts = count_at(lines, "INFO", SEQ_RESTART_INFO);
        if restarts != 0 {
            return Err(format!(
                "a forward gap must NEVER be reported as a client restart, got \
                 {restarts}"
            ));
        }
        let line = lines
            .iter()
            .find(|l| l.contains(SEQ_GAP_WARN))
            .ok_or("no gap line")?;
        let required = [
            format!("service={service}"),
            "expected_sequence=3".to_string(),
            "actual_sequence=4".to_string(),
        ];
        for field in &required {
            if !line.contains(field.as_str()) {
                return Err(format!("gap line must carry `{field}`: {line}"));
            }
        }
        Ok(())
    });
}

/// Flood latch: recovery + re-arm reached from INSIDE one batch of queued
/// requests: the server twin of
/// `recovery_within_one_batch_of_responses_reports_and_rearms`.
///
/// Deleting the match report from
/// `ServiceServer::try_take_one_request` fails this test (no recovery line;
/// the later mismatch is a suppressed repeat instead of a fresh WARN head).
#[test]
#[traced_test]
fn recovery_within_one_batch_of_requests_reports_and_rearms() {
    use cerulion_core::transport::service::request_topic;

    let transport = test_transport("server_batch");
    let service = &unique_service("/server_batch");
    let mut server = transport
        .create_service_server(service, SLICE, req_hash(service), resp_hash(service))
        .expect("server");
    let mut raw_pub = transport
        .create_publisher_simple(&request_topic(service), SLICE)
        .expect("raw pub");
    let client_guid = derive_client_guid("skewed_batch", service, 0);

    let wrong = fnv1a_hash(b"SkewedPeer_Request_Batch");
    assert_ne!(wrong, req_hash(service));

    // Three skewed frames QUEUED, then taken, so one run of takes visits all
    // three and the regime opens with a real suppressed repeat.
    for i in 0..3i64 {
        raw_pub
            .publish_raw(&service_frame(wrong, client_guid, i + 1, b"skew"))
            .expect("publish");
    }
    assert_eq!(
        drain_requests(&mut server, |_, _| {}).expect("take"),
        0,
        "a wrong-hash request must never reach the callback"
    );
    assert_eq!(server.schema_mismatch_count(), 3);

    // Recovery AND re-arm inside ONE backlog: the healed request closes the
    // regime, the skew queued behind it opens a fresh one. Sequence 1 is this
    // client's first, so the loss detector adds no unrelated warning.
    raw_pub
        .publish_raw(&service_frame(req_hash(service), client_guid, 1, b"ok"))
        .expect("publish");
    raw_pub
        .publish_raw(&service_frame(wrong, client_guid, 2, b"skew"))
        .expect("publish");
    let mut good = Vec::new();
    assert_eq!(
        drain_requests(&mut server, |e, p| good.push((e.sequence, p.to_vec()))).expect("take"),
        1
    );
    assert_eq!(good, vec![(1i64, b"ok".to_vec())]);
    assert_eq!(
        server.schema_mismatch_count(),
        4,
        "the batch adds exactly the ONE trailing skew to the running total — a \
         recovery that reset the counter would read 1"
    );

    logs_assert(|lines: &[&str]| {
        let warns = count_at_exclusively(lines, "WARN", &[HASH_LOUD])?;
        never_loud(lines, HASH_SUPPRESSED)?;
        let debugs = count_at_exclusively(lines, "DEBUG", &[HASH_SUPPRESSED])?;
        let recoveries = count_at_exclusively(lines, "INFO", &[HASH_RECOVERY])?;
        if recoveries != 1 {
            return Err(format!(
                "a recovery reached mid-backlog must report exactly once, got {recoveries}"
            ));
        }
        if warns != 2 {
            return Err(format!(
                "the mid-backlog recovery must RE-ARM the loud arm for the skew \
                 queued behind it: expected 2 WARN heads, got {warns}"
            ));
        }
        let want_debugs = debug_lines_expected(2);
        if debugs != want_debugs {
            return Err(format!(
                "expected {want_debugs} DEBUG repeats, got {debugs}"
            ));
        }
        let rec = lines
            .iter()
            .find(|l| l.contains(HASH_RECOVERY))
            .ok_or("no recovery line")?;
        if !rec.contains("suppressed_count=2") {
            return Err(format!("recovery must report the 2 suppressed: {rec}"));
        }
        Ok(())
    });
}

/// Flood latch: the FRAMING-skew twin at the production site, and its
/// SEPARATENESS from the hash regime.
///
/// A frame whose hash MATCHES but that is too short to hold the envelope is
/// the same per-frame flood class (a skewed peer emits it on every frame), so
/// it rides its own latch. "Its own" is the load-bearing word: with the
/// framing regime open, a hash mismatch must STILL produce a loud head, and
/// healing the framing condition must not announce a hash recovery.
#[test]
#[traced_test]
fn short_envelope_frames_latch_separately_from_the_hash_regime() {
    use cerulion_core::transport::service::request_topic;
    use cerulion_core::wire::WireHeader;

    let transport = test_transport("short");
    let service = &unique_service("/short");
    let mut server = transport
        .create_service_server(service, SLICE, req_hash(service), resp_hash(service))
        .expect("server");
    let mut raw_pub = transport
        .create_publisher_simple(&request_topic(service), SLICE)
        .expect("raw pub");
    let client_guid = derive_client_guid("short_peer", service, 0);

    /// A frame with the RIGHT hash but a body below `ServiceEnvelope::SIZE`.
    fn short_frame(hash: u64, sequence: u32) -> Vec<u8> {
        let body = [0u8; 10];
        let total = WireHeader::SIZE + body.len();
        let header = WireHeader {
            schema_hash: hash,
            total_size: total as u32,
            offset_table_offset: 0,
            offset_table_count: 0,
            sequence,
            timestamp_ns: 0,
        };
        let mut frame = vec![0u8; total];
        header.write_to_buf(&mut frame[..WireHeader::SIZE]);
        frame[WireHeader::SIZE..].copy_from_slice(&body);
        frame
    }

    // Open a FRAMING regime: 1 loud head + 2 suppressed repeats.
    for seq in 1..=3u32 {
        raw_pub
            .publish_raw(&short_frame(req_hash(service), seq))
            .expect("publish");
        assert_eq!(drain_requests(&mut server, |_, _| {}).expect("take"), 0);
    }
    assert_eq!(server.short_frame_count(), 3);
    assert_eq!(
        server.schema_mismatch_count(),
        0,
        "a short frame passes the hash gate — it must not touch the hash counter"
    );

    // With the framing regime OPEN, a hash mismatch must still be LOUD.
    let wrong = fnv1a_hash(b"SkewedPeer_Request_Short");
    assert_ne!(wrong, req_hash(service));
    raw_pub
        .publish_raw(&service_frame(wrong, client_guid, 1, b"skew"))
        .expect("publish");
    assert_eq!(drain_requests(&mut server, |_, _| {}).expect("take"), 0);
    assert_eq!(server.schema_mismatch_count(), 1);
    assert_eq!(
        server.short_frame_count(),
        3,
        "a wrong-hash frame returns before the framing gate"
    );

    // The framing regime is still open — the next short frame is a repeat,
    // not a fresh head.
    raw_pub
        .publish_raw(&short_frame(req_hash(service), 4))
        .expect("publish");
    assert_eq!(drain_requests(&mut server, |_, _| {}).expect("take"), 0);
    assert_eq!(server.short_frame_count(), 4);

    // A well-formed request heals the framing condition. The hash regime had
    // ONE failure (no suppressed repeats), so it re-arms SILENTLY.
    raw_pub
        .publish_raw(&service_frame(req_hash(service), client_guid, 2, b"ok"))
        .expect("publish");
    let mut good = Vec::new();
    assert_eq!(
        drain_requests(&mut server, |e, p| good.push((e.sequence, p.to_vec()))).expect("take"),
        1
    );
    assert_eq!(good, vec![(2i64, b"ok".to_vec())]);

    logs_assert(|lines: &[&str]| {
        let short_heads = count_at_exclusively(lines, "WARN", &[SHORT_LOUD])?;
        never_loud(lines, SHORT_SUPPRESSED)?;
        let short_debugs = count_at_exclusively(lines, "DEBUG", &[SHORT_SUPPRESSED])?;
        let short_recoveries = count_at_exclusively(lines, "INFO", &[SHORT_RECOVERY])?;
        let hash_heads = count_at_exclusively(lines, "WARN", &[HASH_LOUD])?;
        let hash_recoveries = count_at_exclusively(lines, "INFO", &[HASH_RECOVERY])?;
        if short_heads != 1 {
            return Err(format!(
                "expected exactly 1 WARN framing head over 4 short frames, got \
                 {short_heads}"
            ));
        }
        let want_short_debugs = debug_lines_expected(3);
        if short_debugs != want_short_debugs {
            return Err(format!(
                "expected {want_short_debugs} DEBUG framing repeats, got {short_debugs}"
            ));
        }
        if hash_heads != 1 {
            return Err(format!(
                "an OPEN framing regime must not swallow the hash regime's loud head: \
                 expected 1 WARN, got {hash_heads}"
            ));
        }
        if short_recoveries != 1 {
            return Err(format!(
                "expected exactly 1 INFO framing recovery, got {short_recoveries}"
            ));
        }
        if hash_recoveries != 0 {
            return Err(format!(
                "a lone hash mismatch must re-arm SILENTLY, got {hash_recoveries} \
                 recovery lines"
            ));
        }
        let rec = lines
            .iter()
            .find(|l| l.contains(SHORT_RECOVERY))
            .ok_or("no framing recovery line")?;
        if !rec.contains("suppressed_count=3") {
            return Err(format!("framing recovery must report 3 suppressed: {rec}"));
        }
        let head = lines
            .iter()
            .find(|l| l.contains(SHORT_LOUD))
            .ok_or("no framing head")?;
        if !head.contains("size_bytes=10") || !head.contains(&format!("service={service}")) {
            return Err(format!(
                "framing head must carry the size and the `service=` field: {head}"
            ));
        }
        Ok(())
    });
}

/// ANTI-TAUTOLOGY control: a HEALTHY service pair logs none of the flood-latch
/// lines and keeps both counters at exactly zero. Without this, every "exactly
/// N" assertion above would still pass if the reporter also fired on matching
/// frames.
#[test]
#[traced_test]
fn a_healthy_service_pair_is_silent_and_counts_zero() {
    let transport = test_transport("quiet");
    let service = &unique_service("/quiet");
    let mut server = transport
        .create_service_server(service, SLICE, req_hash(service), resp_hash(service))
        .expect("server");
    let guid = derive_client_guid("n", service, 0);
    let mut client = transport
        .create_service_client(service, guid, SLICE, req_hash(service), resp_hash(service))
        .expect("client");

    for _ in 0..5 {
        client.send_request(b"ping").expect("send");
        let mut envelopes = Vec::new();
        drain_requests(&mut server, |e, _| envelopes.push(e)).expect("take");
        assert_eq!(envelopes.len(), 1);
        server
            .send_response(&envelopes[0], b"pong")
            .expect("respond");
        let mut replies = 0usize;
        drain_responses(&mut client, |_, _| replies += 1).expect("take response");
        assert_eq!(replies, 1);
    }

    assert_eq!(client.schema_mismatch_count(), 0);
    assert_eq!(server.schema_mismatch_count(), 0);
    assert_eq!(client.short_frame_count(), 0);
    assert_eq!(server.short_frame_count(), 0);
    logs_assert(|lines: &[&str]| {
        let any = lines
            .iter()
            .filter(|l| {
                l.contains(HASH_LOUD)
                    || l.contains(HASH_SUPPRESSED)
                    || l.contains(HASH_RECOVERY)
                    || l.contains(SHORT_LOUD)
                    || l.contains(SHORT_SUPPRESSED)
                    || l.contains(SHORT_RECOVERY)
            })
            .count();
        if any == 0 {
            Ok(())
        } else {
            Err(format!(
                "a healthy service pair must emit no flood-latch lines, got {any}"
            ))
        }
    });
}
