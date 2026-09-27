// SPDX-License-Identifier: AGPL-3.0-only
//! A real control-stream reset must retire readers before the next connection.

use super::*;
use cerulion_link::Connection;
use tokio::task::{JoinHandle, JoinSet};

async fn send_oracles(mut stream: cerulion_link::SendStream, mut sequence: u32) {
    loop {
        if write_frame(&mut stream, &oracle_frame(sequence))
            .await
            .is_err()
        {
            break;
        }
        sequence += 1;
        tokio::time::sleep(Duration::from_millis(5)).await;
    }
}

async fn serve_connection(
    connection: Connection,
    first: bool,
    topics: [String; 2],
    old_closed: Arc<std::sync::Mutex<Option<std::sync::mpsc::SyncSender<()>>>>,
) {
    let (mut send, mut recv) = accept_frame_stream(&connection).await.unwrap();
    let mut readers: HashMap<String, JoinHandle<()>> = HashMap::new();
    while let Ok(bytes) = read_frame(&mut recv, DEFAULT_MAX_FRAME_LEN).await {
        let request: WireRequest = serde_json::from_slice(&bytes).unwrap();
        let reply = match request {
            WireRequest::Catalog => WireResponse::Catalog(CatalogReply {
                version: CATALOG_WIRE_VERSION,
                robot: "robot".into(),
                entries: topics
                    .iter()
                    .map(|topic| CatalogEntry {
                        topic: topic.clone(),
                        schema_hash: Some(ORACLE_SCHEMA_HASH),
                        schema_name: None,
                        provenance: CatalogProvenance::Runtime,
                        producer_count: None,
                        liveness: None,
                    })
                    .collect(),
                error: None,
            }),
            WireRequest::Demand { topic } if first && topic == topics[1] => {
                // Reset ONLY the control reply stream. The first topic continues
                // producing until the desk explicitly closes this connection.
                send.reset(7u32.into()).unwrap();
                tokio::time::timeout(DIAL_TIMEOUT, connection.closed())
                    .await
                    .unwrap();
                for (_, reader) in readers.drain() {
                    reader.abort();
                    let _ = reader.await;
                }
                old_closed.lock().unwrap().take().unwrap().send(()).unwrap();
                return;
            }
            WireRequest::Demand { topic } => {
                assert!(topics.contains(&topic));
                let mut stream = open_uni_frame_stream(&connection).await.unwrap();
                write_frame(
                    &mut stream,
                    &serde_json::to_vec(&StreamPreamble {
                        topic: topic.clone(),
                    })
                    .unwrap(),
                )
                .await
                .unwrap();
                let start = if first { 0 } else { 100_000 };
                let reader = tokio::spawn(send_oracles(stream, start));
                assert!(readers.insert(topic.clone(), reader).is_none());
                WireResponse::DemandAccepted { topic }
            }
            WireRequest::Undemand { topic } => {
                let removed = readers.remove(&topic);
                let was_demanded = removed.is_some();
                if let Some(reader) = removed {
                    reader.abort();
                    let _ = reader.await;
                }
                WireResponse::Undemanded {
                    topic,
                    was_demanded,
                }
            }
            other => panic!("unexpected transport-fixture request: {other:?}"),
        };
        if write_frame(&mut send, &serde_json::to_vec(&reply).unwrap())
            .await
            .is_err()
        {
            break;
        }
    }
    for (_, reader) in readers {
        reader.abort();
        let _ = reader.await;
    }
}

#[test]
fn poisoned_control_retires_other_topic_readers_before_redial() {
    let runtime = tokio::runtime::Runtime::new().unwrap();
    let endpoint = runtime.block_on(disabled_endpoint(unique_secret()));
    let topics = [
        format!("/poison/a/{}", unique_id()),
        format!("/poison/b/{}", unique_id()),
    ];
    let registry = Arc::new(WanRegistry::new(
        HashMap::from([(
            "robot".into(),
            WanRobot {
                eid: endpoint.id(),
                direct_addrs: loopback_sockets(&endpoint.bound_sockets()),
            },
        )]),
        unique_secret(),
        RelayConfig::Disabled,
    ));
    let (closed_tx, closed_rx) = std::sync::mpsc::sync_channel(1);
    let closed_tx = Arc::new(std::sync::Mutex::new(Some(closed_tx)));
    let accepted_count = Arc::new(AtomicU64::new(0));
    let accepted = accepted_count.clone();
    let server_endpoint = endpoint.clone();
    let server_topics = topics.clone();
    let server = runtime.spawn(async move {
        let mut connections = JoinSet::new();
        for first in [true, false, false] {
            let incoming = tokio::time::timeout(DIAL_TIMEOUT, accept_one(&server_endpoint))
                .await
                .unwrap()
                .unwrap()
                .unwrap();
            accepted.fetch_add(1, Ordering::SeqCst);
            connections.spawn(serve_connection(
                incoming.connection,
                first,
                server_topics.clone(),
                closed_tx.clone(),
            ));
        }
        while let Some(result) = connections.join_next().await {
            result.unwrap();
        }
    });
    let desk = test_manager("poison_retirement");
    let plane = IrohMirrorPlane::new(desk.clone(), registry.clone())
        .unwrap()
        .with_dial_timeout(DIAL_TIMEOUT);
    let a = TopicKey::new("robot", &topics[0]);
    let b = TopicKey::new("robot", &topics[1]);
    plane.ensure_mirror(&a, ORACLE_SCHEMA_HASH).unwrap();
    assert_oracle_and_gap_free(&collect_frames(&desk, &a.topic, 3));
    assert_eq!(gather_origin(&desk, &a.topic).as_deref(), Some("robot"));
    assert!(plane.ensure_mirror(&b, ORACLE_SCHEMA_HASH).is_err());
    assert_eq!(plane.connection_count(), 0);
    assert_eq!(plane.reader_count(), 0);

    // Both physical publisher slots must already be free on return. A detached
    // old reader would still occupy one even though reader_count says zero.
    let max_slice =
        MaxSliceLen::const_new(cerulion_core::graph::config::DEFAULT_MAX_SLICE_LEN as u32);
    let slot_one = desk
        .create_ingress_injector(&a.topic, ORACLE_SCHEMA_HASH, max_slice)
        .unwrap();
    let slot_two = desk
        .create_ingress_injector(&a.topic, ORACLE_SCHEMA_HASH, max_slice)
        .unwrap();
    drop((slot_one, slot_two));
    closed_rx.recv_timeout(DIAL_TIMEOUT).unwrap();
    assert!(gather_absent(&desk, &a.topic));
    assert_eq!(plane.last_push_outcome("robot"), None);

    plane.ensure_mirror(&a, ORACLE_SCHEMA_HASH).unwrap();
    plane.ensure_mirror(&b, ORACLE_SCHEMA_HASH).unwrap();
    for key in [&a, &b] {
        let frames = collect_frames(&desk, &key.topic, 3);
        assert!(
            frames[0].0 >= 100_000,
            "only the replacement connection owns these frames"
        );
        assert_oracle_and_gap_free(&frames);
        assert_eq!(gather_origin(&desk, &key.topic).as_deref(), Some("robot"));
    }
    assert_eq!(accepted_count.load(Ordering::SeqCst), 2);
    assert_eq!(plane.reader_count(), 2);
    let peer_before = registry.get("robot").unwrap();
    plane.retire_robot("robot").unwrap();
    plane.retire_robot("robot").unwrap();
    assert_eq!(registry.get("robot").unwrap(), peer_before);
    assert_eq!(plane.reader_count(), 0);
    assert_eq!(plane.connection_count(), 0);
    assert_eq!(plane.last_push_outcome("robot"), None);
    assert!(gather_absent(&desk, &a.topic));
    assert!(gather_absent(&desk, &b.topic));
    for key in [&a, &b] {
        plane.ensure_mirror(key, ORACLE_SCHEMA_HASH).unwrap();
        assert_oracle_and_gap_free(&collect_frames(&desk, &key.topic, 3));
    }
    assert_eq!(accepted_count.load(Ordering::SeqCst), 3);
    assert!(plane.retire_robot(" robot ").is_err());
    plane.invalidate_account().unwrap();
    runtime.block_on(async {
        tokio::time::timeout(DIAL_TIMEOUT, server)
            .await
            .unwrap()
            .unwrap();
        endpoint.close().await;
    });
}
