// SPDX-License-Identifier: AGPL-3.0-only
//! Malformed membership preserves admitted readers; identity fences are explicit.
use super::*;
use serde_json::{json, Value};
use std::io::{Read, Write};
use std::os::unix::net::UnixStream;

struct Producers {
    stop: Arc<AtomicBool>,
    handles: Vec<std::thread::JoinHandle<()>>,
}
impl Drop for Producers {
    fn drop(&mut self) {
        self.stop.store(true, Ordering::Release);
        for handle in self.handles.drain(..) {
            handle.join().unwrap();
        }
    }
}

#[test]
fn shared_endpoint_selected_reads_work_with_300_and_4096_registered_robots() {
    for count in [300, 4096] {
        let seed = unique_secret();
        let first = RobotFixture::start("large_first", Arc::new(claimed_paired(public_of(&seed))));
        let second =
            RobotFixture::start("large_second", Arc::new(claimed_paired(public_of(&seed))));
        let mut membership: HashMap<_, _> = (0..count - 2)
            .map(|index| {
                (
                    format!("unused-{index}"),
                    cerulion_netd::wan::WanRobot {
                        eid: cerulion_link::EndpointId::from_bytes(&public_of(&unique_secret()))
                            .unwrap(),
                        direct_addrs: vec![],
                    },
                )
            })
            .collect();
        membership.insert(
            "first".into(),
            cerulion_netd::wan::WanRobot {
                eid: first.eid,
                direct_addrs: first.addrs.clone(),
            },
        );
        membership.insert(
            "second".into(),
            cerulion_netd::wan::WanRobot {
                eid: second.eid,
                direct_addrs: second.addrs.clone(),
            },
        );
        let manager = test_manager("large_membership");
        let rt = tokio::runtime::Runtime::new().unwrap();
        let endpoint = rt.block_on(disabled_endpoint(seed));
        let root = tempfile::tempdir().unwrap();
        let socket = root.path().join("outbound.sock");
        let server = cerulion_netd::shared_endpoint::OutboundServer::start(
            socket.clone(),
            endpoint.clone(),
            rt.handle().clone(),
            seed,
            manager.clone(),
        )
        .unwrap();
        let registry = Arc::new(cerulion_netd::wan::WanRegistry::new(
            membership.clone(),
            seed,
            RelayConfig::Disabled,
        ));
        let plane = IrohMirrorPlane::new_shared(manager.clone(), registry, socket).unwrap();
        let first_key = TopicKey::new("first", format!("/large/first/{}", unique_id()));
        let second_key = TopicKey::observed("second", format!("/large/second/{}", unique_id()));
        let stop = Arc::new(AtomicBool::new(false));
        let first_producer =
            spawn_producer(first.manager.clone(), first_key.topic.clone(), stop.clone());
        let second_producer = spawn_producer(
            second.manager.clone(),
            second_key.topic.clone(),
            stop.clone(),
        );
        let producers = Producers {
            stop,
            handles: vec![first_producer, second_producer],
        };
        let deadline = Instant::now() + DIAL_TIMEOUT;
        while !first
            .manager
            .list_topics()
            .unwrap()
            .contains(&first_key.topic)
            || !second
                .manager
                .list_topics()
                .unwrap()
                .contains(&second_key.topic)
        {
            assert!(Instant::now() < deadline, "both producers must publish");
            std::thread::sleep(Duration::from_millis(5));
        }
        let catalog = plane
            .query_catalog("first", DIAL_TIMEOUT)
            .expect("unrelated membership must not reject one selected catalog");
        assert!(catalog
            .entries
            .iter()
            .any(|entry| entry.topic == first_key.topic
                && entry.schema_hash == Some(ORACLE_SCHEMA_HASH)));
        plane.ensure_mirror(&first_key, ORACLE_SCHEMA_HASH).unwrap();
        plane
            .ensure_mirror(&second_key, ORACLE_SCHEMA_HASH)
            .unwrap();
        for key in [&first_key, &second_key] {
            assert!(plane.has_reader(key).unwrap());
            assert_oracle_and_gap_free(&collect_frames(&manager, &key.local_topic(), 3));
        }
        plane.query_catalog("second", DIAL_TIMEOUT).unwrap();
        assert!(plane.has_reader(&first_key).unwrap());
        assert_oracle_and_gap_free(&collect_frames(&manager, &first_key.local_topic(), 3));
        let mut removed = membership.clone();
        removed.remove("first");
        assert_eq!(
            plane.update_membership(removed).unwrap(),
            vec![first_key.clone()]
        );
        assert!(!plane.has_reader(&first_key).unwrap());
        assert!(plane.has_reader(&second_key).unwrap());
        assert_oracle_and_gap_free(&collect_frames(&manager, &second_key.local_topic(), 3));
        plane.update_membership(membership).unwrap();
        plane.ensure_mirror(&first_key, ORACLE_SCHEMA_HASH).unwrap();
        assert_oracle_and_gap_free(&collect_frames(&manager, &first_key.local_topic(), 3));
        assert_eq!(plane.release_mirror(&first_key), MirrorRelease::Retired);
        assert_eq!(plane.release_mirror(&second_key), MirrorRelease::Retired);
        drop(producers);
        drop(plane);
        drop(server);
        rt.block_on(endpoint.close());
    }
}

fn request(stream: &mut UnixStream, configuration: &Value, operation: Value) -> Value {
    let bytes =
        serde_json::to_vec(&json!({"configuration":configuration,"operation":operation})).unwrap();
    stream
        .write_all(&(bytes.len() as u32).to_le_bytes())
        .unwrap();
    stream.write_all(&bytes).unwrap();
    let mut header = [0; 4];
    stream.read_exact(&mut header).unwrap();
    let size = u32::from_le_bytes(header) as usize;
    assert!(size > 0 && size <= 256 * 1024);
    let mut bytes = vec![0; size];
    stream.read_exact(&mut bytes).unwrap();
    serde_json::from_slice(&bytes).unwrap()
}

#[test]
fn shared_endpoint_seventeen_address_refusal_preserves_both_readers_and_identity_fences() {
    let seed = unique_secret();
    let remote = RobotFixture::start(
        "membership_source",
        Arc::new(claimed_paired(public_of(&seed))),
    );
    let manager = test_manager("membership_owner");
    let rt = tokio::runtime::Runtime::new().unwrap();
    let endpoint = rt.block_on(disabled_endpoint(seed));
    let directory = tempfile::tempdir().unwrap();
    let socket = directory.path().join("outbound.sock");
    let server = cerulion_netd::shared_endpoint::OutboundServer::start(
        socket.clone(),
        endpoint.clone(),
        rt.handle().clone(),
        seed,
        manager.clone(),
    )
    .unwrap();
    let mut stream = UnixStream::connect(socket).unwrap();
    stream
        .set_read_timeout(Some(Duration::from_secs(5)))
        .unwrap();
    let configuration = json!({
        "key":public_of(&seed), "account":null,"epoch_dir":null,"owner":null,
        "namespace":serde_json::to_string(&manager.iox_config()).unwrap(),
        "peers":[{"name":"producer","key":remote.eid.as_bytes(),"addresses":remote.addrs}]
    });
    let topic = format!("/shared/membership/{}", unique_id());
    let stop = Arc::new(AtomicBool::new(false));
    let producer = spawn_producer(remote.manager.clone(), topic.clone(), stop.clone());
    for observer in [false, true] {
        assert_eq!(
            request(
                &mut stream,
                &configuration,
                json!({"op":"ensure","robot":"producer","topic":topic,"schema_hash":ORACLE_SCHEMA_HASH,"observer":observer})
            ),
            json!({"Ok":"Done"})
        );
    }
    let mut malformed = configuration.clone();
    malformed["peers"][0]["addresses"] = json!((1..=17)
        .map(|port| format!("127.0.0.1:{port}"))
        .collect::<Vec<_>>());
    let refusal = request(
        &mut stream,
        &malformed,
        json!({"op":"ensure","robot":"producer","topic":"/shared/missing","schema_hash":ORACLE_SCHEMA_HASH,"observer":false}),
    );
    assert_eq!(
        refusal,
        json!({"Err":{"message":"outbound robot membership is invalid","fenced":false}})
    );
    for key in [
        TopicKey::new("producer", &topic),
        TopicKey::observed("producer", &topic),
    ] {
        assert_eq!(
            request(
                &mut stream,
                &configuration,
                json!({"op":"has_reader","robot":"producer","topic":topic,"observer":key.observer})
            ),
            json!({"Ok":{"Present":true}})
        );
        assert_oracle_and_gap_free(&collect_frames(&manager, &key.local_topic(), 3));
    }
    assert_eq!(server.active_clients().unwrap(), 1);
    let mut fenced = configuration.clone();
    fenced["account"] = json!([42; 32].to_vec());
    assert_eq!(
        request(&mut stream, &fenced, json!({"op":"sync"})),
        json!({"Err":{"message":"outbound account changed; release the previous connection","fenced":true}})
    );
    let mut byte = [0];
    assert_eq!(stream.read(&mut byte).unwrap(), 0);
    stop.store(true, Ordering::Relaxed);
    producer.join().unwrap();
    drop(stream);
    drop(server);
    rt.block_on(endpoint.close());
}
