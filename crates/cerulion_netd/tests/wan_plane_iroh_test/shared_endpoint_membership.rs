// SPDX-License-Identifier: AGPL-3.0-only
//! Malformed membership preserves admitted readers; identity fences are explicit.
use super::*;
use serde_json::{json, Value};
use std::io::{Read, Write};
use std::os::unix::net::UnixStream;

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
