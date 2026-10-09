// SPDX-License-Identifier: AGPL-3.0-only
//! Real marker ownership, consumer-first attach, and malformed identity refusals.
//! Private SHM roots; no network session or fixture cdylib is needed.

use cerulion_core::transport::{
    mirror_origin::marker_service_name, TransportConfig, TransportManager,
};
use cerulion_core::wire::{MaxSliceLen, WireHeader};
use iceoryx2::prelude::*;
use iceoryx2::service::attribute::AttributeSpecifier;

const SLICE: MaxSliceLen = MaxSliceLen::const_new(256);

#[test]
fn remote_marker_survives_failed_attribution_and_consumer_first_attach() {
    let config = cerulion_core::testing::iceoryx_test_config();
    let writer =
        TransportManager::init_for_test(TransportConfig::default(), config.clone()).unwrap();
    let reader = TransportManager::init_for_test(TransportConfig::default(), config).unwrap();
    let topic = "/marker/consumer_first";
    let subscriber = reader.create_subscriber(topic).unwrap();
    let injector = writer
        .create_remote_ingress_injector(topic, 0x1234, SLICE)
        .unwrap();
    assert!(writer
        .register_mirror_provenance(topic, &"x".repeat(257))
        .is_err());
    assert!(reader.is_network_mirror(topic).unwrap());
    assert_eq!(reader.network_mirror_topics().unwrap(), vec![topic]);
    let mut frame = [0u8; WireHeader::SIZE + 4];
    let mut header = WireHeader::new(0x1234, 42, 100);
    header.total_size = frame.len() as u32;
    header.write_to_buf(&mut frame[..WireHeader::SIZE]);
    frame[WireHeader::SIZE..].copy_from_slice(&[0x11, 0x22, 0x33, 0x44]);
    assert!(matches!(
        injector.reinject_raw(&frame),
        cerulion_core::ReinjectOutcome::Injected { recipients: 1 }
    ));
    let mut received = Vec::new();
    subscriber
        .try_receive(|message| {
            let mut header = [0u8; WireHeader::SIZE];
            message.header().write_to_buf(&mut header);
            received.extend_from_slice(&header);
            received.extend_from_slice(message.payload());
        })
        .unwrap();
    assert_eq!(received, frame);
    drop(injector);
    assert!(!reader.is_network_mirror(topic).unwrap());
    let _local = writer.create_publisher(topic, SLICE, 0).unwrap();
    assert!(!reader.is_network_mirror(topic).unwrap());
}

#[test]
fn failed_remote_publisher_removes_only_its_owned_marker() {
    let config = cerulion_core::testing::iceoryx_test_config();
    let manager = TransportManager::init_for_test(TransportConfig::default(), config).unwrap();
    let topic = "/marker/owned";
    let first = manager
        .create_remote_ingress_injector(topic, 0x1234, SLICE)
        .unwrap();
    let second = manager
        .create_remote_ingress_injector(topic, 0x1234, SLICE)
        .unwrap();
    assert!(manager
        .create_remote_ingress_injector(topic, 0x1234, SLICE)
        .is_err());
    assert!(manager.is_network_mirror(topic).unwrap());
    drop(first);
    assert!(manager.is_network_mirror(topic).unwrap());
    drop(second);
    assert!(!manager.is_network_mirror(topic).unwrap());
}

#[test]
fn refused_remote_publisher_leaves_local_topic_unmarked() {
    let config = cerulion_core::testing::iceoryx_test_config();
    let manager = TransportManager::init_for_test(TransportConfig::default(), config).unwrap();
    let topic = "/marker/local";
    let _first = manager.create_publisher(topic, SLICE, 0).unwrap();
    let _second = manager.create_publisher(topic, SLICE, 0).unwrap();
    assert!(manager
        .create_remote_ingress_injector(topic, 0x1234, SLICE)
        .is_err());
    assert!(!manager.is_network_mirror(topic).unwrap());
    assert!(manager.network_mirror_topics().unwrap().is_empty());
}

#[test]
fn marker_attribute_collision_refuses_observation_listing_and_publisher_creation() {
    let config = cerulion_core::testing::iceoryx_test_config();
    let node = NodeBuilder::new()
        .config(&config)
        .create::<ipc_threadsafe::Service>()
        .unwrap();
    let topic = "/marker/collision";
    let name: ServiceName = marker_service_name(topic).as_str().try_into().unwrap();
    let attributes = AttributeSpecifier::new()
        .define(
            &"topic".try_into().unwrap(),
            &"/another/topic".try_into().unwrap(),
        )
        .unwrap();
    let _service = node
        .service_builder(&name)
        .event()
        .create_with_attributes(&attributes)
        .unwrap();
    let manager = TransportManager::init_for_test(TransportConfig::default(), config).unwrap();
    assert!(manager.is_network_mirror(topic).is_err());
    assert!(manager.network_mirror_topics().is_err());
    assert!(manager
        .create_remote_ingress_injector(topic, 0x1234, SLICE)
        .is_err());
    assert!(manager.data_service_missing(topic));
}

/// Each refusal names its own role and remedy: a local observer losing to a live
/// mirror is told to observe without local scope, not that an identity failed; a
/// mirror losing to a live lease is told which lease; a malformed reserved marker
/// names the service and the `cerulion clean` remedy.
#[test]
fn refusals_name_the_role_and_the_remedy() {
    let config = cerulion_core::testing::iceoryx_test_config();
    let manager = TransportManager::init_for_test(TransportConfig::default(), config).unwrap();
    let topic = "/marker/refusal_text";
    let remote = manager
        .create_remote_ingress_injector(topic, 0x1234, SLICE)
        .unwrap();
    let lease_refused = match manager.acquire_local_topic_lease(topic) {
        Ok(_) => panic!("a live network mirror must refuse the local lease"),
        Err(e) => e.to_string(),
    };
    assert!(
        lease_refused.starts_with(
            "Local observation of topic '/marker/refusal_text' could not hold its source: a live network mirror owns this topic"
        ),
        "{lease_refused}"
    );
    assert!(
        !lease_refused.contains("could not be established"),
        "{lease_refused}"
    );
    assert!(
        !lease_refused.contains("bug in Cerulion"),
        "{lease_refused}"
    );
    drop(remote);
    let _lease = manager.acquire_local_topic_lease(topic).unwrap();
    let mirror_refused = match manager.create_remote_ingress_injector(topic, 0x1234, SLICE) {
        Ok(_) => panic!("a live local lease must refuse the network mirror"),
        Err(e) => e.to_string(),
    };
    assert!(
        mirror_refused.starts_with(
            "Network mirror identity for topic '/marker/refusal_text' could not be established: network mirror refused while a local observer holds its source lease"
        ),
        "{mirror_refused}"
    );
    assert!(
        !mirror_refused.contains("bug in Cerulion"),
        "{mirror_refused}"
    );
}

#[test]
fn missing_reserved_marker_attribute_fails_listing_closed() {
    let config = cerulion_core::testing::iceoryx_test_config();
    let node = NodeBuilder::new()
        .config(&config)
        .create::<ipc_threadsafe::Service>()
        .unwrap();
    let topic = "/marker/missing_attribute";
    let name: ServiceName = marker_service_name(topic).as_str().try_into().unwrap();
    let _service = node.service_builder(&name).event().create().unwrap();
    let manager = TransportManager::init_for_test(TransportConfig::default(), config).unwrap();
    assert!(manager.is_network_mirror(topic).is_err());
    let listing = manager.network_mirror_topics().unwrap_err().to_string();
    assert!(
        listing.starts_with(&format!(
            "Reserved network mirror marker '{}' is malformed: it carries no topic attribute",
            marker_service_name(topic)
        )),
        "{listing}"
    );
    assert!(
        listing.contains("run `cerulion clean` once it has exited"),
        "{listing}"
    );
    assert!(!listing.contains("bug in Cerulion"), "{listing}");
    assert!(manager
        .create_remote_ingress_injector(topic, 0x1234, SLICE)
        .is_err());
    assert!(manager.data_service_missing(topic));
}

/// Exercise the pinned native registry's role-registration ordering separately
/// from production admission. Ports stay alive until both decisions are known.
#[test]
fn fenced_native_role_registration_cannot_admit_both_sources() {
    use std::sync::atomic::{fence, Ordering};
    use std::sync::{Arc, Barrier};
    let config = cerulion_core::testing::iceoryx_test_config();
    let node = NodeBuilder::new()
        .config(&config)
        .create::<ipc_threadsafe::Service>()
        .unwrap();
    for round in 0..100 {
        let name: ServiceName = format!("/fenced-admission/{round}")
            .as_str()
            .try_into()
            .unwrap();
        let service = node
            .service_builder(&name)
            .event()
            .max_notifiers(64)
            .max_listeners(64)
            .create()
            .unwrap();
        let start = Arc::new(Barrier::new(2));
        let decisions = Arc::new(Barrier::new(2));
        let (local, remote) = std::thread::scope(|threads| {
            let local = threads.spawn({
                let start = start.clone();
                let decisions = decisions.clone();
                let service = &service;
                move || {
                    start.wait();
                    let _lease = service.listener_builder().create().unwrap();
                    fence(Ordering::SeqCst);
                    let admitted = service.dynamic_config().number_of_notifiers() == 0;
                    decisions.wait();
                    admitted
                }
            });
            let remote = threads.spawn({
                let service = &service;
                move || {
                    start.wait();
                    let _origin = service.notifier_builder().create().unwrap();
                    fence(Ordering::SeqCst);
                    let admitted = service.dynamic_config().number_of_listeners() == 0;
                    decisions.wait();
                    admitted
                }
            });
            (local.join().unwrap(), remote.join().unwrap())
        });
        assert!(!(local && remote), "both sources admitted in round {round}");
    }
}

#[test]
fn local_leases_keep_source_local_until_the_last_lease_exits() {
    let config = cerulion_core::testing::iceoryx_test_config();
    let manager = TransportManager::init_for_test(TransportConfig::default(), config).unwrap();
    let topic = "/marker/local_leases";
    let local = manager.create_publisher(topic, SLICE, 0).unwrap();
    let first = manager.acquire_local_topic_lease(topic).unwrap();
    let second = manager.acquire_local_topic_lease(topic).unwrap();
    let subscriber = manager.create_subscriber_open_only(topic).unwrap();
    assert!(!manager.is_network_mirror(topic).unwrap());
    assert!(manager.network_mirror_topics().unwrap().is_empty());
    drop(local);
    assert!(manager
        .create_remote_ingress_injector(topic, 0x1234, SLICE)
        .is_err());
    drop(first);
    assert!(manager
        .create_remote_ingress_injector(topic, 0x1234, SLICE)
        .is_err());
    drop(subscriber);
    drop(second);
    let remote = manager
        .create_remote_ingress_injector(topic, 0x1234, SLICE)
        .unwrap();
    assert!(manager.acquire_local_topic_lease(topic).is_err());
    let second_remote = manager
        .create_remote_ingress_injector(topic, 0x1234, SLICE)
        .unwrap();
    drop(remote);
    assert!(manager.is_network_mirror(topic).unwrap());
    assert!(manager.acquire_local_topic_lease(topic).is_err());
    drop(second_remote);
    let _local_again = manager.acquire_local_topic_lease(topic).unwrap();
    assert!(!manager.is_network_mirror(topic).unwrap());
}

#[test]
fn local_lease_quota_preserves_configured_creation_and_existing_open_limits() {
    for quota in [1, 7, 16] {
        let mut config = cerulion_core::testing::iceoryx_test_config();
        config.defaults.event.max_listeners = quota;
        let creator =
            TransportManager::init_for_test(TransportConfig::default(), config.clone()).unwrap();
        let topic = "/marker/listener_quota";
        let first = creator.acquire_local_topic_lease(topic).unwrap();
        // Opening with a different creation default must not impose a new minimum.
        config.defaults.event.max_listeners = 64;
        let opener = TransportManager::init_for_test(TransportConfig::default(), config).unwrap();
        let mut leases = vec![first];
        for _ in 1..quota {
            leases.push(opener.acquire_local_topic_lease(topic).unwrap());
        }
        assert!(opener.acquire_local_topic_lease(topic).is_err());
        assert!(!opener.is_network_mirror(topic).unwrap());
        assert!(opener.network_mirror_topics().unwrap().is_empty());
        assert!(opener
            .create_remote_ingress_injector(topic, 0x1234, SLICE)
            .is_err());
        drop(leases);
        let _remote = opener
            .create_remote_ingress_injector(topic, 0x1234, SLICE)
            .unwrap();
    }
}

/// Exercise the actual production constructors; retain each successful
/// port until both admission decisions are recorded. Same-process native race
/// evidence supplements the pinned ordering audit, not an exhaustive proof.
#[test]
fn source_constructors_never_admit_local_and_remote_together() {
    use std::sync::{Arc, Barrier};
    let config = cerulion_core::testing::iceoryx_test_config();
    let local_manager =
        TransportManager::init_for_test(TransportConfig::default(), config.clone()).unwrap();
    let remote_manager =
        TransportManager::init_for_test(TransportConfig::default(), config).unwrap();
    for round in 0..100 {
        let topic = format!("/marker/constructors/{round}");
        let start = Arc::new(Barrier::new(2));
        let decisions = Arc::new(Barrier::new(2));
        let (local, remote) = std::thread::scope(|threads| {
            let local = threads.spawn({
                let start = start.clone();
                let decisions = decisions.clone();
                let topic = &topic;
                let manager = &local_manager;
                move || {
                    start.wait();
                    let lease = manager.acquire_local_topic_lease(topic);
                    let admitted = lease.is_ok();
                    decisions.wait();
                    drop(lease);
                    admitted
                }
            });
            let remote = threads.spawn({
                let topic = &topic;
                let manager = &remote_manager;
                move || {
                    start.wait();
                    let injector = manager.create_remote_ingress_injector(topic, 0x1234, SLICE);
                    let admitted = injector.is_ok();
                    decisions.wait();
                    drop(injector);
                    admitted
                }
            });
            (local.join().unwrap(), remote.join().unwrap())
        });
        assert!(
            !(local && remote),
            "both source constructors admitted in round {round}"
        );
    }
}

/// Separate processes map the same private SHM registry. The child must refuse
/// a retained local source, then admit and deliver literal bytes after release.
#[test]
#[cfg(unix)]
fn a_child_remote_publisher_respects_the_local_source_lifetime() {
    use std::io::{Read, Write};
    use std::os::unix::net::{UnixListener, UnixStream};
    use std::process::{Child, Command, Stdio};
    use std::time::{Duration, Instant};
    const CHILD_CONFIG: &str = "CERULION_MIRROR_LEASE_TEST_CONFIG";
    const CHILD_SOCKET: &str = "CERULION_MIRROR_LEASE_TEST_SOCKET";
    const TOPIC: &str = "/marker/cross_process";
    // schema=0x1234, size=36, no offset table, seq=42, timestamp=100, payload.
    const FRAME: [u8; 36] = [
        0x34, 0x12, 0, 0, 0, 0, 0, 0, 0x24, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0x2A, 0, 0, 0, 0x64,
        0, 0, 0, 0, 0, 0, 0, 0x11, 0x22, 0x33, 0x44,
    ];
    fn receive(stream: &mut UnixStream) -> u8 {
        let mut byte = [0];
        stream.read_exact(&mut byte).unwrap();
        byte[0]
    }
    fn timeouts(stream: &UnixStream) {
        // Accepted sockets inherit the listener's nonblocking mode on macOS.
        stream.set_nonblocking(false).unwrap();
        stream
            .set_read_timeout(Some(Duration::from_secs(10)))
            .unwrap();
        stream
            .set_write_timeout(Some(Duration::from_secs(10)))
            .unwrap();
    }
    if let Some(path) = std::env::var_os(CHILD_CONFIG) {
        let config = serde_json::from_slice(&std::fs::read(path).unwrap()).unwrap();
        let manager = TransportManager::init_for_test(TransportConfig::default(), config).unwrap();
        let mut stream = UnixStream::connect(std::env::var_os(CHILD_SOCKET).unwrap()).unwrap();
        timeouts(&stream);
        assert_eq!(receive(&mut stream), b'T');
        let error = match manager.create_remote_ingress_injector(TOPIC, 0x1234, SLICE) {
            Err(error) => error.to_string(),
            Ok(_) => panic!("child replaced a parent-held local source"),
        };
        assert!(
            error.contains("local observer holds its source lease"),
            "{error}"
        );
        stream.write_all(b"R").unwrap();
        assert_eq!(receive(&mut stream), b'A');
        let remote = manager
            .create_remote_ingress_injector(TOPIC, 0x1234, SLICE)
            .unwrap();
        stream.write_all(b"O").unwrap();
        assert_eq!(receive(&mut stream), b'P');
        assert!(matches!(
            remote.reinject_raw(&FRAME),
            cerulion_core::ReinjectOutcome::Injected { recipients: 1 }
        ));
        stream.write_all(b"I").unwrap();
        assert_eq!(receive(&mut stream), b'X');
        drop(remote);
        stream.write_all(b"D").unwrap();
        return;
    }
    struct OwnedChild(Child);
    impl Drop for OwnedChild {
        fn drop(&mut self) {
            if self.0.try_wait().ok().flatten().is_none() {
                let _ = self.0.kill();
            }
            let _ = self.0.wait();
        }
    }
    let root = tempfile::tempdir_in("/tmp").unwrap();
    let config = cerulion_core::testing::iceoryx_test_config();
    let path = root.path().join("config.json");
    std::fs::write(&path, serde_json::to_vec(&config).unwrap()).unwrap();
    let socket = root.path().join("control.sock");
    let listener = UnixListener::bind(&socket).unwrap();
    listener.set_nonblocking(true).unwrap();
    let manager = TransportManager::init_for_test(TransportConfig::default(), config).unwrap();
    let local = manager.create_publisher(TOPIC, SLICE, 0x1234).unwrap();
    let lease = manager.acquire_local_topic_lease(TOPIC).unwrap();
    let subscriber = manager.create_subscriber_open_only(TOPIC).unwrap();
    drop(local);
    let mut child = OwnedChild(
        Command::new(std::env::current_exe().unwrap())
            .args([
                "--exact",
                "a_child_remote_publisher_respects_the_local_source_lifetime",
                "--nocapture",
            ])
            .env(CHILD_CONFIG, &path)
            .env(CHILD_SOCKET, &socket)
            .stdout(Stdio::null())
            .stderr(Stdio::inherit())
            .spawn()
            .unwrap(),
    );
    let deadline = Instant::now() + Duration::from_secs(10);
    let mut stream = loop {
        match listener.accept() {
            Ok((stream, _)) => break stream,
            Err(error) if error.kind() == std::io::ErrorKind::WouldBlock => {
                assert!(
                    child.0.try_wait().unwrap().is_none(),
                    "child exited before admission handshake"
                );
                assert!(Instant::now() < deadline, "child never connected");
                std::thread::yield_now();
            }
            Err(error) => panic!("accept child: {error}"),
        }
    };
    timeouts(&stream);
    stream.write_all(b"T").unwrap();
    assert_eq!(receive(&mut stream), b'R');
    assert!(!manager.is_network_mirror(TOPIC).unwrap());
    drop(subscriber);
    drop(lease);
    stream.write_all(b"A").unwrap();
    assert_eq!(receive(&mut stream), b'O');
    assert!(manager.is_network_mirror(TOPIC).unwrap());
    assert!(manager.acquire_local_topic_lease(TOPIC).is_err());
    let subscriber = manager.create_subscriber_open_only(TOPIC).unwrap();
    stream.write_all(b"P").unwrap();
    assert_eq!(receive(&mut stream), b'I');
    let mut received = Vec::new();
    subscriber
        .wait_for_message(Duration::from_secs(3), |message| {
            let mut header = [0; WireHeader::SIZE];
            message.header().write_to_buf(&mut header);
            received.extend_from_slice(&header);
            received.extend_from_slice(message.payload());
        })
        .unwrap();
    assert_eq!(received, FRAME);
    stream.write_all(b"X").unwrap();
    assert_eq!(receive(&mut stream), b'D');
    let deadline = Instant::now() + Duration::from_secs(10);
    let status = loop {
        if let Some(status) = child.0.try_wait().unwrap() {
            break status;
        }
        assert!(
            Instant::now() < deadline,
            "child failed to exit after releasing its source"
        );
        std::thread::yield_now();
    };
    assert!(status.success(), "child failed: {status}");
    assert!(!manager.is_network_mirror(TOPIC).unwrap());
    drop(subscriber);
    let _local_again = manager.acquire_local_topic_lease(TOPIC).unwrap();
}
