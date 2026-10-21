// SPDX-License-Identifier: AGPL-3.0-only
//! The `quic/` carrier end to end on loopback: two `NetworkManager`s, one
//! listening on `quic/127.0.0.1:<port>?mixed_rel=1` with a self-signed
//! certificate, one connecting with that certificate as its only trust anchor.
//!
//! What each test pins:
//! - a frame put through `publish_to_network` arrives byte for byte, and a
//!   topic listed `best_effort` in `NetworkConfig::egress_qos` arrives carrying
//!   zenoh's best-effort marker while an unlisted topic arrives reliable; the
//!   consumer's zenoh session lists a non-streamed best-effort link (the QUIC
//!   datagram link) beside the stream link;
//! - a producer built from a graph `network:` block puts each egress topic
//!   with the marker its own `reliability` names, else the block's
//!   `default_reliability`, in both directions;
//! - a self-signed end-entity certificate is accepted as its own trust anchor
//!   for both ECDSA P-256 and Ed25519 keys;
//! - a connector whose `root_ca` is a DIFFERENT certificate receives nothing,
//!   while a second connector to the same listener that trusts the right
//!   certificate receives frames;
//! - a connector that trusts the listener's certificate but dials an address
//!   the certificate does not name receives nothing, while the same dial to a
//!   listener whose certificate names that address receives frames;
//! - a `tls` path that does not exist, or holds no PEM block, fails the session
//!   open naming the `tls.*` setting and the path.
//!
//! Which link carried a given frame is zenoh's choice and is not observable
//! from inside the process; a packet capture reads the QUIC frame types.

use std::path::{Path, PathBuf};
use std::sync::mpsc;
use std::time::{Duration, Instant};

use cerulion_core::graph::NetworkBlock;
use cerulion_core::{
    EgressQos, EgressReliability, NetworkConfig, NetworkManager, NetworkTls, WireHeader,
};
use zenoh::qos::Reliability;
use zenoh::Wait;

/// A loopback UDP port that was free a moment ago.
fn free_udp_port() -> u16 {
    std::net::UdpSocket::bind("127.0.0.1:0")
        .unwrap()
        .local_addr()
        .unwrap()
        .port()
}

/// Write a self-signed certificate for `127.0.0.1` and its key under `dir`,
/// returning `(certificate path, key path)`.
fn self_signed(
    dir: &Path,
    name: &str,
    alg: &'static rcgen::SignatureAlgorithm,
) -> (PathBuf, PathBuf) {
    let key = rcgen::KeyPair::generate_for(alg).unwrap();
    let cert = rcgen::CertificateParams::new(vec!["127.0.0.1".to_string()])
        .unwrap()
        .self_signed(&key)
        .unwrap();
    let cert_path = dir.join(format!("{name}.pem"));
    let key_path = dir.join(format!("{name}.key"));
    std::fs::write(&cert_path, cert.pem()).unwrap();
    std::fs::write(&key_path, key.serialize_pem()).unwrap();
    (cert_path, key_path)
}

/// A frame whose payload after the header is `seq` then 56 bytes that depend
/// on `seq` and their position, so a truncated or altered frame differs.
fn frame(seq: u64) -> Vec<u8> {
    let mut f = vec![0u8; WireHeader::SIZE + 64];
    f[WireHeader::SIZE..WireHeader::SIZE + 8].copy_from_slice(&seq.to_le_bytes());
    for (i, b) in f[WireHeader::SIZE + 8..].iter_mut().enumerate() {
        *b = (seq as u8).wrapping_mul(31).wrapping_add(i as u8);
    }
    f
}

/// A listener on `locator` presenting `cert` / `key`, with `/q/be` best effort.
fn producer(locator: &str, cert: &Path, key: &Path) -> NetworkManager {
    NetworkManager::new(NetworkConfig {
        listen_endpoints: vec![locator.to_string()],
        tls: Some(NetworkTls {
            certificate: Some(cert.to_path_buf()),
            private_key: Some(key.to_path_buf()),
            ..NetworkTls::default()
        }),
        egress_qos: vec![EgressQos {
            topic: "/q/be".to_string(),
            reliability: EgressReliability::BestEffort,
        }],
        ..NetworkConfig::default()
    })
}

/// A connector dialling `locator` that trusts `anchor`.
fn consumer(locator: &str, anchor: &Path) -> NetworkManager {
    NetworkManager::new(NetworkConfig {
        connect_endpoints: vec![locator.to_string()],
        tls: Some(NetworkTls {
            root_ca: Some(anchor.to_path_buf()),
            ..NetworkTls::default()
        }),
        ..NetworkConfig::default()
    })
}

struct Pair {
    producer: NetworkManager,
    consumer: NetworkManager,
}

/// A listening producer with `cert`/`key` and best-effort `/q/be`, and a
/// connecting consumer that trusts `anchor`, both on `127.0.0.1:<port>`.
fn pair(port: u16, cert: &Path, key: &Path, anchor: &Path) -> Pair {
    let locator = format!("quic/127.0.0.1:{port}?mixed_rel=1");
    Pair {
        producer: producer(&locator, cert, key),
        consumer: consumer(&locator, anchor),
    }
}

/// Subscribe on `consumer` to `topic`, then put frames on `producer` until one
/// arrives or `budget` passes. Each received payload must equal, byte for
/// byte, the frame put with the sequence number it carries. Returns the
/// received sample's reliability.
fn deliver_one(
    producer: &NetworkManager,
    consumer: &NetworkManager,
    topic: &str,
    budget: Duration,
) -> Option<Reliability> {
    let session = consumer.session().expect("consumer session opens");
    let (tx, rx) = mpsc::channel();
    let _sub = session
        .declare_subscriber(NetworkManager::data_key(topic))
        .callback(move |s| {
            let _ = tx.send((s.payload().to_bytes().into_owned(), s.reliability()));
        })
        .wait()
        .unwrap();
    let deadline = Instant::now() + budget;
    let mut seq = 0u64;
    while Instant::now() < deadline {
        producer
            .publish_to_network(topic, frame(seq))
            .expect("the put itself never fails on a live session");
        seq += 1;
        if let Ok((bytes, rel)) = rx.recv_timeout(Duration::from_millis(100)) {
            assert!(
                bytes.len() >= WireHeader::SIZE + 8,
                "a {}-byte payload is shorter than any frame put",
                bytes.len()
            );
            let got_seq = u64::from_le_bytes(
                bytes[WireHeader::SIZE..WireHeader::SIZE + 8]
                    .try_into()
                    .unwrap(),
            );
            assert!(got_seq < seq, "sequence {got_seq} was never put");
            assert_eq!(bytes, frame(got_seq), "the frame arrives verbatim");
            return Some(rel);
        }
    }
    None
}

fn delivers_with_self_signed_anchor(alg: &'static rcgen::SignatureAlgorithm) {
    let dir = tempfile::tempdir().unwrap();
    let (cert, key) = self_signed(dir.path(), "robot", alg);
    let p = pair(free_udp_port(), &cert, &key, &cert);
    p.producer
        .session()
        .expect("a quic/ listener opens with the files");
    assert_eq!(
        deliver_one(&p.producer, &p.consumer, "/q/be", Duration::from_secs(10)),
        Some(Reliability::BestEffort),
        "a best_effort topic arrives with the best-effort marker"
    );
    assert_eq!(
        deliver_one(&p.producer, &p.consumer, "/q/rel", Duration::from_secs(10)),
        Some(Reliability::Reliable),
        "an unlisted topic arrives reliable"
    );
    let links: Vec<_> = p
        .consumer
        .session()
        .unwrap()
        .info()
        .links()
        .wait()
        .map(|l| (l.is_streamed(), l.reliability()))
        .collect();
    assert!(
        links.contains(&(false, Some(Reliability::BestEffort))),
        "a mixed_rel=1 connection carries a non-streamed best-effort (datagram) link: {links:?}"
    );
    assert!(
        links.contains(&(true, Some(Reliability::Reliable))),
        "beside its reliable stream link: {links:?}"
    );
    assert_eq!(
        p.producer.egress_reliability("/q/be"),
        EgressReliability::BestEffort
    );
    assert_eq!(
        p.producer.egress_reliability("/q/rel"),
        EgressReliability::Reliable
    );
}

#[test]
fn quic_mixed_rel_delivers_with_ecdsa_self_signed_anchor() {
    delivers_with_self_signed_anchor(&rcgen::PKCS_ECDSA_P256_SHA256);
}

#[test]
fn quic_mixed_rel_delivers_with_ed25519_self_signed_anchor() {
    delivers_with_self_signed_anchor(&rcgen::PKCS_ED25519);
}

/// The producer's `NetworkConfig` comes from a parsed graph `network:` block:
/// under each `default_reliability`, a bare egress topic arrives with the
/// default's marker, and a topic with its own `reliability` arrives with that
/// marker whatever the default says.
#[test]
fn quic_mixed_rel_puts_each_topic_at_its_effective_reliability() {
    let dir = tempfile::tempdir().unwrap();
    let (cert, key) = self_signed(dir.path(), "robot", &rcgen::PKCS_ECDSA_P256_SHA256);
    for (default, opposite) in [("best_effort", "reliable"), ("reliable", "best_effort")] {
        let port = free_udp_port();
        let yaml = format!(
            "mode: peer\n\
             listen:\n  - quic/127.0.0.1:{port}?mixed_rel=1\n\
             default_reliability: {default}\n\
             egress:\n  - /q/bare\n  - topic: /q/own\n    reliability: {opposite}\n\
             tls:\n  certificate: {}\n  private_key: {}\n",
            cert.display(),
            key.display()
        );
        let block: NetworkBlock = serde_yaml::from_str(&yaml).unwrap();
        let p = Pair {
            producer: NetworkManager::new(block.to_network_config().expect("peer mode")),
            consumer: NetworkManager::new(NetworkConfig {
                connect_endpoints: vec![format!("quic/127.0.0.1:{port}?mixed_rel=1")],
                tls: Some(NetworkTls {
                    root_ca: Some(cert.clone()),
                    ..NetworkTls::default()
                }),
                ..NetworkConfig::default()
            }),
        };
        p.producer.session().expect("a quic/ listener opens");
        let marker = |r: &str| match r {
            "best_effort" => Reliability::BestEffort,
            _ => Reliability::Reliable,
        };
        assert_eq!(
            deliver_one(&p.producer, &p.consumer, "/q/bare", Duration::from_secs(10)),
            Some(marker(default)),
            "default {default}: a bare topic arrives at the default"
        );
        assert_eq!(
            deliver_one(&p.producer, &p.consumer, "/q/own", Duration::from_secs(10)),
            Some(marker(opposite)),
            "default {default}: a topic's own reliability ({opposite}) wins"
        );
    }
}

#[test]
fn quic_connector_with_a_different_anchor_receives_nothing() {
    let dir = tempfile::tempdir().unwrap();
    let (cert, key) = self_signed(dir.path(), "robot", &rcgen::PKCS_ECDSA_P256_SHA256);
    let (stranger, _) = self_signed(dir.path(), "stranger", &rcgen::PKCS_ECDSA_P256_SHA256);
    let p = pair(free_udp_port(), &cert, &key, &stranger);
    p.producer.session().expect("listener opens");
    assert_eq!(
        deliver_one(&p.producer, &p.consumer, "/q/rel", Duration::from_secs(4)),
        None,
        "a peer whose certificate does not chain to root_ca must not connect"
    );
    // The same listener reaches a connector that trusts its certificate, so the
    // anchor is what refused the first one.
    let locator = p.producer.config().listen_endpoints[0].clone();
    let trusting = consumer(&locator, &cert);
    assert_eq!(
        deliver_one(&p.producer, &trusting, "/q/rel", Duration::from_secs(10)),
        Some(Reliability::Reliable),
        "a connector whose root_ca is the listener's certificate receives frames"
    );
}

/// The connector always checks that the listener's certificate names the host
/// or IP it dialled (`verify_name_on_connect`). A certificate whose only name
/// is `localhost`, trusted as `root_ca`, is refused when `127.0.0.1` is
/// dialled. The control is a second listener whose certificate, from the same
/// key, also names `127.0.0.1`: dialled the same way, it delivers.
#[test]
fn quic_connector_dialling_a_name_the_certificate_lacks_receives_nothing() {
    let dir = tempfile::tempdir().unwrap();
    let key_pair = rcgen::KeyPair::generate_for(&rcgen::PKCS_ECDSA_P256_SHA256).unwrap();
    let key_path = dir.path().join("robot.key");
    std::fs::write(&key_path, key_pair.serialize_pem()).unwrap();
    let cert_for = |file: &str, names: &[&str]| {
        let cert =
            rcgen::CertificateParams::new(names.iter().map(|n| n.to_string()).collect::<Vec<_>>())
                .unwrap()
                .self_signed(&key_pair)
                .unwrap();
        let path = dir.path().join(file);
        std::fs::write(&path, cert.pem()).unwrap();
        path
    };
    let name_only = cert_for("localhost.pem", &["localhost"]);
    let with_ip = cert_for("localhost-and-ip.pem", &["localhost", "127.0.0.1"]);

    let wrong = format!("quic/127.0.0.1:{}?mixed_rel=1", free_udp_port());
    let wrong_producer = producer(&wrong, &name_only, &key_path);
    wrong_producer.session().expect("listener opens");
    let wrong_consumer = consumer(&wrong, &name_only);
    assert_eq!(
        deliver_one(
            &wrong_producer,
            &wrong_consumer,
            "/q/rel",
            Duration::from_secs(4)
        ),
        None,
        "a trusted certificate that does not name 127.0.0.1 must be refused when 127.0.0.1 is dialled"
    );

    let right = format!("quic/127.0.0.1:{}?mixed_rel=1", free_udp_port());
    let right_producer = producer(&right, &with_ip, &key_path);
    right_producer.session().expect("listener opens");
    let right_consumer = consumer(&right, &with_ip);
    assert_eq!(
        deliver_one(
            &right_producer,
            &right_consumer,
            "/q/rel",
            Duration::from_secs(10)
        ),
        Some(Reliability::Reliable),
        "the same key, anchor setup and dialled address deliver once the certificate names 127.0.0.1"
    );
}

#[test]
fn quic_tls_path_that_does_not_exist_fails_the_open_naming_it() {
    let missing = PathBuf::from("/nonexistent/cerulion-quic-test/robot.pem");
    let m = NetworkManager::new(NetworkConfig {
        connect_endpoints: vec![format!("quic/127.0.0.1:{}", free_udp_port())],
        tls: Some(NetworkTls {
            root_ca: Some(missing.clone()),
            ..NetworkTls::default()
        }),
        ..NetworkConfig::default()
    });
    let err = m
        .session()
        .expect_err("missing root_ca refuses the open")
        .to_string();
    assert!(err.contains(missing.to_str().unwrap()), "{err}");
    assert!(err.contains("tls.root_ca"), "{err}");
}

#[test]
fn quic_tls_file_without_a_pem_block_fails_the_open_naming_it() {
    let dir = tempfile::tempdir().unwrap();
    let not_pem = dir.path().join("robot.pem");
    std::fs::write(&not_pem, b"this is not a certificate\n").unwrap();
    let m = NetworkManager::new(NetworkConfig {
        listen_endpoints: vec![format!("quic/127.0.0.1:{}", free_udp_port())],
        tls: Some(NetworkTls {
            certificate: Some(not_pem.clone()),
            private_key: Some(not_pem.clone()),
            ..NetworkTls::default()
        }),
        ..NetworkConfig::default()
    });
    let err = m
        .session()
        .expect_err("a non-PEM file refuses the open")
        .to_string();
    assert!(err.contains(not_pem.to_str().unwrap()), "{err}");
    assert!(err.contains("tls.certificate"), "{err}");
    assert!(err.contains("no PEM block"), "{err}");
}

/// A self-signed certificate marked `CA:TRUE` (what `openssl req -x509` writes
/// by default) would be refused at handshake as its own end entity
/// (`CaUsedAsEndEntity`), seen only as a listener-side WARN. The listener's
/// session open refuses it instead, naming the file and the openssl flag.
#[test]
fn quic_listener_certificate_marked_ca_fails_the_open_with_the_fix() {
    let dir = tempfile::tempdir().unwrap();
    let key = rcgen::KeyPair::generate_for(&rcgen::PKCS_ECDSA_P256_SHA256).unwrap();
    let mut params = rcgen::CertificateParams::new(vec!["127.0.0.1".to_string()]).unwrap();
    params.is_ca = rcgen::IsCa::Ca(rcgen::BasicConstraints::Unconstrained);
    let cert = params.self_signed(&key).unwrap();
    let cert_path = dir.path().join("ca.pem");
    let key_path = dir.path().join("ca.key");
    std::fs::write(&cert_path, cert.pem()).unwrap();
    std::fs::write(&key_path, key.serialize_pem()).unwrap();
    let p = pair(free_udp_port(), &cert_path, &key_path, &cert_path);
    let err = p
        .producer
        .session()
        .expect_err("a CA:TRUE listener certificate refuses the open")
        .to_string();
    assert!(err.contains(cert_path.to_str().unwrap()), "{err}");
    assert!(err.contains("tls.certificate"), "{err}");
    assert!(err.contains("CA:TRUE"), "{err}");
    assert!(err.contains("basicConstraints=critical,CA:FALSE"), "{err}");
    // The same file as a CONNECT-side root_ca is fine: a real CA is CA:TRUE.
    p.consumer.session().expect("a CA:TRUE root_ca is accepted");
}
