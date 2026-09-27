// SPDX-License-Identifier: AGPL-3.0-only
//! Scripted socket peers exercise framing, not an account-service substitute.

use super::*;
use crate::account_access::AccountSnapshot;
use std::io::Read;
use std::time::Duration;

const HELLO: &[u8] = b"{\"hello\":\"cerulion-netd\",\"protocol\":8}\n";
const INSTALLED: &[u8] =
    b"{\"id\":1,\"account_access\":{\"operation\":\"installed\",\"robot_count\":0}}\n";

fn deadline() -> Instant {
    Instant::now() + Duration::from_secs(5)
}

fn install() -> AccountAccessRequest {
    AccountAccessRequest::Install {
        snapshot: Box::new(AccountSnapshot {
            auth_account_id: "test-account".into(),
            pairing_account_id: [0; 32],
            device_key: [1; 32],
            owner_chain: None,
            robots: vec![],
        }),
    }
}

fn client_and_peer() -> (NetdClient, UnixStream) {
    let (stream, mut peer) = UnixStream::pair().unwrap();
    peer.set_read_timeout(Some(Duration::from_secs(5))).unwrap();
    peer.set_write_timeout(Some(Duration::from_secs(5)))
        .unwrap();
    peer.write_all(HELLO).unwrap();
    let client =
        NetdClient::finish_bounded_handshake(stream, "test.sock".into(), deadline()).unwrap();
    (client, peer)
}

fn receive_install(peer: &UnixStream) -> String {
    let mut line = String::new();
    BufReader::new(peer).read_line(&mut line).unwrap();
    let value: serde_json::Value = serde_json::from_str(&line).unwrap();
    assert_eq!(value["method"], "account_access");
    assert_eq!(value["id"], 1);
    assert_eq!(value["action"]["operation"], "install");
    line
}

fn assert_peer_closed(mut peer: UnixStream) {
    peer.set_read_timeout(Some(Duration::from_secs(5))).unwrap();
    let mut unexpected = Vec::new();
    match peer.read_to_end(&mut unexpected) {
        Ok(_) => {}
        Err(error)
            if matches!(
                error.kind(),
                io::ErrorKind::ConnectionReset | io::ErrorKind::NotConnected
            ) => {}
        Err(error) => panic!("peer did not observe connection closure: {error}"),
    }
    assert!(
        unexpected.is_empty(),
        "poisoned connection sent another request"
    );
}

fn assert_poisoned(client: &mut NetdClient) {
    assert!(client.poisoned);
    assert!(client
        .account_access_once(install())
        .unwrap_err()
        .to_string()
        .contains("reconnect"));
    assert!(client
        .round_trip(&Request::Status { id: 99 })
        .unwrap_err()
        .to_string()
        .contains("reconnect"));
    // A legacy first-demand retry must not replace the poisoned connection.
    client.next_id = 1;
    assert!(client
        .demand("robot", "/topic", 0)
        .unwrap_err()
        .to_string()
        .contains("reconnect"));
}

#[test]
fn complete_lines_respect_exact_cap_and_preserve_buffered_next_record() {
    let (stream, mut peer) = UnixStream::pair().unwrap();
    peer.write_all(b"first\nsecond\n").unwrap();
    let mut reader = BufReader::new(stream);
    assert_eq!(read_line(&mut reader, deadline(), 5).unwrap(), "first");
    assert_eq!(read_line(&mut reader, deadline(), 6).unwrap(), "second");
}

#[test]
fn partial_eof_oversize_and_invalid_utf8_are_not_complete_lines() {
    for (bytes, cap, expected) in [
        (b"partial".as_slice(), 8, io::ErrorKind::UnexpectedEof),
        (b"12345\n".as_slice(), 4, io::ErrorKind::InvalidData),
        (b"\xff\n".as_slice(), 8, io::ErrorKind::InvalidData),
    ] {
        let (stream, mut peer) = UnixStream::pair().unwrap();
        peer.write_all(bytes).unwrap();
        drop(peer);
        assert_eq!(
            read_line(&mut BufReader::new(stream), deadline(), cap)
                .unwrap_err()
                .kind(),
            expected
        );
    }
}

#[test]
fn progress_cannot_renew_the_absolute_read_deadline() {
    let (stream, mut peer) = UnixStream::pair().unwrap();
    peer.write_all(b"a").unwrap();
    let writer = std::thread::spawn(move || {
        // Each gap is shorter than the whole budget. A per-read timeout would
        // accept the eventual newline; an absolute deadline must refuse it.
        for byte in b"bcdefghijklmnopqrst\n" {
            std::thread::sleep(Duration::from_millis(25));
            if peer.write_all(&[*byte]).is_err() {
                break;
            }
        }
    });
    let mut reader = BufReader::new(stream);
    let error =
        read_line(&mut reader, Instant::now() + Duration::from_millis(100), 64).unwrap_err();
    assert!(matches!(
        error.kind(),
        io::ErrorKind::TimedOut | io::ErrorKind::WouldBlock
    ));
    drop(reader);
    writer.join().unwrap();
}

#[test]
fn expired_deadlines_reject_even_buffered_data_and_unsent_writes() {
    let (mut stream, mut peer) = UnixStream::pair().unwrap();
    peer.write_all(b"ready\n").unwrap();
    let expired = Instant::now();
    assert_eq!(
        read_line(
            &mut BufReader::new(stream.try_clone().unwrap()),
            expired,
            64
        )
        .unwrap_err()
        .kind(),
        io::ErrorKind::TimedOut
    );
    assert_eq!(
        write_line(&mut stream, "request", expired)
            .unwrap_err()
            .kind(),
        io::ErrorKind::TimedOut
    );
    stream.shutdown(Shutdown::Both).unwrap();
    let mut unsent = Vec::new();
    peer.read_to_end(&mut unsent).unwrap();
    assert!(unsent.is_empty());
}

#[test]
fn stalled_partial_write_expires_and_request_size_is_capped() {
    let (mut stream, _peer) = UnixStream::pair().unwrap();
    let oversized = "x".repeat(MAX_REQUEST_LINE_BYTES + 1);
    assert_eq!(
        write_line(&mut stream, &oversized, deadline())
            .unwrap_err()
            .kind(),
        io::ErrorKind::InvalidInput
    );
    let error = write_line(
        &mut stream,
        &oversized[..MAX_REQUEST_LINE_BYTES],
        Instant::now() + Duration::from_millis(100),
    )
    .unwrap_err();
    assert!(matches!(
        error.kind(),
        io::ErrorKind::TimedOut | io::ErrorKind::WouldBlock
    ));
}

#[test]
fn valid_fixed_response_is_repeatable_and_does_not_poison() {
    let mut requests = Vec::new();
    for _ in 0..2 {
        let (mut client, mut peer) = client_and_peer();
        let writer = std::thread::spawn(move || {
            let request = receive_install(&peer);
            peer.write_all(INSTALLED).unwrap();
            request
        });
        assert_eq!(
            client.account_access_once(install()).unwrap(),
            AccountAccessReply::Installed { robot_count: 0 }
        );
        assert!(!client.poisoned);
        assert_eq!(
            client.stream.read_timeout().unwrap(),
            Some(ROUNDTRIP_TIMEOUT)
        );
        requests.push(writer.join().unwrap());
    }
    // The semantic oracle above is independent; this adds the determinism pin.
    assert_eq!(requests[0], requests[1]);
}

#[test]
fn malformed_truncated_wrong_id_and_wrong_operation_poison_and_close() {
    for response in [
        b"{broken}\n".as_slice(),
        b"{\"id\":1".as_slice(),
        b"{\"id\":2,\"account_access\":{\"operation\":\"installed\",\"robot_count\":0}}\n"
            .as_slice(),
        b"{\"id\":1,\"account_access\":{\"operation\":\"installed\",\"robot_count\":1}}\n"
            .as_slice(),
        b"{\"id\":1,\"demands\":[],\"active_connections\":1,\"idle\":false}\n".as_slice(),
        b"{\"id\":1,\"error\":\"refused\",\"robot\":null,\"topic\":null".as_slice(),
    ] {
        let (mut client, mut peer) = client_and_peer();
        let writer = std::thread::spawn(move || {
            receive_install(&peer);
            peer.write_all(response).unwrap();
            let _ = peer.shutdown(Shutdown::Write);
            assert_peer_closed(peer);
        });
        assert!(client.account_access_once(install()).is_err());
        assert_poisoned(&mut client);
        writer.join().unwrap();
    }
}

#[test]
fn oversized_response_poisons_before_parsing_unbounded_input() {
    let (mut client, mut peer) = client_and_peer();
    let writer = std::thread::spawn(move || {
        receive_install(&peer);
        let _ = peer.write_all(&vec![b' '; MAX_RESPONSE_BYTES + 1]);
        assert_peer_closed(peer);
    });
    let error = client.account_access_once(install()).unwrap_err();
    assert!(
        matches!(error, ClientError::Io(ref error) if error.kind() == io::ErrorKind::InvalidData)
    );
    assert_poisoned(&mut client);
    writer.join().unwrap();
}

#[test]
fn delayed_response_cannot_be_reused_for_a_later_request() {
    let (mut client, mut peer) = client_and_peer();
    let (release, wait) = std::sync::mpsc::channel();
    let writer = std::thread::spawn(move || {
        receive_install(&peer);
        wait.recv_timeout(Duration::from_secs(5)).unwrap();
        let _ = peer.write_all(INSTALLED);
        assert_peer_closed(peer);
    });
    let error = client
        .account_access_until(install(), Instant::now() + Duration::from_millis(100))
        .unwrap_err();
    assert!(
        matches!(error, ClientError::Io(ref error) if matches!(error.kind(), io::ErrorKind::TimedOut | io::ErrorKind::WouldBlock))
    );
    assert_poisoned(&mut client);
    release.send(()).unwrap();
    writer.join().unwrap();
}

#[test]
fn correlated_remote_refusal_is_complete_and_keeps_connection_usable() {
    let (mut client, mut peer) = client_and_peer();
    let writer = std::thread::spawn(move || {
        receive_install(&peer);
        peer.write_all(b"{\"id\":1,\"error\":\"refused\",\"robot\":null,\"topic\":null}\n")
            .unwrap();
    });
    assert!(matches!(
        client.account_access_once(install()),
        Err(ClientError::Netd { .. })
    ));
    assert!(!client.poisoned);
    writer.join().unwrap();
}

#[test]
fn schema_for_another_topic_poisons_even_with_matching_id_and_robot() {
    let (mut client, mut peer) = client_and_peer();
    let writer = std::thread::spawn(move || {
        let mut request = String::new();
        BufReader::new(&peer).read_line(&mut request).unwrap();
        let value: serde_json::Value = serde_json::from_str(&request).unwrap();
        assert_eq!(value["action"]["topic"], "/wanted");
        peer.write_all(
            concat!(
                "{\"id\":1,\"account_access\":{\"operation\":\"schema\",",
                "\"robot_id\":[0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],",
                "\"schema\":{\"version\":1,\"robot\":\"robot\",\"requested\":\"/different\",",
                "\"docs\":[],\"error\":\"not found\"}}}\n"
            )
            .as_bytes(),
        )
        .unwrap();
        assert_peer_closed(peer);
    });
    assert!(client
        .account_access_once(AccountAccessRequest::Schema {
            robot_id: [0; 32],
            topic: "/wanted".into()
        })
        .is_err());
    assert_poisoned(&mut client);
    writer.join().unwrap();
}

#[test]
fn malformed_oversized_and_old_hello_close_without_a_request() {
    for bytes in [
        b"{broken}\n".to_vec(),
        b"{\"hello\":\"different\",\"protocol\":8}\n".to_vec(),
        b"{\"hello\":\"cerulion-netd\",\"protocol\":0}\n".to_vec(),
        vec![b'x'; MAX_HELLO_BYTES + 1],
    ] {
        let (stream, mut peer) = UnixStream::pair().unwrap();
        let writer = std::thread::spawn(move || {
            let _ = peer.write_all(&bytes);
            assert_peer_closed(peer);
        });
        assert!(
            NetdClient::finish_bounded_handshake(stream, "test.sock".into(), deadline()).is_err()
        );
        writer.join().unwrap();
    }
}

#[test]
fn supported_hello_with_old_verb_version_refuses_before_send() {
    let (stream, mut peer) = UnixStream::pair().unwrap();
    peer.write_all(b"{\"hello\":\"cerulion-netd\",\"protocol\":7}\n")
        .unwrap();
    let mut client =
        NetdClient::finish_bounded_handshake(stream, "test.sock".into(), deadline()).unwrap();
    let error = client
        .account_access_once(install())
        .unwrap_err()
        .to_string();
    assert!(error.contains("v7") && error.contains("v8") && error.contains("restart"));
    assert!(!client.poisoned);
    drop(client);
    let mut unsent = Vec::new();
    peer.read_to_end(&mut unsent).unwrap();
    assert!(unsent.is_empty());
}

#[test]
fn existing_only_constructor_uses_actual_unix_socket_and_never_creates_missing_path() {
    let dir = tempfile::tempdir_in("/tmp").unwrap();
    let path = dir.path().join("control.sock");
    assert!(matches!(
        NetdClient::connect_existing_at_until(path.clone(), deadline()),
        Err(ClientError::Connect {
            attempt: ConnectAttempt::ExistingOnly,
            ..
        })
    ));
    assert!(!path.exists());
    let listener = std::os::unix::net::UnixListener::bind(&path).unwrap();
    let writer = std::thread::spawn(move || {
        let (mut peer, _) = listener.accept().unwrap();
        peer.write_all(HELLO).unwrap();
        receive_install(&peer);
        peer.write_all(INSTALLED).unwrap();
        assert_peer_closed(peer);
    });
    let mut client = NetdClient::connect_existing_at_until(path, deadline()).unwrap();
    assert_eq!(
        client.account_access_once(install()).unwrap(),
        AccountAccessReply::Installed { robot_count: 0 }
    );
    drop(client);
    writer.join().unwrap();
}

#[test]
fn unix_connect_with_a_full_accept_queue_cannot_block_past_its_deadline() {
    let dir = tempfile::tempdir_in("/tmp").unwrap();
    let path = dir.path().join("full.sock");
    let address = socket2::SockAddr::unix(&path).unwrap();
    let listener =
        socket2::Socket::new(socket2::Domain::UNIX, socket2::Type::STREAM, None).unwrap();
    listener.bind(&address).unwrap();
    listener.listen(1).unwrap();
    let mut held = Vec::new();
    let mut saturated = false;
    for _ in 0..32 {
        let socket =
            socket2::Socket::new(socket2::Domain::UNIX, socket2::Type::STREAM, None).unwrap();
        if socket
            .connect_timeout(&address, Duration::from_millis(20))
            .is_err()
            || socket.peer_addr().is_err()
        {
            saturated = true;
            break;
        }
        held.push(socket);
    }
    assert!(saturated, "the real listener's accept queue must be full");
    let (send, receive) = std::sync::mpsc::channel();
    let worker = std::thread::spawn(move || {
        let result = NetdClient::connect_existing_at_until(
            path,
            Instant::now() + Duration::from_millis(100),
        );
        send.send(result.map(|_| ())).unwrap();
    });
    // This generous watchdog catches an unbounded connect, not small timing jitter.
    let observed = receive.recv_timeout(Duration::from_secs(5));
    drop(held);
    drop(listener);
    worker.join().unwrap();
    assert!(observed.expect("bounded Unix connect must return").is_err());
}

#[test]
fn connection_and_hello_share_the_callers_deadline() {
    let dir = tempfile::tempdir_in("/tmp").unwrap();
    let path = dir.path().join("silent.sock");
    let listener = std::os::unix::net::UnixListener::bind(&path).unwrap();
    let peer = std::thread::spawn(move || {
        let (stream, _) = listener.accept().unwrap();
        assert_peer_closed(stream);
    });
    let error =
        NetdClient::connect_existing_at_until(path, Instant::now() + Duration::from_millis(100))
            .unwrap_err();
    assert!(
        matches!(error, ClientError::Io(ref error) if matches!(error.kind(), io::ErrorKind::TimedOut | io::ErrorKind::WouldBlock))
    );
    peer.join().unwrap();
}
