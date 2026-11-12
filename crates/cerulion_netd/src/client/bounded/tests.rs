// SPDX-License-Identifier: AGPL-3.0-only
//! Scripted socket peers exercise framing, not an account-service substitute.

use super::*;
use crate::account_access::AccountSnapshot;
use std::io::Read;
use std::time::Duration;

const HELLO: &[u8] = b"{\"hello\":\"cerulion-netd\",\"protocol\":9,\"wire_header\":2}\n";
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

// Hand-written control replies; one socket proves retries retain the held key.
fn busy_mirror_peer(
    messages: Vec<&'static str>,
    persistent: bool,
) -> (NetdClient, std::thread::JoinHandle<(usize, Vec<u64>)>) {
    let (stream, mut peer) = UnixStream::pair().unwrap();
    peer.set_read_timeout(Some(Duration::from_secs(5))).unwrap();
    peer.write_all(b"{\"hello\":\"cerulion-netd\",\"protocol\":11,\"wire_header\":2}\n")
        .unwrap();
    let client =
        NetdClient::finish_bounded_handshake(stream, "no-reconnect.sock".into(), deadline())
            .unwrap();
    let worker = std::thread::spawn(move || {
        let mut reader = BufReader::new(peer.try_clone().unwrap());
        let mut attempts = 0;
        let mut ids = Vec::new();
        let mut held = false;
        loop {
            let mut line = String::new();
            if reader.read_line(&mut line).unwrap() == 0 {
                break;
            }
            let request: serde_json::Value = serde_json::from_str(&line).unwrap();
            let id = request["id"].as_u64().unwrap();
            ids.push(id);
            assert_eq!(request["robot"], "arm");
            let method = request["method"].as_str().unwrap();
            let reply = if request["topic"] == "/held" && method == "demand" {
                assert!(!held);
                held = true;
                serde_json::json!({"id":id,"robot":"arm","topic":"/held",
                    "refcount":1,"mirror_created":true})
            } else if method == "release" {
                assert_eq!(request["topic"], "/held");
                assert!(held, "retry lost the pre-existing demand");
                held = false;
                serde_json::json!({"id":id,"robot":"arm","topic":"/held",
                    "refcount":0,"last_release":true})
            } else {
                assert!(held);
                assert!(matches!(method, "observe" | "demand"));
                assert_eq!(request["topic"], "/topic");
                assert_eq!(request["schema_hash"], 7);
                let error = if persistent {
                    messages.first().copied()
                } else {
                    messages.get(attempts).copied()
                };
                attempts += 1;
                if let Some(message) = error {
                    serde_json::json!({"method":"error","id":id,"error":message,
                        "robot":"arm","topic":"/topic"})
                } else if method == "observe" {
                    serde_json::json!({"id":id,"robot":"arm","topic":"/topic",
                        "refcount":1,"mirror_created":true,"local_topic":"/topic"})
                } else {
                    serde_json::json!({"id":id,"robot":"arm","topic":"/topic",
                        "refcount":1,"mirror_created":true})
                }
            };
            writeln!(peer, "{reply}").unwrap();
        }
        assert!(!held, "the healthy demand was not explicitly released");
        (attempts, ids)
    });
    (client, worker)
}

#[test]
fn busy_mirror_observe_and_demand_retry_then_acknowledge_on_the_same_connection() {
    let mut runs = Vec::new();
    for _ in 0..2 {
        for observer in [true, false] {
            let (mut client, worker) = busy_mirror_peer(
                vec![
                    crate::account_access::CONTROLLER_BUSY,
                    crate::account_access::IDENTITY_BUSY,
                ],
                false,
            );
            client.demand("arm", "/held", 7).unwrap();
            let result = if observer {
                client.observe("arm", "/topic", 7).map(|reply| reply.demand)
            } else {
                client.demand("arm", "/topic", 7)
            };
            assert!(client.release("arm", "/held").unwrap().last_release);
            drop(client);
            let counts = worker.join().unwrap();
            let reply = result.unwrap();
            assert_eq!(
                (reply.robot.as_str(), reply.topic.as_str(), reply.refcount),
                ("arm", "/topic", 1)
            );
            assert_eq!(counts, (3, vec![1, 2, 3, 4, 5]));
            runs.push(counts);
        }
    }
    assert_eq!(runs[0..2], runs[2..4]);
}

#[test]
fn busy_mirror_persistent_refusal_keeps_original_error_and_live_hold_within_a_bound() {
    for observer in [true, false] {
        let (mut client, worker) =
            busy_mirror_peer(vec![crate::account_access::CONTROLLER_BUSY], true);
        client.demand("arm", "/held", 7).unwrap();
        let started = Instant::now();
        let result = if observer {
            client.observe("arm", "/topic", 7).map(|reply| reply.demand)
        } else {
            client.demand("arm", "/topic", 7)
        };
        assert!(started.elapsed() < Duration::from_millis(2500));
        assert!(client.release("arm", "/held").unwrap().last_release);
        drop(client);
        let (attempts, ids) = worker.join().unwrap();
        assert!(
            matches!(result, Err(ClientError::Netd { error, robot, topic })
            if error == crate::account_access::CONTROLLER_BUSY
                && robot.as_deref() == Some("arm") && topic.as_deref() == Some("/topic"))
        );
        assert_eq!(attempts, 8);
        assert_eq!(ids, (1..=10).collect::<Vec<_>>());
    }
}

#[test]
fn busy_mirror_non_transient_refusals_are_not_retried_or_replaced() {
    for message in [
        "schema conflict",
        "account robot controller is busy; retry the operation (fatal)",
        "robot access revoked",
    ] {
        for observer in [true, false] {
            let (mut client, worker) = busy_mirror_peer(vec![message], true);
            client.demand("arm", "/held", 7).unwrap();
            let result = if observer {
                client.observe("arm", "/topic", 7).map(|reply| reply.demand)
            } else {
                client.demand("arm", "/topic", 7)
            };
            assert!(client.release("arm", "/held").unwrap().last_release);
            drop(client);
            assert_eq!(worker.join().unwrap(), (1, vec![1, 2, 3]));
            assert!(matches!(result, Err(ClientError::Netd { error, .. }) if error == message));
        }
    }
}

#[test]
fn busy_mirror_wrapped_ensure_refusals_retry_without_matching_another_source() {
    let wrapped = "mirror registration failed: failed to establish the iroh WAN mirror for robot='arm' topic='/topic': account robot controller is busy; retry the operation";
    for _ in 0..2 {
        let (mut client, worker) = busy_mirror_peer(vec![wrapped], false);
        client.demand("arm", "/held", 7).unwrap();
        let reply = client.observe("arm", "/topic", 7).unwrap();
        assert_eq!(reply.local_topic, "/topic");
        assert!(client.release("arm", "/held").unwrap().last_release);
        drop(client);
        assert_eq!(worker.join().unwrap(), (2, vec![1, 2, 3, 4]));
    }
    let key = crate::registry::TopicKey::new("arm", "/topic");
    for message in [
        "mirror registration failed: failed to establish the iroh WAN mirror for robot='other' topic='/topic': account robot controller is busy; retry the operation",
        "mirror registration failed: failed to establish the iroh WAN mirror for robot='arm' topic='/other': account robot controller is busy; retry the operation",
        "mirror registration failed: failed to establish the iroh WAN mirror for robot='arm' topic='/topic': account robot controller is busy; retry the operation (fatal)",
    ] {
        assert!(!super::super::is_transient_mirror_refusal(message, &key));
    }
}

#[test]
fn busy_mirror_retry_schedule_has_attempt_and_time_limits() {
    use super::super::{mirror_busy_delay, MIRROR_BUSY_BUDGET};
    let expected = [23, 47, 83, 139, 211, 307, 389].map(Duration::from_millis);
    let actual: Vec<_> = (0..8)
        .filter_map(|index| mirror_busy_delay(index, MIRROR_BUSY_BUDGET))
        .collect();
    assert_eq!(actual, expected);
    assert_eq!(actual.iter().sum::<Duration>(), Duration::from_millis(1199));
    for (index, delay) in expected.iter().enumerate() {
        assert_eq!(mirror_busy_delay(index, *delay), None);
        assert_eq!(
            mirror_busy_delay(index, *delay + Duration::from_nanos(1)),
            Some(*delay)
        );
    }
    assert_eq!(mirror_busy_delay(0, Duration::ZERO), None);
    assert_eq!(MIRROR_BUSY_BUDGET, Duration::from_secs(2));
}

#[test]
fn busy_mirror_exhaustion_latches_until_a_completed_recovery() {
    let (mut client, worker) =
        busy_mirror_peer(vec![crate::account_access::CONTROLLER_BUSY; 16], false);
    client.demand("arm", "/held", 7).unwrap();
    for total in [1, 2] {
        assert!(matches!(
            client.observe("arm", "/topic", 7),
            Err(ClientError::Netd { .. })
        ));
        assert!(client.mirror_busy_failures.is_failing());
        assert_eq!(client.mirror_busy_failures.total_failures(), total);
    }
    assert_eq!(
        client.observe("arm", "/topic", 7).unwrap().local_topic,
        "/topic"
    );
    assert!(!client.mirror_busy_failures.is_failing());
    assert_eq!(client.mirror_busy_failures.total_failures(), 2);
    assert!(client.release("arm", "/held").unwrap().last_release);
    drop(client);
    assert_eq!(worker.join().unwrap(), (17, (1..=19).collect()));
}

#[test]
fn bounded_status_carries_failures_and_refuses_uncorrelated_replies() {
    for reply in [
        "{\"method\":\"status\",\"id\":1,\"demands\":[],\"active_connections\":1,\"idle\":false,\"robot_startup_error\":\"state directory is not owner-only\"}\n",
        "{\"method\":\"status\",\"id\":2,\"demands\":[],\"active_connections\":1,\"idle\":false}\n",
        "{\"method\":\"error\",\"id\":1,\"error\":\"status refused\"}\n",
    ] {
        let (mut client, mut peer) = client_and_peer();
        peer.write_all(reply.as_bytes()).unwrap();
        let result = client.status_until(deadline());
        if reply.contains("owner-only") {
            assert_eq!(result.unwrap().robot_startup_error.as_deref(), Some("state directory is not owner-only"));
        } else if reply.contains("status refused") {
            assert!(result.unwrap_err().to_string().contains("status refused"));
        } else {
            assert!(result.unwrap_err().to_string().contains("correlated daemon status"));
            assert_poisoned(&mut client);
        }
    }
}

#[test]
fn bounded_status_retires_a_connection_when_its_reply_never_arrives() {
    let (mut client, _peer) = client_and_peer();
    let started = Instant::now();
    assert!(client
        .status_until(started + Duration::from_millis(50))
        .is_err());
    assert!(started.elapsed() < Duration::from_secs(1));
    assert_poisoned(&mut client);
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

/// Reads `peer` to the end and asserts the other end closed without sending more.
///
/// The read timeout bounds a peer that never closes. The other end may already
/// have closed when this runs: macOS then refuses `setsockopt` with EINVAL, since
/// the socket can neither send nor receive. Such a socket is already
/// disconnected, so the read returns its buffered bytes and then EOF without a
/// timeout.
fn assert_peer_closed(mut peer: UnixStream) {
    match peer.set_read_timeout(Some(Duration::from_secs(5))) {
        Ok(()) => {}
        Err(error) if error.raw_os_error() == Some(libc::EINVAL) => {}
        Err(error) => panic!("could not bound the closure read: {error}"),
    }
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
    peer.write_all(b"{\"hello\":\"cerulion-netd\",\"protocol\":8,\"wire_header\":2}\n")
        .unwrap();
    let mut client =
        NetdClient::finish_bounded_handshake(stream, "test.sock".into(), deadline()).unwrap();
    let error = client
        .account_access_once(install())
        .unwrap_err()
        .to_string();
    assert!(error.contains("v8") && error.contains("v9") && error.contains("restart"));
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

#[cfg(target_os = "linux")]
#[test]
fn the_bounded_connect_reaches_an_abstract_control_socket() {
    use std::os::linux::net::SocketAddrExt;
    use std::os::unix::net::{SocketAddr, UnixListener};
    let name = format!("cerulion-bounded-test-{}", std::process::id());
    let address = SocketAddr::from_abstract_name(name.as_bytes()).unwrap();
    let listener = UnixListener::bind_addr(&address).unwrap();
    let server = std::thread::spawn(move || {
        let (mut peer, _) = listener.accept().unwrap();
        peer.write_all(HELLO).unwrap();
        let mut rest = Vec::new();
        let _ = peer.read_to_end(&mut rest);
    });
    let path = PathBuf::from(&name);
    // No filesystem socket has that name: the pathname branch cannot reach it.
    assert!(connect_control(&path, deadline(), false).is_err());
    let stream = connect_control(&path, deadline(), true).unwrap();
    let client = NetdClient::finish_bounded_handshake(stream, path, deadline()).unwrap();
    drop(client);
    server.join().unwrap();
}

#[test]
fn closure_check_accepts_a_peer_that_closed_before_it_ran() {
    let (stream, peer) = UnixStream::pair().unwrap();
    drop(stream);
    assert_peer_closed(peer);
}

#[test]
#[should_panic(expected = "poisoned connection sent another request")]
fn closure_check_still_reports_bytes_left_by_a_peer_that_already_closed() {
    let (mut stream, peer) = UnixStream::pair().unwrap();
    stream.write_all(b"unexpected").unwrap();
    drop(stream);
    assert_peer_closed(peer);
}
