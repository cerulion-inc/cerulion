// SPDX-License-Identifier: MIT OR Apache-2.0
//! Literal protocol and state oracles for the transport-independent exec client.

use std::io::Cursor;

use cerulion_ops_proto::exec_client::{
    ExecCaptureLimits, ExecClient, ExecEvent, ExecPartial, ExecResult,
};
use cerulion_ops_proto::session::{encode_session, read_session_frame, write_session_frame};
use cerulion_ops_proto::{
    ClientFrame, ExitReason, HandshakeReply, OpsProtoError, OutputStream, RecordingWitness,
    ServerFrame, SessionExit, MAX_SESSION_FRAME_BYTES,
};

const HELLO_JSON: &str = r#"{"min_version":2,"max_version":2,"session":{"type":"exec","argv":["printf","%s","a b","","--json","$(x)","é"]}}"#;
const ACK_JSON: &[u8] = br#"{"type":"ack","chosen_version":2,"server_name":"robot"}"#;
const EOF_JSON: &[u8] = br#"{"type":"stdin_eof"}"#;
const STARTED_JSON: &str = r#"{"type":"started","session_id":"exec-1"}"#;
const STDOUT_JSON: &str = r#"{"type":"data","stream":"stdout","data":"AP9hCg=="}"#;
const STDERR_JSON: &str = r#"{"type":"data","stream":"stderr","data":"RXJy"}"#;
const H1: &str = "7668ad26affbdd0fe5ac260fa107f93ab31b7ff2baacc64204f1c471d6f92232";
const H2: &str = "3f1352e989697121f70af74c842c8cb73833a47a9a19361995af4b77d5a2055c";

fn client(stdout_bytes: usize, stderr_bytes: usize) -> ExecClient {
    ExecClient::new(
        vec!["printf".into(), "hello".into()],
        ExecCaptureLimits {
            stdout_bytes,
            stderr_bytes,
        },
    )
    .unwrap()
}

fn ack() -> HandshakeReply {
    serde_json::from_slice(ACK_JSON).unwrap()
}

fn frame(json: &str) -> ServerFrame {
    serde_json::from_str(json).unwrap()
}

fn ready(stdout_bytes: usize, stderr_bytes: usize) -> ExecClient {
    let mut client = client(stdout_bytes, stderr_bytes);
    assert_eq!(
        client.accept_handshake(ack()).unwrap(),
        ClientFrame::StdinEof
    );
    assert_eq!(
        client.accept_frame(frame(STARTED_JSON)).unwrap(),
        ExecEvent::Progress
    );
    client
}

fn data(stream: OutputStream, encoded: &str) -> ServerFrame {
    ServerFrame::Data {
        stream,
        data: encoded.into(),
    }
}

fn witness(session_id: &str, head: &str, events: u64) -> RecordingWitness {
    RecordingWitness {
        session_id: session_id.into(),
        head: head.into(),
        events,
    }
}

fn checkpoint(session_id: &str, head: &str, events: u64) -> ServerFrame {
    ServerFrame::Witness {
        witness: witness(session_id, head, events),
    }
}

fn exit() -> SessionExit {
    serde_json::from_str(r#"{"code":7,"signal":null,"reason":"exited","recording_head":"7668ad26affbdd0fe5ac260fa107f93ab31b7ff2baacc64204f1c471d6f92232","events":6,"stdout_truncated":false,"stderr_truncated":false}"#).unwrap()
}

fn complete(client: &mut ExecClient, exit: SessionExit) -> ExecResult {
    match client.accept_frame(ServerFrame::Exit(exit)).unwrap() {
        ExecEvent::Complete(result) => result,
        other => panic!("expected a final result, got {other:?}"),
    }
}

fn assert_decode(error: OpsProtoError) {
    assert!(matches!(error, OpsProtoError::Decode(_)), "{error:?}");
}

fn assert_poisoned(client: &mut ExecClient) {
    let retained = client.partial();
    if retained.session_id.is_some() {
        // This is valid while streaming: probe before any invalid repeated Ack
        // or Started could itself turn a recoverable session error terminal.
        assert_decode(
            client
                .accept_frame(data(OutputStream::Stdout, "QQ=="))
                .unwrap_err(),
        );
        assert_eq!(client.partial(), retained);
    }
    assert_decode(client.accept_handshake(ack()).unwrap_err());
    for frame in [
        frame(STARTED_JSON),
        frame(STDOUT_JSON),
        checkpoint("exec-1", H1, 4),
        ServerFrame::Exit(exit()),
    ] {
        assert_decode(client.accept_frame(frame).unwrap_err());
        assert_eq!(client.partial(), retained);
    }
}

fn captured(partial: &ExecPartial, stream: OutputStream) -> (&[u8], bool) {
    match stream {
        OutputStream::Stdout => (&partial.stdout, partial.stdout_truncated),
        OutputStream::Stderr => (&partial.stderr, partial.stderr_truncated),
        OutputStream::Pty => panic!("exec has no PTY capture"),
    }
}

#[test]
fn hello_preserves_literal_argv_and_exec_mode() {
    let client = ExecClient::new(
        ["printf", "%s", "a b", "", "--json", "$(x)", "é"]
            .map(str::to_owned)
            .to_vec(),
        ExecCaptureLimits {
            stdout_bytes: 64,
            stderr_bytes: 64,
        },
    )
    .unwrap();
    let payload = encode_session(client.hello()).unwrap();
    assert_eq!(payload, HELLO_JSON.as_bytes());
    let mut wire = Vec::new();
    write_session_frame(&mut wire, &payload).unwrap();
    // Hand-counted UTF-8 payload length: 112, including two bytes for é.
    let expected = [b"\x00\x00\x00\x70".as_slice(), HELLO_JSON.as_bytes()].concat();
    assert_eq!(wire, expected);
    assert_eq!(encode_session(client.hello()).unwrap(), payload);
}

#[test]
fn invalid_argv_cannot_start_a_client_and_full_hello_is_bounded() {
    let limits = ExecCaptureLimits {
        stdout_bytes: 1,
        stderr_bytes: 1,
    };
    for argv in [
        vec![],
        vec![String::new()],
        vec!["pr\0intf".into()],
        vec!["printf".into(), "arg\0ument".into()],
    ] {
        match ExecClient::new(argv, limits) {
            Err(error) => assert_decode(error),
            Ok(_) => panic!("invalid argv produced a client"),
        }
    }
    // The literal full Hello with argv:[""] has 71 bytes outside the argument.
    // Bounding only the inner session object would accept the one-byte overflow.
    let argument = "a".repeat(MAX_SESSION_FRAME_BYTES - 71);
    let exact = ExecClient::new(vec![argument.clone()], limits).unwrap();
    assert_eq!(encode_session(exact.hello()).unwrap().len(), 1_048_576);
    assert!(matches!(
        ExecClient::new(vec![format!("{argument}a")], limits),
        Err(OpsProtoError::FrameTooLarge {
            size: 1_048_577,
            max: 1_048_576
        })
    ));
}

#[test]
fn ack_yields_child_stdin_eof_once() {
    let mut client = client(8, 8);
    let literal_ack = [b"\x00\x00\x00\x37".as_slice(), ACK_JSON].concat();
    let payload = read_session_frame(&mut Cursor::new(literal_ack)).unwrap();
    let reply = serde_json::from_slice(&payload).unwrap();
    let eof = client.accept_handshake(reply).unwrap();
    assert_eq!(eof, ClientFrame::StdinEof);
    let mut wire = Vec::new();
    write_session_frame(&mut wire, &encode_session(&eof).unwrap()).unwrap();
    assert_eq!(wire, [b"\x00\x00\x00\x14".as_slice(), EOF_JSON].concat());
    assert_decode(client.accept_handshake(ack()).unwrap_err());
    assert_decode(client.accept_frame(frame(STARTED_JSON)).unwrap_err());
    assert_poisoned(&mut client);
}

#[test]
fn handshake_refusal_and_unoffered_versions_are_terminal() {
    for chosen_version in [0, 1, 3, u16::MAX] {
        let mut client = client(8, 8);
        assert_decode(
            client
                .accept_handshake(HandshakeReply::Ack {
                    chosen_version,
                    server_name: "robot".into(),
                })
                .unwrap_err(),
        );
        assert_poisoned(&mut client);
    }
    let mut client = client(8, 8);
    let rejection = serde_json::from_str(r#"{"type":"reject","server_min":1,"server_max":1,"message":"unpaired\u001b[31m\nrefused"}"#).unwrap();
    match client.accept_handshake(rejection).unwrap_err() {
        OpsProtoError::HandshakeRejected {
            server_min,
            server_max,
            reason,
        } => {
            assert_eq!((server_min, server_max), (1, 1));
            assert_eq!(reason, "unpaired\u{1b}[31m\nrefused");
        }
        other => panic!("wrong refusal envelope: {other:?}"),
    }
    assert_poisoned(&mut client);
}

#[test]
fn server_frames_require_the_handshake() {
    for frame in [
        frame(STARTED_JSON),
        frame(STDOUT_JSON),
        checkpoint("exec-1", H1, 1),
        ServerFrame::Exit(exit()),
    ] {
        let mut client = client(8, 8);
        let before = client.partial();
        assert_decode(client.accept_frame(frame).unwrap_err());
        assert_eq!(client.partial(), before);
        assert_poisoned(&mut client);
    }
}

#[test]
fn streams_preserve_binary_bytes_and_order_independently() {
    let mut client = ready(8, 8);
    for (prefix, json) in [
        (b"\x00\x00\x00\x33", STDOUT_JSON),
        (b"\x00\x00\x00\x2f", STDERR_JSON),
    ] {
        let wire = [prefix.as_slice(), json.as_bytes()].concat();
        let payload = read_session_frame(&mut Cursor::new(wire)).unwrap();
        assert_eq!(
            client
                .accept_frame(serde_json::from_slice(&payload).unwrap())
                .unwrap(),
            ExecEvent::Progress
        );
    }
    client
        .accept_frame(data(OutputStream::Stdout, "QQ=="))
        .unwrap();
    client
        .accept_frame(data(OutputStream::Stderr, "Qg=="))
        .unwrap();
    let result = complete(&mut client, exit());
    assert_eq!(result.stdout, [0, 255, 97, 10, 65]);
    assert_eq!(result.stderr, b"ErrB");
    assert!(!result.stdout_truncated);
    assert!(!result.stderr_truncated);
}

#[test]
fn zero_capture_distinguishes_empty_from_discarded_bytes() {
    let mut client = ready(0, 0);
    for stream in [OutputStream::Stdout, OutputStream::Stderr] {
        client.accept_frame(data(stream, "")).unwrap();
    }
    assert!(!client.partial().stdout_truncated);
    assert!(!client.partial().stderr_truncated);
    client
        .accept_frame(data(OutputStream::Stdout, "QQ=="))
        .unwrap();
    assert!(client.partial().stdout_truncated);
    assert!(!client.partial().stderr_truncated);
    client
        .accept_frame(data(OutputStream::Stderr, "Qg=="))
        .unwrap();
    let result = complete(&mut client, exit());
    assert!(result.stdout.is_empty());
    assert!(result.stderr.is_empty());
    assert!(result.stdout_truncated);
    assert!(result.stderr_truncated);
}

#[test]
fn capture_limits_have_exact_boundaries_and_sticky_truncation() {
    for stream in [OutputStream::Stdout, OutputStream::Stderr] {
        for (chunks, truncated) in [
            (["QQ==", "Qg==", ""], false),
            (["QQ==", "QkM=", ""], true),
            (["QUJD", "REVG", ""], true),
        ] {
            let mut client = ready(2, 2);
            for chunk in chunks {
                client.accept_frame(data(stream, chunk)).unwrap();
            }
            let partial = client.partial();
            assert_eq!(captured(&partial, stream), (b"AB".as_slice(), truncated));
            let other = match stream {
                OutputStream::Stdout => OutputStream::Stderr,
                _ => OutputStream::Stdout,
            };
            assert_eq!(captured(&partial, other), (b"".as_slice(), false));
        }
    }
}

#[test]
fn independent_capture_limits_do_not_stop_checkpoints_or_exit() {
    let mut client = ready(0, 2);
    for stream in [OutputStream::Stdout, OutputStream::Stderr] {
        client.accept_frame(data(stream, "QUJD")).unwrap();
        client.accept_frame(data(stream, "REVG")).unwrap();
    }
    assert_eq!(
        client.accept_frame(checkpoint("exec-1", H1, 4)).unwrap(),
        ExecEvent::Checkpoint(witness("exec-1", H1, 4))
    );
    let result = complete(&mut client, exit());
    assert_eq!(result.stdout, b"");
    assert_eq!(result.stderr, b"AB");
    assert!(result.stdout_truncated);
    assert!(result.stderr_truncated);
    assert_eq!(result.exit.code, Some(7));
}

#[test]
fn hostile_session_order_and_pty_fail_before_state_changes() {
    for bad in [
        data(OutputStream::Pty, "QQ=="),
        frame(STARTED_JSON),
        frame(r#"{"type":"started","session_id":"other"}"#),
    ] {
        let mut client = ready(8, 8);
        client.accept_frame(frame(STDOUT_JSON)).unwrap();
        let before = client.partial();
        assert_decode(client.accept_frame(bad).unwrap_err());
        assert_eq!(client.partial(), before);
        assert_poisoned(&mut client);
    }
    for bad in [
        frame(r#"{"type":"started","session_id":""}"#),
        frame(STDOUT_JSON),
        checkpoint("exec-1", H1, 1),
        ServerFrame::Exit(exit()),
    ] {
        let mut client = client(8, 8);
        client.accept_handshake(ack()).unwrap();
        let before = client.partial();
        assert_decode(client.accept_frame(bad).unwrap_err());
        assert_eq!(client.partial(), before);
        assert_decode(client.accept_frame(frame(STARTED_JSON)).unwrap_err());
        assert_eq!(client.partial(), before);
        assert_poisoned(&mut client);
    }
}

#[test]
fn directly_constructed_frames_still_require_validation() {
    let mut conflicting = exit();
    conflicting.signal = Some(15);
    for bad in [
        data(OutputStream::Stdout, "!!!!"),
        data(OutputStream::Stdout, "AP9hCg"),
        data(OutputStream::Stdout, "Zh=="),
        checkpoint("exec-1", "ABC", 4),
        checkpoint("exec-1", H1, 0),
        ServerFrame::Exit(conflicting),
        ServerFrame::Exit(SessionExit {
            events: 1,
            ..exit()
        }),
        ServerFrame::Exit(SessionExit {
            recording_head: H1.to_uppercase(),
            ..exit()
        }),
        ServerFrame::Exit(SessionExit {
            recording_head: "0".into(),
            ..exit()
        }),
    ] {
        let mut client = ready(8, 8);
        client.accept_frame(frame(STDOUT_JSON)).unwrap();
        let before = client.partial();
        assert_decode(client.accept_frame(bad).unwrap_err());
        assert_eq!(client.partial(), before);
        assert_poisoned(&mut client);
    }
    // 21,846 complete base64 quanta decode to 65,538 bytes, above the raw cap.
    let mut client = ready(8, 8);
    let before = client.partial();
    assert!(matches!(
        client.accept_frame(data(OutputStream::Stdout, &"AAAA".repeat(21_846))),
        Err(OpsProtoError::FrameTooLarge {
            size: 65_538,
            max: 65_536
        })
    ));
    assert_eq!(client.partial(), before);
    assert_poisoned(&mut client);
}

#[test]
fn checkpoints_require_monotonic_identity_and_identical_repeats_are_progress() {
    let mut client = ready(8, 8);
    assert_eq!(
        client.accept_frame(checkpoint("exec-1", H1, 4)).unwrap(),
        ExecEvent::Checkpoint(witness("exec-1", H1, 4))
    );
    assert_eq!(
        client.accept_frame(checkpoint("exec-1", H1, 4)).unwrap(),
        ExecEvent::Progress
    );
    assert_eq!(
        client
            .accept_frame(checkpoint("exec-1", H2, u64::MAX))
            .unwrap(),
        ExecEvent::Checkpoint(witness("exec-1", H2, u64::MAX))
    );
    assert_eq!(
        client.partial().checkpoint,
        Some(witness("exec-1", H2, u64::MAX))
    );
}

#[test]
fn bad_checkpoint_cannot_replace_the_last_valid_partial_state() {
    for bad in [
        checkpoint("other", H2, 5),
        checkpoint("exec-1", H1, 3),
        checkpoint("exec-1", H2, 4),
        checkpoint("exec-1", H2, 0),
        checkpoint("exec-1", &H2.to_uppercase(), 5),
        checkpoint("exec-1", "0", 5),
    ] {
        let mut client = ready(8, 8);
        client.accept_frame(frame(STDOUT_JSON)).unwrap();
        client.accept_frame(checkpoint("exec-1", H1, 4)).unwrap();
        let before = client.partial();
        assert_decode(client.accept_frame(bad).unwrap_err());
        assert_eq!(client.partial(), before);
        assert_eq!(client.partial().checkpoint, Some(witness("exec-1", H1, 4)));
        assert_poisoned(&mut client);
    }
}

#[test]
fn final_commitment_is_consistent_with_the_started_id_and_latest_checkpoint() {
    for (events, head, allowed) in [(6, H2, true), (4, H1, true), (3, H1, false), (4, H2, false)] {
        let mut client = ready(8, 8);
        client.accept_frame(checkpoint("exec-1", H1, 4)).unwrap();
        let before = client.partial();
        let final_exit = SessionExit {
            recording_head: head.into(),
            events,
            ..exit()
        };
        if allowed {
            let result = complete(&mut client, final_exit);
            assert_eq!(result.recording, witness("exec-1", head, events));
            assert_eq!(result.exit.recording_head, head);
            assert_eq!(result.exit.events, events);
        } else {
            assert_decode(
                client
                    .accept_frame(ServerFrame::Exit(final_exit))
                    .unwrap_err(),
            );
            assert_eq!(client.partial(), before);
            assert_poisoned(&mut client);
        }
    }
    let mut client = ready(8, 8);
    let result = complete(&mut client, exit());
    assert_eq!(result.recording, witness("exec-1", H1, 6));
}

#[test]
fn final_status_preserves_nonzero_signal_and_absent_status_metadata() {
    for (code, signal, reason) in [
        (Some(7), None, ExitReason::Exited),
        (None, Some(15), ExitReason::Exited),
        (None, None, ExitReason::Exited),
        (None, None, ExitReason::ClientClosed),
        (None, None, ExitReason::Disconnected),
        (None, None, ExitReason::Revoked),
        (None, None, ExitReason::ProtocolError),
        (None, None, ExitReason::RecordingFailed),
        (Some(0), None, ExitReason::CleanupFailed),
        (Some(7), None, ExitReason::CleanupFailed),
        (None, None, ExitReason::SpawnFailed),
        (None, None, ExitReason::TimedOut),
        (None, None, ExitReason::ResourceLimit),
    ] {
        let mut client = ready(8, 8);
        let result = complete(
            &mut client,
            SessionExit {
                code,
                signal,
                reason,
                ..exit()
            },
        );
        assert_eq!((result.exit.code, result.exit.signal), (code, signal));
        assert_eq!(result.exit.reason, reason);
        assert_eq!(result.recording, witness("exec-1", H1, 6));
    }
}

#[test]
fn server_and_local_truncation_are_combined_without_changing_server_metadata() {
    let cases = [
        (false, false, false),
        (false, true, true),
        (true, false, true),
        (true, true, true),
    ];
    for (local_out, server_out, expected_out) in cases {
        for (local_err, server_err, expected_err) in cases {
            let mut client = ready(0, 0);
            if local_out {
                client
                    .accept_frame(data(OutputStream::Stdout, "QQ=="))
                    .unwrap();
            }
            if local_err {
                client
                    .accept_frame(data(OutputStream::Stderr, "Qg=="))
                    .unwrap();
            }
            let result = complete(
                &mut client,
                SessionExit {
                    stdout_truncated: server_out,
                    stderr_truncated: server_err,
                    ..exit()
                },
            );
            assert_eq!(result.stdout_truncated, expected_out);
            assert_eq!(result.stderr_truncated, expected_err);
            assert_eq!(result.exit.stdout_truncated, server_out);
            assert_eq!(result.exit.stderr_truncated, server_err);
            assert_eq!(client.partial().stdout_truncated, local_out);
            assert_eq!(client.partial().stderr_truncated, local_err);
        }
    }
}

#[test]
fn partial_state_never_invents_an_exit_and_session_error_keeps_valid_evidence() {
    let empty = ExecPartial {
        session_id: None,
        stdout: Vec::new(),
        stderr: Vec::new(),
        stdout_truncated: false,
        stderr_truncated: false,
        checkpoint: None,
    };
    let mut early_error = client(3, 2);
    early_error.accept_handshake(ack()).unwrap();
    match early_error
        .accept_frame(frame(r#"{"type":"error","message":"spawn unavailable"}"#))
        .unwrap_err()
    {
        OpsProtoError::Remote { kind, message } => {
            assert_eq!(kind, "exec_session");
            assert_eq!(message, "spawn unavailable");
        }
        other => panic!("wrong pre-start error envelope: {other:?}"),
    }
    assert_eq!(early_error.partial(), empty);
    assert_decode(early_error.accept_frame(frame(STARTED_JSON)).unwrap_err());
    assert_eq!(early_error.partial(), empty);
    assert_poisoned(&mut early_error);

    let mut client = client(3, 2);
    assert_eq!(client.partial(), empty);
    client.accept_handshake(ack()).unwrap();
    assert_eq!(client.partial(), empty);
    client.accept_frame(frame(STARTED_JSON)).unwrap();
    let started = client.partial();
    assert_eq!(started.session_id.as_deref(), Some("exec-1"));
    assert!(started.stdout.is_empty());
    assert!(started.stderr.is_empty());
    assert!(started.checkpoint.is_none());
    client.accept_frame(frame(STDOUT_JSON)).unwrap();
    client.accept_frame(frame(STDERR_JSON)).unwrap();
    client.accept_frame(checkpoint("exec-1", H1, 4)).unwrap();
    let expected = ExecPartial {
        session_id: Some("exec-1".into()),
        stdout: vec![0, 255, 97],
        stderr: b"Er".to_vec(),
        stdout_truncated: true,
        stderr_truncated: true,
        checkpoint: Some(witness("exec-1", H1, 4)),
    };
    assert_eq!(client.partial(), expected);
    match client
        .accept_frame(frame(
            r#"{"type":"error","message":"recording\u001b[31m\nfailed"}"#,
        ))
        .unwrap_err()
    {
        OpsProtoError::Remote { kind, message } => {
            assert_eq!(kind, "exec_session");
            assert_eq!(message, "recording\u{1b}[31m\nfailed");
        }
        other => panic!("wrong session error envelope: {other:?}"),
    }
    assert_eq!(client.partial(), expected);
    assert_poisoned(&mut client);
}

#[test]
fn completion_moves_bytes_and_cannot_be_replayed() {
    for late_frame in [
        frame(STARTED_JSON),
        frame(STDOUT_JSON),
        checkpoint("exec-1", H1, 6),
        ServerFrame::Exit(exit()),
    ] {
        let mut completed = ready(8, 8);
        complete(&mut completed, exit());
        let retained = completed.partial();
        assert_decode(completed.accept_frame(late_frame).unwrap_err());
        assert_eq!(completed.partial(), retained);
        assert_poisoned(&mut completed);
    }
    let mut client = ready(8, 8);
    client.accept_frame(frame(STDOUT_JSON)).unwrap();
    client.accept_frame(frame(STDERR_JSON)).unwrap();
    client.accept_frame(checkpoint("exec-1", H2, 4)).unwrap();
    let result = complete(&mut client, exit());
    assert_eq!(result.stdout, [0, 255, 97, 10]);
    assert_eq!(result.stderr, b"Err");
    assert_eq!(result.recording, witness("exec-1", H1, 6));
    let partial = client.partial();
    assert!(partial.stdout.is_empty());
    assert!(partial.stderr.is_empty());
    assert_eq!(partial.session_id.as_deref(), Some("exec-1"));
    assert_eq!(partial.checkpoint, Some(witness("exec-1", H2, 4)));
    assert_decode(client.accept_handshake(ack()).unwrap_err());
    assert_poisoned(&mut client);
    assert_eq!(result.stdout, [0, 255, 97, 10]);
    assert_eq!(result.stderr, b"Err");
}

#[test]
fn partial_is_an_independent_snapshot_of_validated_state() {
    let mut client = ready(8, 8);
    client.accept_frame(frame(STDOUT_JSON)).unwrap();
    client.accept_frame(checkpoint("exec-1", H1, 4)).unwrap();
    let mut snapshot = client.partial();
    snapshot.stdout.clear();
    snapshot.stderr.extend_from_slice(b"injected");
    snapshot.session_id = Some("other".into());
    snapshot.checkpoint.as_mut().unwrap().events = 99;
    let retained = client.partial();
    assert_eq!(retained.stdout, [0, 255, 97, 10]);
    assert!(retained.stderr.is_empty());
    assert_eq!(retained.session_id.as_deref(), Some("exec-1"));
    assert_eq!(retained.checkpoint, Some(witness("exec-1", H1, 4)));
}
