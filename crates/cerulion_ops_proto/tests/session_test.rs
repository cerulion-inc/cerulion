// SPDX-License-Identifier: MIT OR Apache-2.0
use std::io::Cursor;

use cerulion_ops_proto::session::{
    decode_client_frame, decode_data, decode_server_frame, encode_data, read_session_frame,
    write_session_frame,
};
use cerulion_ops_proto::{
    negotiate_version, ExecRequest, Hello, OpsProtoError, SessionKind, ShellRequest, WindowSize,
    MAX_DATA_BYTES, MAX_SESSION_FRAME_BYTES, MIN_PROTOCOL_VERSION, PROTOCOL_VERSION,
};

#[test]
fn cleanup_failure_frame_preserves_the_leader_status() {
    let bytes = br#"{"type":"exit","code":7,"signal":null,"reason":"cleanup_failed","recording_head":"0000000000000000000000000000000000000000000000000000000000000000","events":2,"stdout_truncated":false,"stderr_truncated":false}"#;
    let frame = decode_server_frame(bytes).unwrap();
    let cerulion_ops_proto::ServerFrame::Exit(exit) = &frame else {
        panic!("expected exit frame");
    };
    assert_eq!(exit.code, Some(7));
    assert_eq!(exit.reason, cerulion_ops_proto::ExitReason::CleanupFailed);
    assert_eq!(serde_json::to_vec(&frame).unwrap(), bytes);
}

#[test]
fn live_witness_requires_a_session_count_and_canonical_hash() {
    let valid = serde_json::json!({
        "type": "witness",
        "witness": {
            "session_id": "shell-1",
            "head": "d78f2237b17c59bb9fb87b7dcf41b6b3473800ef66a0a52e3f5e16ab6ddfefca",
            "events": 1
        }
    });
    assert!(decode_server_frame(&serde_json::to_vec(&valid).unwrap()).is_ok());
    for (key, value) in [
        ("session_id", serde_json::json!("")),
        ("events", serde_json::json!(0)),
        ("events", serde_json::json!(-1)),
        ("head", serde_json::json!("0")),
        ("head", serde_json::json!("A".repeat(64))),
        ("uncommitted", serde_json::json!(true)),
    ] {
        let mut invalid = valid.clone();
        invalid["witness"][key] = value;
        assert!(decode_server_frame(&serde_json::to_vec(&invalid).unwrap()).is_err());
    }
    let mut invalid = valid;
    invalid["uncommitted"] = serde_json::json!(true);
    assert!(decode_server_frame(&serde_json::to_vec(&invalid).unwrap()).is_err());
}

#[test]
fn binary_chunks_have_standard_base64_and_exact_size_boundary() {
    assert_eq!(encode_data(&[0, 255, b'a', b'\n']).unwrap(), "AP9hCg==");
    assert_eq!(decode_data("AP9hCg==").unwrap(), [0, 255, b'a', b'\n']);
    assert_eq!(decode_data("").unwrap(), Vec::<u8>::new());
    let encoded = encode_data(&vec![0; MAX_DATA_BYTES]).unwrap();
    assert_eq!(decode_data(&encoded).unwrap().len(), MAX_DATA_BYTES);
    assert!(encode_data(&vec![0; MAX_DATA_BYTES + 1]).is_err());
    for malformed in ["AP9hCg", "AP9hCg===", "!!!!", "Zh==", "Zg==\n"] {
        assert!(decode_data(malformed).is_err(), "accepted {malformed}");
    }
    // A complete base64 quantum may decode above the raw cap while still
    // fitting the encoded cap. Both limits must be enforced.
    assert!(decode_data(&"A".repeat(MAX_DATA_BYTES.div_ceil(3) * 4)).is_err());
}

#[test]
fn session_lengths_are_bounded_before_reading_or_writing_body() {
    let mut source = Cursor::new(((MAX_SESSION_FRAME_BYTES + 1) as u32).to_be_bytes());
    assert!(matches!(
        read_session_frame(&mut source),
        Err(OpsProtoError::FrameTooLarge {
            max: MAX_SESSION_FRAME_BYTES,
            ..
        })
    ));
    assert_eq!(source.position(), 4);
    let mut sink = Vec::new();
    assert!(write_session_frame(&mut sink, &vec![0; MAX_SESSION_FRAME_BYTES + 1]).is_err());
    assert!(sink.is_empty());
    assert!(decode_client_frame(&vec![b' '; MAX_SESSION_FRAME_BYTES + 1]).is_err());
    assert!(decode_server_frame(&vec![b' '; MAX_SESSION_FRAME_BYTES + 1]).is_err());
    let bytes = vec![b'x'; MAX_SESSION_FRAME_BYTES];
    write_session_frame(&mut sink, &bytes).unwrap();
    assert_eq!(read_session_frame(&mut Cursor::new(sink)).unwrap(), bytes);
}

#[test]
fn partial_session_prefix_and_payload_are_errors_not_clean_disconnects() {
    assert!(matches!(
        read_session_frame(&mut Cursor::new([])),
        Err(OpsProtoError::ConnectionClosed)
    ));
    for bytes in [vec![0], vec![0, 0, 0], vec![0, 0, 0, 2, b'x']] {
        assert!(matches!(
            read_session_frame(&mut Cursor::new(bytes)),
            Err(OpsProtoError::Io(_))
        ));
    }
}

#[test]
fn malformed_session_parameters_and_frames_are_refused() {
    let size = WindowSize {
        rows: 24,
        cols: 80,
        pixel_width: 0,
        pixel_height: 0,
    };
    SessionKind::Shell(ShellRequest {
        term: "xterm-256color".into(),
        size,
    })
    .validate()
    .unwrap();
    SessionKind::Exec(ExecRequest {
        argv: vec!["printf".into(), String::new()],
    })
    .validate()
    .unwrap();
    for argv in [vec![], vec![String::new()], vec!["a\0b".into()]] {
        assert!(SessionKind::Exec(ExecRequest { argv }).validate().is_err());
    }
    for term in [String::new(), "bad/term".into(), "x".repeat(65)] {
        assert!(SessionKind::Shell(ShellRequest { term, size })
            .validate()
            .is_err());
    }
    assert!(WindowSize { rows: 0, ..size }.validate().is_err());
    assert!(WindowSize { cols: 0, ..size }.validate().is_err());
    for bytes in [
        br#"{"type":"stdin","data":"!!!!"}"#.as_slice(),
        br#"{"type":"resize","size":{"rows":0,"cols":80,"pixel_width":0,"pixel_height":0}}"#,
        br#"{"type":"close","unexpected":true}"#,
        br#"{"type":"stdin_eof","unexpected":true}"#,
        br#"{"type":"unknown"}"#,
    ] {
        assert!(decode_client_frame(bytes).is_err(), "{bytes:?}");
    }
    for bytes in [
        br#"{"type":"started","session_id":""}"#.as_slice(),
        br#"{"type":"data","stream":"stdout","data":"!!!!"}"#,
        br#"{"type":"exit","code":0,"signal":null,"reason":"exited","recording_head":"","events":2,"stdout_truncated":false,"stderr_truncated":false}"#,
    ] { assert!(decode_server_frame(bytes).is_err()); }
}

#[test]
fn empty_variants_reject_extra_fields_at_the_serde_boundary() {
    use cerulion_ops_proto::ClientFrame;

    for kind in ["stdin_eof", "close"] {
        let valid = format!(r#"{{"type":"{kind}"}}"#);
        assert!(serde_json::from_str::<ClientFrame>(&valid).is_ok());
        let extra = format!(r#"{{"type":"{kind}","uncommitted":true}}"#);
        assert!(serde_json::from_str::<ClientFrame>(&extra).is_err());
    }
    assert_eq!(
        serde_json::from_str::<SessionKind>(r#"{"type":"ops"}"#).unwrap(),
        SessionKind::Ops
    );
    assert!(serde_json::from_str::<SessionKind>(r#"{"type":"ops","argv":["id"]}"#).is_err());
}

#[test]
fn session_frame_bytes_match_literal_oracles_on_both_encodes() {
    use cerulion_ops_proto::session::encode_session;
    use cerulion_ops_proto::{ClientFrame, OutputStream, ServerFrame};

    let clients = [
        (
            ClientFrame::Stdin {
                data: "AP9hCg==".into(),
            },
            br#"{"type":"stdin","data":"AP9hCg=="}"#.as_slice(),
        ),
        (ClientFrame::StdinEof, br#"{"type":"stdin_eof"}"#),
        (ClientFrame::Close, br#"{"type":"close"}"#),
        (
            ClientFrame::Resize {
                size: WindowSize {
                    rows: 24,
                    cols: 80,
                    pixel_width: 0,
                    pixel_height: 0,
                },
            },
            br#"{"type":"resize","size":{"rows":24,"cols":80,"pixel_width":0,"pixel_height":0}}"#,
        ),
    ];
    for (frame, oracle) in clients {
        assert_eq!(decode_client_frame(oracle).unwrap(), frame);
        for _ in 0..2 {
            assert_eq!(encode_session(&frame).unwrap(), oracle);
        }
    }
    let servers = [
        (
            ServerFrame::Started {
                session_id: "shell-1".into(),
            },
            br#"{"type":"started","session_id":"shell-1"}"#.as_slice(),
        ),
        (
            ServerFrame::Data {
                stream: OutputStream::Stdout,
                data: "Zg==".into(),
            },
            br#"{"type":"data","stream":"stdout","data":"Zg=="}"#,
        ),
        (
            ServerFrame::Data {
                stream: OutputStream::Stderr,
                data: "Zg==".into(),
            },
            br#"{"type":"data","stream":"stderr","data":"Zg=="}"#,
        ),
        (
            ServerFrame::Data {
                stream: OutputStream::Pty,
                data: "Zg==".into(),
            },
            br#"{"type":"data","stream":"pty","data":"Zg=="}"#,
        ),
        (
            ServerFrame::Error {
                message: "session refused".into(),
            },
            br#"{"type":"error","message":"session refused"}"#,
        ),
    ];
    for (frame, oracle) in servers {
        assert_eq!(decode_server_frame(oracle).unwrap(), frame);
        for _ in 0..2 {
            assert_eq!(encode_session(&frame).unwrap(), oracle);
        }
    }
    let mut framed = Vec::new();
    write_session_frame(&mut framed, br#"{"type":"close"}"#).unwrap();
    assert_eq!(framed, b"\x00\x00\x00\x10{\"type\":\"close\"}");
}

#[test]
fn session_metadata_has_a_size_limit_and_explicit_authorization_verbs() {
    use cerulion_ops_proto::session::encode_session;

    assert_eq!(SessionKind::Ops.verb(), None);
    SessionKind::Ops.validate().unwrap();
    let exec = SessionKind::Exec(ExecRequest {
        argv: vec!["id".into()],
    });
    assert_eq!(exec.verb(), Some("exec"));
    for _ in 0..2 {
        assert_eq!(
            encode_session(&exec).unwrap(),
            br#"{"type":"exec","argv":["id"]}"#
        );
    }
    let shell: SessionKind = serde_json::from_str(r#"{"type":"shell","term":"vt100","size":{"rows":1,"cols":1,"pixel_width":0,"pixel_height":0}}"#).unwrap();
    assert_eq!(shell.verb(), Some("shell"));
    shell.validate().unwrap();
    assert!(matches!(
        SessionKind::Exec(ExecRequest {
            argv: vec!["x".repeat(MAX_SESSION_FRAME_BYTES)]
        })
        .validate(),
        Err(OpsProtoError::FrameTooLarge { .. })
    ));
    assert!(decode_data(&"A".repeat(MAX_DATA_BYTES.div_ceil(3) * 4 + 1)).is_err());
    let invalid_json_map = std::collections::BTreeMap::from([(vec![1, 2], 0)]);
    assert!(encode_session(&invalid_json_map).is_err());
}

#[test]
fn exit_commitments_refuse_inconsistent_status_and_uncommitted_fields() {
    let valid = serde_json::json!({ "type": "exit", "code": 0, "signal": null,
        "reason": "exited", "recording_head": "0".repeat(64), "events": 2,
        "stdout_truncated": false, "stderr_truncated": false });
    for (key, value) in [
        ("events", serde_json::json!(1)),
        ("recording_head", serde_json::json!("g".repeat(64))),
        ("recording_head", serde_json::json!("A".repeat(64))),
        ("signal", serde_json::json!(9)),
        ("uncommitted", serde_json::json!(true)),
    ] {
        let mut invalid = valid.clone();
        invalid[key] = value;
        assert!(decode_server_frame(&serde_json::to_vec(&invalid).unwrap()).is_err());
    }
    for bytes in [
        b"{".as_slice(),
        br#"{"type":"data","stream":"unknown","data":""}"#,
        br#"{"type":"error","message":"x","uncommitted":true}"#,
    ] {
        assert!(decode_server_frame(bytes).is_err());
    }
    assert!(decode_client_frame(b"{").is_err());
}

#[test]
fn frame_io_errors_propagate_without_becoming_clean_disconnects() {
    struct FailedIo;
    impl std::io::Read for FailedIo {
        fn read(&mut self, _: &mut [u8]) -> std::io::Result<usize> {
            Err(std::io::ErrorKind::BrokenPipe.into())
        }
    }
    impl std::io::Write for FailedIo {
        fn write(&mut self, _: &[u8]) -> std::io::Result<usize> {
            Err(std::io::ErrorKind::BrokenPipe.into())
        }
        fn flush(&mut self) -> std::io::Result<()> {
            Ok(())
        }
    }
    for result in [
        read_session_frame(&mut FailedIo).map(|_| ()),
        write_session_frame(&mut FailedIo, b"{}"),
    ] {
        let Err(OpsProtoError::Io(error)) = result else {
            panic!("expected I/O error");
        };
        assert_eq!(error.kind(), std::io::ErrorKind::BrokenPipe);
    }
}

#[test]
fn ordinary_ops_negotiates_legacy_and_current_peers_without_downgrading_sessions() {
    assert_eq!((MIN_PROTOCOL_VERSION, PROTOCOL_VERSION), (1, 2));
    assert_eq!(cerulion_ops_proto::MIN_SESSION_PROTOCOL_VERSION, 2);
    for (offered, supported, chosen) in [
        ((1, 1), (1, 2), 1),
        ((1, 2), (1, 1), 1),
        ((1, 2), (1, 2), 2),
        ((2, 2), (1, 2), 2),
    ] {
        assert_eq!(
            negotiate_version(offered.0, offered.1, supported.0, supported.1).unwrap(),
            chosen
        );
    }
    let legacy: Hello = serde_json::from_str(r#"{"min_version":1,"max_version":1}"#).unwrap();
    assert_eq!(legacy.session, SessionKind::Ops);
    assert_eq!(
        serde_json::to_string(&legacy).unwrap(),
        r#"{"min_version":1,"max_version":1}"#
    );
    assert_eq!(
        serde_json::to_string(&Hello::current()).unwrap(),
        r#"{"min_version":1,"max_version":2}"#
    );
    assert!(negotiate_version(legacy.min_version, legacy.max_version, 2, 2).is_err());
}

#[test]
fn hello_rejects_unknown_connection_and_session_fields() {
    for json in [
        r#"{"min_version":1,"max_version":2,"extra":true}"#,
        r#"{"min_version":2,"max_version":2,"session":{"type":"ops","argv":["id"]}}"#,
        r#"{"min_version":2,"max_version":2,"session":{"type":"unknown"}}"#,
    ] {
        assert!(serde_json::from_str::<Hello>(json).is_err());
    }
}

#[test]
fn client_refuses_an_ack_outside_the_offer_or_the_builds_supported_range() {
    struct Peer(Cursor<Vec<u8>>);
    impl std::io::Read for Peer {
        fn read(&mut self, bytes: &mut [u8]) -> std::io::Result<usize> {
            std::io::Read::read(&mut self.0, bytes)
        }
    }
    impl std::io::Write for Peer {
        fn write(&mut self, bytes: &[u8]) -> std::io::Result<usize> {
            Ok(bytes.len())
        }
        fn flush(&mut self) -> std::io::Result<()> {
            Ok(())
        }
    }
    for (json, offered) in [
        (
            br#"{"type":"ack","chosen_version":1,"server_name":"old"}"#.as_slice(),
            (2, 2),
        ),
        (
            br#"{"type":"ack","chosen_version":0,"server_name":"unsupported"}"#.as_slice(),
            (0, 2),
        ),
        (
            br#"{"type":"ack","chosen_version":3,"server_name":"future"}"#.as_slice(),
            (1, 3),
        ),
        (
            br#"{"type":"ack","chosen_version":2,"server_name":"current"}"#.as_slice(),
            (1, 1),
        ),
    ] {
        let mut frame = Vec::new();
        cerulion_ops_proto::write_frame(&mut frame, json).unwrap();
        assert!(matches!(
            cerulion_ops_proto::OpsClient::connect(Peer(Cursor::new(frame)), offered.0, offered.1),
            Err(OpsProtoError::Decode(_))
        ));
    }
}

#[test]
fn ordinary_client_accepts_only_supported_versions_inside_its_actual_offer() {
    struct Peer(Cursor<Vec<u8>>);
    impl std::io::Read for Peer {
        fn read(&mut self, bytes: &mut [u8]) -> std::io::Result<usize> {
            std::io::Read::read(&mut self.0, bytes)
        }
    }
    impl std::io::Write for Peer {
        fn write(&mut self, bytes: &[u8]) -> std::io::Result<usize> {
            Ok(bytes.len())
        }
        fn flush(&mut self) -> std::io::Result<()> {
            Ok(())
        }
    }
    for (json, offered, expected) in [
        (
            br#"{"type":"ack","chosen_version":1,"server_name":"old"}"#.as_slice(),
            (1, 1),
            1,
        ),
        (
            br#"{"type":"ack","chosen_version":1,"server_name":"old"}"#.as_slice(),
            (1, 2),
            1,
        ),
        (
            br#"{"type":"ack","chosen_version":2,"server_name":"current"}"#.as_slice(),
            (1, 2),
            2,
        ),
        (
            br#"{"type":"ack","chosen_version":2,"server_name":"current"}"#.as_slice(),
            (2, 2),
            2,
        ),
    ] {
        let mut frame = Vec::new();
        cerulion_ops_proto::write_frame(&mut frame, json).unwrap();
        let client =
            cerulion_ops_proto::OpsClient::connect(Peer(Cursor::new(frame)), offered.0, offered.1)
                .unwrap();
        assert_eq!(client.negotiated_version(), expected);
    }
}
