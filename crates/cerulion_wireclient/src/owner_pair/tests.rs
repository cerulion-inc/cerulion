// SPDX-License-Identifier: AGPL-3.0-only

use std::io::{self, Cursor, Read, Write};

use super::*;

struct ScriptedStream {
    input: Cursor<Vec<u8>>,
    output: Vec<u8>,
}

impl ScriptedStream {
    fn replies(replies: &[&[u8]]) -> Self {
        let mut input = Vec::new();
        for reply in replies {
            input.extend_from_slice(&(reply.len() as u32).to_be_bytes());
            input.extend_from_slice(reply);
        }
        Self {
            input: Cursor::new(input),
            output: Vec::new(),
        }
    }
}

impl Read for ScriptedStream {
    fn read(&mut self, bytes: &mut [u8]) -> io::Result<usize> {
        self.input.read(bytes)
    }
}

impl Write for ScriptedStream {
    fn write(&mut self, bytes: &[u8]) -> io::Result<usize> {
        self.output.extend_from_slice(bytes);
        Ok(bytes.len())
    }
    fn flush(&mut self) -> io::Result<()> {
        Ok(())
    }
}

const HELLO: &[u8] = br#"{"min_version":1,"max_version":2}"#;
const ACK: &[u8] = br#"{"type":"ack","chosen_version":2,"server_name":"robot"}"#;
const LEGACY_ACK: &[u8] = br#"{"type":"ack","chosen_version":1,"server_name":"robot"}"#;
const PAIRED: &[u8] =
    br#"{"status":"ok","id":0,"result":{"paired":true,"account":"owner","source":"strong_chain"}}"#;

#[test]
fn certificate_exchange_writes_one_versioned_pair_request() {
    for ack_bytes in [LEGACY_ACK, ACK] {
        let mut stream = ScriptedStream::replies(&[ack_bytes, PAIRED]);
        pair_encoded(&mut stream, "0123", "owner").unwrap();
        // Literal envelope oracle: this test isolates framing from certificate crypto.
        let request = br#"{"id":0,"verb":"pair","args":{"owner_certificate_postcard":"0123"}}"#;
        let mut expected = Vec::new();
        for bytes in [HELLO, request.as_slice()] {
            expected.extend_from_slice(&(bytes.len() as u32).to_be_bytes());
            expected.extend_from_slice(bytes);
        }
        assert_eq!(stream.output, expected);
    }
}

#[test]
fn owner_handshake_matches_the_shared_ordinary_ops_protocol() {
    for (ack_bytes, expected_version) in [(LEGACY_ACK, 1), (ACK, 2)] {
        let mut stream = ScriptedStream::replies(&[ack_bytes, PAIRED]);
        pair_encoded(&mut stream, "0123", "owner").unwrap();
        let bytes = cerulion_ops_proto::read_frame(&mut Cursor::new(stream.output)).unwrap();
        let hello: cerulion_ops_proto::Hello = cerulion_ops_proto::decode(&bytes).unwrap();
        assert_eq!(hello, cerulion_ops_proto::Hello::current());
        assert_eq!(hello.session, cerulion_ops_proto::SessionKind::Ops);
        let ack: cerulion_ops_proto::HandshakeReply =
            cerulion_ops_proto::decode(ack_bytes).unwrap();
        assert!(matches!(
            ack,
            cerulion_ops_proto::HandshakeReply::Ack { chosen_version, .. }
                if chosen_version == expected_version
        ));
    }
}

#[test]
fn handshake_refusal_or_wrong_version_never_sends_pair() {
    for reply in [
        br#"{"type":"ack","chosen_version":0,"server_name":"unsupported"}"#.as_slice(),
        br#"{"type":"ack","chosen_version":3,"server_name":"robot"}"#,
        br#"{"type":"reject","server_min":1,"server_max":1,"message":"unsupported"}"#,
        br#"{"type":"ack","chosen_version":2,"chosen_version":1,"server_name":"robot"}"#,
        b"invalid JSON",
    ] {
        let mut stream = ScriptedStream::replies(&[reply]);
        assert!(pair_encoded(&mut stream, "0123", "owner").is_err());
        assert_eq!(&stream.output[4..], HELLO);
    }
}

#[test]
fn mismatched_ids_and_nonconfirming_results_are_terminal() {
    for reply in [
        br#"{"status":"ok","id":1,"result":{"paired":true,"account":"owner","source":"strong_chain"}}"#.as_slice(),
        br#"{"status":"err","id":1,"kind":"denied","message":"no"}"#,
        br#"{"status":"ok","id":0,"result":{"paired":false,"account":"owner","source":"strong_chain"}}"#,
        br#"{"status":"ok","id":0,"result":{"paired":true,"account":"other","source":"strong_chain"}}"#,
        br#"{"status":"ok","id":0,"result":{"paired":true,"account":"owner","source":"code_paired"}}"#,
        br#"{"status":"ok","id":0,"result":{"paired":true,"account":"owner"}}"#,
        br#"{"status":"ok","id":0,"id":1,"result":{"paired":true,"account":"owner","source":"strong_chain"}}"#,
    ] {
        let mut stream = ScriptedStream::replies(&[ACK, reply]);
        assert!(matches!(pair_encoded(&mut stream, "0123", "owner"), Err(ConnectError::Protocol(_))));
    }
}

#[test]
fn peer_refusal_text_is_sanitized_at_the_boundary() {
    for replies in [
        vec![
            br#"{"type":"reject","server_min":2,"server_max":2,"message":"stop\u001b[2J\nnow"}"#
                .as_slice(),
        ],
        vec![
            ACK,
            br#"{"status":"err","id":0,"kind":"denied","message":"stop\u001b[2J\nnow"}"#,
        ],
    ] {
        let error =
            pair_encoded(&mut ScriptedStream::replies(&replies), "0123", "owner").unwrap_err();
        assert!(matches!(error, ConnectError::Control(_)));
        let text = error.to_string();
        assert!(text.contains("stop"));
        assert!(!text.contains('\u{1b}'));
        assert!(!text.contains('\n'));
    }
}

#[test]
fn oversize_prefix_is_rejected_before_reading_or_allocating_its_body() {
    let mut stream = ScriptedStream {
        input: Cursor::new((64 * 1024 + 1_u32).to_be_bytes().to_vec()),
        output: Vec::new(),
    };
    assert!(matches!(
        pair_encoded(&mut stream, "0123", "owner"),
        Err(ConnectError::Protocol(_))
    ));
    assert_eq!(stream.input.position(), 4);
}

#[test]
fn oversized_requests_and_truncated_replies_fail_without_another_request() {
    let mut stream = ScriptedStream::replies(&[ACK]);
    assert!(matches!(
        pair_encoded(
            &mut stream,
            &"a".repeat(OWNER_PAIR_MAX_FRAME_BYTES),
            "owner"
        ),
        Err(ConnectError::Protocol(_))
    ));
    assert_eq!(&stream.output[4..], HELLO);
    for truncated in [vec![], vec![0, 0], vec![0, 0, 0, 3, b'{']] {
        let mut stream = ScriptedStream {
            input: Cursor::new(truncated),
            output: Vec::new(),
        };
        assert!(matches!(
            pair_encoded(&mut stream, "0123", "owner"),
            Err(ConnectError::Io(_))
        ));
    }
}

#[test]
fn transport_error_text_is_sanitized_and_write_failures_propagate() {
    struct FailingStream {
        fail_write: bool,
    }
    impl Read for FailingStream {
        fn read(&mut self, _: &mut [u8]) -> io::Result<usize> {
            Err(io::Error::new(
                io::ErrorKind::ConnectionReset,
                "peer\u{1b}[2J\nclose",
            ))
        }
    }
    impl Write for FailingStream {
        fn write(&mut self, bytes: &[u8]) -> io::Result<usize> {
            if self.fail_write {
                Err(io::Error::new(
                    io::ErrorKind::BrokenPipe,
                    "peer\u{1b}[2J\nclose",
                ))
            } else {
                Ok(bytes.len())
            }
        }
        fn flush(&mut self) -> io::Result<()> {
            Ok(())
        }
    }
    for fail_write in [false, true] {
        let error = pair_encoded(&mut FailingStream { fail_write }, "0123", "owner").unwrap_err();
        assert!(matches!(error, ConnectError::Io(_)));
        assert!(!error.to_string().contains('\u{1b}'));
        assert!(!error.to_string().contains('\n'));
    }
}

#[test]
fn changed_identity_after_ack_writes_only_hello_and_never_pair() {
    for ack_bytes in [LEGACY_ACK, ACK] {
        let mut stream = ScriptedStream::replies(&[ack_bytes, PAIRED]);
        let mut checks = 0;
        let result = pair_encoded_with_check(&mut stream, "0123", "owner", || {
            checks += 1;
            Err(ConnectError::Protocol("identity changed".into()))
        });
        assert!(
            matches!(result, Err(ConnectError::Protocol(message)) if message == "identity changed")
        );
        assert_eq!(checks, 1);
        let mut expected = (HELLO.len() as u32).to_be_bytes().to_vec();
        expected.extend_from_slice(HELLO);
        assert_eq!(stream.output, expected);
        // Only the acknowledgement was consumed: no pairing reply or request exists.
        assert_eq!(stream.input.position(), 4 + ack_bytes.len() as u64);
    }
}

#[test]
fn identity_guard_runs_once_only_after_a_supported_ack() {
    for ack_bytes in [LEGACY_ACK, ACK] {
        let mut stream = ScriptedStream::replies(&[ack_bytes, PAIRED]);
        let mut checks = 0;
        pair_encoded_with_check(&mut stream, "0123", "owner", || {
            checks += 1;
            Ok(())
        })
        .unwrap();
        assert_eq!(checks, 1);
        let request = br#"{"id":0,"verb":"pair","args":{"owner_certificate_postcard":"0123"}}"#;
        let mut expected = Vec::new();
        for bytes in [HELLO, request.as_slice()] {
            expected.extend_from_slice(&(bytes.len() as u32).to_be_bytes());
            expected.extend_from_slice(bytes);
        }
        assert_eq!(stream.output, expected);
    }
    for reply in [
        br#"{"type":"ack","chosen_version":0,"server_name":"unsupported"}"#.as_slice(),
        br#"{"type":"reject","server_min":2,"server_max":2,"message":"denied"}"#,
        b"invalid JSON",
    ] {
        let mut stream = ScriptedStream::replies(&[reply]);
        let result = pair_encoded_with_check(&mut stream, "0123", "owner", || {
            panic!("identity guard ran without a supported Ack")
        });
        assert!(result.is_err());
        assert_eq!(&stream.output[4..], HELLO);
    }
}
