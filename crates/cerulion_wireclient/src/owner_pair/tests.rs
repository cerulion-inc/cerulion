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

const ACK: &[u8] = br#"{"type":"ack","chosen_version":1,"server_name":"robot"}"#;
const PAIRED: &[u8] =
    br#"{"status":"ok","id":0,"result":{"paired":true,"account":"owner","source":"strong_chain"}}"#;

#[test]
fn certificate_exchange_writes_one_versioned_pair_request() {
    let mut stream = ScriptedStream::replies(&[ACK, PAIRED]);
    pair_encoded(&mut stream, "0123", "owner").unwrap();
    // Literal envelope oracle: this test isolates framing from certificate crypto.
    let hello = br#"{"min_version":1,"max_version":1}"#;
    let request = br#"{"id":0,"verb":"pair","args":{"owner_certificate_postcard":"0123"}}"#;
    let mut expected = Vec::new();
    for bytes in [hello.as_slice(), request.as_slice()] {
        expected.extend_from_slice(&(bytes.len() as u32).to_be_bytes());
        expected.extend_from_slice(bytes);
    }
    assert_eq!(stream.output, expected);
}

#[test]
fn handshake_refusal_or_wrong_version_never_sends_pair() {
    for reply in [
        br#"{"type":"ack","chosen_version":2,"server_name":"robot"}"#.as_slice(),
        br#"{"type":"reject","server_min":2,"server_max":2,"message":"unsupported"}"#,
        br#"{"type":"ack","chosen_version":1,"chosen_version":2,"server_name":"robot"}"#,
        b"invalid JSON",
    ] {
        let mut stream = ScriptedStream::replies(&[reply]);
        assert!(pair_encoded(&mut stream, "0123", "owner").is_err());
        assert_eq!(&stream.output[4..], br#"{"min_version":1,"max_version":1}"#);
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
        input: Cursor::new(
            ((OWNER_PAIR_MAX_FRAME_BYTES + 1) as u32)
                .to_be_bytes()
                .to_vec(),
        ),
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
    assert_eq!(&stream.output[4..], br#"{"min_version":1,"max_version":1}"#);
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
