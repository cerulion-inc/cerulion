// SPDX-License-Identifier: AGPL-3.0-only
//! Protocol framing + version-negotiation oracle tests.

use std::io::Cursor;

use cerud::error::CerudError;
use cerud::protocol::{
    self, negotiate_version, Hello, Request, Response, MAX_FRAME_BYTES, MIN_PROTOCOL_VERSION,
    PROTOCOL_VERSION,
};

#[test]
fn frame_roundtrip_preserves_payload() {
    let payload = b"hello cerud, this is a framed payload".to_vec();
    let mut buf = Vec::new();
    protocol::write_frame(&mut buf, &payload).unwrap();
    // 4-byte length prefix + payload.
    assert_eq!(buf.len(), 4 + payload.len());
    assert_eq!(&buf[..4], &(payload.len() as u32).to_be_bytes());

    let mut cursor = Cursor::new(buf);
    let read = protocol::read_frame(&mut cursor).unwrap();
    assert_eq!(read, payload);
}

#[test]
fn empty_frame_roundtrips() {
    let mut buf = Vec::new();
    protocol::write_frame(&mut buf, &[]).unwrap();
    let mut cursor = Cursor::new(buf);
    let read = protocol::read_frame(&mut cursor).unwrap();
    assert!(read.is_empty());
}

#[test]
fn clean_eof_is_connection_closed() {
    // Nothing on the wire → a clean close, not an I/O error.
    let mut cursor = Cursor::new(Vec::new());
    match protocol::read_frame(&mut cursor) {
        Err(CerudError::ConnectionClosed) => {}
        other => panic!("expected ConnectionClosed, got {other:?}"),
    }
}

#[test]
fn oversized_declared_length_refused_without_allocating() {
    // A hostile length prefix of MAX+1 must be refused BEFORE allocating.
    let bogus_len = (MAX_FRAME_BYTES as u64 + 1) as u32;
    let mut bytes = bogus_len.to_be_bytes().to_vec();
    bytes.push(0); // one stray body byte; we should never get this far
    let mut cursor = Cursor::new(bytes);
    match protocol::read_frame(&mut cursor) {
        Err(CerudError::FrameTooLarge { size, max }) => {
            assert_eq!(size, MAX_FRAME_BYTES + 1);
            assert_eq!(max, MAX_FRAME_BYTES);
        }
        other => panic!("expected FrameTooLarge, got {other:?}"),
    }
}

#[test]
fn truncated_frame_mid_payload_is_an_io_error_not_a_clean_close() {
    // A length prefix claiming 10 bytes, but only 3 body bytes on the wire: a
    // TRUNCATED frame (crash/partial write), not a clean connection close.
    let mut bytes = 10u32.to_be_bytes().to_vec();
    bytes.extend_from_slice(b"abc"); // 3 of the promised 10
    let mut cursor = Cursor::new(bytes);
    match protocol::read_frame(&mut cursor) {
        // A mid-payload EOF is a genuine I/O error, distinct from the clean
        // EOF-at-length-prefix ConnectionClosed.
        Err(CerudError::Io(_)) => {}
        other => panic!("expected an Io (truncated frame) error, got {other:?}"),
    }
}

#[test]
fn write_frame_refuses_oversized_payload() {
    let mut sink = Vec::new();
    let payload = vec![0u8; MAX_FRAME_BYTES + 1];
    match protocol::write_frame(&mut sink, &payload) {
        Err(CerudError::FrameTooLarge { size, max }) => {
            assert_eq!(size, MAX_FRAME_BYTES + 1);
            assert_eq!(max, MAX_FRAME_BYTES);
        }
        other => panic!("expected FrameTooLarge, got {other:?}"),
    }
    // Nothing should have been written.
    assert!(sink.is_empty());
}

#[test]
fn negotiate_version_picks_highest_common() {
    // Exact overlap.
    assert_eq!(negotiate_version(1, 1, 1, 1).unwrap(), 1);
    // Both support a range → the highest common version.
    assert_eq!(negotiate_version(1, 5, 2, 3).unwrap(), 3);
    assert_eq!(negotiate_version(2, 4, 1, 7).unwrap(), 4);
    // Server single, client wide.
    assert_eq!(negotiate_version(1, 9, 4, 4).unwrap(), 4);
}

#[test]
fn negotiate_version_refuses_disjoint_ranges() {
    // Client only speaks a newer version than the server supports.
    match negotiate_version(2, 3, 1, 1) {
        Err(CerudError::VersionMismatch {
            client_min,
            client_max,
            server_min,
            server_max,
        }) => {
            assert_eq!(
                (client_min, client_max, server_min, server_max),
                (2, 3, 1, 1)
            );
        }
        other => panic!("expected VersionMismatch, got {other:?}"),
    }
    // Client only speaks an older version than the server supports.
    assert!(matches!(
        negotiate_version(1, 1, 5, 9),
        Err(CerudError::VersionMismatch { .. })
    ));
}

#[test]
fn negotiate_version_rejects_inverted_client_range() {
    match negotiate_version(5, 2, 1, 9) {
        Err(CerudError::Decode(msg)) => assert!(msg.contains("inverted")),
        other => panic!("expected Decode, got {other:?}"),
    }
}

#[test]
fn hello_current_advertises_build_range() {
    let hello = Hello::current();
    assert_eq!(hello.min_version, MIN_PROTOCOL_VERSION);
    assert_eq!(hello.max_version, PROTOCOL_VERSION);
}

#[test]
fn request_response_envelopes_roundtrip_through_json() {
    let req = Request {
        id: 42,
        verb: "inventory".to_string(),
        args: serde_json::json!({"k": "v"}),
    };
    let bytes = protocol::encode(&req).unwrap();
    let back: Request = protocol::decode(&bytes).unwrap();
    assert_eq!(back, req);

    let resp = Response::Ok {
        id: 42,
        result: serde_json::json!({"arch": "aarch64"}),
    };
    let bytes = protocol::encode(&resp).unwrap();
    let back: Response = protocol::decode(&bytes).unwrap();
    assert_eq!(back, resp);
}

#[test]
fn response_from_error_stamps_machine_kind() {
    let err = CerudError::UnknownVerb("frobnicate".to_string());
    match Response::from_error(7, &err) {
        Response::Err { id, kind, message } => {
            assert_eq!(id, 7);
            assert_eq!(kind, "unknown_verb");
            assert!(message.contains("frobnicate"));
        }
        other => panic!("expected Err response, got {other:?}"),
    }
}
