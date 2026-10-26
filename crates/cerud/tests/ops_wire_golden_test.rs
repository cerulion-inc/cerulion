// SPDX-License-Identifier: AGPL-3.0-only
//! Golden `cerulion/ops/1` frames: the encoder must emit, and the decoder must
//! accept, the exact bytes in `tests/fixtures/ops_wire_golden.txt`.
//!
//! The fixture's hex was built from hand-written JSON text plus a big-endian
//! `u32` length prefix, never from this crate's encoder, so a change to the
//! framing, to a serde field name or tag, or to field order fails here by
//! name. Each data line is `<case> <hex>`; the `#` line above it carries the
//! same payload as text, and [`the_fixture_hex_is_the_commented_json_behind_a_big_endian_length`]
//! holds the two together.

use std::collections::BTreeMap;
use std::fmt::Debug;
use std::io::Cursor;

use serde::de::DeserializeOwned;
use serde::Serialize;

use cerud::protocol::{self, HandshakeReply, Hello, Request, Response};
use cerud::verbs::inventory::Inventory;

const FIXTURE: &str = include_str!("fixtures/ops_wire_golden.txt");

/// One fixture case: the whole frame, and the payload text from its comment.
struct Case {
    frame: Vec<u8>,
    commented_json: String,
}

fn decode_hex(hex: &str) -> Vec<u8> {
    assert!(hex.len() % 2 == 0, "odd-length hex in the fixture: {hex}");
    (0..hex.len())
        .step_by(2)
        .map(|i| u8::from_str_radix(&hex[i..i + 2], 16).expect("fixture hex digit"))
        .collect()
}

/// Parse the fixture into `case -> Case`. A comment line of the form
/// `# <case> (<n> payload bytes): <json>` precedes each data line.
fn fixture() -> BTreeMap<String, Case> {
    let mut cases = BTreeMap::new();
    let mut pending: Option<(String, String)> = None;
    for line in FIXTURE.lines() {
        if let Some(comment) = line.strip_prefix("# ") {
            if let Some((head, json)) = comment.split_once(": {") {
                let name = head
                    .split_whitespace()
                    .next()
                    .expect("case name")
                    .to_string();
                pending = Some((name, format!("{{{json}")));
            }
            continue;
        }
        if line.trim().is_empty() {
            continue;
        }
        let (name, hex) = line.split_once(' ').expect("data line is `<case> <hex>`");
        let (commented, json) = pending
            .take()
            .expect("a `#` payload line precedes each case");
        assert_eq!(
            commented, name,
            "comment and data line name different cases"
        );
        let previous = cases.insert(
            name.to_string(),
            Case {
                frame: decode_hex(hex),
                commented_json: json,
            },
        );
        assert!(previous.is_none(), "case {name} appears twice");
    }
    assert!(pending.is_none(), "a payload comment has no data line");
    cases
}

/// The fixture frame for `case`.
fn golden(case: &str) -> Vec<u8> {
    fixture()
        .remove(case)
        .unwrap_or_else(|| panic!("no fixture case named {case}"))
        .frame
}

/// Encode `value` as one frame and require the fixture's bytes; then read the
/// fixture's frame back and require `value`.
fn assert_golden<T>(case: &str, value: &T)
where
    T: Serialize + DeserializeOwned + PartialEq + Debug,
{
    let expected = golden(case);

    let mut written = Vec::new();
    let payload = protocol::encode(value).expect("encode");
    protocol::write_frame(&mut written, &payload).expect("write_frame");
    assert_eq!(
        written,
        expected,
        "{case}: encoder bytes differ from the golden frame\n encoder: {}\n golden:  {}",
        String::from_utf8_lossy(&written[4.min(written.len())..]),
        String::from_utf8_lossy(&expected[4..]),
    );

    let mut cursor = Cursor::new(expected);
    let frame = protocol::read_frame(&mut cursor).expect("read_frame");
    let back: T = protocol::decode(&frame).expect("decode");
    assert_eq!(
        &back, value,
        "{case}: golden frame decodes to a different value"
    );
}

#[test]
fn the_fixture_hex_is_the_commented_json_behind_a_big_endian_length() {
    let cases = fixture();
    assert_eq!(cases.len(), 10, "fixture case count changed");
    for (name, case) in &cases {
        let (prefix, payload) = case.frame.split_at(4);
        let declared = u32::from_be_bytes(prefix.try_into().unwrap()) as usize;
        assert_eq!(declared, payload.len(), "{name}: length prefix");
        assert_eq!(
            payload,
            case.commented_json.as_bytes(),
            "{name}: hex payload differs from its comment"
        );
    }
}

#[test]
fn frame_length_prefix_is_big_endian() {
    // 300 = 0x0000012c: big-endian puts the 0x01 at index 2 and 0x2c last.
    let payload = vec![b'x'; 300];
    let mut buf = Vec::new();
    protocol::write_frame(&mut buf, &payload).unwrap();
    assert_eq!(&buf[..4], &[0x00, 0x00, 0x01, 0x2c]);

    let mut framed = vec![0x00, 0x00, 0x01, 0x2c];
    framed.extend_from_slice(&payload);
    let read = protocol::read_frame(&mut Cursor::new(framed)).unwrap();
    assert_eq!(read, payload);
}

#[test]
fn golden_hello_frames() {
    assert_golden("hello_current", &Hello::current());
    assert_golden(
        "hello_range",
        &Hello {
            min_version: 2,
            max_version: 3,
        },
    );
}

#[test]
fn golden_handshake_reply_frames() {
    assert_golden(
        "handshake_ack",
        &HandshakeReply::Ack {
            chosen_version: 1,
            server_name: "cerud".to_string(),
        },
    );
    assert_golden(
        "handshake_reject",
        &HandshakeReply::Reject {
            server_min: 1,
            server_max: 1,
            message: "protocol version mismatch: server supports 1..=1, client offered 2..=3"
                .to_string(),
        },
    );
}

#[test]
fn golden_request_frames() {
    assert_golden(
        "request_inventory",
        &Request {
            id: 0,
            verb: "inventory".to_string(),
            args: serde_json::Value::Null,
        },
    );
    assert_golden(
        "request_log_tail",
        &Request {
            id: 7,
            verb: "log-tail".to_string(),
            args: serde_json::json!({"lines": 50, "name": "app.log"}),
        },
    );
    assert_golden(
        "request_restart",
        &Request {
            id: 2,
            verb: "restart".to_string(),
            args: serde_json::json!({"graph": "demo"}),
        },
    );
}

#[test]
fn a_request_without_args_decodes_to_null_args() {
    let back: Request = protocol::decode(br#"{"id":5,"verb":"inventory"}"#).unwrap();
    assert_eq!(back.args, serde_json::Value::Null);
}

#[test]
fn golden_response_frames() {
    assert_golden(
        "response_ok",
        &Response::Ok {
            id: 7,
            result: serde_json::json!({"lines": ["boot ok"], "truncated": false}),
        },
    );
    assert_golden(
        "response_err",
        &Response::Err {
            id: 3,
            kind: "denied".to_string(),
            message: "authorization denied for caller 'desk' verb 'restart': no grant".to_string(),
        },
    );
}

#[test]
fn golden_inventory_frame() {
    assert_golden(
        "inventory",
        &Inventory {
            arch: "aarch64".to_string(),
            os: "linux".to_string(),
            kernel: Some("6.8.0".to_string()),
            glibc: Some("2.39".to_string()),
            cuda: None,
            jetpack_l4t: None,
            disk_free_bytes: Some(1_048_576),
            cerulion_version: "1.0.0".to_string(),
        },
    );
}
