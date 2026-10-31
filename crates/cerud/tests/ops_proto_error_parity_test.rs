// SPDX-License-Identifier: AGPL-3.0-only
//! `CerudError::from(OpsProtoError)` keeps the variant and the message: for
//! every `cerulion_ops_proto::OpsProtoError` variant, the converted
//! `CerudError` prints the same text and carries the `kind` string of the
//! variant of the same name.

use cerud::error::CerudError;
use cerulion_ops_proto::OpsProtoError;

fn every_variant() -> Vec<(OpsProtoError, &'static str)> {
    let json_err = serde_json::from_str::<u8>("not json").unwrap_err();
    vec![
        (
            OpsProtoError::Io(std::io::Error::other("pipe broke")),
            "internal",
        ),
        (OpsProtoError::Json(json_err), "internal"),
        (
            OpsProtoError::FrameTooLarge { size: 9, max: 8 },
            "frame_too_large",
        ),
        (
            OpsProtoError::Decode("bad frame".to_string()),
            "decode_error",
        ),
        (
            OpsProtoError::VersionMismatch {
                client_min: 2,
                client_max: 3,
                server_min: 1,
                server_max: 1,
            },
            "version_mismatch",
        ),
        (
            OpsProtoError::HandshakeRejected {
                reason: "not paired".to_string(),
                server_min: 1,
                server_max: 1,
            },
            "handshake_rejected",
        ),
        (OpsProtoError::ConnectionClosed, "connection_closed"),
        (
            OpsProtoError::Remote {
                kind: "denied".to_string(),
                message: "no grant".to_string(),
            },
            "remote_error",
        ),
    ]
}

#[test]
fn every_ops_proto_error_converts_with_the_same_text_and_kind() {
    let cases = every_variant();
    assert_eq!(cases.len(), 8, "one case per OpsProtoError variant");
    for (err, kind) in cases {
        let text = err.to_string();
        let converted = CerudError::from(err);
        assert_eq!(converted.to_string(), text, "message changed in conversion");
        assert_eq!(converted.kind(), kind, "{text}: wrong CerudError variant");
    }
}

#[test]
fn the_converted_messages_are_the_ones_cerud_printed_before_the_move() {
    // Literal oracles: the CerudError Display strings for these variants.
    let pairs = [
        (
            OpsProtoError::FrameTooLarge { size: 9, max: 8 },
            "protocol frame too large: 9 bytes exceeds the 8-byte cap",
        ),
        (
            OpsProtoError::VersionMismatch {
                client_min: 2,
                client_max: 3,
                server_min: 1,
                server_max: 1,
            },
            "protocol version mismatch: server supports 1..=1, client offered 2..=3",
        ),
        (
            OpsProtoError::HandshakeRejected {
                reason: "not paired".to_string(),
                server_min: 1,
                server_max: 1,
            },
            "handshake rejected by server (supports 1..=1): not paired",
        ),
        (OpsProtoError::ConnectionClosed, "connection closed by peer"),
        (
            OpsProtoError::Remote {
                kind: "denied".to_string(),
                message: "no grant".to_string(),
            },
            "remote error [denied]: no grant",
        ),
    ];
    for (err, expected) in pairs {
        assert_eq!(err.to_string(), expected);
        assert_eq!(CerudError::from(err).to_string(), expected);
    }
}
