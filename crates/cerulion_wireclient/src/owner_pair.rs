// SPDX-License-Identifier: AGPL-3.0-only
//! Bounded owner-certificate exchange on an already authenticated ops stream.
//!
//! This client mirrors the small version-1 ops envelope so publishable desk
//! crates do not depend on the robot server. Integration tests exercise it against
//! that server. The caller owns transport deadlines and must close a timed-out
//! connection; a partially read frame must never be reused.

use std::io::{Read, Write};

use cerulion_pairing::verify::OwnerCertificatePresentationWire;
use serde::de::DeserializeOwned;
use serde::{Deserialize, Serialize};

use crate::epoch::sanitize_peer_text;
use crate::{ConnectError, ConnectResult};

/// Maximum request or reply payload for the certificate-only pairing exchange.
pub const OWNER_PAIR_MAX_FRAME_BYTES: usize = 64 * 1024;

#[derive(Serialize)]
struct Hello {
    min_version: u16,
    max_version: u16,
}

#[derive(Deserialize)]
#[serde(tag = "type", rename_all = "snake_case")]
enum HandshakeReply {
    Ack {
        chosen_version: u16,
        #[serde(rename = "server_name")]
        _server_name: String,
    },
    Reject {
        #[serde(rename = "server_min")]
        _server_min: u16,
        #[serde(rename = "server_max")]
        _server_max: u16,
        message: String,
    },
}

#[derive(Serialize)]
struct PairArgs<'a> {
    owner_certificate_postcard: &'a str,
}

#[derive(Serialize)]
struct PairRequest<'a> {
    id: u64,
    verb: &'static str,
    args: PairArgs<'a>,
}

#[derive(Deserialize)]
struct PairResult {
    paired: bool,
    account: String,
    source: String,
}

#[derive(Deserialize)]
#[serde(tag = "status", rename_all = "snake_case")]
enum PairResponse {
    Ok {
        id: u64,
        result: PairResult,
    },
    Err {
        id: u64,
        #[serde(rename = "kind")]
        _kind: String,
        message: String,
    },
}

/// Present one owner certificate to the pinned robot over its ops stream.
///
/// The stream must authenticate the same desk key as the certificate. This
/// function does not retry or fall back to code pairing. The robot verifies the
/// certificate chain, ownership, validity, and revocation before admitting it.
/// Run on a blocking thread when the stream wraps an asynchronous transport.
pub fn present_owner_certificate<S: Read + Write>(
    stream: &mut S,
    presentation: &OwnerCertificatePresentationWire,
) -> ConnectResult<()> {
    let bytes = presentation
        .to_postcard()
        .map_err(|e| ConnectError::DeviceCert(e.to_string()))?;
    pair_encoded(
        stream,
        &hex::encode(bytes),
        &hex::encode(presentation.device_cert.cert.account.0),
    )
}

fn pair_encoded<S: Read + Write>(
    stream: &mut S,
    presentation_hex: &str,
    expected_account: &str,
) -> ConnectResult<()> {
    write_json(
        stream,
        &Hello {
            min_version: 1,
            max_version: 1,
        },
    )?;
    match read_json::<HandshakeReply>(stream)? {
        HandshakeReply::Ack {
            chosen_version: 1, ..
        } => {}
        HandshakeReply::Ack { chosen_version, .. } => {
            return Err(ConnectError::Protocol(format!(
                "ops server selected unsupported version {chosen_version}"
            )));
        }
        HandshakeReply::Reject { message, .. } => {
            return Err(ConnectError::Control(format!(
                "owner certificate handshake refused: {}",
                sanitize_peer_text(&message)
            )));
        }
    }
    write_json(
        stream,
        &PairRequest {
            id: 0,
            verb: "pair",
            args: PairArgs {
                owner_certificate_postcard: presentation_hex,
            },
        },
    )?;
    match read_json::<PairResponse>(stream)? {
        PairResponse::Ok { id: 0, result }
            if result.paired
                && result.account == expected_account
                && result.source == "strong_chain" =>
        {
            Ok(())
        }
        PairResponse::Ok { id, .. } | PairResponse::Err { id, .. } if id != 0 => Err(
            ConnectError::Protocol("owner certificate response id mismatch".into()),
        ),
        PairResponse::Ok { .. } => Err(ConnectError::Protocol(
            "owner certificate response did not confirm the expected account and strong chain"
                .into(),
        )),
        PairResponse::Err { message, .. } => Err(ConnectError::Control(format!(
            "owner certificate refused: {}",
            sanitize_peer_text(&message)
        ))),
    }
}

fn write_json<S: Write>(stream: &mut S, value: &impl Serialize) -> ConnectResult<()> {
    let bytes = serde_json::to_vec(value)
        .map_err(|_| ConnectError::Protocol("cannot encode owner certificate request".into()))?;
    if bytes.len() > OWNER_PAIR_MAX_FRAME_BYTES {
        return Err(ConnectError::Protocol(
            "owner certificate request exceeds frame limit".into(),
        ));
    }
    stream
        .write_all(&(bytes.len() as u32).to_be_bytes())
        .map_err(sanitized_io)?;
    stream.write_all(&bytes).map_err(sanitized_io)?;
    stream.flush().map_err(sanitized_io)?;
    Ok(())
}

fn read_json<T: DeserializeOwned>(stream: &mut impl Read) -> ConnectResult<T> {
    let mut prefix = [0; 4];
    stream.read_exact(&mut prefix).map_err(sanitized_io)?;
    let len = u32::from_be_bytes(prefix) as usize;
    if len > OWNER_PAIR_MAX_FRAME_BYTES {
        return Err(ConnectError::Protocol(
            "owner certificate reply exceeds frame limit".into(),
        ));
    }
    let mut bytes = vec![0; len];
    stream.read_exact(&mut bytes).map_err(sanitized_io)?;
    serde_json::from_slice(&bytes)
        .map_err(|_| ConnectError::Protocol("invalid owner certificate reply".into()))
}

fn sanitized_io(error: std::io::Error) -> ConnectError {
    // Transport errors may include a peer's application-close reason.
    ConnectError::Io(std::io::Error::new(
        error.kind(),
        sanitize_peer_text(&error.to_string()),
    ))
}

#[cfg(test)]
mod tests;
