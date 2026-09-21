// SPDX-License-Identifier: AGPL-3.0-only
//! The desk-side MIRROR of the robot's `cerulion/wire/1` control protocol.
//!
//! # Why mirror instead of importing `cerulion_remoted::wire`
//!
//! The robot serves this vocabulary from `cerulion_remoted::wire` — but
//! depending on `cerulion_remoted` in the PRODUCTION build would drag `cerud`
//! (the robot ops server) into the desk client, which has no business carrying
//! the robot's ops-server implementation. So the request/response shapes are
//! mirrored HERE (the desk client's production dep graph stays lean: no `cerud`),
//! and a dev-dependency SHAPE-PARITY test (`tests/protocol_parity_test.rs`)
//! round-trips bytes between these types and `cerulion_remoted`'s serve types on
//! every test run — so a drift on EITHER side fails loudly.
//!
//! The payload types that carry structured data — [`CatalogReply`],
//! [`SchemaReply`] — are the SHARED `cerulion_core::transport::cerulion_q` types
//! (imported, not mirrored), so those halves cannot diverge by construction.

use cerulion_core::transport::cerulion_q::{CatalogReply, SchemaReply};
use serde::{Deserialize, Serialize};

/// The first frame of every per-topic uni data stream: names the topic the
/// stream carries, so the desk correlates stream ↔ topic. All subsequent frames
/// on the stream are raw Cerulion wire frames (32-byte `WireHeader` + payload).
///
/// Mirror of `cerulion_remoted::StreamPreamble`.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct StreamPreamble {
    /// The canonical absolute topic name the following frames belong to.
    pub topic: String,
}

/// A control-stream request (desk → robot), length-prefixed JSON. The `verb` tag
/// dispatches; each verb is answered with a [`WireResponse`].
///
/// Mirror of `cerulion_remoted::WireRequest` (the `#[serde(tag = "verb")]`
/// snake_case shape must stay byte-identical — pinned by the parity test).
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[serde(tag = "verb", rename_all = "snake_case")]
pub enum WireRequest {
    /// Return the robot's full topic catalog ([`CatalogReply`]).
    Catalog,
    /// Attach a tap for `topic` + open its robot→desk uni data stream.
    Demand {
        /// The canonical absolute topic to stream.
        topic: String,
    },
    /// Drop the tap for `topic` (its uni stream ends).
    Undemand {
        /// The canonical absolute topic to stop streaming.
        topic: String,
    },
    /// Serve `topic`'s `.msg`/YAML closure ([`SchemaReply`]) so a schema-less desk
    /// can decode its frames.
    Schema {
        /// The canonical absolute topic whose type closure is requested.
        topic: String,
    },
    /// Return the per-topic WAN counters ([`StatusReply`]).
    Status,
    /// PUSH the robot's latest access-list revocation epoch on connect
    /// (the desk-push sync, by design). `epoch_postcard` is
    /// `hex(postcard(cerulion_pairing::verify::EpochSyncWire))`.
    ///
    /// Mirror of `cerulion_remoted::WireRequest::SyncEpoch`.
    SyncEpoch {
        /// `hex(postcard(EpochSyncWire))` — the intermediate + the signed epoch.
        epoch_postcard: String,
    },
}

/// A control-stream response (robot → desk), length-prefixed JSON. The `reply`
/// tag mirrors the request verb.
///
/// Mirror of `cerulion_remoted::WireResponse`. The `Catalog` / `Schema` payloads
/// are the SHARED `cerulion_q` types, so those arms cannot diverge.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(tag = "reply", rename_all = "snake_case")]
pub enum WireResponse {
    /// The `catalog` reply — the shared `cerulion_q` [`CatalogReply`] shape.
    Catalog(CatalogReply),
    /// A `demand` was accepted: the topic's uni data stream is now open.
    DemandAccepted {
        /// The topic now streaming.
        topic: String,
    },
    /// An `undemand` result: `was_demanded` is `true` if a live tap was torn down.
    Undemanded {
        /// The topic.
        topic: String,
        /// Whether a live tap existed and was dropped.
        was_demanded: bool,
    },
    /// The `schema` reply — the shared `cerulion_q` [`SchemaReply`] shape.
    Schema(SchemaReply),
    /// The `status` reply — per-topic WAN counters + Hz.
    Status(StatusReply),
    /// The `sync_epoch` result — the robot's CURRENT epoch after the call,
    /// and whether this push was newer and took effect (`false` = a stale no-op,
    /// the expected steady state once the robot is already up to date).
    ///
    /// Mirror of `cerulion_remoted::WireResponse::EpochSynced`.
    EpochSynced {
        /// The robot's current epoch number after this call.
        epoch: u64,
        /// Whether this push was newer and took effect (`false` = stale no-op).
        applied: bool,
    },
    /// A verb-level error (unknown/silent-topic demand, malformed request). Never
    /// silent — always surfaced here.
    Error {
        /// The topic the error concerns, when applicable.
        topic: Option<String>,
        /// The actionable human-readable message (names the topic where relevant).
        message: String,
    },
}

/// The `status` reply payload: one row per demanded topic.
///
/// Mirror of `cerulion_remoted::StatusReply`.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct StatusReply {
    /// Per-topic WAN counter rows, sorted by topic.
    pub topics: Vec<TopicStatus>,
}

/// One demanded topic's WAN counters + free wire-timestamp Hz + live/dead state.
///
/// Mirror of `cerulion_remoted::TopicStatus` (kept field-for-field — the parity
/// test fails if the robot's shape drifts).
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct TopicStatus {
    /// The canonical absolute topic name.
    pub topic: String,
    /// Frames written to QUIC by the robot.
    pub wan_forwarded: u64,
    /// Frames dropped at the robot's bounded drop-to-live queue.
    pub wan_dropped: u64,
    /// Frames read off SHM by the robot.
    pub frames_seen: u64,
    /// The topic's publish rate in Hz (0.0 until ≥2 timestamped frames span a
    /// positive interval).
    pub hz: f64,
    /// Whether the topic's forward stream is still LIVE (`false` once its
    /// uni-stream writer has died — the robot tore the tap down; a re-demand
    /// re-attaches). Surfaced so `status` never misreports a dead stream.
    pub stream_alive: bool,
}

/// The accept-time routing decision the robot writes on the skeleton control
/// frame when it REFUSES a wire connection (unpaired / unclaimed / unknown ALPN).
///
/// A partial MIRROR of `cerulion_remoted::AcceptDecision` — only the arms the
/// desk must recognize as a refusal. The distinct `#[serde(tag = "decision")]`
/// (vs [`WireResponse`]'s `"reply"` tag) lets the desk unambiguously tell an
/// admit-reply from a refusal on the FIRST control read (see
/// [`decode_first_reply`]). Pinned against the real enum by the parity test.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[serde(tag = "decision", rename_all = "snake_case")]
pub enum AcceptDecision {
    /// Ops plane, full surface — never seen on the wire ALPN.
    OpsAdmit,
    /// Ops plane, bootstrap surface only — never seen on the wire ALPN.
    OpsBootstrapOnly,
    /// Wire plane admitted (a paired `CAP_OBSERVE` account). Seen only if the
    /// robot mis-routes; the desk treats it as "admitted" and proceeds.
    WireAdmit,
    /// Refused outright (unpaired/unclaimed). Carries the robot's stated reason.
    Refuse {
        /// The diagnosable reason (surfaced verbatim to the operator).
        reason: String,
    },
    /// An ALPN the daemon does not serve.
    UnknownAlpn {
        /// The negotiated ALPN bytes.
        alpn: Vec<u8>,
    },
}

/// Classify a robot's FIRST control-stream reply. On an ADMITTED wire connection
/// the robot answers a [`WireResponse`]; on a REFUSED one the daemon's skeleton
/// path answers an [`AcceptDecision`] instead (a distinct `"decision"` tag). This
/// tries the wire response first, then the refusal decision, so the desk can tell
/// them apart WITHOUT guessing.
///
/// Returns:
/// - `Ok(response)` — a decoded [`WireResponse`] (the connection is admitted);
/// - `Err(Some(reason))` — an explicit refusal (or unknown-ALPN / unexpected
///   ops decision) with a human reason;
/// - `Err(None)` — the bytes decode as NEITHER (a protocol mismatch).
pub fn decode_first_reply(bytes: &[u8]) -> Result<WireResponse, Option<String>> {
    if let Ok(response) = serde_json::from_slice::<WireResponse>(bytes) {
        return Ok(response);
    }
    match decode_refusal(bytes) {
        Some(reason) => Err(Some(reason)),
        None => Err(None),
    }
}

/// If `bytes` is an [`AcceptDecision`] describing a NON-admit outcome, return the
/// human reason; otherwise `None`. A [`AcceptDecision::WireAdmit`] returns `None`
/// (it is not a refusal). Used only after a [`WireResponse`] decode has failed.
fn decode_refusal(bytes: &[u8]) -> Option<String> {
    match serde_json::from_slice::<AcceptDecision>(bytes).ok()? {
        AcceptDecision::Refuse { reason } => Some(reason),
        AcceptDecision::UnknownAlpn { alpn } => Some(format!(
            "robot does not serve the wire ALPN (negotiated {:?}) — incompatible robot",
            String::from_utf8_lossy(&alpn)
        )),
        AcceptDecision::OpsAdmit | AcceptDecision::OpsBootstrapOnly => Some(
            "robot routed the wire dial onto the OPS plane (a protocol mismatch) — \
             the desk dialed cerulion/wire/1 but got an ops decision"
                .to_string(),
        ),
        // WireAdmit is not a refusal — the caller keeps trying to decode a real
        // WireResponse (this arm is unreachable in practice: an admitted wire
        // connection never writes an AcceptDecision).
        AcceptDecision::WireAdmit => None,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The request tags match the robot's documented `verb` vocabulary (hand
    /// oracle — the exact wire strings the robot dispatches on).
    #[test]
    fn wire_request_json_tags_are_the_wire_vocabulary() {
        assert_eq!(
            serde_json::to_string(&WireRequest::Catalog).unwrap(),
            r#"{"verb":"catalog"}"#
        );
        assert_eq!(
            serde_json::to_string(&WireRequest::Demand {
                topic: "/imu".to_string()
            })
            .unwrap(),
            r#"{"verb":"demand","topic":"/imu"}"#
        );
        assert_eq!(
            serde_json::to_string(&WireRequest::Undemand {
                topic: "/imu".to_string()
            })
            .unwrap(),
            r#"{"verb":"undemand","topic":"/imu"}"#
        );
        assert_eq!(
            serde_json::to_string(&WireRequest::Schema {
                topic: "/imu".to_string()
            })
            .unwrap(),
            r#"{"verb":"schema","topic":"/imu"}"#
        );
        assert_eq!(
            serde_json::to_string(&WireRequest::Status).unwrap(),
            r#"{"verb":"status"}"#
        );
        // The epoch push. Its tag matters MOST — a desk whose `sync_epoch`
        // bytes the robot cannot parse would silently stop delivering revocations
        // while every other verb kept working. (The robot's twin oracle lives in
        // `cerulion_remoted::wire`'s tests; the cross-crate byte parity in
        // `cerulion_connectd/tests/protocol_parity_test.rs`.)
        assert_eq!(
            serde_json::to_string(&WireRequest::SyncEpoch {
                epoch_postcard: "00ff10".to_string()
            })
            .unwrap(),
            r#"{"verb":"sync_epoch","epoch_postcard":"00ff10"}"#
        );
    }

    /// The reply tags + both healthy-outcome fields (the desk distinguishes
    /// `applied` from the stale no-op). Hand oracle.
    #[test]
    fn epoch_synced_reply_json_tag_is_the_wire_vocabulary() {
        assert_eq!(
            serde_json::to_string(&WireResponse::EpochSynced {
                epoch: 7,
                applied: true
            })
            .unwrap(),
            r#"{"reply":"epoch_synced","epoch":7,"applied":true}"#
        );
        assert_eq!(
            serde_json::to_string(&WireResponse::EpochSynced {
                epoch: 7,
                applied: false
            })
            .unwrap(),
            r#"{"reply":"epoch_synced","epoch":7,"applied":false}"#
        );
    }

    /// Every request variant round-trips through JSON.
    #[test]
    fn wire_request_round_trips() {
        for req in [
            WireRequest::Catalog,
            WireRequest::Demand {
                topic: "/a".to_string(),
            },
            WireRequest::Undemand {
                topic: "/a".to_string(),
            },
            WireRequest::Schema {
                topic: "/a".to_string(),
            },
            WireRequest::Status,
            WireRequest::SyncEpoch {
                epoch_postcard: "abcdef".to_string(),
            },
        ] {
            let bytes = serde_json::to_vec(&req).unwrap();
            let back: WireRequest = serde_json::from_slice(&bytes).unwrap();
            assert_eq!(back, req);
        }
    }

    /// The stream preamble round-trips (the uni-stream topic-correlation frame).
    #[test]
    fn stream_preamble_round_trips() {
        let p = StreamPreamble {
            topic: "/utlidar/cloud".to_string(),
        };
        let bytes = serde_json::to_vec(&p).unwrap();
        assert_eq!(serde_json::from_slice::<StreamPreamble>(&bytes).unwrap(), p);
    }

    /// A `DemandAccepted` reply decodes as a `WireResponse` (admitted path).
    #[test]
    fn decode_first_reply_accepts_a_wire_response() {
        let reply = WireResponse::DemandAccepted {
            topic: "/imu".to_string(),
        };
        let bytes = serde_json::to_vec(&reply).unwrap();
        assert_eq!(decode_first_reply(&bytes), Ok(reply));
    }

    /// A refusal `AcceptDecision` (the daemon's skeleton report on an unpaired
    /// wire dial) decodes as an explicit refusal, NOT a wire response — and the
    /// reason is surfaced verbatim. Hand oracle (the exact refusal JSON shape the
    /// robot writes).
    #[test]
    fn decode_first_reply_recognizes_a_refusal() {
        let refuse = AcceptDecision::Refuse {
            reason: "wire plane refused: unpaired device key".to_string(),
        };
        let bytes = serde_json::to_vec(&refuse).unwrap();
        // It must NOT decode as a WireResponse (distinct tag), and it must be an
        // explicit refusal carrying the reason.
        assert!(serde_json::from_slice::<WireResponse>(&bytes).is_err());
        match decode_first_reply(&bytes) {
            Err(Some(reason)) => assert!(reason.contains("unpaired"), "reason: {reason}"),
            other => panic!("expected an explicit refusal, got {other:?}"),
        }
    }

    /// An unknown-ALPN decision surfaces an actionable incompatible-robot reason.
    #[test]
    fn decode_first_reply_recognizes_unknown_alpn() {
        let bytes = serde_json::to_vec(&AcceptDecision::UnknownAlpn {
            alpn: b"cerulion/bogus/9".to_vec(),
        })
        .unwrap();
        match decode_first_reply(&bytes) {
            Err(Some(reason)) => assert!(reason.contains("incompatible"), "reason: {reason}"),
            other => panic!("expected an unknown-ALPN refusal, got {other:?}"),
        }
    }

    /// Garbage bytes decode as neither — a protocol mismatch (`Err(None)`).
    #[test]
    fn decode_first_reply_rejects_garbage() {
        assert_eq!(decode_first_reply(b"not json at all"), Err(None));
        // Valid JSON but neither shape (no `reply` and no `decision` tag).
        assert_eq!(decode_first_reply(br#"{"hello":"world"}"#), Err(None));
    }
}
