// SPDX-License-Identifier: AGPL-3.0-only
//! The RAW-CDR sample adapter — how the bridge obtains the
//! verbatim CDR payload (+ endianness) of received DDS samples instead of a
//! typed serde deserialize, so the generic
//! [`CdrCodec`](cerulion_core::codegen::CdrCodec) can transcode ANY
//! schema-known type with zero per-type code.
//!
//! # The raw path on rustdds 0.13.1 / ros2-client 0.10.0
//!
//! What the pinned sources of those two crates allow, and the constraints
//! that follow from them:
//!
//! 1. **ros2-client's `Subscription<M>` hard-wires the serde-CDR adapter.**
//!    `Subscription<M>` wraps `no_key::SimpleDataReaderCdr<M>` (=
//!    `SimpleDataReader<M, CDRDeserializerAdapter<M>>`); the only
//!    constructor path (`Node::create_subscription` →
//!    `Context::create_subscription`) bakes that adapter in, and the
//!    adapter-generic seam (`Node::create_simpledatareader<D, DA>`) is
//!    `pub(crate)`. The seed variants (`take_seed`/`async_stream_seed`)
//!    still route through the serde-CDR engine (`CdrDeserializeSeedDecoder`)
//!    — serde offers no "slurp the remaining raw bytes" escape, so no raw
//!    payload that way either. **Conclusion: raw drain = drop to the
//!    rustdds layer for raw-drained topics.**
//!
//! 2. **The rustdds drop-down is fully public** (the custom-
//!    `DeserializerAdapter` seam — the same pattern DDS-bridge ecosystems
//!    use for payload-agnostic transport):
//!    `ros2_client::Context::domain_participant()` (pub) → rustdds
//!    `DomainParticipant::create_subscriber(&QosPolicies)` (pub) → rustdds
//!    `Subscriber::create_simple_datareader_no_key::<D, DA>`
//!    (pub, adapter-generic; the stale `pub(crate) from_keyed` TODO inside
//!    `no_key/simpledatareader.rs` notwithstanding). Topic creation STAYS on
//!    ros2-client (`Node::create_topic`) — it returns a plain
//!    `rustdds::Topic` with the ROS name mangling (`rt/` prefix via
//!    `Name::to_dds_name`) and `MessageTypeName` type mangling already
//!    applied, so raw readers match CycloneDDS peers exactly like typed
//!    ones.
//!
//! 3. **The adapter receives exactly `(CDR body, RepresentationIdentifier)`.**
//!    rustdds parses the 4-byte DDS encapsulation header itself
//!    (`SerializedPayload { representation_identifier,
//!    representation_options, value: Bytes }`) and calls
//!    `DA::from_bytes_with(&serialized_payload.value, recognized_rep_id,
//!    decoder)` — `value` is the body with the encapsulation ALREADY
//!    stripped. Declaring `supported_encodings() = [CDR_LE, CDR_BE]` makes
//!    rustdds itself reject PL_CDR/XCDR2 frames with a loud
//!    `ReadError::Deserialization` naming topic + rep id BEFORE the
//!    bridge's own code runs (`deserialize_with` filters on
//!    `supported_encodings`).
//!
//! 4. **`async_stream` works on the raw path.** `SimpleDataReader::
//!    as_async_stream()` requires only `DA: DefaultDecoder<D>` — satisfied
//!    below by a unit `RawDecoder` (`Clone`, zero-size). The stream shape is
//!    identical to the typed pump path (`FusedStream<Item =
//!    ReadResult<DeserializedCacheChange<D>>>`, value via `.into_value()`,
//!    writer stamp via `.source_timestamp()`), so the pump's proven
//!    `async_stream()`-pinned drain pattern ports verbatim and the
//!    take()-never-yields constraint never applies (this path never uses
//!    take-polling). All DDS objects — including raw readers — are still created
//!    ON the drain thread (the pump module contract).
//!
//! 5. **One cosmetic trade-off:** a rustdds-level reader bypasses
//!    ros2-client's `Node::add_reader` ROS-graph bookkeeping, so raw
//!    subscriptions won't appear in `ros2 node info` for the bridge node.
//!    DDS discovery/matching is RTPS-level and unaffected (the reader still
//!    matches CycloneDDS publishers by topic/type/QoS).
//!
//! # Chosen design
//!
//! - [`RawCdrAdapter`] implements `rustdds::no_key::DeserializerAdapter<
//!   RawSample>` + `DefaultDecoder`: decode = capture the body bytes +
//!   map the rep id to [`CdrEndianness`]. One `Bytes → Vec` copy per sample
//!   — this is the DDS ingress edge, not the SHM hot path (the zero-copy
//!   contract starts at `publish_raw`'s loan).
//! - [`create_raw_reader`] pins the exact construction the drain-thread
//!   wiring uses (`pump::build_raw_binding`).
//! - The Cerulion side ([`super::RawIngressRoute`]) decodes the
//!   [`RawSample`] through the generic codec and `publish_raw`s the frame on
//!   a dynamically-created ingress publisher.

use cerulion_core::codegen::CdrEndianness;
use cerulion_go2_dds::ros2_client::rustdds::dds::CreateResult;
use cerulion_go2_dds::ros2_client::rustdds::no_key::{
    Decode, DefaultDecoder, DeserializerAdapter, SimpleDataReader,
};
use cerulion_go2_dds::ros2_client::rustdds::{
    QosPolicies, RepresentationIdentifier, Subscriber, Topic,
};

/// A raw-decoder failure. Defensive only: rustdds filters representation ids
/// against the adapter's `supported_encodings` before invoking the decoder,
/// so this variant firing means a rustdds contract change — loud, never a
/// silent mis-decode.
#[derive(Debug, Clone, PartialEq, Eq, thiserror::Error)]
pub enum RawDecodeError {
    /// A representation id outside the declared supported set reached the
    /// decoder.
    #[error(
        "unsupported CDR representation id {rep_id:02x?} reached the raw decoder — only \
         CDR_LE/CDR_BE are declared supported (rustdds should have filtered this frame)"
    )]
    UnsupportedRepresentation { rep_id: [u8; 2] },
}

/// One raw DDS sample: the verbatim CDR BODY (the 4-byte DDS encapsulation
/// header is already parsed off by rustdds) plus its byte order,
/// exactly the `(endian, cdr_body)` pair [`CdrCodec::decode`] takes.
///
/// [`CdrCodec::decode`]: cerulion_core::codegen::CdrCodec::decode
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct RawSample {
    /// Byte order from the DDS representation identifier.
    pub endianness: CdrEndianness,
    /// The CDR body bytes, verbatim (no encapsulation header).
    pub body: Vec<u8>,
}

/// The rustdds `DeserializerAdapter` that captures raw CDR instead of
/// serde-decoding.
pub struct RawCdrAdapter;

/// The declared decodable encodings (returned by the adapter's
/// `supported_encodings`). EXACTLY the two plain-CDR ids the generic codec
/// implements — deliberately NOT `PL_CDR_LE` (which the stock
/// `CDRDeserializerAdapter` claims): a parameter-list frame must be rejected
/// loudly by rustdds's rep-id filter, never handed to a plain-CDR walk.
const RAW_SUPPORTED_ENCODINGS: [RepresentationIdentifier; 2] = [
    RepresentationIdentifier::CDR_LE,
    RepresentationIdentifier::CDR_BE,
];

impl DeserializerAdapter<RawSample> for RawCdrAdapter {
    type Error = RawDecodeError;
    type Decoded = RawSample;

    fn supported_encodings() -> &'static [RepresentationIdentifier] {
        &RAW_SUPPORTED_ENCODINGS
    }

    fn transform_decoded(decoded: RawSample) -> RawSample {
        decoded
    }
}

/// The zero-size default decoder (`DefaultDecoder` is what unlocks
/// `as_async_stream()` without a per-call seed).
#[derive(Debug, Clone, Copy)]
pub struct RawDecoder;

impl<'de> Decode<'de, RawSample> for RawDecoder {
    type Error = RawDecodeError;

    fn decode_bytes(
        self,
        input_bytes: &'de [u8],
        encoding: RepresentationIdentifier,
    ) -> Result<RawSample, RawDecodeError> {
        let endianness = if encoding == RepresentationIdentifier::CDR_LE {
            CdrEndianness::Little
        } else if encoding == RepresentationIdentifier::CDR_BE {
            CdrEndianness::Big
        } else {
            return Err(RawDecodeError::UnsupportedRepresentation {
                rep_id: encoding.to_bytes(),
            });
        };
        Ok(RawSample {
            endianness,
            body: input_bytes.to_vec(),
        })
    }
}

impl DefaultDecoder<RawSample> for RawCdrAdapter {
    type Decoder = RawDecoder;
    const DECODER: RawDecoder = RawDecoder;
}

/// A raw-CDR DDS reader — the type the drain thread drives via
/// `as_async_stream()` exactly like the typed subscriptions.
pub type RawReader = SimpleDataReader<RawSample, RawCdrAdapter>;

/// Create a raw-CDR reader on `topic`, the module-doc drop-down to the
/// rustdds layer: `subscriber` comes from
/// `context.domain_participant().create_subscriber(&qos)`, `topic` from the
/// ros2-client `Node::create_topic` (ROS name/type mangling applied). The
/// pump wiring is the production caller.
pub fn create_raw_reader(
    subscriber: &Subscriber,
    topic: &Topic,
    qos: Option<QosPolicies>,
) -> CreateResult<RawReader> {
    subscriber.create_simple_datareader_no_key::<RawSample, RawCdrAdapter>(topic, qos)
}

#[cfg(test)]
mod tests {
    use super::*;
    use cerulion_core::codegen::split_encapsulation;
    use cerulion_go2_dds::cdr::{encode_twist, CDR_LE_HEADER};
    use cerulion_go2_dds::messages::{Twist, Vector3};

    #[test]
    fn test_supported_encodings_are_exactly_plain_cdr_le_and_be() {
        // The filter pin: PL_CDR/XCDR2 must NOT be claimed (a parameter-list
        // frame handed to the plain-CDR walk would silently mis-decode —
        // rustdds's rep-id filter is the loud rejection, and this list is
        // what arms it). Order + content, vs hand constants.
        let enc = RawCdrAdapter::supported_encodings();
        assert_eq!(
            enc,
            &[
                RepresentationIdentifier::CDR_LE,
                RepresentationIdentifier::CDR_BE
            ]
        );
        assert!(!enc.contains(&RepresentationIdentifier::PL_CDR_LE));
        assert!(!enc.contains(&RepresentationIdentifier::PL_CDR_BE));
    }

    #[test]
    fn test_decode_bytes_le_captures_body_verbatim() {
        let body = [1u8, 2, 3, 0xFF, 0, 42];
        let got = RawDecoder
            .decode_bytes(&body, RepresentationIdentifier::CDR_LE)
            .expect("LE decodes");
        assert_eq!(got.endianness, CdrEndianness::Little);
        assert_eq!(got.body, body.to_vec());
    }

    #[test]
    fn test_decode_bytes_be_maps_endianness() {
        let body = [0u8; 4];
        let got = RawDecoder
            .decode_bytes(&body, RepresentationIdentifier::CDR_BE)
            .expect("BE decodes");
        assert_eq!(got.endianness, CdrEndianness::Big);
        assert_eq!(got.body, vec![0u8; 4]);
    }

    #[test]
    fn test_decode_bytes_rejects_unsupported_rep_id_defensively() {
        // Defensive arm: rustdds filters first, but a contract drift must be
        // a loud Err naming the id, never a silent Little default.
        let err = RawDecoder
            .decode_bytes(&[0u8; 8], RepresentationIdentifier::PL_CDR_LE)
            .unwrap_err();
        assert_eq!(
            err,
            RawDecodeError::UnsupportedRepresentation {
                rep_id: RepresentationIdentifier::PL_CDR_LE.to_bytes()
            }
        );
        assert!(err.to_string().contains("unsupported CDR representation"));
    }

    #[test]
    fn test_adapter_from_bytes_default_decoder_path_matches_direct_decode() {
        // `as_async_stream()` routes through `DefaultDecoder::DECODER` →
        // `DeserializerAdapter::from_bytes` — prove that wiring yields the
        // SAME RawSample as calling the decoder directly.
        let body = [9u8, 8, 7, 6];
        let via_adapter =
            RawCdrAdapter::from_bytes(&body, RepresentationIdentifier::CDR_LE).expect("adapter");
        let direct = RawDecoder
            .decode_bytes(&body, RepresentationIdentifier::CDR_LE)
            .expect("direct");
        assert_eq!(via_adapter, direct);
        assert_eq!(via_adapter.body, body.to_vec());
    }

    #[test]
    fn test_encapsulation_parity_with_the_codec_split_on_a_real_hand_codec_payload() {
        // The two encapsulation handlers in play — rustdds's (rep id parsed
        // off, body handed to the adapter) and the codec's
        // `split_encapsulation` (used by `decode_dds_payload`) — must agree
        // on a REAL full DDS payload from the `cerulion_go2_dds` hand codec.
        let payload = encode_twist(&Twist {
            linear: Vector3 {
                x: 0.5,
                y: -1.25,
                z: 0.0,
            },
            angular: Vector3 {
                x: 0.0,
                y: 0.0,
                z: 2.0,
            },
        })
        .expect("hand codec encodes");
        assert_eq!(&payload[..4], &CDR_LE_HEADER, "hand codec emits CDR_LE");

        // Codec-side split.
        let (endian, body) = split_encapsulation(&payload).expect("split");
        assert_eq!(endian, CdrEndianness::Little);

        // rustdds-side shape: rep id = first two bytes, options = next two,
        // adapter receives payload[4..].
        let rep_id = RepresentationIdentifier::from_bytes(&payload[..2]).expect("rep id");
        assert_eq!(rep_id, RepresentationIdentifier::CDR_LE);
        let raw = RawCdrAdapter::from_bytes(&payload[4..], rep_id).expect("adapter decodes");

        assert_eq!(raw.endianness, endian);
        assert_eq!(raw.body, body.to_vec());
    }
}
