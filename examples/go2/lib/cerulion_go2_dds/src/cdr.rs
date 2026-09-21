// SPDX-License-Identifier: AGPL-3.0-only
//! Pure CDR codecs for the Go2 DDS bridge (v1 message set).
//!
//! These are PURE functions over byte slices — no DDS, no network, no
//! participant. They exist so the wire encoding can be oracle-tested in
//! isolation (the tests at the bottom hand-build CDR buffers), and so the
//! bridge can go bytes<->struct directly when bridging a DDS sample to a Cerulion
//! wire frame (or vice versa) without routing through a typed reader.
//!
//! # The engine is the same one DDS pub/sub uses
//!
//! The body encoder/decoder is `cdr-encoding` — the exact serde-CDR engine
//! `rustdds`/`ros2-client` use for typed pub/sub — so a payload produced here
//! is byte-identical to one a real DDS publisher would put on the wire (XCDR1 /
//! CDR, little-endian). This module adds only the piece `cdr-encoding` leaves to the DDS
//! layer: the 4-byte **encapsulation header** (representation identifier).
//!
//! # Wire shape (full DDS `SerializedPayload`)
//!
//! ```text
//! payload := rep_id:u16(BE) | options:u16 | cdr_body
//! ```
//!
//! `rep_id` is `0x0001` for CDR_LE (little-endian, what this module emits) or
//! `0x0000` for CDR_BE; the header bytes are `00 01 00 00` for CDR_LE. Decode
//! reads the rep_id and dispatches the body decode by endianness — the
//! "endianness header handling" the header exists for. Alignment inside the
//! body is relative to the body start (byte 0 AFTER the header), which is
//! exactly where `cdr-encoding` begins counting.
//!
//! `cdr-encoding`'s `to_vec`/`from_bytes` write/read the body ONLY (verified
//! against its source: serialization begins directly at the first field, no
//! preamble), so this module prepends and strips the 4-byte header itself.

use byteorder::{BigEndian, LittleEndian};
use serde::{de::DeserializeOwned, Serialize};
use thiserror::Error;

use crate::messages::{PointCloud2, PointField, Request, SportModeState, Twist};

/// The 4-byte CDR little-endian encapsulation header (`rep_id = 0x0001`,
/// options `0x0000`). Every payload this module encodes starts with these
/// bytes.
pub const CDR_LE_HEADER: [u8; 4] = [0x00, 0x01, 0x00, 0x00];

/// Representation identifier for plain CDR, little-endian (XCDR1).
pub const REP_ID_CDR_LE: u16 = 0x0001;
/// Representation identifier for plain CDR, big-endian (XCDR1).
pub const REP_ID_CDR_BE: u16 = 0x0000;

/// A CDR encode/decode failure. Total (never panics); every variant names the
/// message type so a mis-decode is loud, not silent.
#[derive(Debug, Clone, PartialEq, Eq, Error)]
pub enum CdrError {
    /// The payload was shorter than the 4-byte encapsulation header.
    #[error("CDR payload for {what} truncated: need {need} bytes for the encapsulation header, have {len}")]
    MissingHeader {
        what: &'static str,
        need: usize,
        len: usize,
    },
    /// The representation identifier is neither CDR_LE nor CDR_BE (e.g.
    /// PL_CDR / a parameter-list encapsulation, or garbage). Decoding is
    /// refused rather than mis-reading the body.
    #[error("CDR payload for {what} has unsupported representation identifier {rep_id:#06x} (only CDR_LE 0x0001 and CDR_BE 0x0000 are supported)")]
    UnsupportedRepresentation { what: &'static str, rep_id: u16 },
    /// The serde-CDR engine failed to serialize the value. `cause` is the
    /// engine error's Debug text (NOT a `#[source]` chain: the field must stay
    /// stringly so the enum keeps `Clone + PartialEq + Eq` for oracle tests —
    /// and a field literally named `source` would make thiserror demand
    /// `std::error::Error` on it).
    #[error("CDR encode failed for {what}: {cause}")]
    Encode { what: &'static str, cause: String },
    /// The serde-CDR engine failed to deserialize the body (short buffer, bad
    /// length prefix, non-UTF-8 string, ...). Same stringly-`cause` rationale
    /// as [`CdrError::Encode`].
    #[error("CDR decode failed for {what}: {cause}")]
    Decode { what: &'static str, cause: String },
}

/// Encode any `Serialize` value into a full CDR-LE DDS payload (4-byte
/// encapsulation header + little-endian CDR body). `what` names the type for
/// error context.
pub fn encode_cdr<T: Serialize>(value: &T, what: &'static str) -> Result<Vec<u8>, CdrError> {
    let body = cdr_encoding::to_vec::<T, LittleEndian>(value).map_err(|e| CdrError::Encode {
        what,
        cause: format!("{e:?}"),
    })?;
    let mut out = Vec::with_capacity(CDR_LE_HEADER.len() + body.len());
    out.extend_from_slice(&CDR_LE_HEADER);
    out.extend_from_slice(&body);
    Ok(out)
}

/// Decode a full CDR DDS payload (encapsulation header + body) into `T`. Reads
/// the representation identifier and dispatches the body decode by endianness;
/// trailing body bytes are ignored (CDR may pad the tail). `what` names the
/// type for error context.
pub fn decode_cdr<T: DeserializeOwned>(payload: &[u8], what: &'static str) -> Result<T, CdrError> {
    if payload.len() < 4 {
        return Err(CdrError::MissingHeader {
            what,
            need: 4,
            len: payload.len(),
        });
    }
    // The representation identifier is a 16-bit value stored big-endian in the
    // first two header bytes (RTPS 9.4.2). CDR_LE -> [0x00, 0x01].
    let rep_id = u16::from_be_bytes([payload[0], payload[1]]);
    let body = &payload[4..];
    let decoded = match rep_id {
        REP_ID_CDR_LE => cdr_encoding::from_bytes::<T, LittleEndian>(body),
        REP_ID_CDR_BE => cdr_encoding::from_bytes::<T, BigEndian>(body),
        other => {
            return Err(CdrError::UnsupportedRepresentation {
                what,
                rep_id: other,
            })
        }
    };
    decoded
        .map(|(value, _consumed)| value)
        .map_err(|e| CdrError::Decode {
            what,
            cause: format!("{e:?}"),
        })
}

// --- Typed wrappers (the clean API for the node crates) --------------------

/// Encode a [`PointCloud2`] to a CDR-LE DDS payload.
pub fn encode_point_cloud2(msg: &PointCloud2) -> Result<Vec<u8>, CdrError> {
    encode_cdr(msg, "sensor_msgs/PointCloud2")
}
/// Decode a CDR DDS payload into a [`PointCloud2`].
pub fn decode_point_cloud2(payload: &[u8]) -> Result<PointCloud2, CdrError> {
    decode_cdr(payload, "sensor_msgs/PointCloud2")
}

/// Encode a [`Twist`] to a CDR-LE DDS payload.
pub fn encode_twist(msg: &Twist) -> Result<Vec<u8>, CdrError> {
    encode_cdr(msg, "geometry_msgs/Twist")
}
/// Decode a CDR DDS payload into a [`Twist`].
pub fn decode_twist(payload: &[u8]) -> Result<Twist, CdrError> {
    decode_cdr(payload, "geometry_msgs/Twist")
}

/// Encode a [`SportModeState`] to a CDR-LE DDS payload.
pub fn encode_sport_mode_state(msg: &SportModeState) -> Result<Vec<u8>, CdrError> {
    encode_cdr(msg, "unitree_go/SportModeState")
}
/// Decode a CDR DDS payload into a [`SportModeState`].
pub fn decode_sport_mode_state(payload: &[u8]) -> Result<SportModeState, CdrError> {
    decode_cdr(payload, "unitree_go/SportModeState")
}

/// Encode a [`Request`] to a CDR-LE DDS payload.
pub fn encode_request(msg: &Request) -> Result<Vec<u8>, CdrError> {
    encode_cdr(msg, "unitree_api/Request")
}
/// Decode a CDR DDS payload into a [`Request`].
pub fn decode_request(payload: &[u8]) -> Result<Request, CdrError> {
    decode_cdr(payload, "unitree_api/Request")
}

// --- PointCloud2 point-data extraction -------------------------------------
//
// Separate from CDR: this decodes the RAW packed `data` blob (already
// CDR-decoded off the wire) into xyz points.
// The point layout is described by the `fields` /
// `point_step`, not by CDR.

/// Read a little-endian `f32` at `off` in `data`, or `None` if out of bounds.
pub fn read_f32_le(data: &[u8], off: usize) -> Option<f32> {
    data.get(off..off + 4)
        .map(|b| f32::from_le_bytes([b[0], b[1], b[2], b[3]]))
}

/// Read a big-endian `f32` at `off` in `data`, or `None` if out of bounds.
/// The BE mirror of [`read_f32_le`] — [`decode_xyz_points`] dispatches on
/// `PointCloud2::is_bigendian` between the two.
pub fn read_f32_be(data: &[u8], off: usize) -> Option<f32> {
    data.get(off..off + 4)
        .map(|b| f32::from_be_bytes([b[0], b[1], b[2], b[3]]))
}

/// Find the byte offset of the point field named `name` (e.g. `"x"`).
pub fn point_field_offset(fields: &[PointField], name: &str) -> Option<usize> {
    fields
        .iter()
        .find(|f| f.name == name)
        .map(|f| f.offset as usize)
}

/// Decode up to `max_points` `[x, y, z]` points from a cloud's packed `data`
/// blob, using the `x`/`y`/`z` FLOAT32 field offsets and `point_step`.
/// Honors `is_bigendian`: each coordinate is read via [`read_f32_le`] or
/// [`read_f32_be`] accordingly (same LE/BE dispatch precedent as the
/// encapsulation header in [`decode_cdr`]).
///
/// Returns only COMPLETE points (stops at the first point whose x/y/z would
/// read out of bounds — identical contract in both endianness arms). Returns
/// an empty vec when the cloud has no x/y/z fields, `point_step == 0`, or no
/// data.
///
/// LIMITATION (refused loudly, never silently mis-read): a ROW-PADDED
/// organized cloud — `height > 1` with `row_step != width * point_step` — is
/// NOT supported. The walk is a dense `point_step` stride over `data`, so row
/// padding would be decoded as garbage points; such a cloud returns an empty
/// vec with a `tracing::debug!` naming the mismatch. Dense organized clouds
/// (`row_step == width * point_step`) and unorganized clouds (`height <= 1`,
/// the Go2 lidar shape) decode normally.
pub fn decode_xyz_points(pc: &PointCloud2, max_points: usize) -> Vec<[f32; 3]> {
    let (ox, oy, oz) = match (
        point_field_offset(&pc.fields, "x"),
        point_field_offset(&pc.fields, "y"),
        point_field_offset(&pc.fields, "z"),
    ) {
        (Some(x), Some(y), Some(z)) => (x, y, z),
        _ => return Vec::new(),
    };
    let step = pc.point_step as usize;
    if step == 0 {
        return Vec::new();
    }
    // Row-padded organized cloud: the dense stride below would walk straight
    // through the per-row padding and fabricate points from it. Refuse loudly
    // (u64 math so a hostile width cannot overflow the check).
    if pc.height > 1 && (pc.row_step as u64) != (pc.width as u64) * (pc.point_step as u64) {
        tracing::debug!(
            height = pc.height,
            width = pc.width,
            point_step = pc.point_step,
            row_step = pc.row_step,
            "decode_xyz_points: unsupported ROW-PADDED organized cloud \
             (row_step != width * point_step) — returning no points rather than \
             mis-reading row padding as data"
        );
        return Vec::new();
    }
    // The endianness dispatch: robot traffic declares its point-data byte
    // order in `is_bigendian`; honor it (both arms share the identical
    // complete-points-only break contract below).
    let read: fn(&[u8], usize) -> Option<f32> = if pc.is_bigendian {
        read_f32_be
    } else {
        read_f32_le
    };
    let n = (pc.data.len() / step).min(max_points);
    let mut out = Vec::with_capacity(n);
    for i in 0..n {
        let base = i * step;
        match (
            read(&pc.data, base + ox),
            read(&pc.data, base + oy),
            read(&pc.data, base + oz),
        ) {
            (Some(x), Some(y), Some(z)) => out.push([x, y, z]),
            _ => break,
        }
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::messages::{
        Header, ImuState, PointField, RequestHeader, RequestIdentity, RequestLease, RequestPolicy,
        Time, Vector3, POINT_FIELD_FLOAT32, SPORT_API_ID_MOVE,
    };

    // -- An INDEPENDENT hand CDR-LE body writer (the oracle) ----------------
    //
    // Re-implements the confirmed cdr-encoding rules by hand so the generated
    // payloads can be cross-checked against the real engine (equality proves
    // the field-order/alignment/framing model here matches the engine — NOT a
    // self-compare; the engine is the source of truth). Rules (verified
    // against cdr-encoding source): primitives are aligned to their natural
    // size; a string is `u32(len+1) | utf8 | 0x00`; a `Vec` sequence is
    // `u32(count) | elements`; a fixed array has NO count.
    #[derive(Default)]
    struct CdrLeBody {
        b: Vec<u8>,
    }
    impl CdrLeBody {
        fn align(&mut self, a: usize) {
            while self.b.len() % a != 0 {
                self.b.push(0);
            }
        }
        fn bool(&mut self, v: bool) -> &mut Self {
            self.b.push(v as u8);
            self
        }
        // NOTE: `u32` is used internally by `string`/`byte_seq` (their length
        // prefixes). Only the primitive writers the oracle tests actually need
        // are defined — a fixed-array writer would be dead code here since the
        // SportModeState oracle is a structural body-length + field-offset
        // check, not a full hand-built buffer.
        fn u32(&mut self, v: u32) -> &mut Self {
            self.align(4);
            self.b.extend_from_slice(&v.to_le_bytes());
            self
        }
        fn i32(&mut self, v: i32) -> &mut Self {
            self.align(4);
            self.b.extend_from_slice(&v.to_le_bytes());
            self
        }
        fn i64(&mut self, v: i64) -> &mut Self {
            self.align(8);
            self.b.extend_from_slice(&v.to_le_bytes());
            self
        }
        fn f64(&mut self, v: f64) -> &mut Self {
            self.align(8);
            self.b.extend_from_slice(&v.to_le_bytes());
            self
        }
        fn string(&mut self, s: &str) -> &mut Self {
            self.u32(s.len() as u32 + 1);
            self.b.extend_from_slice(s.as_bytes());
            self.b.push(0);
            self
        }
        fn byte_seq(&mut self, bytes: &[u8]) -> &mut Self {
            self.u32(bytes.len() as u32);
            self.b.extend_from_slice(bytes);
            self
        }
        /// Prepend the CDR-LE encapsulation header to make a full payload.
        fn payload(&self) -> Vec<u8> {
            let mut v = CDR_LE_HEADER.to_vec();
            v.extend_from_slice(&self.b);
            v
        }
    }

    // -- Encapsulation header handling --------------------------------------

    #[test]
    fn header_too_short_is_missing_header() {
        assert_eq!(
            decode_twist(&[0x00, 0x01, 0x00]),
            Err(CdrError::MissingHeader {
                what: "geometry_msgs/Twist",
                need: 4,
                len: 3
            })
        );
    }

    #[test]
    fn unknown_representation_id_is_rejected_not_misdecoded() {
        // rep_id 0x0003 = PL_CDR_LE (parameter-list encapsulation) — not a
        // plain-CDR body; refuse rather than mis-read.
        let mut payload = vec![0x00, 0x03, 0x00, 0x00];
        payload.extend_from_slice(&[0u8; 48]);
        assert_eq!(
            decode_twist(&payload),
            Err(CdrError::UnsupportedRepresentation {
                what: "geometry_msgs/Twist",
                rep_id: 0x0003
            })
        );
    }

    #[test]
    fn big_endian_header_is_dispatched_big_endian() {
        // Hand-build a CDR_BE Twist: header 00 00 00 00, then 6 BIG-endian f64.
        let mut payload = vec![0x00, 0x00, 0x00, 0x00];
        for v in [1.0f64, 2.0, 3.0, 4.0, 5.0, 6.0] {
            payload.extend_from_slice(&v.to_be_bytes());
        }
        let t = decode_twist(&payload).expect("decode BE");
        assert_eq!(
            t.linear,
            Vector3 {
                x: 1.0,
                y: 2.0,
                z: 3.0
            }
        );
        assert_eq!(
            t.angular,
            Vector3 {
                x: 4.0,
                y: 5.0,
                z: 6.0
            }
        );
    }

    // -- Twist: the definitive CDR-engine anchor (no strings/seqs/padding) --

    fn sample_twist() -> Twist {
        // All values exactly representable in f64, so the oracle bytes are
        // unambiguous.
        Twist {
            linear: Vector3 {
                x: 0.5,
                y: 0.0,
                z: 0.0,
            },
            angular: Vector3 {
                x: 0.0,
                y: 0.0,
                z: 0.25,
            },
        }
    }

    #[test]
    fn twist_encode_matches_hand_oracle_bytes() {
        let mut o = CdrLeBody::default();
        o.f64(0.5).f64(0.0).f64(0.0).f64(0.0).f64(0.0).f64(0.25);
        // Fully hand-computable: header + 6 f64 LE = 52 bytes, no padding.
        assert_eq!(o.payload().len(), 52);
        assert_eq!(encode_twist(&sample_twist()).unwrap(), o.payload());
    }

    #[test]
    fn twist_decode_matches_hand_oracle_struct() {
        let mut o = CdrLeBody::default();
        o.f64(0.5).f64(0.0).f64(0.0).f64(0.0).f64(0.0).f64(0.25);
        assert_eq!(decode_twist(&o.payload()).unwrap(), sample_twist());
    }

    #[test]
    fn twist_round_trips() {
        let t = sample_twist();
        assert_eq!(decode_twist(&encode_twist(&t).unwrap()).unwrap(), t);
    }

    // -- Request: nested fixed header + string + byte seq -------------------

    fn move_request(id: i64, api_id: i64, parameter: &str) -> Request {
        Request {
            header: RequestHeader {
                identity: RequestIdentity { id, api_id },
                lease: RequestLease { id: 0 },
                policy: RequestPolicy {
                    priority: 0,
                    noreply: false,
                },
            },
            parameter: parameter.to_string(),
            binary: Vec::new(),
        }
    }

    fn request_oracle(id: i64, api_id: i64, parameter: &str) -> Vec<u8> {
        let mut o = CdrLeBody::default();
        // header.identity.id / api_id (i64, 8-aligned, body starts at 0)
        o.i64(id).i64(api_id);
        // header.lease.id (i64)
        o.i64(0);
        // header.policy.priority (i32) / noreply (bool, 1 byte)
        o.i32(0).bool(false);
        // parameter (string — the u32 length re-aligns to 4, injecting the
        // padding after the trailing bool)
        o.string(parameter);
        // binary (uint8[] sequence)
        o.byte_seq(&[]);
        o.payload()
    }

    #[test]
    fn request_encode_matches_hand_oracle_bytes() {
        let param = r#"{"x":0.1,"y":0.0,"z":0.0}"#;
        let req = move_request(7, SPORT_API_ID_MOVE, param);
        assert_eq!(
            encode_request(&req).unwrap(),
            request_oracle(7, SPORT_API_ID_MOVE, param)
        );
    }

    #[test]
    fn request_decode_matches_hand_oracle_struct() {
        // The task anchor: api_id 1008 + the JSON Move parameter, decoded from
        // an INDEPENDENT hand-built buffer.
        let param = r#"{"x":0.1,"y":0.0,"z":0.0}"#;
        let decoded = decode_request(&request_oracle(42, SPORT_API_ID_MOVE, param)).unwrap();
        assert_eq!(decoded.header.identity.id, 42);
        assert_eq!(decoded.header.identity.api_id, 1008);
        assert_eq!(decoded.parameter, param);
        assert!(decoded.binary.is_empty());
        assert_eq!(decoded, move_request(42, SPORT_API_ID_MOVE, param));
    }

    #[test]
    fn request_string_bounds_edges_round_trip() {
        // Empty parameter (CDR string len=1, just the null terminator) and a
        // long UTF-8 parameter both survive round-trip, and the empty case
        // matches its hand oracle byte-for-byte.
        let empty = move_request(1, SPORT_API_ID_MOVE, "");
        assert_eq!(
            encode_request(&empty).unwrap(),
            request_oracle(1, SPORT_API_ID_MOVE, "")
        );
        assert_eq!(
            decode_request(&encode_request(&empty).unwrap()).unwrap(),
            empty
        );

        let long = move_request(
            2,
            SPORT_API_ID_MOVE,
            r#"{"x":-1.5,"y":2.5,"z":-3.14159,"note":"a longer parameter payload"}"#,
        );
        assert_eq!(
            decode_request(&encode_request(&long).unwrap()).unwrap(),
            long
        );
    }

    #[test]
    fn request_with_binary_payload_round_trips() {
        let mut req = move_request(3, SPORT_API_ID_MOVE, "{}");
        req.binary = vec![0xDE, 0xAD, 0xBE, 0xEF, 0x00, 0x7F];
        // Cross-check the full byte layout incl. the trailing byte sequence.
        let mut o = CdrLeBody::default();
        o.i64(3)
            .i64(SPORT_API_ID_MOVE)
            .i64(0)
            .i32(0)
            .bool(false)
            .string("{}")
            .byte_seq(&[0xDE, 0xAD, 0xBE, 0xEF, 0x00, 0x7F]);
        assert_eq!(encode_request(&req).unwrap(), o.payload());
        assert_eq!(decode_request(&encode_request(&req).unwrap()).unwrap(), req);
    }

    // -- PointCloud2: point-data extraction (the task anchor) + CDR struct --

    fn two_point_cloud() -> PointCloud2 {
        // Little-endian f32 point blob: point0 = (1,2,3), point1 = (4,5,6).
        let data = vec![
            0, 0, 128, 63, 0, 0, 0, 64, 0, 0, 64, 64, // (1.0, 2.0, 3.0)
            0, 0, 128, 64, 0, 0, 160, 64, 0, 0, 192, 64, // (4.0, 5.0, 6.0)
        ];
        PointCloud2 {
            header: Header {
                stamp: Time {
                    sec: 12,
                    nanosec: 34,
                },
                frame_id: "lidar".to_string(),
            },
            height: 1,
            width: 2,
            fields: vec![
                PointField {
                    name: "x".to_string(),
                    offset: 0,
                    datatype: POINT_FIELD_FLOAT32,
                    count: 1,
                },
                PointField {
                    name: "y".to_string(),
                    offset: 4,
                    datatype: POINT_FIELD_FLOAT32,
                    count: 1,
                },
                PointField {
                    name: "z".to_string(),
                    offset: 8,
                    datatype: POINT_FIELD_FLOAT32,
                    count: 1,
                },
            ],
            is_bigendian: false,
            point_step: 12,
            row_step: 24,
            data,
            is_dense: true,
        }
    }

    #[test]
    fn pointcloud_xyz_decodes_exact_points_at_offsets_0_4_8() {
        let pc = two_point_cloud();
        assert_eq!(point_field_offset(&pc.fields, "x"), Some(0));
        assert_eq!(point_field_offset(&pc.fields, "y"), Some(4));
        assert_eq!(point_field_offset(&pc.fields, "z"), Some(8));
        assert_eq!(
            decode_xyz_points(&pc, 16),
            vec![[1.0, 2.0, 3.0], [4.0, 5.0, 6.0]]
        );
        // max_points caps the count.
        assert_eq!(decode_xyz_points(&pc, 1), vec![[1.0, 2.0, 3.0]]);
    }

    #[test]
    fn pointcloud_empty_data_decodes_no_points() {
        let mut pc = two_point_cloud();
        pc.data.clear();
        assert_eq!(decode_xyz_points(&pc, 16), Vec::<[f32; 3]>::new());
        // No x/y/z fields => also empty (not a panic).
        let mut no_fields = two_point_cloud();
        no_fields.fields.clear();
        assert_eq!(decode_xyz_points(&no_fields, 16), Vec::<[f32; 3]>::new());
        // point_step == 0 => empty, no divide-by-zero.
        let mut zero_step = two_point_cloud();
        zero_step.point_step = 0;
        assert_eq!(decode_xyz_points(&zero_step, 16), Vec::<[f32; 3]>::new());
    }

    #[test]
    fn pointcloud_cdr_round_trips_and_is_le_headed() {
        // The full PointCloud2 (nested Header string + PointField sequence +
        // data sequence) survives CDR round-trip. The CDR engine itself is
        // anchored byte-for-byte by the Twist/Request oracles above; here we
        // additionally pin the encapsulation header and that the whole message
        // round-trips through the real serde struct.
        let pc = two_point_cloud();
        let payload = encode_point_cloud2(&pc).unwrap();
        assert_eq!(&payload[..4], &CDR_LE_HEADER);
        assert_eq!(decode_point_cloud2(&payload).unwrap(), pc);
        // Empty cloud (empty frame_id, no fields, no data) also round-trips.
        let empty = PointCloud2::default();
        assert_eq!(
            decode_point_cloud2(&encode_point_cloud2(&empty).unwrap()).unwrap(),
            empty
        );
    }

    // -- SportModeState: fixed-array-heavy struct ---------------------------

    fn sample_sport_state() -> SportModeState {
        SportModeState {
            stamp: Time {
                sec: 100,
                nanosec: 250_000_000,
            },
            error_code: 0xDEAD_BEEF,
            imu_state: ImuState {
                quaternion: [1.0, 0.0, 0.0, 0.0],
                gyroscope: [0.1, 0.2, 0.3],
                accelerometer: [0.0, 0.0, 9.81],
                rpy: [0.01, -0.02, 0.03],
                temperature: 42,
            },
            mode: 1,
            progress: 0.5,
            gait_type: 2,
            foot_raise_height: 0.08,
            position: [1.5, -2.5, 0.3],
            body_height: 0.32,
            velocity: [0.4, 0.0, 0.0],
            yaw_speed: 0.25,
            range_obstacle: [1.0, 2.0, 3.0, 4.0],
            foot_force: [10, 20, 30, 40],
            foot_position_body: [0.1; 12],
            foot_speed_body: [0.2; 12],
        }
    }

    #[test]
    fn sport_state_body_is_exactly_the_fixed_size() {
        // Structural anchor: with every `floatN[K]`/`int16[4]` a FIXED array
        // (no CDR count), the body is exactly 232 bytes (hand-computed with
        // alignment). A regression that used a `Vec` for a fixed array would
        // inject a spurious u32 count and change this length. See the field
        // table in messages.rs for the offset walk.
        let payload = encode_sport_mode_state(&sample_sport_state()).unwrap();
        assert_eq!(payload.len(), 4 + 232);
        assert_eq!(&payload[..4], &CDR_LE_HEADER);
    }

    #[test]
    fn sport_state_error_code_is_at_body_offset_8() {
        // error_code (u32) follows stamp.sec (i32) + stamp.nanosec (u32) = body
        // offset 8 => payload offset 12. Hand-anchored field position.
        let payload = encode_sport_mode_state(&sample_sport_state()).unwrap();
        assert_eq!(&payload[12..16], &0xDEAD_BEEFu32.to_le_bytes());
    }

    #[test]
    fn sport_state_round_trips() {
        let s = sample_sport_state();
        assert_eq!(
            decode_sport_mode_state(&encode_sport_mode_state(&s).unwrap()).unwrap(),
            s
        );
    }

    // -- ADVERSARIAL decode surface ------------------------------------------
    //
    // Robot traffic is UNTRUSTED input (the FrameWalker precedent): these
    // codecs decode bytes off the network and encode commands to a physical
    // robot. A review verified that cdr-encoding 0.11's current behavior is
    // safe on hostile input — bounds-checked Eof errors (no OOB read), NO
    // size_hint-driven pre-allocation (a lying length prefix cannot trigger a
    // huge alloc), and `from_utf8` validation on strings. The tests below are
    // the REGRESSION GUARD pinning that safety: a future codec swap or version
    // bump that reintroduces a panic / pre-alloc-OOM / silent partial decode
    // fails HERE, in a pure parallel-safe test, not on the robot.

    /// The valid CDR body prefix of a `Request` up to (but excluding) the
    /// `parameter` string — the launch pad for hostile-string bodies.
    fn hostile_request_prefix() -> CdrLeBody {
        let mut o = CdrLeBody::default();
        o.i64(1).i64(SPORT_API_ID_MOVE).i64(0).i32(0).bool(false);
        o
    }

    #[test]
    fn truncated_body_is_decode_err_not_panic() {
        // (a) A valid CDR_LE header followed by 3 bytes of what should be a
        // 232-byte SportModeState body: the engine must hit its bounds-checked
        // Eof and surface CdrError::Decode — never panic, never fabricate a
        // partial struct.
        let mut payload = CDR_LE_HEADER.to_vec();
        payload.extend_from_slice(&[0x01, 0x02, 0x03]);
        assert!(matches!(
            decode_sport_mode_state(&payload),
            Err(CdrError::Decode {
                what: "unitree_go/SportModeState",
                ..
            })
        ));
        // Same class on the string-bearing type (cuts off inside the fixed
        // header, before the parameter string).
        let mut short_req = CDR_LE_HEADER.to_vec();
        short_req.extend_from_slice(&7i64.to_le_bytes());
        assert!(matches!(
            decode_request(&short_req),
            Err(CdrError::Decode {
                what: "unitree_api/Request",
                ..
            })
        ));
        // Header-only (zero-byte body) is also a Decode err, not a panic.
        assert!(matches!(
            decode_twist(&CDR_LE_HEADER),
            Err(CdrError::Decode {
                what: "geometry_msgs/Twist",
                ..
            })
        ));
    }

    #[test]
    fn hostile_string_length_prefix_errs_promptly_no_prealloc() {
        // (b) The DoS guard: a Request whose `parameter` string claims
        // 0xFFFFFFFF (~4 GiB) bytes over a buffer that only has 3. cdr-encoding
        // 0.11 has NO size_hint pre-allocation, so this errs promptly on the
        // bounds check. The buffer is deliberately TINY (< 50 bytes): if a
        // future engine version pre-allocated from the lying prefix, this test
        // would become a multi-GiB allocation — observable as an OOM/timeout
        // here rather than shipped to the robot.
        let mut o = hostile_request_prefix();
        o.u32(0xFFFF_FFFF); // hostile string length prefix (aligns to 4 first)
        o.b.extend_from_slice(&[0x41, 0x42, 0x43]); // 3 bytes, not 4 GiB
        let payload = o.payload();
        assert!(payload.len() < 50, "guard buffer must stay tiny");
        assert!(matches!(
            decode_request(&payload),
            Err(CdrError::Decode {
                what: "unitree_api/Request",
                ..
            })
        ));
        // Same hostile prefix on the `binary` sequence (u8[]) — an empty-string
        // parameter followed by a 0xFFFFFFFF element count over 2 real bytes.
        let mut o2 = hostile_request_prefix();
        o2.string(""); // valid empty parameter
        o2.u32(0xFFFF_FFFF); // hostile sequence count
        o2.b.extend_from_slice(&[0x00, 0x01]);
        assert!(matches!(
            decode_request(&o2.payload()),
            Err(CdrError::Decode {
                what: "unitree_api/Request",
                ..
            })
        ));
    }

    #[test]
    fn non_utf8_string_bytes_are_decode_err_not_panic() {
        // (c) A well-formed length prefix whose content bytes are not UTF-8
        // where a `String` field is expected: cdr-encoding validates via
        // from_utf8 and errors — pinned so a future engine can't silently
        // accept (or panic on) invalid text from the robot.
        let mut o = hostile_request_prefix();
        // len = 2 (1 content byte + null terminator), content 0xFF = invalid
        // UTF-8, then the terminator.
        o.u32(2);
        o.b.extend_from_slice(&[0xFF, 0x00]);
        // Close out the message shape (empty binary seq) so UTF-8 validation is
        // the ONLY hostile thing about this buffer.
        o.byte_seq(&[]);
        assert!(matches!(
            decode_request(&o.payload()),
            Err(CdrError::Decode {
                what: "unitree_api/Request",
                ..
            })
        ));
    }

    #[test]
    fn hostile_point_field_offsets_never_panic() {
        // (d) The `_ => break` OOB guard in decode_xyz_points, exercised for
        // real: a field table claiming x lives at offset 1000 in a 24-byte
        // blob. First read is None -> break at i == 0 -> empty vec, no panic.
        let mut pc = two_point_cloud();
        pc.fields[0].offset = 1000;
        assert_eq!(decode_xyz_points(&pc, 16), Vec::<[f32; 3]>::new());
        // Variant near the u32 ceiling: offset u32::MAX. base + ox stays well
        // inside usize on 64-bit and read_f32_le's .get() bounds-checks it —
        // no overflow, no panic, no points.
        let mut pc_max = two_point_cloud();
        pc_max.fields[2].offset = u32::MAX;
        assert_eq!(decode_xyz_points(&pc_max, 16), Vec::<[f32; 3]>::new());
        // (c) The SAME break guard under the BE arm: a hostile offset on a
        // big-endian cloud routes through read_f32_be's identical bounds check
        // — no panic, no points.
        let mut pc_be = two_point_cloud();
        pc_be.is_bigendian = true;
        pc_be.fields[0].offset = 1000;
        assert_eq!(decode_xyz_points(&pc_be, 16), Vec::<[f32; 3]>::new());
    }

    #[test]
    fn straddling_tail_returns_only_complete_points() {
        // (e) data.len() NOT a multiple of point_step: 30 bytes at step 12 is
        // 2 complete points + 6 stray tail bytes. The contract is COMPLETE
        // points only — hand oracle: exactly the 2 real points, tail ignored.
        let mut pc = two_point_cloud();
        pc.data.extend_from_slice(&[0xAA; 6]); // 24 -> 30 bytes
        assert_eq!(
            decode_xyz_points(&pc, 16),
            vec![[1.0, 2.0, 3.0], [4.0, 5.0, 6.0]]
        );
    }

    #[test]
    fn point_step_smaller_than_field_offset_breaks_not_panics() {
        // (f) A hostile step (4) smaller than the z offset (8) over 12 bytes of
        // data (LE f32s 1.0, 2.0, 3.0): n = 12/4 = 3 candidate points, but only
        // point 0 has a complete z read (bytes 8..12). Point 1's z needs bytes
        // 12..16 -> None -> the break fires MID-LOOP. Hand oracle: exactly one
        // point, its overlapping reads = (1.0, 2.0, 3.0).
        let mut pc = two_point_cloud();
        pc.data.truncate(12);
        pc.point_step = 4;
        assert_eq!(decode_xyz_points(&pc, 16), vec![[1.0, 2.0, 3.0]]);
    }

    // -- is_bigendian: honored, not ignored ----------------------------------

    #[test]
    fn big_endian_cloud_decodes_via_hand_be_oracle_bytes() {
        // (a) The BE twin of the LE anchor test: the SAME points
        // (1,2,3)/(4,5,6) as HAND-WRITTEN big-endian f32 bytes (IEEE-754 bit
        // patterns transcribed by hand — 1.0 = 0x3F800000 etc. — NOT generated
        // by byte-reversing in code under test) with is_bigendian = true.
        let mut pc = two_point_cloud();
        pc.is_bigendian = true;
        pc.data = vec![
            0x3F, 0x80, 0x00, 0x00, // 1.0 BE
            0x40, 0x00, 0x00, 0x00, // 2.0 BE
            0x40, 0x40, 0x00, 0x00, // 3.0 BE
            0x40, 0x80, 0x00, 0x00, // 4.0 BE
            0x40, 0xA0, 0x00, 0x00, // 5.0 BE
            0x40, 0xC0, 0x00, 0x00, // 6.0 BE
        ];
        assert_eq!(
            decode_xyz_points(&pc, 16),
            vec![[1.0, 2.0, 3.0], [4.0, 5.0, 6.0]]
        );
    }

    #[test]
    fn mixed_endianness_flag_changes_the_decode_anti_tautology() {
        // (b) The MIXED trap proving the dispatch is LIVE: is_bigendian = true
        // over the LE data bytes must NOT reproduce the LE decode, and the
        // garbled values must be exactly the BE interpretation of those bytes
        // (hand-computed bit patterns: LE 1.0 = bytes 00 00 80 3F, read BE =
        // bits 0x0000_803F; LE 2.0 = 00 00 00 40 -> 0x0000_0040; LE 3.0 =
        // 00 00 40 40 -> 0x0000_4040 — all subnormals, compared via to_bits
        // for exactness).
        let mut pc = two_point_cloud(); // LE bytes stay as-is
        pc.is_bigendian = true;
        let pts = decode_xyz_points(&pc, 16);
        assert_eq!(pts.len(), 2);
        assert_ne!(pts[0], [1.0, 2.0, 3.0], "BE flag over LE bytes must garble");
        assert_eq!(pts[0][0].to_bits(), 0x0000_803F);
        assert_eq!(pts[0][1].to_bits(), 0x0000_0040);
        assert_eq!(pts[0][2].to_bits(), 0x0000_4040);
    }

    // -- row_step: dense clouds walk, row-padded clouds are refused loudly ---

    #[test]
    fn row_padded_organized_cloud_is_refused_not_misread() {
        // height = 2, width = 2, point_step = 12, but row_step = 32 (24 bytes
        // of points + 8 pad bytes per row): the dense stride would fabricate
        // points out of the padding, so the documented limitation refuses it
        // (empty vec + tracing::debug!), never a silent mis-read.
        let mut pc = two_point_cloud();
        pc.height = 2;
        pc.row_step = 32;
        pc.data = vec![0x41; 64]; // 2 rows x 32 bytes, content irrelevant
        assert_eq!(decode_xyz_points(&pc, 16), Vec::<[f32; 3]>::new());
    }

    #[test]
    fn dense_organized_cloud_still_decodes() {
        // The control for the refusal: an organized cloud (height = 2) that IS
        // dense (row_step == width * point_step == 24) walks normally — the
        // refusal is scoped to real row padding, not to organization itself.
        let mut pc = two_point_cloud();
        let row = pc.data.clone(); // the (1,2,3)/(4,5,6) row, 24 bytes
        pc.data.extend_from_slice(&row); // second identical row
        pc.height = 2;
        // row_step is already 24 == 2 * 12 in the fixture.
        assert_eq!(
            decode_xyz_points(&pc, 16),
            vec![
                [1.0, 2.0, 3.0],
                [4.0, 5.0, 6.0],
                [1.0, 2.0, 3.0],
                [4.0, 5.0, 6.0]
            ]
        );
    }
}
