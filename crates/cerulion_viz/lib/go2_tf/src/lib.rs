// SPDX-License-Identifier: AGPL-3.0-only
//! Pure Go2 TF transforms element codec + robot geometry constants.
//!
//! `tf2_msgs/TFMessage` carries `geometry_msgs/TransformStamped[] transforms`
//! — a `DynamicArray<Nested>`, for which the generated type exposes only
//! raw-bytes accessors (`transforms_bytes()` / `set_transforms_bytes(&[u8])`).
//!
//! The element layout below is a documented PRODUCER↔CONSUMER CONVENTION,
//! exactly like `cerulion_viz::pointcloud::parse_point_fields` is the
//! convention for the packed `PointCloud2.fields` blob.
//!
//! # It is NOT the framework's canonical element framing
//!
//! A later change gave `cerulion_core::codegen::FrameWalker` a CANONICAL element
//! framing for this shape (`u32 count` + per element `u32 len` + a headerless
//! `TransformStamped` sub-frame). These records are NOT written that way — no
//! per-element length, no offset table — so the walker keeps surfacing the
//! field as `NestedArrayOpaque`, and `cerulion_viz::tf` / `cerulion_viz::sink`
//! keep reading the raw bytes this module encodes.
//!
//! That refusal is NOT free, and a future edit here must not assume it: the
//! bytes below are only 4 bytes away from satisfying the canonical framing.
//! `sec` sits exactly where the canonical per-element `u32 len` does, so the
//! 84-byte one-record blob (`identity_odom_base`) is EXACTLY consumed whenever
//! `sec == 76` — a value a monotonic robot clock passes through every boot. It
//! stays opaque only because the walker also audits each element's INTERNAL
//! accounting, which those 4 leftover bytes fail. A layout
//! change that alters a record's length moves that alias, so
//! `frame_walker.rs`'s `go2_tf_bespoke_blob_stays_opaque_at_every_stamp`
//! DERIVES the alias from the blob rather than hardcoding it — keep it in sync
//! with this layout. This module is the single home of that
//! convention: [`encode_tf_transforms`] (the producer half — `go2_tf_source`
//! writes `set_transforms_bytes(&encode_tf_transforms(..))`) and
//! [`decode_tf_transforms`] (the consumer half — the Rerun TF sinks decode
//! `transforms_bytes()`), which are exact inverses.
//!
//! The layout is byte-identical to the network /tf pipe test
//! (`cerulion_core/tests/network_tf_e2e_test.rs`'s `transform_record` /
//! `tf_transforms`), so the WHOLE /tf pipeline is coherent: the network link carries
//! these bytes VERBATIM across the zenoh hop, `go2_tf_source` encodes them, the sink
//! decodes them. That test file is the convention's OTHER pin: a layout
//! change here MUST be mirrored in its hand-built records (each side's
//! byte-level oracle tests catch a one-sided edit).
//!
//! # Wire layout (all little-endian)
//!
//! ```text
//! transforms := count:u32, then `count` × record
//! record     := sec:i32 | nanosec:u32
//!             | frame_id_len:u32  | frame_id  (UTF-8)   // PARENT frame
//!             | child_len:u32     | child     (UTF-8)   // CHILD frame
//!             | translation x,y,z : f64 × 3
//!             | rotation    x,y,z,w : f64 × 4           // quaternion
//! ```
//!
//! An EMPTY blob (`&[]` — the `set_transforms_bytes(&[])` idiom the humanoid
//! bench `tf_broadcaster` and the pipe test's empty-array edge write) decodes to
//! ZERO transforms (Ok). `encode_tf_transforms(&[])` emits the 4-byte
//! `count = 0` prefix; both forms decode to an empty vec.
//!
//! # NOT ROS 2 CDR
//!
//! This is a Cerulion-native element encoding (the framework wire format is
//! not CDR anywhere). The on-robot ROS 2 path (go2_ros2_sdk over
//! rmw_cerulion) produces whatever rmw_cerulion emits for a
//! `TransformStamped[]`; reconciling the two encodings is a bring-up concern
//! deliberately OUT of scope here (the demo must not depend on the real
//! transform library). The native path (`go2_tf_source`) uses THIS
//! encoding end to end.

// Principle #12 (logging): library code never prints — it logs through
// `tracing`. Scoped `not(test)` so unit tests keep printing diagnostics, and
// applied at the crate root rather than in `[workspace.lints]` because that
// table cannot distinguish a lib target from a test binary. Pinned by
// `cerulion_cli_engine/tests/library_print_ban_test.rs`.
#![cfg_attr(not(test), deny(clippy::print_stdout, clippy::print_stderr))]

use thiserror::Error;

/// One decoded transform: a parent→child rigid transform with a stamp.
///
/// `frame_id` is the PARENT frame, `child_frame_id` the CHILD (ROS
/// `TransformStamped` naming). `rotation` is a quaternion in `[x, y, z, w]`
/// order (the ROS `geometry_msgs/Quaternion` field order).
#[derive(Debug, Clone, PartialEq)]
pub struct TfTransform {
    /// Transform stamp seconds (the transform's OWN time, not the wire stamp).
    pub stamp_sec: i32,
    /// Transform stamp nanoseconds.
    pub stamp_nanosec: u32,
    /// Parent frame id (`header.frame_id`).
    pub frame_id: String,
    /// Child frame id.
    pub child_frame_id: String,
    /// Translation `[x, y, z]` (metres).
    pub translation: [f64; 3],
    /// Rotation quaternion `[x, y, z, w]`.
    pub rotation: [f64; 4],
}

impl TfTransform {
    /// Build a transform with an identity-friendly constructor.
    pub fn new(
        frame_id: impl Into<String>,
        child_frame_id: impl Into<String>,
        translation: [f64; 3],
        rotation: [f64; 4],
        stamp_sec: i32,
        stamp_nanosec: u32,
    ) -> Self {
        Self {
            stamp_sec,
            stamp_nanosec,
            frame_id: frame_id.into(),
            child_frame_id: child_frame_id.into(),
            translation,
            rotation,
        }
    }
}

/// The identity quaternion `[x, y, z, w]` — no rotation.
pub const IDENTITY_QUAT: [f64; 4] = [0.0, 0.0, 0.0, 1.0];

/// A decode failure on the opaque `transforms` blob. Every variant names the
/// offending record; the codec is total (never panics) and refuses partial
/// reads loudly rather than fabricating a transform (Principle: loud over
/// silent).
#[derive(Debug, Clone, PartialEq, Eq, Error)]
pub enum TfDecodeError {
    /// The blob ended before the `count` prefix could be read.
    #[error("transforms blob truncated: need 4 bytes for the count prefix, have {len}")]
    MissingCount { len: usize },
    /// Record `index` extends past the end of the blob. `need` is the FULL
    /// size of the field being read (not the shortfall — the message words it
    /// accordingly; the shortfall is `at + need - len`).
    #[error(
        "transforms record {index} truncated: reading '{field}' at byte {at} needs {need} bytes, \
         blob len {len}"
    )]
    Truncated {
        index: usize,
        field: &'static str,
        at: usize,
        need: usize,
        len: usize,
    },
    /// A frame-id field held non-UTF-8 bytes.
    #[error("transforms record {index} has non-UTF-8 {which} frame id")]
    BadUtf8 { index: usize, which: &'static str },
    /// Extra bytes remained after the declared `count` records were read.
    #[error("transforms blob has {trailing} trailing bytes after {count} record(s)")]
    TrailingBytes { count: usize, trailing: usize },
}

/// Encode a slice of transforms into the opaque `transforms` blob (the exact
/// inverse of [`decode_tf_transforms`]). Always emits the `count` prefix, so
/// `encode_tf_transforms(&[])` is a 4-byte `count = 0` blob.
pub fn encode_tf_transforms(transforms: &[TfTransform]) -> Vec<u8> {
    let mut bytes = Vec::new();
    encode_tf_transforms_into(transforms, &mut bytes);
    bytes
}

/// Encode the same wire format as [`encode_tf_transforms`], replacing the
/// destination contents while retaining its capacity. Repeated fixed-shape TF
/// publications can reuse their startup allocation.
pub fn encode_tf_transforms_into(transforms: &[TfTransform], v: &mut Vec<u8>) {
    v.clear();
    v.extend_from_slice(&(transforms.len() as u32).to_le_bytes());
    for t in transforms {
        v.extend_from_slice(&t.stamp_sec.to_le_bytes());
        v.extend_from_slice(&t.stamp_nanosec.to_le_bytes());
        v.extend_from_slice(&(t.frame_id.len() as u32).to_le_bytes());
        v.extend_from_slice(t.frame_id.as_bytes());
        v.extend_from_slice(&(t.child_frame_id.len() as u32).to_le_bytes());
        v.extend_from_slice(t.child_frame_id.as_bytes());
        for c in t.translation {
            v.extend_from_slice(&c.to_le_bytes());
        }
        for c in t.rotation {
            v.extend_from_slice(&c.to_le_bytes());
        }
    }
}

/// Cursor reader over the blob, refusing every over-read loudly.
struct Reader<'a> {
    blob: &'a [u8],
    cur: usize,
    index: usize,
}

impl<'a> Reader<'a> {
    fn take(&mut self, n: usize, field: &'static str) -> Result<&'a [u8], TfDecodeError> {
        // Copy the `&'a` slice out first so the returned subslice is tied to
        // the blob's lifetime, not to the `&mut self` borrow.
        let blob = self.blob;
        let end = self.cur.checked_add(n);
        match end {
            Some(e) if e <= blob.len() => {
                let s = &blob[self.cur..e];
                self.cur = e;
                Ok(s)
            }
            _ => Err(TfDecodeError::Truncated {
                index: self.index,
                field,
                at: self.cur,
                need: n,
                len: self.blob.len(),
            }),
        }
    }

    fn u32(&mut self, field: &'static str) -> Result<u32, TfDecodeError> {
        let b = self.take(4, field)?;
        Ok(u32::from_le_bytes([b[0], b[1], b[2], b[3]]))
    }

    fn i32(&mut self, field: &'static str) -> Result<i32, TfDecodeError> {
        let b = self.take(4, field)?;
        Ok(i32::from_le_bytes([b[0], b[1], b[2], b[3]]))
    }

    fn f64(&mut self, field: &'static str) -> Result<f64, TfDecodeError> {
        let b = self.take(8, field)?;
        Ok(f64::from_le_bytes([
            b[0], b[1], b[2], b[3], b[4], b[5], b[6], b[7],
        ]))
    }

    fn string(
        &mut self,
        which: &'static str,
        field: &'static str,
    ) -> Result<String, TfDecodeError> {
        let len = self.u32(field)? as usize;
        let bytes = self.take(len, field)?;
        std::str::from_utf8(bytes)
            .map(|s| s.to_string())
            .map_err(|_| TfDecodeError::BadUtf8 {
                index: self.index,
                which,
            })
    }
}

/// Decode the opaque `transforms` blob into a vec of transforms (the exact
/// inverse of [`encode_tf_transforms`]).
///
/// An empty blob (`&[]`) decodes to zero transforms (the
/// `set_transforms_bytes(&[])` producer idiom). A truncated record, a
/// non-UTF-8 frame id, or trailing bytes after the declared count is a loud
/// [`TfDecodeError`] (never a silent partial read).
pub fn decode_tf_transforms(blob: &[u8]) -> Result<Vec<TfTransform>, TfDecodeError> {
    // The empty-array idiom: `set_transforms_bytes(&[])` writes NOTHING (no
    // count prefix) — the humanoid bench + the pipe test's empty-array edge.
    if blob.is_empty() {
        return Ok(Vec::new());
    }
    if blob.len() < 4 {
        return Err(TfDecodeError::MissingCount { len: blob.len() });
    }
    let count = u32::from_le_bytes([blob[0], blob[1], blob[2], blob[3]]) as usize;
    let mut r = Reader {
        blob,
        cur: 4,
        index: 0,
    };
    let mut out = Vec::with_capacity(count.min(1024)); // cap the pre-alloc; a lying count still reads bounded
    for i in 0..count {
        r.index = i;
        let stamp_sec = r.i32("sec")?;
        let stamp_nanosec = r.u32("nanosec")?;
        let frame_id = r.string("parent", "frame_id")?;
        let child_frame_id = r.string("child", "child_frame_id")?;
        let translation = [r.f64("tx")?, r.f64("ty")?, r.f64("tz")?];
        let rotation = [r.f64("qx")?, r.f64("qy")?, r.f64("qz")?, r.f64("qw")?];
        out.push(TfTransform {
            stamp_sec,
            stamp_nanosec,
            frame_id,
            child_frame_id,
            translation,
            rotation,
        });
    }
    if r.cur != blob.len() {
        return Err(TfDecodeError::TrailingBytes {
            count,
            trailing: blob.len() - r.cur,
        });
    }
    Ok(out)
}

// ===========================================================================
// Go2 robot geometry — representative estimates, NOT measurements.
// ===========================================================================
//
// Placeholder values: these mount offsets are
// representative published Go2 values, NOT measured from the actual robot.
// The real transforms come from the Go2 URDF (go2_ros2_sdk's
// robot_state_publisher on /tf_static) or from mounts you measure. Replace
// them with values verified on your robot.

/// The odom (world-fixed) frame name.
pub const ODOM_FRAME: &str = "odom";
/// The robot base frame name.
pub const BASE_FRAME: &str = "base";
/// The lidar sensor frame name.
pub const LIDAR_FRAME: &str = "lidar";
/// The camera sensor frame name.
pub const CAMERA_FRAME: &str = "camera";

/// `base → lidar` mount translation (metres). Placeholder: measure it on your robot.
pub const BASE_TO_LIDAR_XYZ: [f64; 3] = [0.17, 0.0, 0.11];
/// `base → camera` mount translation (metres). Placeholder: measure it on your robot.
pub const BASE_TO_CAMERA_XYZ: [f64; 3] = [0.27, 0.0, 0.05];

/// The static mount table (`base → lidar`, `base → camera`), identity
/// rotation. Published on `/tf_static` by `go2_tf_source` and
/// decoded by the Rerun static sink. `stamp` is 0/0 (latched-static
/// convention — the wire stamp is the sink's timeline).
pub fn static_mounts() -> Vec<TfTransform> {
    vec![
        TfTransform::new(
            BASE_FRAME,
            LIDAR_FRAME,
            BASE_TO_LIDAR_XYZ,
            IDENTITY_QUAT,
            0,
            0,
        ),
        TfTransform::new(
            BASE_FRAME,
            CAMERA_FRAME,
            BASE_TO_CAMERA_XYZ,
            IDENTITY_QUAT,
            0,
            0,
        ),
    ]
}

/// The `odom` to `base` dynamic transform. v1 is a hardcoded IDENTITY stub.
/// The feed a real one derives from is already bridged: the Go2 example's
/// `dds_bridge` projects the firmware's `SportModeState` onto `/go2/odom` as
/// `nav_msgs/Odometry`; what gates consuming it is verifying that byte layout
/// against samples captured from a robot.
/// Until then the tree is complete and correct with the robot at the odom
/// origin; the lidar/camera still render posed via the static mounts.
///
/// `stamp` carries the transform's own time; the sink uses the WIRE stamp for
/// its Rerun timeline (so a replay is deterministic — Principle #7).
pub fn identity_odom_base(stamp_sec: i32, stamp_nanosec: u32) -> TfTransform {
    TfTransform::new(
        ODOM_FRAME,
        BASE_FRAME,
        [0.0, 0.0, 0.0],
        IDENTITY_QUAT,
        stamp_sec,
        stamp_nanosec,
    )
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Hand-build the blob for one record (an ORACLE, independent of
    /// `encode_tf_transforms` — the two are cross-checked below).
    fn oracle_record(
        sec: i32,
        nanosec: u32,
        frame_id: &str,
        child: &str,
        t: [f64; 3],
        r: [f64; 4],
    ) -> Vec<u8> {
        let mut v = Vec::new();
        v.extend_from_slice(&sec.to_le_bytes());
        v.extend_from_slice(&nanosec.to_le_bytes());
        v.extend_from_slice(&(frame_id.len() as u32).to_le_bytes());
        v.extend_from_slice(frame_id.as_bytes());
        v.extend_from_slice(&(child.len() as u32).to_le_bytes());
        v.extend_from_slice(child.as_bytes());
        for c in t {
            v.extend_from_slice(&c.to_le_bytes());
        }
        for c in r {
            v.extend_from_slice(&c.to_le_bytes());
        }
        v
    }

    fn oracle_blob(records: &[Vec<u8>]) -> Vec<u8> {
        let mut v = Vec::new();
        v.extend_from_slice(&(records.len() as u32).to_le_bytes());
        for rec in records {
            v.extend_from_slice(rec);
        }
        v
    }

    fn two_transforms() -> Vec<TfTransform> {
        vec![
            TfTransform::new(
                "odom",
                "base_link",
                [1.5, -2.5, 3.25],
                [0.0, 0.0, 0.25, 0.96875],
                100,
                250_000_000,
            ),
            TfTransform::new(
                "base_link",
                "camera_link",
                [-0.125, 0.0625, -4.75],
                [0.5, -0.5, 0.5, 0.5],
                100,
                250_000_001,
            ),
        ]
    }

    #[test]
    fn encode_matches_hand_oracle_bytes() {
        // ENCODE end: the produced bytes equal an INDEPENDENT hand-built blob
        // (not a self-round-trip — an oracle at the byte level).
        let oracle = oracle_blob(&[
            oracle_record(
                100,
                250_000_000,
                "odom",
                "base_link",
                [1.5, -2.5, 3.25],
                [0.0, 0.0, 0.25, 0.96875],
            ),
            oracle_record(
                100,
                250_000_001,
                "base_link",
                "camera_link",
                [-0.125, 0.0625, -4.75],
                [0.5, -0.5, 0.5, 0.5],
            ),
        ]);
        assert_eq!(encode_tf_transforms(&two_transforms()), oracle);
    }

    #[test]
    fn decode_matches_hand_oracle_records() {
        // DECODE end: a hand-built blob decodes to the expected records (not a
        // self-round-trip — an oracle at the record level).
        let blob = oracle_blob(&[oracle_record(
            200,
            500_000_000,
            "map",
            "odom",
            [10.0, -20.0, 0.0],
            [0.0, 0.0, 0.0, 1.0],
        )]);
        assert_eq!(
            decode_tf_transforms(&blob).expect("decode"),
            vec![TfTransform::new(
                "map",
                "odom",
                [10.0, -20.0, 0.0],
                [0.0, 0.0, 0.0, 1.0],
                200,
                500_000_000,
            )]
        );
    }

    #[test]
    fn encode_decode_round_trips() {
        let ts = two_transforms();
        assert_eq!(
            decode_tf_transforms(&encode_tf_transforms(&ts)).unwrap(),
            ts
        );
    }

    #[test]
    fn empty_blob_and_count_zero_both_decode_empty() {
        assert_eq!(decode_tf_transforms(&[]).unwrap(), Vec::new());
        assert_eq!(
            decode_tf_transforms(&0u32.to_le_bytes()).unwrap(),
            Vec::new()
        );
        // encode(&[]) is the 4-byte count=0 form (not the empty idiom).
        assert_eq!(encode_tf_transforms(&[]), 0u32.to_le_bytes().to_vec());
    }

    #[test]
    fn short_count_prefix_errors() {
        assert_eq!(
            decode_tf_transforms(&[1, 2, 3]),
            Err(TfDecodeError::MissingCount { len: 3 })
        );
    }

    #[test]
    fn truncated_record_errors_loudly() {
        // count = 1 but no record bytes follow.
        let mut blob = 1u32.to_le_bytes().to_vec();
        // add only the sec field (4 bytes) then stop — nanosec read fails.
        blob.extend_from_slice(&5i32.to_le_bytes());
        assert!(matches!(
            decode_tf_transforms(&blob),
            Err(TfDecodeError::Truncated {
                index: 0,
                field: "nanosec",
                ..
            })
        ));
    }

    #[test]
    fn trailing_bytes_error() {
        let mut blob = oracle_blob(&[oracle_record(
            0,
            0,
            "a",
            "b",
            [0.0, 0.0, 0.0],
            IDENTITY_QUAT,
        )]);
        blob.push(0xFF); // one extra byte
        assert!(matches!(
            decode_tf_transforms(&blob),
            Err(TfDecodeError::TrailingBytes {
                count: 1,
                trailing: 1
            })
        ));
    }

    #[test]
    fn bad_utf8_frame_id_errors() {
        // count=1, sec=0, nanosec=0, frame_id_len=1, frame_id=0xFF (invalid).
        let mut blob = 1u32.to_le_bytes().to_vec();
        blob.extend_from_slice(&0i32.to_le_bytes());
        blob.extend_from_slice(&0u32.to_le_bytes());
        blob.extend_from_slice(&1u32.to_le_bytes());
        blob.push(0xFF);
        assert_eq!(
            decode_tf_transforms(&blob),
            Err(TfDecodeError::BadUtf8 {
                index: 0,
                which: "parent"
            })
        );
    }

    #[test]
    fn reusable_encoder_matches_oracle_and_keeps_capacity() {
        let transforms = [TfTransform::new(
            "odom",
            "base",
            [0.0, 0.0, 0.0],
            IDENTITY_QUAT,
            7,
            42,
        )];
        let oracle = oracle_blob(&[oracle_record(
            7,
            42,
            "odom",
            "base",
            [0.0, 0.0, 0.0],
            IDENTITY_QUAT,
        )]);
        let mut bytes = Vec::with_capacity(256);
        let pointer = bytes.as_ptr();
        let capacity = bytes.capacity();
        for _ in 0..3 {
            encode_tf_transforms_into(&transforms, &mut bytes);
            assert_eq!(bytes, oracle);
            assert_eq!(bytes.as_ptr(), pointer);
            assert_eq!(bytes.capacity(), capacity);
        }
        encode_tf_transforms_into(&[], &mut bytes);
        assert_eq!(bytes, [0, 0, 0, 0]);
        assert_eq!(bytes.as_ptr(), pointer);
        encode_tf_transforms_into(&transforms, &mut bytes);
        assert_eq!(bytes, oracle, "a shorter prior frame leaves no stale tail");
    }

    #[test]
    fn static_mounts_round_trip_through_the_codec() {
        let mounts = static_mounts();
        assert_eq!(mounts.len(), 2);
        assert_eq!(mounts[0].frame_id, "base");
        assert_eq!(mounts[0].child_frame_id, "lidar");
        assert_eq!(mounts[0].translation, [0.17, 0.0, 0.11]);
        assert_eq!(mounts[1].child_frame_id, "camera");
        assert_eq!(mounts[1].translation, [0.27, 0.0, 0.05]);
        // Codec round-trip on the real production table.
        assert_eq!(
            decode_tf_transforms(&encode_tf_transforms(&mounts)).unwrap(),
            mounts
        );
    }

    #[test]
    fn identity_odom_base_is_identity() {
        let t = identity_odom_base(7, 42);
        assert_eq!(t.frame_id, "odom");
        assert_eq!(t.child_frame_id, "base");
        assert_eq!(t.translation, [0.0, 0.0, 0.0]);
        assert_eq!(t.rotation, IDENTITY_QUAT);
        assert_eq!((t.stamp_sec, t.stamp_nanosec), (7, 42));
    }
}
