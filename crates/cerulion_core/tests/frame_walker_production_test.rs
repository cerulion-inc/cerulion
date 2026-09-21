// SPDX-License-Identifier: AGPL-3.0-only
//! Production-writer ↔ frame-walker cross-check.
//!
//! Every in-module walker test decodes HAND-BUILT frames, so a shared
//! misunderstanding between the walker and the real writer could pass them
//! all. This file closes the encode↔decode loop: frames are produced by the
//! REAL writer path (`CerulionPublisher::loan_proxy` → generated `<Name>Shm`
//! setters → `OutputProxy::Drop` publish) over an isolated iceoryx2
//! `TestTransport` (per-test SHM root — parallel-safe), captured via
//! `try_receive`, re-assembled (header via `WireHeader::write_to_buf` —
//! `read_from_buf`'s exact inverse, the `network_ingress_test`
//! full-frame-compare convention), and decoded by a [`FrameWalker`] built
//! from `BUILTIN_MSGS`. Decoded values are asserted against the HAND-WRITTEN
//! values the test wrote — never a self-compare.
//!
//! Decoding goes through `walk_by_hash`, which ALSO pins that the generated
//! type's `SCHEMA_HASH` (stamped into the wire header by `loan_proxy`)
//! equals the hash of the `parse_rosmsg`-derived schema the walker builds
//! its layout from — the two recipe paths must agree or nothing decodes.
//!
//! Two schema shapes: fixed-only (`geometry_msgs/Vector3` — pure
//! fixed-section reads) and variable + nested (`sensor_msgs/Image` — fixed
//! fields, a string, a bytes blob, and the empty-`header` producer idiom
//! `set_header_bytes(&[])`, which the walker decodes as a present nested
//! value with zero fields).
//!
//! # The `CdrCodec` ⇄ walker canonical-v1 cross-check
//!
//! [`FrameWalker`] decodes the **canonical v1** element framing for
//! `Nested[]` / `string[]` fields. Every one of its in-module tests decodes a
//! HAND-BUILT frame, so a shared misunderstanding between the walker and the
//! real *writer* of those bytes would pass them all: the same gap the tests above
//! closed for the fixed/variable field layer, now re-opened one level down at
//! the ELEMENT layer.
//!
//! The production writer of canonical v1 is
//! [`cerulion_core::codegen::CdrCodec`]'s `decode_cdr_seq`: it is what a
//! never-seen DDS robot's frames actually travel through (`cerulion ros2
//! attach` → the `dds_bridge` graph → `MappingRoute::RawGeneric` →
//! `CdrCodec::decode`), and it is the only in-tree producer that emits the
//! encoding for every field shape. So the arms below run the FULL production
//! ingress direction:
//!
//! ```text
//! hand-written CDR body  →  CdrCodec::decode  →  real Cerulion frame
//!                                             →  FrameWalker::walk_by_hash
//!                                             →  assert == the hand values
//! ```
//!
//! **Why this is an oracle and not a self-compare.** The values asserted are
//! the ones hand-written INTO the CDR body, not a re-encode of the walker's
//! own output. The two sides in between are independent implementations that
//! never call each other: the codec resolves element layouts through
//! `resolve_nested_qname` and writes the framing in `decode_cdr_seq`, while
//! the walker resolves through `lookup_nested` and reads the framing in its
//! own bounded cursor. Their agreement is exactly what needs asserting.
//! Where the framing arithmetic is fully determined by the schema (no
//! interior alignment padding — `variable_payload_align` is 1 for nested and
//! string payloads) the arms ALSO assert the blob's byte length against a
//! hand-computed structural oracle, so a framing change is caught even if
//! every leaf value still happens to round-trip.
//!
//! One arm covers the other direction of "production": a NATIVE publisher
//! (`loan_proxy` + `set_transforms_bytes`) carrying a hand-built canonical
//! blob through real shared memory — the one place a native Cerulion producer
//! can emit canonical bytes today, since the generated array API is
//! byte-oriented (no typed `push_<field>`; deliberately out of scope). Its
//! companion pins the DEGRADATION contract over the same real path: bytes
//! that are not canonically framed come back as
//! [`FrameValueKind::NestedArrayOpaque`] byte-for-byte, which is what keeps
//! the shipping bespoke consumers (`cerulion_viz::tf`,
//! `cerulion_viz::pointcloud`) working unchanged.

use cerulion_core::codegen::{
    parse_rosmsg, CdrCodec, CdrEndianness, FrameValue, FrameValueKind, FrameWalker, MessageSchema,
    PrimType,
};
use cerulion_core::shm_runtime::write_offset_entry;
use cerulion_core::testing::TestTransport;
use cerulion_core::wire::{MaxSliceLen, WireHeader};
use cerulion_core::CerulionSubscriber;
use native_ros2_messages::geometry_msgs::Vector3;
use native_ros2_messages::sensor_msgs::Image;
use native_ros2_messages::tf2_msgs::TFMessage;

/// Walker over every built-in ROS 2 schema (the exact `.msg` text the
/// generated writer types compiled against).
fn builtin_walker() -> FrameWalker {
    let mut schemas: Vec<MessageSchema> = Vec::new();
    for (pkg, name, text) in native_ros2_messages::BUILTIN_MSGS {
        if let Ok(s) = parse_rosmsg(text, name, Some(pkg)) {
            schemas.push(s);
        }
    }
    let (walker, warnings) = FrameWalker::new(schemas);
    assert!(
        warnings.is_empty(),
        "built-in schema set must resolve cleanly: {warnings:?}"
    );
    walker
}

/// Drain the subscriber and return the LAST delivered frame re-assembled as
/// full wire bytes (32-byte header + payload). `write_to_buf` is
/// `read_from_buf`'s exact inverse, so the reassembled bytes equal the bytes
/// the publisher committed.
fn capture_last_frame(sub: &CerulionSubscriber) -> Vec<u8> {
    let mut frames: Vec<Vec<u8>> = Vec::new();
    sub.try_receive(|msg| {
        let mut frame = vec![0u8; WireHeader::SIZE];
        msg.header().write_to_buf(&mut frame);
        frame.extend_from_slice(msg.payload());
        frames.push(frame);
    })
    .expect("drain subscriber");
    match frames.pop() {
        Some(last) => last,
        None => panic!("expected at least one delivered frame"),
    }
}

/// Fixed-only: a production-published `Vector3` walks to the exact values
/// the test wrote (exact binary floats — no rounding ambiguity).
#[test]
fn production_vector3_frame_walks_to_written_values() {
    let tt = TestTransport::new();
    let mut publisher = tt.publisher("fwp/vec3", MaxSliceLen::const_new(256), 0);
    let sub = tt.subscriber("fwp/vec3");

    {
        let mut proxy = publisher.loan_proxy::<Vector3>().expect("loan Vector3");
        proxy.x = 1.5;
        proxy.y = -2.25;
        proxy.z = 3.125;
        // Drop publishes (fixed schema — no variable gate).
    }

    let frame = capture_last_frame(&sub);
    let walker = builtin_walker();
    let fv = walker
        .walk_by_hash(&frame)
        .expect("production Vector3 frame must walk by its own header hash");

    assert_eq!(fv.schema_name, "geometry_msgs/Vector3");
    assert_eq!(fv.field("x"), Some(&FrameValueKind::F64(1.5)));
    assert_eq!(fv.field("y"), Some(&FrameValueKind::F64(-2.25)));
    assert_eq!(fv.field("z"), Some(&FrameValueKind::F64(3.125)));
}

/// Variable + nested: a production-published `Image` (fixed u32/u8 fields,
/// string `encoding`, bytes `data`, empty-`header` idiom) walks to the exact
/// hand-written values.
#[test]
fn production_image_frame_walks_to_written_values() {
    let tt = TestTransport::new();
    let mut publisher = tt.publisher("fwp/image", MaxSliceLen::const_new(64 * 1024), 0);
    let sub = tt.subscriber("fwp/image");

    // Deterministic 64-byte payload oracle.
    let data_oracle: Vec<u8> = (0u8..64).collect();

    {
        let mut proxy = publisher.loan_proxy::<Image>().expect("loan Image");
        proxy.height = 480;
        proxy.width = 640;
        proxy.is_bigendian = 0;
        proxy.step = 1920;
        proxy.set_header_bytes(&[]).expect("empty header idiom");
        proxy.set_encoding("rgb8").expect("set encoding");
        proxy.set_data(&data_oracle).expect("set data");
        // Drop publishes (all variable fields written).
    }

    let frame = capture_last_frame(&sub);
    let walker = builtin_walker();
    let fv = walker
        .walk_by_hash(&frame)
        .expect("production Image frame must walk by its own header hash");

    assert_eq!(fv.schema_name, "sensor_msgs/Image");
    assert_eq!(fv.field("height"), Some(&FrameValueKind::U32(480)));
    assert_eq!(fv.field("width"), Some(&FrameValueKind::U32(640)));
    assert_eq!(fv.field("is_bigendian"), Some(&FrameValueKind::U8(0)));
    assert_eq!(fv.field("step"), Some(&FrameValueKind::U32(1920)));
    assert_eq!(fv.field("encoding"), Some(&FrameValueKind::Str("rgb8")));
    assert_eq!(fv.field("data"), Some(&FrameValueKind::Bytes(&data_oracle)));
    // The empty-`header` producer idiom (`set_header_bytes(&[])`) decodes
    // as a PRESENT nested value with zero fields — the documented
    // degradation, not an error.
    match fv.field("header") {
        Some(FrameValueKind::Nested(h)) => {
            assert_eq!(h.schema_name, "std_msgs/Header");
            assert!(h.fields.is_empty(), "empty header blob → zero fields");
        }
        other => panic!("expected empty Nested header, got {other:?}"),
    }
}

// ===========================================================================
// CdrCodec ⇄ FrameWalker canonical-v1 cross-check
// ===========================================================================

/// The production canonical-v1 WRITER over every built-in ROS 2 schema — the
/// exact codec `cerulion ros2 attach`'s `dds_bridge` graph runs on the
/// `MappingRoute::RawGeneric` path. Built from the SAME `.msg` corpus as
/// [`builtin_walker`], but through an independent pipeline
/// (`resolve_fixed_nested` + `LayoutResolver` inside `CdrCodec::new`), so the
/// two agreeing is a real cross-check.
fn builtin_codec() -> CdrCodec {
    let mut schemas: Vec<MessageSchema> = Vec::new();
    for (pkg, name, text) in native_ros2_messages::BUILTIN_MSGS {
        if let Ok(s) = parse_rosmsg(text, name, Some(pkg)) {
            schemas.push(s);
        }
    }
    let (codec, warnings) = CdrCodec::new(schemas);
    assert!(
        warnings.is_empty(),
        "built-in schema set must resolve cleanly for the codec too: {warnings:?}"
    );
    codec
}

/// A minimal CDR body writer mirroring `CdrReader`'s rules exactly: align to
/// a primitive's own width before writing it, and spell a string as a
/// 4-aligned `u32(content_len + 1)` then the UTF-8 then a NUL. Byte 0 is the
/// field-aligned origin (the byte after the 4-byte encapsulation header,
/// which [`CdrCodec::decode`] takes as already stripped).
///
/// Deliberately hand-rolled rather than reusing the codec's own `CdrWriter`:
/// the CDR body is the test's INPUT ORACLE, so it must not come from the
/// implementation under test.
#[derive(Default)]
struct CdrBody {
    buf: Vec<u8>,
}

impl CdrBody {
    fn align(&mut self, a: usize) {
        while !self.buf.len().is_multiple_of(a) {
            self.buf.push(0);
        }
    }

    fn u32(&mut self, v: u32) {
        self.align(4);
        self.buf.extend_from_slice(&v.to_le_bytes());
    }

    fn i32(&mut self, v: i32) {
        self.align(4);
        self.buf.extend_from_slice(&v.to_le_bytes());
    }

    fn f64(&mut self, v: f64) {
        self.align(8);
        self.buf.extend_from_slice(&v.to_le_bytes());
    }

    /// CDR string: `u32(len + 1)` | UTF-8 | NUL. The `+1` and the trailing
    /// NUL are the CDR contract `read_cdr_string` enforces STRICTLY (a zero
    /// prefix or a missing NUL is a loud error, never a lenient trim).
    fn string(&mut self, s: &str) {
        self.u32(s.len() as u32 + 1);
        self.buf.extend_from_slice(s.as_bytes());
        self.buf.push(0);
    }

    /// `std_msgs/Header` = `builtin_interfaces/Time stamp` (`int32 sec`,
    /// `uint32 nanosec`) + `string frame_id`, in declaration order.
    fn header(&mut self, sec: i32, nanosec: u32, frame_id: &str) {
        self.i32(sec);
        self.u32(nanosec);
        self.string(frame_id);
    }

    /// `geometry_msgs/Pose` = `Point position` (3 × f64) + `Quaternion
    /// orientation` (4 × f64) — 7 CDR doubles, no padding between them.
    fn pose(&mut self, position: [f64; 3], orientation: [f64; 4]) {
        for c in position {
            self.f64(c);
        }
        for c in orientation {
            self.f64(c);
        }
    }

    /// A CDR sequence of `float64` (`u32 count` + count × f64).
    fn f64_seq(&mut self, values: &[f64]) {
        self.u32(values.len() as u32);
        for v in values {
            self.f64(*v);
        }
    }
}

/// Destructure a decoded canonical element array into its elements AND its
/// raw bytes (the `raw` slice every pre-existing
/// bespoke-convention consumer still reads).
fn expect_nested_array<'v, 'a>(
    v: Option<&'v FrameValueKind<'a>>,
    what: &str,
) -> (&'v [FrameValueKind<'a>], &'a [u8]) {
    match v {
        Some(FrameValueKind::NestedArray { elements, raw }) => (elements, raw),
        other => panic!("expected a decoded NestedArray for {what}, got {other:?}"),
    }
}

fn expect_nested<'v, 'a>(v: &'v FrameValueKind<'a>, what: &str) -> &'v FrameValue<'a> {
    match v {
        FrameValueKind::Nested(inner) => inner,
        other => panic!("expected Nested for {what}, got {other:?}"),
    }
}

fn expect_opaque<'a>(v: Option<&FrameValueKind<'a>>, what: &str) -> &'a [u8] {
    match v {
        Some(FrameValueKind::NestedArrayOpaque(bytes)) => bytes,
        other => panic!("expected NestedArrayOpaque for {what}, got {other:?}"),
    }
}

/// Assert a decoded `geometry_msgs/Pose` sub-tree equals hand-written values.
/// Every leaf is an exact binary float (no rounding ambiguity), and the values
/// are asymmetric across x/y/z/w so a swapped or short span shows up as a
/// wrong value rather than passing.
fn assert_pose_tree(pose: &FrameValue<'_>, position: [f64; 3], orientation: [f64; 4]) {
    assert_eq!(pose.schema_name, "geometry_msgs/Pose");
    let p = expect_nested(
        pose.field("position").expect("position present"),
        "position",
    );
    assert_eq!(p.schema_name, "geometry_msgs/Point");
    assert_eq!(p.field("x"), Some(&FrameValueKind::F64(position[0])));
    assert_eq!(p.field("y"), Some(&FrameValueKind::F64(position[1])));
    assert_eq!(p.field("z"), Some(&FrameValueKind::F64(position[2])));
    let o = expect_nested(
        pose.field("orientation").expect("orientation present"),
        "orientation",
    );
    assert_eq!(o.schema_name, "geometry_msgs/Quaternion");
    assert_eq!(o.field("x"), Some(&FrameValueKind::F64(orientation[0])));
    assert_eq!(o.field("y"), Some(&FrameValueKind::F64(orientation[1])));
    assert_eq!(o.field("z"), Some(&FrameValueKind::F64(orientation[2])));
    assert_eq!(o.field("w"), Some(&FrameValueKind::F64(orientation[3])));
}

/// Assert a decoded `std_msgs/Header` sub-tree equals hand-written values.
fn assert_header_tree(header: &FrameValue<'_>, sec: i32, nanosec: u32, frame_id: &str) {
    assert_eq!(header.schema_name, "std_msgs/Header");
    assert_eq!(
        header.field("frame_id"),
        Some(&FrameValueKind::Str(frame_id))
    );
    let stamp = expect_nested(header.field("stamp").expect("stamp present"), "stamp");
    assert_eq!(stamp.schema_name, "builtin_interfaces/Time");
    assert_eq!(stamp.field("sec"), Some(&FrameValueKind::I32(sec)));
    assert_eq!(stamp.field("nanosec"), Some(&FrameValueKind::U32(nanosec)));
}

/// The `geometry_msgs/Pose` wire stride: `Point`(24) + `Quaternion`(32). The
/// FIXED-element canonical form carries NO count, so this value alone decides
/// the element count — a codec/walker disagreement about it changes how many
/// elements come back, not just their contents.
const POSE_STRIDE: usize = 56;

/// A `geometry_msgs/PoseStamped` element body: fixed `pose` (56) + one
/// offset-table entry (8) + its `std_msgs/Header` sub-frame (16 + frame_id).
/// Hand-derived from the schema, never read back from the codec.
fn pose_stamped_element_len(frame_id: &str) -> usize {
    POSE_STRIDE + 8 + (16 + frame_id.len())
}

/// The canonical counted framing's own overhead: `u32 count` up front, then a
/// `u32 len` per element.
fn counted_blob_len(element_lens: &[usize]) -> usize {
    4 + element_lens.iter().map(|n| 4 + n).sum::<usize>()
}

/// Hand oracle for the fixed-stride arm: three distinct `Pose`s.
const POSE_ARRAY_ORACLE: [([f64; 3], [f64; 4]); 3] = [
    ([1.0, 2.0, 3.0], [0.0, 0.0, 0.0, 1.0]),
    ([-4.5, 5.25, 6.125], [0.5, -0.5, 0.25, 0.75]),
    ([7.0, 8.0, 9.0], [0.125, 0.25, 0.5, 0.875]),
];

/// Hand oracle for the variable-element arm. The three `frame_id`s have
/// DELIBERATELY different lengths (3 / 10 / 20), so every element body is a
/// different size and the per-element `u32 len` genuinely has to be read — a
/// decoder that assumed a uniform stride could not pass this.
/// One `nav_msgs/Path` element's hand-written values:
/// `(position, orientation, stamp.sec, stamp.nanosec, header.frame_id)`.
type PathElementOracle = ([f64; 3], [f64; 4], i32, u32, &'static str);

const PATH_ORACLE: [PathElementOracle; 3] = [
    ([1.0, 2.0, 3.0], [0.0, 0.0, 0.0, 1.0], 11, 12, "map"),
    (
        [-4.5, 5.25, 6.125],
        [0.5, -0.5, 0.25, 0.75],
        21,
        22,
        "odom_frame",
    ),
    (
        [7.0, 8.0, 9.0],
        [0.125, 0.25, 0.5, 0.875],
        31,
        32,
        "base_link_extra_long",
    ),
];

/// Hand oracle for the `string[]` arm, including the EMPTY-string element
/// (CDR spells it `u32(1) | NUL` — one byte on the CDR side, zero content
/// bytes on the Cerulion side).
const JOINT_NAMES_ORACLE: [&str; 4] = ["shoulder_pan", "", "elbow_flex", "wrist_roll_joint"];

/// Build the `geometry_msgs/PoseArray` CDR body: `header` then the `Pose[]`
/// sequence, in declaration order (the order the codec reads them).
fn pose_array_cdr(poses: &[([f64; 3], [f64; 4])], frame_id: &str) -> Vec<u8> {
    let mut cdr = CdrBody::default();
    cdr.header(1, 2, frame_id);
    cdr.u32(poses.len() as u32);
    for (position, orientation) in poses {
        cdr.pose(*position, *orientation);
    }
    cdr.buf
}

/// Build the `nav_msgs/Path` CDR body. `PoseStamped` is declared
/// `header` then `pose`, so each element is written in that order.
fn path_cdr(elements: &[PathElementOracle]) -> Vec<u8> {
    let mut cdr = CdrBody::default();
    cdr.header(1, 2, "map");
    cdr.u32(elements.len() as u32);
    for (position, orientation, sec, nanosec, frame_id) in elements {
        cdr.header(*sec, *nanosec, frame_id);
        cdr.pose(*position, *orientation);
    }
    cdr.buf
}

/// Build the `sensor_msgs/JointState` CDR body: `header`, `string[] name`,
/// then the three `float64[]` sequences.
fn joint_state_cdr(names: &[&str], position: &[f64]) -> Vec<u8> {
    let mut cdr = CdrBody::default();
    cdr.header(5, 6, "base_link");
    cdr.u32(names.len() as u32);
    for n in names {
        cdr.string(n);
    }
    cdr.f64_seq(position);
    cdr.f64_seq(&[]);
    cdr.f64_seq(&[]);
    cdr.buf
}

/// Decode a hand-built CDR body through the PRODUCTION codec and walk the
/// resulting frame by its own stamped header hash. Also pins that the codec
/// and the walker agree on the schema's wire hash — the two derive it from the
/// same recipe through independent resolvers, and nothing decodes if they
/// disagree.
fn cdr_to_walked<'f>(
    codec: &CdrCodec,
    walker: &FrameWalker,
    qname: &str,
    cdr_body: &[u8],
    frame_out: &'f mut Vec<u8>,
) -> FrameValue<'f> {
    assert_eq!(
        codec.schema_hash(qname),
        walker.schema_hash_for(qname),
        "codec and walker must agree on {qname}'s wire schema_hash"
    );
    *frame_out = codec
        .decode(qname, CdrEndianness::Little, cdr_body, 0, 0)
        .unwrap_or_else(|e| panic!("production CdrCodec::decode of {qname} failed: {e}"));
    let fv = walker
        .walk_by_hash(frame_out)
        .unwrap_or_else(|e| panic!("walk_by_hash of the codec-produced {qname} frame failed: {e}"));
    assert_eq!(fv.schema_name, qname);
    fv
}

/// **Nested[] with a recursively FIXED element** (`geometry_msgs/PoseArray`,
/// `Pose` = 56 B): the codec writes back-to-back fixed sections at
/// `WireLayout::fixed_size` stride with NO count prefix, and the walker
/// recovers the count as `len / stride`. Asserted against the hand-written
/// CDR values, plus the structural byte-length oracle (`3 × 56`) that pins
/// the stride agreement independently of the leaf values.
#[test]
fn cdr_pose_array_fixed_stride_walks_to_hand_written_cdr_values() {
    let codec = builtin_codec();
    let walker = builtin_walker();
    let cdr = pose_array_cdr(&POSE_ARRAY_ORACLE, "map");

    let mut frame = Vec::new();
    let fv = cdr_to_walked(&codec, &walker, "geometry_msgs/PoseArray", &cdr, &mut frame);

    let (elements, raw) = expect_nested_array(fv.field("poses"), "poses");
    assert_eq!(
        raw.len(),
        POSE_ARRAY_ORACLE.len() * POSE_STRIDE,
        "the fixed-element form is pure stride: no count prefix, no per-element length"
    );
    assert_eq!(elements.len(), POSE_ARRAY_ORACLE.len());
    for (i, (position, orientation)) in POSE_ARRAY_ORACLE.iter().enumerate() {
        assert_pose_tree(
            expect_nested(&elements[i], "Pose element"),
            *position,
            *orientation,
        );
    }
    // The untouched sibling still decodes (the array did not eat the frame).
    assert_header_tree(
        expect_nested(fv.field("header").expect("header present"), "header"),
        1,
        2,
        "map",
    );

    // The codec's OWN reference decoder (`encode_cdr_seq`) reads the
    // canonical bytes back to the byte-identical CDR body. The walker and
    // that decoder therefore agree on the same blob, read two different ways.
    assert_eq!(
        codec
            .encode("geometry_msgs/PoseArray", CdrEndianness::Little, &frame)
            .expect("re-encode"),
        cdr,
        "encode(decode(cdr)) == cdr"
    );
}

/// **Nested[] with a VARIABLE element** (`nav_msgs/Path`, `PoseStamped`
/// carries a `Header` string): the codec writes `u32 count` + per element
/// `u32 len` + a headerless element sub-frame, and the walker reads that
/// framing and recurses into each element — including the element's OWN
/// offset table and its nested `Header`. The headline user story
/// (`/plan` → `poses[i].pose.position`) over the real production writer.
#[test]
fn cdr_path_variable_elements_walk_to_hand_written_cdr_values() {
    let codec = builtin_codec();
    let walker = builtin_walker();
    let cdr = path_cdr(&PATH_ORACLE);

    let mut frame = Vec::new();
    let fv = cdr_to_walked(&codec, &walker, "nav_msgs/Path", &cdr, &mut frame);

    let (elements, raw) = expect_nested_array(fv.field("poses"), "poses");
    // Structural oracle, hand-derived from the schema: every element body is
    // a DIFFERENT size, so this catches a framing change even if the leaf
    // values still happen to decode.
    let element_lens: Vec<usize> = PATH_ORACLE
        .iter()
        .map(|(_, _, _, _, frame_id)| pose_stamped_element_len(frame_id))
        .collect();
    assert_eq!(element_lens, vec![83, 90, 100], "80 + frame_id.len() each");
    assert_eq!(raw.len(), counted_blob_len(&element_lens));
    assert_eq!(raw.len(), 289, "4 + (4+83) + (4+90) + (4+100)");
    assert_eq!(
        u32::from_le_bytes([raw[0], raw[1], raw[2], raw[3]]) as usize,
        PATH_ORACLE.len(),
        "the counted form leads with the element count"
    );

    assert_eq!(elements.len(), PATH_ORACLE.len());
    for (i, (position, orientation, sec, nanosec, frame_id)) in PATH_ORACLE.iter().enumerate() {
        let ps = expect_nested(&elements[i], "PoseStamped element");
        assert_eq!(ps.schema_name, "geometry_msgs/PoseStamped");
        assert_pose_tree(
            expect_nested(ps.field("pose").expect("pose present"), "pose"),
            *position,
            *orientation,
        );
        assert_header_tree(
            expect_nested(ps.field("header").expect("header present"), "header"),
            *sec,
            *nanosec,
            frame_id,
        );
    }

    assert_eq!(
        codec
            .encode("nav_msgs/Path", CdrEndianness::Little, &frame)
            .expect("re-encode"),
        cdr,
        "encode(decode(cdr)) == cdr"
    );
}

/// **`string[]`** (`sensor_msgs/JointState.name`): the codec rewrites each
/// CDR string (`u32(len+1) | UTF-8 | NUL`) into the canonical
/// `u32 count` + per element `u32 len` + UTF-8 content form, and the walker
/// reads it back to exactly the hand-written strings — including the EMPTY
/// element, whose CDR spelling (`u32(1) | NUL`) carries zero content bytes.
#[test]
fn cdr_string_array_walks_to_hand_written_cdr_values() {
    let codec = builtin_codec();
    let walker = builtin_walker();
    let positions = [0.25f64, -0.5, 1.75];
    let cdr = joint_state_cdr(&JOINT_NAMES_ORACLE, &positions);

    let mut frame = Vec::new();
    let fv = cdr_to_walked(&codec, &walker, "sensor_msgs/JointState", &cdr, &mut frame);

    let (elements, raw) = expect_nested_array(fv.field("name"), "name");
    let expected: Vec<FrameValueKind<'_>> = JOINT_NAMES_ORACLE
        .iter()
        .map(|s| FrameValueKind::Str(s))
        .collect();
    assert_eq!(elements, expected.as_slice());
    let name_lens: Vec<usize> = JOINT_NAMES_ORACLE.iter().map(|s| s.len()).collect();
    assert_eq!(raw.len(), counted_blob_len(&name_lens));
    assert_eq!(raw.len(), 58, "4 + (4+12) + (4+0) + (4+10) + (4+16)");

    // The sibling primitive arrays are untouched by the element decode: a
    // populated one and two empty ones, all through the same codec pass.
    match fv.field("position") {
        Some(FrameValueKind::PrimArray(pa)) => {
            assert_eq!(pa.elem, PrimType::F64);
            let got: Vec<f64> = pa.iter_f64().collect();
            assert_eq!(got, positions.to_vec());
        }
        other => panic!("expected a PrimArray for position, got {other:?}"),
    }
    for empty in ["velocity", "effort"] {
        match fv.field(empty) {
            Some(FrameValueKind::PrimArray(pa)) => {
                assert_eq!(pa.count, 0, "{empty} is an empty sequence");
                assert!(pa.bytes.is_empty());
            }
            other => panic!("expected a PrimArray for {empty}, got {other:?}"),
        }
    }

    assert_eq!(
        codec
            .encode("sensor_msgs/JointState", CdrEndianness::Little, &frame)
            .expect("re-encode"),
        cdr,
        "encode(decode(cdr)) == cdr"
    );
}

/// The EMPTY and SINGLE-element edges of all three canonical shapes, over the
/// production codec. The two edges matter for different reasons: an empty
/// array must be zero elements and NOT opaque (so a consumer can tell "the
/// producer sent none" from "these bytes are undecodable"), and a single
/// element must still be framed as an array rather than collapsing into a
/// scalar. The two forms' empty encodings DIFFER, and that difference is
/// itself asserted: the fixed-stride form writes nothing at all, while the
/// counted form still writes its `u32 count = 0`.
#[test]
fn cdr_empty_and_single_element_edges_across_all_three_shapes() {
    let codec = builtin_codec();
    let walker = builtin_walker();

    // --- empty: fixed-element form ⇒ a zero-byte blob -------------------
    let mut frame = Vec::new();
    let fv = cdr_to_walked(
        &codec,
        &walker,
        "geometry_msgs/PoseArray",
        &pose_array_cdr(&[], "map"),
        &mut frame,
    );
    let (elements, raw) = expect_nested_array(fv.field("poses"), "empty poses");
    assert!(elements.is_empty(), "zero elements, NOT opaque");
    assert!(
        raw.is_empty(),
        "the fixed-stride form writes nothing for an empty array"
    );

    // --- empty: counted form ⇒ a 4-byte `count = 0` blob ----------------
    let mut frame = Vec::new();
    let fv = cdr_to_walked(&codec, &walker, "nav_msgs/Path", &path_cdr(&[]), &mut frame);
    let (elements, raw) = expect_nested_array(fv.field("poses"), "empty Path poses");
    assert!(elements.is_empty());
    assert_eq!(
        raw,
        &0u32.to_le_bytes(),
        "the counted form always writes its count, even at zero"
    );

    // --- empty: string[] ------------------------------------------------
    let mut frame = Vec::new();
    let fv = cdr_to_walked(
        &codec,
        &walker,
        "sensor_msgs/JointState",
        &joint_state_cdr(&[], &[]),
        &mut frame,
    );
    let (elements, raw) = expect_nested_array(fv.field("name"), "empty name");
    assert!(elements.is_empty());
    assert_eq!(raw, &0u32.to_le_bytes());

    // --- single element: fixed-element form -----------------------------
    let one_pose = [POSE_ARRAY_ORACLE[1]];
    let mut frame = Vec::new();
    let fv = cdr_to_walked(
        &codec,
        &walker,
        "geometry_msgs/PoseArray",
        &pose_array_cdr(&one_pose, "map"),
        &mut frame,
    );
    let (elements, raw) = expect_nested_array(fv.field("poses"), "one pose");
    assert_eq!(raw.len(), POSE_STRIDE);
    assert_eq!(elements.len(), 1);
    assert_pose_tree(
        expect_nested(&elements[0], "the one Pose"),
        POSE_ARRAY_ORACLE[1].0,
        POSE_ARRAY_ORACLE[1].1,
    );

    // --- single element: counted form -----------------------------------
    let one_stamped = [PATH_ORACLE[2]];
    let mut frame = Vec::new();
    let fv = cdr_to_walked(
        &codec,
        &walker,
        "nav_msgs/Path",
        &path_cdr(&one_stamped),
        &mut frame,
    );
    let (elements, raw) = expect_nested_array(fv.field("poses"), "one PoseStamped");
    assert_eq!(
        raw.len(),
        counted_blob_len(&[pose_stamped_element_len(PATH_ORACLE[2].4)])
    );
    assert_eq!(elements.len(), 1);
    let ps = expect_nested(&elements[0], "the one PoseStamped");
    assert_pose_tree(
        expect_nested(ps.field("pose").expect("pose"), "pose"),
        PATH_ORACLE[2].0,
        PATH_ORACLE[2].1,
    );
    assert_header_tree(
        expect_nested(ps.field("header").expect("header"), "header"),
        PATH_ORACLE[2].2,
        PATH_ORACLE[2].3,
        PATH_ORACLE[2].4,
    );

    // --- single element: string[] ---------------------------------------
    let mut frame = Vec::new();
    let fv = cdr_to_walked(
        &codec,
        &walker,
        "sensor_msgs/JointState",
        &joint_state_cdr(&["only_joint"], &[]),
        &mut frame,
    );
    let (elements, raw) = expect_nested_array(fv.field("name"), "one name");
    assert_eq!(elements, &[FrameValueKind::Str("only_joint")]);
    assert_eq!(raw.len(), counted_blob_len(&["only_joint".len()]));
}

/// A `geometry_msgs/TransformStamped` element body, hand-laid from the
/// schema: fixed `transform` (`Vector3`(24) + `Quaternion`(32) = 56) | two
/// offset-table entries (16) | the `Header` sub-frame | the `child_frame_id`
/// UTF-8. Element-internal offsets are ELEMENT-relative, and the accounting
/// is EXACT — the canonical form allows no unaccounted interior bytes.
fn transform_stamped_element(
    translation: [f64; 3],
    rotation: [f64; 4],
    frame_id: &str,
    child_frame_id: &str,
) -> Vec<u8> {
    // std_msgs/Header sub-frame: fixed Time (8) | entry[0] frame_id (8) | UTF-8.
    let mut header = vec![0u8; 16];
    write_offset_entry(&mut header, 8, 0, 16, frame_id.len() as u32);
    header.extend_from_slice(frame_id.as_bytes());

    let mut element = Vec::with_capacity(72);
    for c in translation {
        element.extend_from_slice(&c.to_le_bytes());
    }
    for c in rotation {
        element.extend_from_slice(&c.to_le_bytes());
    }
    assert_eq!(element.len(), 56, "Vector3(24) + Quaternion(32)");
    element.extend_from_slice(&[0u8; 16]); // the two entry slots
    write_offset_entry(&mut element, 56, 0, 72, header.len() as u32);
    write_offset_entry(
        &mut element,
        56,
        1,
        72 + header.len() as u32,
        child_frame_id.len() as u32,
    );
    element.extend_from_slice(&header);
    element.extend_from_slice(child_frame_id.as_bytes());
    assert_eq!(element.len(), 88 + frame_id.len() + child_frame_id.len());
    element
}

/// Wrap element bodies in the canonical counted framing.
fn counted_blob(elements: &[Vec<u8>]) -> Vec<u8> {
    let mut blob = (elements.len() as u32).to_le_bytes().to_vec();
    for e in elements {
        blob.extend_from_slice(&(e.len() as u32).to_le_bytes());
        blob.extend_from_slice(e);
    }
    blob
}

/// Publish `blob` as a `tf2_msgs/TFMessage`'s `transforms` field through the
/// REAL native producer (`loan_proxy` + the generated byte setter + the
/// `OutputProxy::Drop` publish) over an isolated iceoryx2 `TestTransport`,
/// and return the captured frame.
fn publish_tf_transforms_blob(topic: &str, blob: &[u8]) -> Vec<u8> {
    let tt = TestTransport::new();
    let mut publisher = tt.publisher(topic, MaxSliceLen::const_new(64 * 1024), 0);
    let sub = tt.subscriber(topic);
    {
        let mut proxy = publisher.loan_proxy::<TFMessage>().expect("loan TFMessage");
        proxy
            .set_transforms_bytes(blob)
            .expect("set transforms bytes");
    }
    capture_last_frame(&sub)
}

/// The NATIVE-producer half of the cross-check: a real
/// `loan_proxy`/`set_transforms_bytes` publish over real shared memory
/// carries a canonical blob through byte-intact, and the walker decodes it
/// into the hand-written values. This is the one place a native Cerulion
/// producer can emit canonical bytes today — the generated array API is
/// byte-oriented (no typed `push_<field>`; deliberately out of the canonical
/// framing's scope), so the blob is hand-built and remains the oracle.
///
/// `TransformStamped` is the worst-case element shape: TWO variable fields
/// per element, so each element's own offset table is genuinely exercised,
/// with the elements at different sizes.
#[test]
fn native_producer_canonical_tf_blob_walks_to_hand_written_values() {
    let walker = builtin_walker();
    let oracle: [([f64; 3], [f64; 4], &str, &str); 2] = [
        (
            [1.5, -2.25, 3.125],
            [0.0, 0.0, 0.0, 1.0],
            "odom",
            "base_link",
        ),
        (
            [-4.0, 5.5, 6.75],
            [0.5, -0.5, 0.25, 0.75],
            "base_link",
            "imu_link",
        ),
    ];
    let bodies: Vec<Vec<u8>> = oracle
        .iter()
        .map(|(t, r, f, c)| transform_stamped_element(*t, *r, f, c))
        .collect();
    let blob = counted_blob(&bodies);
    assert_eq!(blob.len(), 218, "4 + (4+101) + (4+105)");

    let frame = publish_tf_transforms_blob("fwp/tf", &blob);
    let fv = walker
        .walk_by_hash(&frame)
        .expect("production TFMessage frame must walk by its own header hash");
    assert_eq!(fv.schema_name, "tf2_msgs/TFMessage");

    let (elements, raw) = expect_nested_array(fv.field("transforms"), "transforms");
    assert_eq!(
        raw, &blob,
        "the real SHM publish carried the blob through byte-for-byte"
    );
    assert_eq!(elements.len(), 2);
    for (i, (translation, rotation, frame_id, child_frame_id)) in oracle.iter().enumerate() {
        let ts = expect_nested(&elements[i], "TransformStamped element");
        assert_eq!(ts.schema_name, "geometry_msgs/TransformStamped");
        let tf = expect_nested(ts.field("transform").expect("transform"), "transform");
        let t = expect_nested(tf.field("translation").expect("translation"), "translation");
        assert_eq!(t.field("x"), Some(&FrameValueKind::F64(translation[0])));
        assert_eq!(t.field("y"), Some(&FrameValueKind::F64(translation[1])));
        assert_eq!(t.field("z"), Some(&FrameValueKind::F64(translation[2])));
        let r = expect_nested(tf.field("rotation").expect("rotation"), "rotation");
        assert_eq!(r.field("x"), Some(&FrameValueKind::F64(rotation[0])));
        assert_eq!(r.field("y"), Some(&FrameValueKind::F64(rotation[1])));
        assert_eq!(r.field("z"), Some(&FrameValueKind::F64(rotation[2])));
        assert_eq!(r.field("w"), Some(&FrameValueKind::F64(rotation[3])));
        assert_header_tree(
            expect_nested(ts.field("header").expect("header"), "header"),
            0,
            0,
            frame_id,
        );
        assert_eq!(
            ts.field("child_frame_id"),
            Some(&FrameValueKind::Str(child_frame_id))
        );
    }

    // The native empty-array idiom over the same real path: zero elements,
    // NOT opaque, and the raw slice agrees.
    let empty_frame = publish_tf_transforms_blob("fwp/tf_empty", &[]);
    let empty_fv = walker.walk_by_hash(&empty_frame).expect("walk empty");
    let (elements, raw) = expect_nested_array(empty_fv.field("transforms"), "empty transforms");
    assert!(
        elements.is_empty(),
        "set_transforms_bytes(&[]) ⇒ no elements"
    );
    assert!(raw.is_empty());
}

/// The DEGRADATION contract over the same real producer path: bytes that are
/// NOT canonically framed come back as `NestedArrayOpaque` carrying the
/// original blob byte-for-byte — never a partial or plausible-but-wrong
/// element list. This is what keeps the shipping bespoke consumers
/// (`cerulion_viz::tf`'s TF convention, `pointcloud`'s packed
/// `PointCloud2.fields`) reading exactly the bytes they always read.
///
/// Each case is non-canonical for a DIFFERENT reason, and each reason maps to
/// one of the walker's strictness rules — so this is a positive pin on those
/// rules, not merely "some junk stays opaque". The paired anti-tautology
/// control is the arm above: the SAME publish path with a canonical blob
/// decodes, so the opacity here is attributable to the bytes and not to the
/// producer path.
#[test]
fn native_producer_non_canonical_blobs_stay_opaque_byte_for_byte() {
    let walker = builtin_walker();
    let good = transform_stamped_element([1.0, 2.0, 3.0], [0.0, 0.0, 0.0, 1.0], "odom", "base");

    let mut cases: Vec<(&str, Vec<u8>)> = Vec::new();

    // 1. Trailing slack after the last element: every read is in bounds and
    //    the count is correct, but the blob is not exactly consumed. This is
    //    the single strongest discriminator against a foreign convention
    //    decoding by accident.
    let mut trailing = counted_blob(std::slice::from_ref(&good));
    trailing.extend_from_slice(&[0u8; 4]);
    cases.push(("trailing slack", trailing));

    // 2. A count that overruns the bytes: `count = 2` with one element's
    //    worth of data. The second element's length prefix reads past the
    //    end, so the array degrades whole rather than yielding the one
    //    element it could have read.
    let mut overrun = 2u32.to_le_bytes().to_vec();
    overrun.extend_from_slice(&(good.len() as u32).to_le_bytes());
    overrun.extend_from_slice(&good);
    cases.push(("count overruns the blob", overrun));

    // 3. A truncated final element: the length prefix promises more bytes
    //    than remain.
    let mut truncated = counted_blob(std::slice::from_ref(&good));
    truncated.truncate(truncated.len() - 8);
    cases.push(("final element truncated", truncated));

    // 4. The flat-record shape (a `u32 count` followed by element bodies with
    //    NO per-element length) — the structural family the in-repo bespoke
    //    conventions (`go2_tf`'s TF encoding, the packed `PointCloud2.fields`
    //    blob) belong to. Read as canonical, the four bytes where the first
    //    element's length prefix would be are instead the low half of the
    //    translation's leading `f64` (`1.0` ⇒ `00 00 00 00`), so the length
    //    reads as ZERO — which the canonical form forbids for a variable
    //    element, since such an element always carries at least its own
    //    8-byte offset-table entry.
    let mut flat = 1u32.to_le_bytes().to_vec();
    flat.extend_from_slice(&good);
    assert_eq!(
        &flat[4..8],
        &[0, 0, 0, 0],
        "the mis-read length really is zero here — the stated reason, asserted"
    );
    cases.push(("flat records, no per-element length", flat));

    for (i, (why, blob)) in cases.iter().enumerate() {
        let topic = format!("fwp/opaque{i}");
        let frame = publish_tf_transforms_blob(&topic, blob);
        let fv = walker.walk_by_hash(&frame).expect("walk");
        let bytes = expect_opaque(fv.field("transforms"), why);
        assert_eq!(
            bytes, blob,
            "{why}: the opaque value must carry the ORIGINAL bytes verbatim"
        );
    }
}
