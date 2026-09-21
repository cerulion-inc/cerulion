// SPDX-License-Identifier: AGPL-3.0-only
//! TFMessage → Rerun mapping into an in-memory store, WITHOUT
//! a transport. Two things are proven here:
//!
//! 1. **Walker cross-check** — a hand-built TFMessage wire frame walked by the
//!    generic [`FrameWalker`](cerulion_core::codegen::FrameWalker) surfaces the
//!    opaque `transforms` bytes, and `transforms_from_frame` decodes them to
//!    the SAME records the `go2_tf` codec produced (a hand oracle, not a
//!    self-compare — the frame is built independently of the codec).
//! 2. **Memory recording** — `log_tf_bytes` records one Rerun message per
//!    transform (temporal AND static), and an undecodable blob records NOTHING
//!    (best-effort, warn-and-skip).
//!
//! No iceoryx2 here — frames are hand-built, so this runs parallel-safe.

use cerulion_core::message::ShmMessage;
use cerulion_core::wire::WireHeader;
use cerulion_viz::schema_registry::builtin_walker;
use cerulion_viz::tf::{log_tf_bytes, transforms_from_frame, UnknownFrameLog};
use go2_tf::{encode_tf_transforms, TfTransform, IDENTITY_QUAT};
use native_ros2_messages::tf2_msgs::TFMessage;

fn memory() -> (rerun::RecordingStream, rerun::sink::MemorySinkStorage) {
    rerun::RecordingStreamBuilder::new("go2_test")
        .recording_id("go2_tf_arch_test")
        .memory()
        .expect("memory sink")
}

/// Hand-build a FULL TFMessage wire frame carrying `transforms_bytes` — the
/// same layout as the pipe test's `tfmessage_oracle_frame`
/// (`[WireHeader][OffsetEntry{offset:8, len}][transforms bytes]`). Independent
/// of `encode_tf_transforms`, so a walk-then-decode is a real cross-check.
fn build_tf_frame(seq: u32, ts: u64, transforms_bytes: &[u8]) -> Vec<u8> {
    assert_eq!(TFMessage::WIRE_FIXED_SIZE, 0);
    assert_eq!(TFMessage::VARIABLE_FIELD_COUNT, 1);
    let offset = (TFMessage::WIRE_FIXED_SIZE + 8 * TFMessage::VARIABLE_FIELD_COUNT) as u32;
    let length = transforms_bytes.len() as u32;

    let mut payload = Vec::with_capacity(offset as usize + transforms_bytes.len());
    payload.extend_from_slice(&offset.to_le_bytes());
    payload.extend_from_slice(&length.to_le_bytes());
    payload.extend_from_slice(transforms_bytes);

    let header = WireHeader {
        schema_hash: TFMessage::SCHEMA_HASH,
        total_size: (WireHeader::SIZE + payload.len()) as u32,
        offset_table_offset: (WireHeader::SIZE + TFMessage::WIRE_FIXED_SIZE) as u32,
        offset_table_count: TFMessage::VARIABLE_FIELD_COUNT as u32,
        sequence: seq,
        timestamp_ns: ts,
    };
    let mut frame = vec![0u8; WireHeader::SIZE + payload.len()];
    header.write_to_buf(&mut frame[..WireHeader::SIZE]);
    frame[WireHeader::SIZE..].copy_from_slice(&payload);
    frame
}

/// The Go2 tree as three transforms (odom→base, base→lidar, base→camera).
fn go2_tree() -> Vec<TfTransform> {
    vec![
        TfTransform::new("odom", "base", [0.0, 0.0, 0.0], IDENTITY_QUAT, 5, 0),
        TfTransform::new("base", "lidar", [0.17, 0.0, 0.11], IDENTITY_QUAT, 5, 0),
        TfTransform::new("base", "camera", [0.27, 0.0, 0.05], IDENTITY_QUAT, 5, 0),
    ]
}

#[test]
fn walker_surfaces_transforms_bytes_that_codec_decodes() {
    let walker = builtin_walker();
    let records = go2_tree();
    let frame = build_tf_frame(0, 42_000, &encode_tf_transforms(&records));

    // The walker path: extract + decode the opaque `transforms` array.
    let (ts, decoded) = transforms_from_frame(&walker, &frame).expect("walk + decode");
    assert_eq!(ts, 42_000, "wire timestamp comes from the header");
    assert_eq!(
        decoded, records,
        "walker → codec decodes to the hand oracle"
    );
}

#[test]
fn empty_transforms_frame_walks_to_zero_records() {
    let walker = builtin_walker();
    // The empty-array edge: set_transforms_bytes(&[]) → offset entry length 0.
    let frame = build_tf_frame(0, 7, &[]);
    let (ts, decoded) = transforms_from_frame(&walker, &frame).expect("walk empty");
    assert_eq!(ts, 7);
    assert!(decoded.is_empty(), "empty transforms array → zero records");
}

/// The pure-function half of the canonical-framing decode contract. A
/// CANONICALLY-framed `transforms` array (the shape the `ros2 attach` CDR
/// ingress writes, and the shape the canonical framing taught the walker to DECODE) must
/// reach this module's bespoke decoder as its RAW bytes and surface a LOUD
/// decode error — never a silent "zero transforms, everything is fine".
///
/// The first change handled only the EMPTY decoded array, so a non-empty one fell to the
/// unexpected-kind arm and returned `Ok((ts, vec![]))`: the frame's transforms
/// vanished and the diagnosis blamed walker drift. Before the canonical framing the identical
/// bytes reached the decoder and errored, which is what this restores.
#[test]
fn canonically_framed_transforms_reach_the_decoder_and_error_loudly() {
    let walker = builtin_walker();
    // One canonical element: `u32 count` + `u32 len` + a headerless
    // `TransformStamped` sub-frame (`[fixed 56][table 16][header][child]`),
    // whose `header` is `[fixed 8][table 8][frame_id]`.
    let frame_id = "odom";
    let child = "base";
    let mut hdr = vec![0u8; 16];
    hdr[8..12].copy_from_slice(&16u32.to_le_bytes());
    hdr[12..16].copy_from_slice(&(frame_id.len() as u32).to_le_bytes());
    hdr.extend_from_slice(frame_id.as_bytes());
    let mut body = Vec::new();
    for v in [0.1f64, 0.2, 0.3] {
        body.extend_from_slice(&v.to_le_bytes());
    }
    for v in IDENTITY_QUAT {
        body.extend_from_slice(&v.to_le_bytes());
    }
    assert_eq!(body.len(), 56);
    body.extend_from_slice(&72u32.to_le_bytes());
    body.extend_from_slice(&(hdr.len() as u32).to_le_bytes());
    body.extend_from_slice(&(72 + hdr.len() as u32).to_le_bytes());
    body.extend_from_slice(&(child.len() as u32).to_le_bytes());
    body.extend_from_slice(&hdr);
    body.extend_from_slice(child.as_bytes());
    let mut blob = 1u32.to_le_bytes().to_vec();
    blob.extend_from_slice(&(body.len() as u32).to_le_bytes());
    blob.extend_from_slice(&body);

    let frame = build_tf_frame(0, 9_000, &blob);
    // Precondition: the walker really DECODES it (else this arm is vacuous) and
    // still carries the field's bytes verbatim.
    let fv = walker.walk_by_hash(&frame).expect("walk");
    match fv.field("transforms") {
        Some(cerulion_core::codegen::FrameValueKind::NestedArray { elements, raw }) => {
            assert_eq!(elements.len(), 1);
            assert_eq!(*raw, blob.as_slice());
        }
        other => panic!("expected a DECODED canonical array, got {other:?}"),
    }

    // THE PIN: loud error, not a silent empty decode.
    let err = transforms_from_frame(&walker, &frame)
        .expect_err("canonical bytes are not this module's convention");
    let text = err.to_string();
    assert!(
        text.contains("decode"),
        "the error must name the DECODE failure, got: {text}"
    );
}

#[test]
fn log_tf_bytes_records_one_message_per_transform_temporal() {
    let (rec, storage) = memory();
    let mut latch = UnknownFrameLog::new();
    let records = go2_tree();
    let bytes = encode_tf_transforms(&records);

    let baseline = storage.num_msgs();
    log_tf_bytes(&rec, &bytes, 1_000, false, &mut latch);
    rec.flush_blocking().expect("flush");
    let delta = storage.num_msgs() - baseline;
    assert_eq!(
        delta, 3,
        "3 transforms → 3 temporal Transform3D messages (got {delta})"
    );
}

#[test]
fn log_tf_bytes_records_static_transforms() {
    let (rec, storage) = memory();
    let mut latch = UnknownFrameLog::new();
    let mounts = go2_tf::static_mounts(); // base→lidar, base→camera
    let bytes = encode_tf_transforms(&mounts);

    let baseline = storage.num_msgs();
    // is_static = true → log_static (Rerun's timeless API).
    log_tf_bytes(&rec, &bytes, 0, true, &mut latch);
    rec.flush_blocking().expect("flush");
    let delta = storage.num_msgs() - baseline;
    assert_eq!(
        delta, 2,
        "2 static mount transforms → 2 static Transform3D messages (got {delta})"
    );
}

#[test]
fn undecodable_blob_records_nothing_and_does_not_panic() {
    let (rec, storage) = memory();
    let mut latch = UnknownFrameLog::new();
    // A 3-byte blob can't even hold the count prefix → decode Err → skip.
    let baseline = storage.num_msgs();
    log_tf_bytes(&rec, &[0xFF, 0x00, 0x01], 1, false, &mut latch);
    rec.flush_blocking().expect("flush");
    assert_eq!(
        storage.num_msgs() - baseline,
        0,
        "an undecodable transforms blob logs nothing (warn-and-skip)"
    );
}
