// SPDX-License-Identifier: AGPL-3.0-only
//! Generated readers never panic on a source at an arbitrary address.
//!
//! A variable nested field written with `with_<field>` is flushed at the
//! writer's cursor, after whatever variable fields were written before it, so
//! its sub-frame can start at any offset inside the payload. The frame crosses
//! zenoh and `bag play` unchanged, so a receiver sees the same offset. These
//! tests build such frames and read them the way a node does:
//!
//! - `HeaderShm::from_bytes(image.header_bytes())` on the `sensor_msgs/Image`
//!   frame from the report (1 MiB of `mono8` data puts the header sub-frame at
//!   payload offset 1 mod 4), and on frames whose header lands at every
//!   offset mod 8;
//! - the whole payload copied into a buffer at every alignment, as a frame
//!   arrives after a copy out of a network buffer or a bag record;
//! - random bytes at random offsets, read through every generated reader
//!   (a property test: no input panics);
//! - a sub-frame aligned to its reader's `READ_ALIGN` but not to 8 (the
//!   `LaserScan` header) is read in place, with no copy;
//! - `<Fixed>Shm::read_unaligned` over the elements of a `PoseArray`, whose
//!   back-to-back fixed sections start wherever the writer's cursor was.
//!
//! No transport is involved: the payload is written by the generated writer
//! over a plain 8-aligned buffer, exactly as the publisher's loan path does
//! after the 40-byte `WireHeader`.

use std::sync::Arc;

use cerulion_core::message::ShmMessage;
use cerulion_core::shm_runtime::{u64s_as_bytes, u64s_as_bytes_mut};
use cerulion_core::wire::MaxPayloadCapacity;
use native_ros2_messages::geometry_msgs::{PoseArray, PoseArrayShm, PoseShm};
use native_ros2_messages::sensor_msgs::{Image, ImageShm, LaserScan, LaserScanShm};
use native_ros2_messages::std_msgs::{Float64MultiArray, Float64MultiArrayShm, HeaderShm};
use proptest::prelude::*;

const STAMP_SEC: i32 = 1_761_609_600;
const STAMP_NSEC: u32 = 123_456_789;
const FRAME_ID: &str = "camera_optical_frame";

/// Write a `sensor_msgs/Image` payload with the documented nested form and
/// return the 8-aligned backing plus the payload length.
fn write_image(encoding: &str, data: &[u8]) -> (Vec<u64>, usize) {
    let mut backing = vec![0u64; (data.len() + 4096).div_ceil(8)];
    let bytes = u64s_as_bytes_mut(&mut backing);
    let cap = MaxPayloadCapacity::const_new(bytes.len() as u32);
    let mut w = Image::build_writer(bytes, cap, Arc::from("/camera/image"));
    w.height = 1024;
    w.width = 1024;
    w.step = 1024;
    w.with_header(|h| {
        h.stamp.sec = STAMP_SEC;
        h.stamp.nanosec = STAMP_NSEC;
        h.set_frame_id(FRAME_ID)
    })
    .unwrap();
    w.set_encoding(encoding).unwrap();
    w.set_data(data).unwrap();
    Image::flush_staged_nested(&mut w).unwrap();
    assert!(Image::all_variables_written(&w) && !Image::has_overflow(&w));
    let n = Image::payload_wire_size(&w);
    drop(w);
    (backing, n)
}

/// Offset of `inner` inside `outer`, in bytes.
fn offset_in(outer: &[u8], inner: &[u8]) -> usize {
    inner.as_ptr() as usize - outer.as_ptr() as usize
}

fn assert_header_reads_back(header: &HeaderShm<'_>) {
    assert_eq!(header.stamp.sec, STAMP_SEC);
    assert_eq!(header.stamp.nanosec, STAMP_NSEC);
    assert_eq!(header.frame_id().unwrap(), FRAME_ID);
}

/// The frame from the report: 1 MiB of `mono8` data. The header sub-frame
/// starts at payload offset 1 048 621 (1 mod 4), and reading it used to panic
/// with "misaligned source pointer (need align 4)".
#[test]
fn image_mono8_header_reads_back_from_a_misaligned_sub_frame() {
    let data = vec![0x5a_u8; 1 << 20];
    let (backing, n) = write_image("mono8", &data);
    let payload = &u64s_as_bytes(&backing)[..n];
    let image = ImageShm::from_bytes(payload);
    let header_bytes = image.header_bytes();
    let offset = offset_in(payload, header_bytes);
    assert_eq!(offset, 1_048_621, "the frame from the report");
    assert_eq!(offset % 4, 1, "the sub-frame is misaligned for HeaderShm");

    let header = HeaderShm::from_bytes(header_bytes);
    assert_header_reads_back(&header);
    assert_eq!(image.encoding().unwrap(), "mono8");
    assert_eq!(image.data(), &data[..]);
}

/// Encodings of length 0 to 7 put the header sub-frame at every offset mod 8;
/// the header reads back from each.
#[test]
fn header_reads_back_at_every_offset_mod_8() {
    let data = vec![0x11_u8; 1000];
    let mut residues = [false; 8];
    for len in 0..8 {
        let encoding = "x".repeat(len);
        let (backing, n) = write_image(&encoding, &data);
        let payload = &u64s_as_bytes(&backing)[..n];
        let image = ImageShm::from_bytes(payload);
        let header_bytes = image.header_bytes();
        residues[offset_in(payload, header_bytes) % 8] = true;
        assert_header_reads_back(&HeaderShm::from_bytes(header_bytes));
        assert_eq!(image.encoding().unwrap(), encoding);
    }
    assert_eq!(residues, [true; 8], "every offset mod 8 was exercised");
}

/// A frame copied into a buffer at any alignment (a zenoh receive buffer, a bag
/// record) reads back field for field, the nested header included.
#[test]
fn image_copied_to_every_alignment_reads_back() {
    let data: Vec<u8> = (0..4096u32).map(|i| (i * 7) as u8).collect();
    let (backing, n) = write_image("mono16", &data);
    let payload = &u64s_as_bytes(&backing)[..n];
    for skew in 0..8 {
        let mut copy = vec![0u64; n.div_ceil(8) + 1];
        let dst = &mut u64s_as_bytes_mut(&mut copy)[skew..skew + n];
        dst.copy_from_slice(payload);
        let image = ImageShm::from_bytes(dst);
        assert_eq!((image.height, image.width, image.step), (1024, 1024, 1024));
        assert_eq!(image.encoding().unwrap(), "mono16");
        assert_eq!(image.data(), &data[..]);
        assert_header_reads_back(&HeaderShm::from_bytes(image.header_bytes()));
        assert_eq!(image.snapshot(), ImageShm::from_bytes(payload).snapshot());
    }
}

/// A source shorter than the fixed section and offset table reads as zero
/// instead of panicking.
#[test]
fn truncated_source_reads_as_zero() {
    let (backing, n) = write_image("mono8", &[1, 2, 3]);
    let payload = &u64s_as_bytes(&backing)[..n];
    for len in [0, 1, 7, 15, 16, Image::WIRE_FIXED_SIZE + 7] {
        let image = ImageShm::from_bytes(&payload[..len]);
        assert_eq!(image.encoding().unwrap(), "");
        assert!(image.data().is_empty());
        assert!(image.header_bytes().is_empty());
        let header = HeaderShm::from_bytes(image.header_bytes());
        assert_eq!(header.stamp.sec, 0);
        assert_eq!(header.frame_id().unwrap(), "");
    }
    // A prefix that holds the fixed section reads the fixed fields.
    let image = ImageShm::from_bytes(&payload[..Image::WIRE_FIXED_SIZE]);
    assert_eq!(image.height, 1024);
}

/// `READ_ALIGN` is the larger of the fixed section's alignment and the widest
/// primitive-array element, never the frame alignment by default.
#[test]
fn read_align_is_per_schema() {
    // Header: `Time` (i32, u32) and a string.
    assert_eq!(HeaderShm::READ_ALIGN, 4);
    // Image: u32 fixed fields, u8 arrays.
    assert_eq!(ImageShm::READ_ALIGN, 4);
    // LaserScan: f32 fixed fields, f32 arrays.
    assert_eq!(LaserScanShm::READ_ALIGN, 4);
    // Float64MultiArray: an f64 array.
    assert_eq!(Float64MultiArrayShm::READ_ALIGN, 8);
}

/// Write a `sensor_msgs/LaserScan` with `n` ranges and `n` intensities and the
/// header written by `with_header` (flushed last, after both arrays).
fn write_laser_scan(n: usize) -> (Vec<u64>, usize) {
    let mut backing = vec![0u64; 4096];
    let bytes = u64s_as_bytes_mut(&mut backing);
    let cap = MaxPayloadCapacity::const_new(bytes.len() as u32);
    let mut w = LaserScan::build_writer(bytes, cap, Arc::from("/scan"));
    w.with_header(|h| {
        h.stamp.sec = STAMP_SEC;
        h.stamp.nanosec = STAMP_NSEC;
        h.set_frame_id(FRAME_ID)
    })
    .unwrap();
    w.set_ranges(&vec![1.5f32; n]).unwrap();
    w.set_intensities(&vec![2.5f32; n]).unwrap();
    LaserScan::flush_staged_nested(&mut w).unwrap();
    let len = LaserScan::payload_wire_size(&w);
    drop(w);
    (backing, len)
}

/// Equal-length ranges and intensities put the header sub-frame at 4 mod 8.
/// `HeaderShm` needs 4, so the header is read in place: `frame_id` points
/// into the parent payload, not into a copy.
#[test]
fn laser_scan_header_at_4_mod_8_is_read_in_place() {
    for n in [360usize, 1081] {
        let (backing, len) = write_laser_scan(n);
        let payload = &u64s_as_bytes(&backing)[..len];
        let scan = LaserScanShm::from_bytes(payload);
        let header_bytes = scan.header_bytes();
        assert_eq!(offset_in(payload, header_bytes) % 8, 4, "n={n}");
        let header = HeaderShm::from_bytes(header_bytes);
        assert_header_reads_back(&header);
        let frame_id = header.frame_id().unwrap().as_ptr() as usize;
        let start = payload.as_ptr() as usize;
        assert!(
            (start..start + payload.len()).contains(&frame_id),
            "n={n}: frame_id was read from a copy, not in place"
        );
        assert_eq!(scan.ranges(), &vec![1.5f32; n][..]);
        assert_eq!(scan.intensities(), &vec![2.5f32; n][..]);
    }
}

/// A schema with an f64 array needs 8: at 4 mod 8 it is read from a copy and
/// its `data` reads back (an in-place read would hand back an empty slice).
#[test]
fn f64_array_at_4_mod_8_reads_back_from_a_copy() {
    let data: Vec<f64> = (0..37).map(|i| i as f64 * 0.25 - 3.0).collect();
    let mut frame = vec![0u64; 256];
    let bytes = u64s_as_bytes_mut(&mut frame);
    let cap = MaxPayloadCapacity::const_new(bytes.len() as u32);
    let mut w = Float64MultiArray::build_writer(bytes, cap, Arc::from("/f64"));
    w.set_data(&data).unwrap();
    Float64MultiArray::flush_staged_nested(&mut w).unwrap();
    let n = Float64MultiArray::payload_wire_size(&w);
    drop(w);
    let payload = &u64s_as_bytes(&frame)[..n];
    assert_eq!(Float64MultiArrayShm::from_bytes(payload).data(), &data[..]);
    let mut backing = vec![0u64; n.div_ceil(8) + 1];
    let dst = &mut u64s_as_bytes_mut(&mut backing)[4..4 + n];
    dst.copy_from_slice(payload);
    let array = Float64MultiArrayShm::from_bytes(dst);
    assert_eq!(array.data(), &data[..]);
    let start = dst.as_ptr() as usize;
    assert!(!(start..start + n).contains(&(array.data().as_ptr() as usize)));
}

/// Write a `geometry_msgs/PoseArray` whose `poses` blob starts after the
/// header sub-frame (so at an offset set by `frame_id`'s length).
fn write_pose_array(frame_id: &str, poses: &[[f64; 7]]) -> (Vec<u64>, usize) {
    let mut blob = Vec::new();
    for p in poses {
        for v in p {
            blob.extend_from_slice(&v.to_le_bytes());
        }
    }
    let mut backing = vec![0u64; 512];
    let bytes = u64s_as_bytes_mut(&mut backing);
    let cap = MaxPayloadCapacity::const_new(bytes.len() as u32);
    let mut w = PoseArray::build_writer(bytes, cap, Arc::from("/poses"));
    w.with_header(|h| h.set_frame_id(frame_id)).unwrap();
    PoseArray::flush_staged_nested(&mut w).unwrap();
    w.set_poses_bytes(&blob).unwrap();
    let n = PoseArray::payload_wire_size(&w);
    drop(w);
    (backing, n)
}

/// The elements of an array of fixed nested messages are back-to-back fixed
/// sections with no alignment guarantee; `read_unaligned` reads each one.
#[test]
fn pose_array_elements_read_unaligned_at_every_offset() {
    let poses = [
        [1.0, 2.0, 3.0, 0.0, 0.0, 0.0, 1.0],
        [-4.5, 5.25, 6.125, 0.5, 0.5, 0.5, 0.5],
    ];
    let stride = std::mem::size_of::<PoseShm>();
    assert_eq!(stride, 56);
    let mut residues = [false; 8];
    for len in 0..8 {
        let frame_id = "m".repeat(len);
        let (backing, n) = write_pose_array(&frame_id, &poses);
        let payload = &u64s_as_bytes(&backing)[..n];
        let array = PoseArrayShm::from_bytes(payload);
        let blob = array.poses_bytes();
        residues[offset_in(payload, blob) % 8] = true;
        assert_eq!(blob.len(), stride * poses.len());
        for (i, expected) in poses.iter().enumerate() {
            let pose = PoseShm::read_unaligned(&blob[i * stride..]).unwrap();
            let got = [
                pose.position.x,
                pose.position.y,
                pose.position.z,
                pose.orientation.x,
                pose.orientation.y,
                pose.orientation.z,
                pose.orientation.w,
            ];
            assert_eq!(&got, expected);
        }
        assert!(PoseShm::read_unaligned(&blob[..stride - 1]).is_none());
        assert_eq!(
            HeaderShm::from_bytes(array.header_bytes())
                .frame_id()
                .unwrap(),
            frame_id
        );
    }
    assert_eq!(residues, [true; 8], "every offset mod 8 was exercised");
}

/// Read every field of an `Image` and of its nested `Header`.
fn read_everything(bytes: &[u8]) {
    let image = ImageShm::from_bytes(bytes);
    let _ = (image.height, image.width, image.is_bigendian, image.step);
    let _ = image.encoding();
    let _ = image.data();
    let _ = image.snapshot();
    let header = HeaderShm::from_bytes(image.header_bytes());
    let _ = (header.stamp.sec, header.stamp.nanosec, header.frame_id());
    let _ = header.snapshot();
    let header = HeaderShm::from_bytes(bytes);
    let _ = (header.stamp.sec, header.frame_id());
    let array = PoseArrayShm::from_bytes(bytes);
    let _ = PoseShm::read_unaligned(array.poses_bytes());
    let _ = PoseShm::read_unaligned(bytes);
}

proptest! {
    #![proptest_config(ProptestConfig::with_cases(2048))]

    /// Random bytes at a random offset 0..8 never panic any reader.
    #[test]
    fn random_bytes_at_any_offset_never_panic(
        skew in 0usize..8,
        bytes in prop::collection::vec(any::<u8>(), 0..256),
    ) {
        let mut backing = vec![0u64; bytes.len().div_ceil(8) + 1];
        let dst = &mut u64s_as_bytes_mut(&mut backing)[skew..skew + bytes.len()];
        dst.copy_from_slice(&bytes);
        read_everything(dst);
    }

    /// A real frame with random bytes overwritten, at a random offset 0..8,
    /// never panics any reader; untouched frames read back exactly.
    #[test]
    fn corrupted_frames_at_any_offset_never_panic(
        skew in 0usize..8,
        enc_len in 0usize..8,
        edits in prop::collection::vec((any::<u16>(), any::<u8>()), 0..16),
    ) {
        let encoding = "e".repeat(enc_len);
        let (frame, n) = write_image(&encoding, &[7u8; 300]);
        let mut backing = vec![0u64; n.div_ceil(8) + 1];
        let dst = &mut u64s_as_bytes_mut(&mut backing)[skew..skew + n];
        dst.copy_from_slice(&u64s_as_bytes(&frame)[..n]);
        if edits.is_empty() {
            let image = ImageShm::from_bytes(dst);
            prop_assert_eq!(image.encoding().unwrap(), encoding.as_str());
            assert_header_reads_back(&HeaderShm::from_bytes(image.header_bytes()));
        }
        for (at, value) in edits {
            dst[at as usize % n] = value;
        }
        read_everything(dst);
    }
}
