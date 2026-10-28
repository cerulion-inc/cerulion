//! Wire-format microbenchmark: Cerulion vs FlatBuffers vs Protobuf (prost) vs
//! Cap'n Proto, on the same logical content.
//!
//! Every `*_encode_*` function writes one message into a caller-provided,
//! preallocated buffer (the shared-memory slot case) and returns the byte range
//! the message occupies in it. Every `*_read_*` function takes bytes already in
//! memory and reads 3 scalar fields plus the payload's length and first byte,
//! including whatever check the format needs before the read.

#![allow(clippy::too_many_arguments)]

use std::sync::Arc;

use cerulion_core::message::ShmMessage;
use cerulion_core::wire::{MaxPayloadCapacity, StepTag, WireHeader};

include!(concat!(env!("OUT_DIR"), "/bench_msgs.rs"));
pub use bench_msgs::{ImuSample, ImuSampleShm};

// Vendored: `flatc --rust -o src schema/wire.fbs`, flatc 25.12.19 (README.md).
#[rustfmt::skip]
#[allow(clippy::all, unused_imports, dead_code, mismatched_lifetime_syntaxes)]
#[path = "wire_generated.rs"]
mod wire_generated;
pub use wire_generated::wirecmp::fb;

// Vendored: `capnp compile -orust:src --src-prefix=schema schema/wire.capnp`,
// capnp 1.5.0 with the capnpc-rust 0.27.0 plugin (README.md).
#[rustfmt::skip]
#[allow(clippy::all, unused_imports, dead_code, unused_parens, mismatched_lifetime_syntaxes)]
#[path = "wire_capnp.rs"]
pub mod wire_capnp;
pub mod pb;

use native_ros2_messages::sensor_msgs::{Image, PointCloud2, PointField};

// ---------------------------------------------------------------------------
// Shared logical content
// ---------------------------------------------------------------------------

pub const FRAME_ID: &str = "camera_optical_frame";
pub const STAMP_SEC: i32 = 1_761_609_600;
pub const STAMP_NSEC: u32 = 123_456_789;

pub const IMU_STAMP_NS: u64 = 1_761_609_600_123_456_789;
pub const IMU_SEQ: u32 = 42;
pub const IMU_STATUS: u32 = 1;
// A unit quaternion (90 degrees about z), not a stand-in for FRAC_1_SQRT_2.
#[allow(clippy::approx_constant)]
pub const IMU_ORI: [f64; 4] = [0.0, 0.0, 0.707_106_781_186_547_5, 0.707_106_781_186_547_6];
pub const IMU_AV: [f64; 3] = [0.01, -0.02, 0.03];
pub const IMU_LA: [f64; 3] = [0.1, 0.2, 9.81];

/// Image geometry plus its payload.
pub struct ImageSrc {
    pub height: u32,
    pub width: u32,
    pub encoding: &'static str,
    pub step: u32,
    pub data: Vec<u8>,
    /// The same payload as a `bytes::Bytes` (for prost's Bytes variant).
    pub data_bytes: bytes::Bytes,
}

pub struct CloudSrc {
    pub height: u32,
    pub width: u32,
    pub point_step: u32,
    pub row_step: u32,
    pub fields: [(&'static str, u32, u8, u32); 4],
    pub data: Vec<u8>,
    pub data_bytes: bytes::Bytes,
}

/// Deterministic, incompressible-looking payload (xorshift64*).
pub fn payload(n: usize, seed: u64) -> Vec<u8> {
    let mut x = seed | 1;
    let mut v = Vec::with_capacity(n);
    while v.len() < n {
        x ^= x >> 12;
        x ^= x << 25;
        x ^= x >> 27;
        let w = x.wrapping_mul(0x2545_F491_4F6C_DD1D);
        for b in w.to_le_bytes() {
            if v.len() < n {
                v.push(b);
            }
        }
    }
    v
}

/// 1 MiB: 1024 x 1024 mono8. 6 MiB: 2048 x 1024 rgb8.
pub fn image_src(mib: usize) -> ImageSrc {
    let (height, width, encoding, step) = match mib {
        1 => (1024, 1024, "mono8", 1024),
        6 => (1024, 2048, "rgb8", 2048 * 3),
        _ => panic!("1 or 6 MiB"),
    };
    let data = payload(mib << 20, 0x5EED_0000 + mib as u64);
    assert_eq!(data.len(), (height * step) as usize);
    ImageSrc {
        height,
        width,
        encoding,
        step,
        data_bytes: bytes::Bytes::from(data.clone()),
        data,
    }
}

/// 4 MiB: 262 144 points of x, y, z, intensity as float32 (point_step 16).
pub fn cloud_src() -> CloudSrc {
    let data = payload(4 << 20, 0xC10D);
    CloudSrc {
        height: 1,
        width: 262_144,
        point_step: 16,
        row_step: 4 << 20,
        fields: [
            ("x", 0, 7, 1),
            ("y", 4, 7, 1),
            ("z", 8, 7, 1),
            ("intensity", 12, 7, 1),
        ],
        data_bytes: bytes::Bytes::from(data.clone()),
        data,
    }
}

/// An 8-byte-aligned, zeroed buffer of at least `n` bytes. Shared-memory
/// slots are 8-aligned; Cerulion and Cap'n Proto require it.
pub struct AlignedBuf(Vec<u64>);
impl AlignedBuf {
    pub fn new(n: usize) -> Self {
        AlignedBuf(vec![0u64; n.div_ceil(8)])
    }
    pub fn bytes(&self) -> &[u8] {
        cerulion_core::shm_runtime::u64s_as_bytes(&self.0)
    }
    pub fn bytes_mut(&mut self) -> &mut [u8] {
        cerulion_core::shm_runtime::u64s_as_bytes_mut(&mut self.0)
    }
    /// An aligned copy of `src`, the "bytes already in memory" input of a read.
    pub fn from_slice(src: &[u8]) -> Self {
        let mut b = Self::new(src.len());
        b.bytes_mut()[..src.len()].copy_from_slice(src);
        b
    }
}

/// Slot size used for every encode: payload plus 64 KiB of headroom.
pub fn slot_len(payload: usize) -> usize {
    payload + (64 << 10)
}

// ---------------------------------------------------------------------------
// Cerulion: the publisher's loan path, minus iceoryx2
// ---------------------------------------------------------------------------
//
// Mirrors CerulionPublisher::loan (transport/publisher.rs:1406-1429: stamp the
// WireHeader, then T::build_writer over the post-header bytes) and
// OutputProxy::drop (transport/output_proxy.rs:276, 497-519:
// flush_staged_nested, then total_size = 40 + payload_wire_size and the
// sequence stamped at [8..12] and [20..24]).

#[inline(always)]
fn cer_stamp_header<T: ShmMessage>(slot: &mut [u8], ts: u64) {
    let header = WireHeader {
        schema_hash: T::SCHEMA_HASH,
        total_size: slot.len() as u32,
        offset_table_offset: (WireHeader::SIZE + T::WIRE_FIXED_SIZE) as u32,
        offset_table_count: T::VARIABLE_FIELD_COUNT as u32,
        sequence: 0,
        timestamp_ns: ts,
        step_tag: StepTag::UNTAGGED,
    };
    header.write_to_buf(&mut slot[..WireHeader::SIZE]);
}

#[inline(always)]
fn cer_commit(slot: &mut [u8], payload_size: usize, seq: u32) -> usize {
    let total = WireHeader::SIZE + payload_size;
    slot[8..12].copy_from_slice(&(total as u32).to_le_bytes());
    slot[20..24].copy_from_slice(&seq.to_le_bytes());
    total
}

/// `topic` is the publisher's own `Arc<str>`, cloned per loan as the
/// publisher does (a refcount increment, no allocation).
pub fn cer_encode_imu(slot: &mut [u8], topic: &Arc<str>) -> usize {
    cer_stamp_header::<ImuSample>(slot, IMU_STAMP_NS);
    let cap = MaxPayloadCapacity::const_new((slot.len() - WireHeader::SIZE) as u32);
    let w = ImuSample::build_writer(&mut slot[WireHeader::SIZE..], cap, Arc::clone(topic));
    w.stamp_ns = IMU_STAMP_NS;
    w.seq = IMU_SEQ;
    w.status = IMU_STATUS;
    w.orientation = IMU_ORI;
    w.angular_velocity = IMU_AV;
    w.linear_acceleration = IMU_LA;
    let n = ImuSample::payload_wire_size(&w);
    cer_commit(slot, n, IMU_SEQ)
}

pub fn cer_encode_image(slot: &mut [u8], src: &ImageSrc, topic: &Arc<str>) -> usize {
    cer_stamp_header::<Image>(slot, IMU_STAMP_NS);
    let cap = MaxPayloadCapacity::const_new((slot.len() - WireHeader::SIZE) as u32);
    let mut w = Image::build_writer(&mut slot[WireHeader::SIZE..], cap, Arc::clone(topic));
    w.height = src.height;
    w.width = src.width;
    w.is_bigendian = 0;
    w.step = src.step;
    w.with_header(|h| {
        h.stamp.sec = STAMP_SEC;
        h.stamp.nanosec = STAMP_NSEC;
        h.set_frame_id(FRAME_ID)
    })
    .unwrap();
    w.set_encoding(src.encoding).unwrap();
    w.set_data(&src.data).unwrap();
    Image::flush_staged_nested(&mut w).unwrap();
    assert!(Image::all_variables_written(&w) && !Image::has_overflow(&w));
    let n = Image::payload_wire_size(&w);
    drop(w);
    cer_commit(slot, n, 7)
}

/// `PointField[]` in the canonical element encoding (frame_walker.rs:44-50):
/// u32 count, then per element u32 length + a headerless PointField sub-frame,
/// each sub-frame written by the generated PointField writer over a scratch.
fn cer_point_fields_blob(src: &CloudSrc, topic: &Arc<str>) -> Vec<u8> {
    let mut blob = Vec::with_capacity(128);
    blob.extend_from_slice(&(src.fields.len() as u32).to_le_bytes());
    let mut scratch = [0u64; 8];
    for &(name, offset, datatype, count) in &src.fields {
        scratch.fill(0);
        let bytes = cerulion_core::shm_runtime::u64s_as_bytes_mut(&mut scratch);
        let cap = MaxPayloadCapacity::const_new(bytes.len() as u32);
        let mut f = PointField::build_writer(bytes, cap, Arc::clone(topic));
        f.offset = offset;
        f.datatype = datatype;
        f.count = count;
        f.set_name(name).unwrap();
        let end = f.cursor() as usize;
        drop(f);
        blob.extend_from_slice(&(end as u32).to_le_bytes());
        blob.extend_from_slice(&cerulion_core::shm_runtime::u64s_as_bytes(&scratch)[..end]);
    }
    blob
}

pub fn cer_encode_cloud(slot: &mut [u8], src: &CloudSrc, topic: &Arc<str>) -> usize {
    cer_stamp_header::<PointCloud2>(slot, IMU_STAMP_NS);
    let cap = MaxPayloadCapacity::const_new((slot.len() - WireHeader::SIZE) as u32);
    let mut w = PointCloud2::build_writer(&mut slot[WireHeader::SIZE..], cap, Arc::clone(topic));
    w.height = src.height;
    w.width = src.width;
    w.is_bigendian = 0;
    w.point_step = src.point_step;
    w.row_step = src.row_step;
    w.is_dense = 1;
    w.with_header(|h| {
        h.stamp.sec = STAMP_SEC;
        h.stamp.nanosec = STAMP_NSEC;
        h.set_frame_id(FRAME_ID)
    })
    .unwrap();
    let blob = cer_point_fields_blob(src, topic);
    w.set_fields_bytes(&blob).unwrap();
    w.set_data(&src.data).unwrap();
    PointCloud2::flush_staged_nested(&mut w).unwrap();
    assert!(PointCloud2::all_variables_written(&w) && !PointCloud2::has_overflow(&w));
    let n = PointCloud2::payload_wire_size(&w);
    drop(w);
    cer_commit(slot, n, 7)
}

/// The typed receive gate, `validate_wire_frame::<T>` (transport/input_view.rs:
/// 176-290), restated here because it is `pub(crate)`: header length, header
/// parse, schema hash, total_size bounds, fixed section + offset table room,
/// 8-byte alignment. Returns the payload range the reader is built over.
#[inline(always)]
pub fn cer_gate<T: ShmMessage>(raw: &[u8]) -> Option<&[u8]> {
    if raw.len() < WireHeader::SIZE {
        return None;
    }
    let h = WireHeader::read_from_buf(raw)?;
    if h.schema_hash != T::SCHEMA_HASH {
        return None;
    }
    let total = h.total_size as usize;
    if total < WireHeader::SIZE || total > raw.len() {
        return None;
    }
    let required =
        cerulion_core::wire::frame_prefix_size(T::WIRE_FIXED_SIZE, T::VARIABLE_FIELD_COUNT)?;
    if total < required || raw.as_ptr().align_offset(8) != 0 {
        return None;
    }
    Some(&raw[WireHeader::SIZE..total])
}

pub fn cer_read_imu(frame: &[u8]) -> u64 {
    let p = cer_gate::<ImuSample>(frame).expect("gate");
    let r = ImuSample::build_reader(p);
    r.stamp_ns ^ r.seq as u64 ^ r.linear_acceleration[2].to_bits()
}

pub fn cer_read_image(frame: &[u8]) -> u64 {
    let p = cer_gate::<Image>(frame).expect("gate");
    let r = Image::build_reader(p);
    let d = r.data();
    r.height as u64 ^ r.width as u64 ^ r.step as u64 ^ d.len() as u64 ^ d[0] as u64
}

pub fn cer_read_cloud(frame: &[u8]) -> u64 {
    let p = cer_gate::<PointCloud2>(frame).expect("gate");
    let r = PointCloud2::build_reader(p);
    let d = r.data();
    r.height as u64 ^ r.width as u64 ^ r.point_step as u64 ^ d.len() as u64 ^ d[0] as u64
}

/// The untrusted-input decoder (codegen/frame_walker.rs:1-36): a schema-driven
/// walk of the whole frame, then the same field reads from its value tree.
pub fn cer_walker() -> cerulion_core::codegen::FrameWalker {
    use cerulion_core::codegen::{parse_rosmsg, resolve_fixed_nested, FrameWalker};
    let mut schemas: Vec<_> = native_ros2_messages::BUILTIN_MSGS
        .iter()
        .map(|&(pkg, name, text)| parse_rosmsg(text, name, Some(pkg)).unwrap())
        .collect();
    schemas.push(
        parse_rosmsg(
            include_str!("../msg/ImuSample.msg"),
            "ImuSample",
            Some("bench_msgs"),
        )
        .unwrap(),
    );
    let _ = resolve_fixed_nested(&mut schemas);
    let (w, _warnings) = FrameWalker::new(schemas);
    w
}

fn fv_u(v: Option<&cerulion_core::codegen::FrameValueKind<'_>>) -> u64 {
    use cerulion_core::codegen::FrameValueKind as K;
    match v {
        Some(K::U32(x)) => *x as u64,
        Some(K::U64(x)) => *x,
        Some(K::F64(x)) => x.to_bits(),
        other => panic!("unexpected {other:?}"),
    }
}
fn fv_bytes<'a>(v: Option<&cerulion_core::codegen::FrameValueKind<'a>>) -> &'a [u8] {
    use cerulion_core::codegen::FrameValueKind as K;
    match v {
        Some(K::Bytes(b)) => b,
        Some(K::PrimArray(p)) => p.bytes,
        other => panic!("unexpected {other:?}"),
    }
}

pub fn cer_walk_imu(w: &cerulion_core::codegen::FrameWalker, frame: &[u8]) -> u64 {
    use cerulion_core::codegen::FrameValueKind as K;
    let v = w.walk_by_hash(frame).expect("walk");
    let la = match v.field("linear_acceleration") {
        Some(K::PrimArray(p)) => p.iter_f64().nth(2).unwrap().to_bits(),
        Some(K::Array(a)) => fv_u(a.get(2)),
        other => panic!("unexpected {other:?}"),
    };
    fv_u(v.field("stamp_ns")) ^ fv_u(v.field("seq")) ^ la
}

pub fn cer_walk_image(w: &cerulion_core::codegen::FrameWalker, frame: &[u8]) -> u64 {
    let v = w.walk_by_hash(frame).expect("walk");
    let d = fv_bytes(v.field("data"));
    fv_u(v.field("height"))
        ^ fv_u(v.field("width"))
        ^ fv_u(v.field("step"))
        ^ d.len() as u64
        ^ d[0] as u64
}

pub fn cer_walk_cloud(w: &cerulion_core::codegen::FrameWalker, frame: &[u8]) -> u64 {
    let v = w.walk_by_hash(frame).expect("walk");
    let d = fv_bytes(v.field("data"));
    fv_u(v.field("height"))
        ^ fv_u(v.field("width"))
        ^ fv_u(v.field("point_step"))
        ^ d.len() as u64
        ^ d[0] as u64
}

// ---------------------------------------------------------------------------
// FlatBuffers: FlatBufferBuilder over a caller-provided slice
// ---------------------------------------------------------------------------

/// A FlatBuffers allocator over a preallocated slice. It never grows: the
/// builder writes back to front inside the slice, and the finished message is
/// the TAIL of the slice (its start is known only after `finish`).
pub struct SliceAlloc<'a>(pub &'a mut [u8]);
impl std::ops::Deref for SliceAlloc<'_> {
    type Target = [u8];
    fn deref(&self) -> &[u8] {
        self.0
    }
}
impl std::ops::DerefMut for SliceAlloc<'_> {
    fn deref_mut(&mut self) -> &mut [u8] {
        self.0
    }
}
unsafe impl flatbuffers::Allocator for SliceAlloc<'_> {
    type Error = &'static str;
    fn grow_downwards(&mut self) -> Result<(), Self::Error> {
        Err("slot too small")
    }
    fn len(&self) -> usize {
        self.0.len()
    }
}

/// The builder over a slot. FlatBuffers' Rust builder needs the bytes it
/// writes into to start zeroed: built over a slot still holding an earlier
/// message, it produced a buffer its own verifier rejects (Unaligned at
/// PointField.offset; src/bin/flatbuffers_dirty_slot.rs). `FlatBufferBuilder::reset` zeroes
/// exactly the previous message's span (builder.rs:224-227), so every encode
/// below starts with `reset()`, and that memset is inside the timed iteration.
pub type FbB<'s> = flatbuffers::FlatBufferBuilder<'s, SliceAlloc<'s>>;
pub fn fb_builder(slot: &mut [u8]) -> FbB<'_> {
    flatbuffers::FlatBufferBuilder::new_in(SliceAlloc(slot))
}

/// Each returns the finished length; the message is the last `len` bytes of
/// the slot (`b.finished_data()`).
pub fn fb_encode_imu(b: &mut FbB<'_>) -> usize {
    b.reset();
    let s = fb::ImuSample::new(
        IMU_STAMP_NS,
        IMU_SEQ,
        IMU_STATUS,
        &IMU_ORI,
        &IMU_AV,
        &IMU_LA,
    );
    let root = fb::ImuMsg::create(b, &fb::ImuMsgArgs { sample: Some(&s) });
    b.finish_minimal(root);
    b.finished_data().len()
}

fn fb_header<'a, A: flatbuffers::Allocator + 'a>(
    b: &mut flatbuffers::FlatBufferBuilder<'a, A>,
) -> flatbuffers::WIPOffset<fb::Header<'a>> {
    let frame_id = b.create_string(FRAME_ID);
    let stamp = fb::Time::new(STAMP_SEC, STAMP_NSEC);
    fb::Header::create(
        b,
        &fb::HeaderArgs {
            stamp: Some(&stamp),
            frame_id: Some(frame_id),
        },
    )
}

pub fn fb_encode_image(b: &mut FbB<'_>, src: &ImageSrc) -> usize {
    b.reset();
    let data = b.create_vector(&src.data[..]);
    let encoding = b.create_string(src.encoding);
    let header = fb_header(b);
    let root = fb::Image::create(
        b,
        &fb::ImageArgs {
            header: Some(header),
            height: src.height,
            width: src.width,
            encoding: Some(encoding),
            is_bigendian: 0,
            step: src.step,
            data: Some(data),
        },
    );
    b.finish_minimal(root);
    b.finished_data().len()
}

pub fn fb_encode_cloud(b: &mut FbB<'_>, src: &CloudSrc) -> usize {
    b.reset();
    let data = b.create_vector(&src.data[..]);
    let mut fs = [flatbuffers::WIPOffset::new(0); 4];
    for (i, &(name, offset, datatype, count)) in src.fields.iter().enumerate() {
        let name = b.create_string(name);
        fs[i] = fb::PointField::create(
            b,
            &fb::PointFieldArgs {
                name: Some(name),
                offset,
                datatype,
                count,
            },
        );
    }
    let fields = b.create_vector(&fs);
    let header = fb_header(b);
    let root = fb::PointCloud2::create(
        b,
        &fb::PointCloud2Args {
            header: Some(header),
            height: src.height,
            width: src.width,
            fields: Some(fields),
            is_bigendian: false,
            point_step: src.point_step,
            row_step: src.row_step,
            data: Some(data),
            is_dense: true,
        },
    );
    b.finish_minimal(root);
    b.finished_data().len()
}

pub fn fb_read_imu(buf: &[u8], verify: bool) -> u64 {
    let m = if verify {
        flatbuffers::root::<fb::ImuMsg>(buf).expect("verify")
    } else {
        unsafe { flatbuffers::root_unchecked::<fb::ImuMsg>(buf) }
    };
    let s = m.sample().unwrap();
    s.stamp_ns() ^ s.seq() as u64 ^ s.linear_acceleration().get(2).to_bits()
}

pub fn fb_read_image(buf: &[u8], verify: bool) -> u64 {
    let m = if verify {
        flatbuffers::root::<fb::Image>(buf).expect("verify")
    } else {
        unsafe { flatbuffers::root_unchecked::<fb::Image>(buf) }
    };
    let d = m.data().unwrap().bytes();
    m.height() as u64 ^ m.width() as u64 ^ m.step() as u64 ^ d.len() as u64 ^ d[0] as u64
}

pub fn fb_read_cloud(buf: &[u8], verify: bool) -> u64 {
    let m = if verify {
        flatbuffers::root::<fb::PointCloud2>(buf).expect("verify")
    } else {
        unsafe { flatbuffers::root_unchecked::<fb::PointCloud2>(buf) }
    };
    let d = m.data().unwrap().bytes();
    m.height() as u64 ^ m.width() as u64 ^ m.point_step() as u64 ^ d.len() as u64 ^ d[0] as u64
}

// ---------------------------------------------------------------------------
// Cap'n Proto: message::Builder over a ScratchSpaceHeapAllocator on the slot
// ---------------------------------------------------------------------------

use capnp::message::{Builder as CBuilder, ReaderOptions, ScratchSpaceHeapAllocator};
use wire_capnp::{image, imu_sample, point_cloud2};

/// A Cap'n Proto output slot. `table` is the slot's first 8 bytes, where the
/// single-segment segment table goes; `alloc` is a `ScratchSpaceHeapAllocator`
/// over the rest, so the one segment is built in place right after its table
/// and the complete flat message is `slot[..8 + segment]`. The allocator is
/// built once per slot: `ScratchSpaceHeapAllocator::new` zeroes the whole
/// scratch (capnp message.rs:912-915), and the builder's drop re-zeroes only
/// the used words, which Cap'n Proto needs before the next build in it
/// (message.rs:957-975). That re-zero is inside every timed iteration.
pub struct CapnpSlot<'a> {
    table: &'a mut [u8],
    alloc: ScratchSpaceHeapAllocator<'a>,
}
impl<'a> CapnpSlot<'a> {
    pub fn new(slot: &'a mut [u8]) -> Self {
        let (table, scratch) = slot.split_at_mut(8);
        CapnpSlot {
            table,
            alloc: ScratchSpaceHeapAllocator::new(scratch),
        }
    }
}

/// Builds the message in the slot, writes its segment table (segment count
/// minus one = 0, then the segment's length in words, both u32 LE) into the
/// slot's first 8 bytes, and returns the flat message length. `inspect` gets
/// the table and the segment while the builder is still alive, before its
/// drop re-zeroes the segment.
#[inline(always)]
fn capnp_encode_in<'b, 's, R>(
    s: &'b mut CapnpSlot<'s>,
    build: impl FnOnce(&mut CBuilder<&'b mut ScratchSpaceHeapAllocator<'s>>),
    inspect: impl FnOnce(&[u8], &[u8]) -> R,
) -> (usize, R) {
    let mut m = CBuilder::new(&mut s.alloc);
    build(&mut m);
    let segs = m.get_segments_for_output();
    assert_eq!(segs.len(), 1, "single segment in the slot");
    let seg = segs[0];
    s.table[..4].copy_from_slice(&0u32.to_le_bytes());
    s.table[4..8].copy_from_slice(&((seg.len() / 8) as u32).to_le_bytes());
    let r = inspect(s.table, seg);
    (8 + seg.len(), r)
}

/// The complete flat message an encode left in its slot, copied out of the
/// `inspect` arguments. Asserts the segment starts right after the table, so
/// the copy is the slot's own contiguous bytes.
pub fn capnp_message_bytes(table: &[u8], seg: &[u8]) -> Vec<u8> {
    assert_eq!(table.len(), 8);
    assert_eq!(
        table.as_ptr().wrapping_add(8),
        seg.as_ptr(),
        "segment follows its table"
    );
    let mut v = Vec::with_capacity(8 + seg.len());
    v.extend_from_slice(table);
    v.extend_from_slice(seg);
    v
}

/// The timed encode: the complete flat message (segment table + segment) in
/// the slot. Returns its length.
pub fn capnp_encode_imu(s: &mut CapnpSlot<'_>) -> usize {
    capnp_encode_in(s, capnp_build_imu, |_, _| ()).0
}
/// The same encode, with `inspect` run on the slot's bytes before the builder drops.
pub fn capnp_encode_imu_with<R>(
    s: &mut CapnpSlot<'_>,
    inspect: impl FnOnce(&[u8], &[u8]) -> R,
) -> (usize, R) {
    capnp_encode_in(s, capnp_build_imu, inspect)
}

fn capnp_build_imu<A: capnp::message::Allocator>(m: &mut CBuilder<A>) {
    {
        let mut r = m.init_root::<imu_sample::Builder>();
        r.set_stamp_ns(IMU_STAMP_NS);
        r.set_seq(IMU_SEQ);
        r.set_status(IMU_STATUS);
        r.set_ox(IMU_ORI[0]);
        r.set_oy(IMU_ORI[1]);
        r.set_oz(IMU_ORI[2]);
        r.set_ow(IMU_ORI[3]);
        r.set_avx(IMU_AV[0]);
        r.set_avy(IMU_AV[1]);
        r.set_avz(IMU_AV[2]);
        r.set_lax(IMU_LA[0]);
        r.set_lay(IMU_LA[1]);
        r.set_laz(IMU_LA[2]);
    }
}

pub fn capnp_encode_image(s: &mut CapnpSlot<'_>, src: &ImageSrc) -> usize {
    capnp_encode_in(s, |m| capnp_build_image(m, src), |_, _| ()).0
}
pub fn capnp_encode_image_with<R>(
    s: &mut CapnpSlot<'_>,
    src: &ImageSrc,
    inspect: impl FnOnce(&[u8], &[u8]) -> R,
) -> (usize, R) {
    capnp_encode_in(s, |m| capnp_build_image(m, src), inspect)
}

fn capnp_build_image<A: capnp::message::Allocator>(m: &mut CBuilder<A>, src: &ImageSrc) {
    {
        let mut r = m.init_root::<image::Builder>();
        {
            let mut h = r.reborrow().init_header();
            let mut st = h.reborrow().init_stamp();
            st.set_sec(STAMP_SEC);
            st.set_nanosec(STAMP_NSEC);
            h.set_frame_id(FRAME_ID);
        }
        r.set_height(src.height);
        r.set_width(src.width);
        r.set_encoding(src.encoding);
        r.set_is_bigendian(0);
        r.set_step(src.step);
        r.init_data(src.data.len() as u32)
            .copy_from_slice(&src.data);
    }
}

pub fn capnp_encode_cloud(s: &mut CapnpSlot<'_>, src: &CloudSrc) -> usize {
    capnp_encode_in(s, |m| capnp_build_cloud(m, src), |_, _| ()).0
}
pub fn capnp_encode_cloud_with<R>(
    s: &mut CapnpSlot<'_>,
    src: &CloudSrc,
    inspect: impl FnOnce(&[u8], &[u8]) -> R,
) -> (usize, R) {
    capnp_encode_in(s, |m| capnp_build_cloud(m, src), inspect)
}

fn capnp_build_cloud<A: capnp::message::Allocator>(m: &mut CBuilder<A>, src: &CloudSrc) {
    {
        let mut r = m.init_root::<point_cloud2::Builder>();
        {
            let mut h = r.reborrow().init_header();
            let mut st = h.reborrow().init_stamp();
            st.set_sec(STAMP_SEC);
            st.set_nanosec(STAMP_NSEC);
            h.set_frame_id(FRAME_ID);
        }
        r.set_height(src.height);
        r.set_width(src.width);
        {
            let mut fl = r.reborrow().init_fields(src.fields.len() as u32);
            for (i, &(name, offset, datatype, count)) in src.fields.iter().enumerate() {
                let mut f = fl.reborrow().get(i as u32);
                f.set_name(name);
                f.set_offset(offset);
                f.set_datatype(datatype);
                f.set_count(count);
            }
        }
        r.set_is_bigendian(false);
        r.set_point_step(src.point_step);
        r.set_row_step(src.row_step);
        r.set_is_dense(true);
        r.init_data(src.data.len() as u32)
            .copy_from_slice(&src.data);
    }
}

/// The read inputs: the bytes the timed encode leaves in its slot, copied
/// out before the builder drops.
pub fn capnp_flat_imu() -> Vec<u8> {
    let mut slot = AlignedBuf::new(4096);
    let mut s = CapnpSlot::new(slot.bytes_mut());
    capnp_encode_imu_with(&mut s, capnp_message_bytes).1
}
pub fn capnp_flat_image(src: &ImageSrc) -> Vec<u8> {
    let mut slot = AlignedBuf::new(slot_len(src.data.len()));
    let mut s = CapnpSlot::new(slot.bytes_mut());
    capnp_encode_image_with(&mut s, src, capnp_message_bytes).1
}
pub fn capnp_flat_cloud(src: &CloudSrc) -> Vec<u8> {
    let mut slot = AlignedBuf::new(slot_len(src.data.len()));
    let mut s = CapnpSlot::new(slot.bytes_mut());
    capnp_encode_cloud_with(&mut s, src, capnp_message_bytes).1
}

/// Cap'n Proto's own serializer output for the same content, built in a
/// separate scratch and written by `serialize::write_message_to_words`.
/// `check` asserts the bytes each timed encode leaves in its slot equal these.
pub fn capnp_reference_imu() -> Vec<u8> {
    let mut scratch = AlignedBuf::new(4096);
    let mut m = CBuilder::new(ScratchSpaceHeapAllocator::new(scratch.bytes_mut()));
    capnp_build_imu(&mut m);
    assert_eq!(m.get_segments_for_output().len(), 1);
    capnp::serialize::write_message_to_words(&m)
}
pub fn capnp_reference_image(src: &ImageSrc) -> Vec<u8> {
    let mut scratch = AlignedBuf::new(slot_len(src.data.len()));
    let mut m = CBuilder::new(ScratchSpaceHeapAllocator::new(scratch.bytes_mut()));
    capnp_build_image(&mut m, src);
    assert_eq!(m.get_segments_for_output().len(), 1);
    capnp::serialize::write_message_to_words(&m)
}
pub fn capnp_reference_cloud(src: &CloudSrc) -> Vec<u8> {
    let mut scratch = AlignedBuf::new(slot_len(src.data.len()));
    let mut m = CBuilder::new(ScratchSpaceHeapAllocator::new(scratch.bytes_mut()));
    capnp_build_cloud(&mut m, src);
    assert_eq!(m.get_segments_for_output().len(), 1);
    capnp::serialize::write_message_to_words(&m)
}

/// Reader options for every Cap'n Proto read: the traversal limit raised to
/// 64 Mi words, because the default would count the 6 MiB Data.
pub fn capnp_opts() -> ReaderOptions {
    let mut o = ReaderOptions::new();
    o.traversal_limit_in_words(Some(64 << 20));
    o
}

pub fn capnp_read_imu(buf: &[u8]) -> u64 {
    let mut s = buf;
    let m = capnp::serialize::read_message_from_flat_slice_no_alloc(&mut s, capnp_opts()).unwrap();
    let r = m.get_root::<imu_sample::Reader>().unwrap();
    r.get_stamp_ns() ^ r.get_seq() as u64 ^ r.get_laz().to_bits()
}

pub fn capnp_read_image(buf: &[u8]) -> u64 {
    let mut s = buf;
    let m = capnp::serialize::read_message_from_flat_slice_no_alloc(&mut s, capnp_opts()).unwrap();
    let r = m.get_root::<image::Reader>().unwrap();
    let d = r.get_data().unwrap();
    r.get_height() as u64
        ^ r.get_width() as u64
        ^ r.get_step() as u64
        ^ d.len() as u64
        ^ d[0] as u64
}

pub fn capnp_read_cloud(buf: &[u8]) -> u64 {
    let mut s = buf;
    let m = capnp::serialize::read_message_from_flat_slice_no_alloc(&mut s, capnp_opts()).unwrap();
    let r = m.get_root::<point_cloud2::Reader>().unwrap();
    let d = r.get_data().unwrap();
    r.get_height() as u64
        ^ r.get_width() as u64
        ^ r.get_point_step() as u64
        ^ d.len() as u64
        ^ d[0] as u64
}

// ---------------------------------------------------------------------------
// Protobuf (prost): build the message struct, encode into the slot
// ---------------------------------------------------------------------------

use prost::Message as _;

fn pb_header() -> Option<pb::Header> {
    Some(pb::Header {
        stamp: Some(pb::Time {
            sec: STAMP_SEC,
            nanosec: STAMP_NSEC,
        }),
        frame_id: FRAME_ID.to_string(),
    })
}

pub fn pb_imu() -> pb::ImuSample {
    pb::ImuSample {
        stamp_ns: IMU_STAMP_NS,
        seq: IMU_SEQ,
        status: IMU_STATUS,
        ox: IMU_ORI[0],
        oy: IMU_ORI[1],
        oz: IMU_ORI[2],
        ow: IMU_ORI[3],
        avx: IMU_AV[0],
        avy: IMU_AV[1],
        avz: IMU_AV[2],
        lax: IMU_LA[0],
        lay: IMU_LA[1],
        laz: IMU_LA[2],
    }
}

/// Encodes straight into `slot` (`&mut [u8]` is a `BufMut`): no intermediate
/// buffer. Returns the encoded length.
#[inline(always)]
fn pb_into<M: prost::Message>(m: &M, slot: &mut [u8]) -> usize {
    let cap = slot.len();
    let mut s: &mut [u8] = slot;
    m.encode(&mut s).unwrap();
    cap - s.len()
}

pub fn pb_encode_imu(slot: &mut [u8]) -> usize {
    pb_into(&pb_imu(), slot)
}

/// Builds the message per call, payload as a refcounted `Bytes` clone (no
/// payload copy in the build; the one payload copy is inside `encode`).
pub fn pb_encode_image(slot: &mut [u8], src: &ImageSrc) -> usize {
    let m = pb::ImageB {
        header: pb_header(),
        height: src.height,
        width: src.width,
        encoding: src.encoding.to_string(),
        is_bigendian: 0,
        step: src.step,
        data: src.data_bytes.clone(),
    };
    pb_into(&m, slot)
}

fn pb_fields(src: &CloudSrc) -> Vec<pb::PointField> {
    src.fields
        .iter()
        .map(|&(name, offset, datatype, count)| pb::PointField {
            name: name.to_string(),
            offset,
            datatype: datatype as u32,
            count,
        })
        .collect()
}

pub fn pb_encode_cloud(slot: &mut [u8], src: &CloudSrc) -> usize {
    let m = pb::PointCloud2B {
        header: pb_header(),
        height: src.height,
        width: src.width,
        fields: pb_fields(src),
        is_bigendian: false,
        point_step: src.point_step,
        row_step: src.row_step,
        data: src.data_bytes.clone(),
        is_dense: true,
    };
    pb_into(&m, slot)
}

pub fn pb_read_imu(buf: &[u8]) -> u64 {
    let m = pb::ImuSample::decode(buf).unwrap();
    m.stamp_ns ^ m.seq as u64 ^ m.laz.to_bits()
}

/// Default prost: `bytes` as `Vec<u8>`, so decode allocates and copies the payload.
pub fn pb_read_image_vec(buf: &[u8]) -> u64 {
    let m = pb::ImageV::decode(buf).unwrap();
    m.height as u64 ^ m.width as u64 ^ m.step as u64 ^ m.data.len() as u64 ^ m.data[0] as u64
}

/// prost with `bytes::Bytes` fields, decoding from a `Bytes`: the payload is
/// a refcounted slice of the input, no copy.
pub fn pb_read_image_bytes(buf: &bytes::Bytes) -> u64 {
    let m = pb::ImageB::decode(buf.clone()).unwrap();
    m.height as u64 ^ m.width as u64 ^ m.step as u64 ^ m.data.len() as u64 ^ m.data[0] as u64
}

pub fn pb_read_cloud_vec(buf: &[u8]) -> u64 {
    let m = pb::PointCloud2V::decode(buf).unwrap();
    m.height as u64 ^ m.width as u64 ^ m.point_step as u64 ^ m.data.len() as u64 ^ m.data[0] as u64
}

pub fn pb_read_cloud_bytes(buf: &bytes::Bytes) -> u64 {
    let m = pb::PointCloud2B::decode(buf.clone()).unwrap();
    m.height as u64 ^ m.width as u64 ^ m.point_step as u64 ^ m.data.len() as u64 ^ m.data[0] as u64
}

/// The value every read must return for each message, from the source.
pub fn expect_imu() -> u64 {
    IMU_STAMP_NS ^ IMU_SEQ as u64 ^ IMU_LA[2].to_bits()
}
pub fn expect_image(s: &ImageSrc) -> u64 {
    s.height as u64 ^ s.width as u64 ^ s.step as u64 ^ s.data.len() as u64 ^ s.data[0] as u64
}
pub fn expect_cloud(s: &CloudSrc) -> u64 {
    s.height as u64 ^ s.width as u64 ^ s.point_step as u64 ^ s.data.len() as u64 ^ s.data[0] as u64
}
