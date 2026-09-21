// SPDX-License-Identifier: MIT OR Apache-2.0
//! Structural wire-format tests for `cerulion-wire`.
//!
//! Discipline cribbed from `cerulion_core/tests/wire_test.rs`: every frame is
//! HAND-BUILT from known byte offsets and checked against a HAND-WRITTEN
//! oracle — never a re-run of the code under test (Principle #13: no
//! self-compares). The little-endian primitives used to build the oracles come
//! from `std` (`to_le_bytes`), not from this crate, so the encode/decode paths
//! are genuinely cross-checked.

use cerulion_wire::{Frame, OffsetEntry, WireError, WireHeader};

// ---- byte-builder helpers (std primitives only) ---------------------------

fn push_u32(v: &mut Vec<u8>, x: u32) {
    v.extend_from_slice(&x.to_le_bytes());
}
fn push_u64(v: &mut Vec<u8>, x: u64) {
    v.extend_from_slice(&x.to_le_bytes());
}

/// Hand-assemble a 32-byte header from field values (independent of
/// `WireHeader::encode` — this is the oracle byte stream).
fn header_bytes(
    schema_hash: u64,
    total_size: u32,
    offset_table_offset: u32,
    offset_table_count: u32,
    sequence: u32,
    timestamp_ns: u64,
) -> Vec<u8> {
    let mut b = Vec::with_capacity(32);
    push_u64(&mut b, schema_hash);
    push_u32(&mut b, total_size);
    push_u32(&mut b, offset_table_offset);
    push_u32(&mut b, offset_table_count);
    push_u32(&mut b, sequence);
    push_u64(&mut b, timestamp_ns);
    assert_eq!(b.len(), 32, "hand-built header must be 32 bytes");
    b
}

// ===========================================================================
// Header
// ===========================================================================

#[test]
fn header_parse_matches_hand_oracle() {
    // Hand-written 32-byte oracle with distinctive per-field values.
    let oracle = header_bytes(0xDEAD_BEEF_CAFE_BABE, 1024, 40, 3, 42, 1_000_000_000);
    let hdr = WireHeader::parse(&oracle).expect("parse");
    assert_eq!(hdr.schema_hash, 0xDEAD_BEEF_CAFE_BABE);
    assert_eq!(hdr.total_size, 1024);
    assert_eq!(hdr.offset_table_offset, 40);
    assert_eq!(hdr.offset_table_count, 3);
    assert_eq!(hdr.sequence, 42);
    assert_eq!(hdr.timestamp_ns, 1_000_000_000);

    // Encode side: `to_bytes` must reproduce the oracle byte-for-byte.
    assert_eq!(hdr.to_bytes().as_slice(), oracle.as_slice());
}

#[test]
fn header_encode_parse_roundtrip_including_extremes() {
    for hdr in [
        WireHeader {
            schema_hash: 0,
            total_size: 32,
            offset_table_offset: 0,
            offset_table_count: 0,
            sequence: 0,
            timestamp_ns: 0,
        },
        WireHeader {
            schema_hash: u64::MAX,
            total_size: u32::MAX,
            offset_table_offset: u32::MAX,
            offset_table_count: u32::MAX,
            sequence: u32::MAX,
            timestamp_ns: u64::MAX,
        },
    ] {
        let bytes = hdr.to_bytes();
        assert_eq!(WireHeader::parse(&bytes).unwrap(), hdr);
    }
}

#[test]
fn header_parse_rejects_short_buffer() {
    let buf = [0u8; 31];
    assert_eq!(
        WireHeader::parse(&buf),
        Err(WireError::FrameTooShort { have: 31, need: 32 })
    );
}

#[test]
fn header_encode_rejects_short_buffer() {
    let hdr = WireHeader {
        schema_hash: 1,
        total_size: 32,
        offset_table_offset: 0,
        offset_table_count: 0,
        sequence: 0,
        timestamp_ns: 0,
    };
    let mut buf = [0u8; 16];
    assert_eq!(
        hdr.encode(&mut buf),
        Err(WireError::FrameTooShort { have: 16, need: 32 })
    );
}

// ===========================================================================
// OffsetEntry
// ===========================================================================

#[test]
fn offset_entry_roundtrip_and_hand_oracle() {
    // Oracle: offset=24 (0x18), length=5 (0x05), little-endian.
    let oracle: [u8; 8] = [0x18, 0x00, 0x00, 0x00, 0x05, 0x00, 0x00, 0x00];
    let e = OffsetEntry::parse(&oracle).expect("parse");
    assert_eq!(e, OffsetEntry::new(24, 5));

    let mut buf = [0u8; 8];
    e.encode(&mut buf).unwrap();
    assert_eq!(buf, oracle);
}

#[test]
fn offset_entry_parse_rejects_short_buffer() {
    assert_eq!(OffsetEntry::parse(&[0u8; 7]), None);
}

// ===========================================================================
// Frame — fixed + variable slicing
// ===========================================================================

/// Build an Imglike frame by hand:
/// fixed { height:u32=480, width:u32=640 } (fixed_size = 8)
/// variable { encoding:"rgb8", data:[1,2,3,4,5] }
///
/// Byte map:
///   [0..32)   header
///   [32..40)  fixed section (height, width)
///   [40..56)  offset table (2 entries)
///   [56..60)  "rgb8"
///   [60..65)  data
/// total_size = 65, offset_table_offset = 40, count = 2.
/// Entry offsets are PAYLOAD-RELATIVE: encoding at 24 (= 8 fixed + 16 table),
/// data at 28.
fn imglike_frame() -> Vec<u8> {
    let mut f = header_bytes(0x1111_2222_3333_4444, 65, 40, 2, 7, 123);
    // fixed section
    push_u32(&mut f, 480);
    push_u32(&mut f, 640);
    // offset table: encoding (24, 4), data (28, 5)
    push_u32(&mut f, 24);
    push_u32(&mut f, 4);
    push_u32(&mut f, 28);
    push_u32(&mut f, 5);
    // variable payload
    f.extend_from_slice(b"rgb8");
    f.extend_from_slice(&[1, 2, 3, 4, 5]);
    assert_eq!(f.len(), 65);
    f
}

#[test]
fn frame_fixed_and_variable_slicing() {
    let raw = imglike_frame();
    let frame = Frame::parse(&raw).expect("parse");

    // Header.
    assert_eq!(frame.header().schema_hash, 0x1111_2222_3333_4444);
    assert_eq!(frame.header().total_size, 65);
    assert_eq!(frame.offset_entry_count(), 2);

    // Fixed section == the raw 8 bytes of {480, 640}.
    let mut fixed_oracle = Vec::new();
    push_u32(&mut fixed_oracle, 480);
    push_u32(&mut fixed_oracle, 640);
    assert_eq!(frame.fixed_section(), fixed_oracle.as_slice());

    // Offset table entries.
    let entries: Vec<OffsetEntry> = frame.offset_entries().collect();
    assert_eq!(
        entries,
        vec![OffsetEntry::new(24, 4), OffsetEntry::new(28, 5)]
    );
    assert_eq!(frame.offset_entry(0), Some(OffsetEntry::new(24, 4)));
    assert_eq!(frame.offset_entry(2), None);

    // Variable field slices against hand oracles.
    assert_eq!(frame.variable_field(0).unwrap().unwrap(), b"rgb8");
    assert_eq!(
        frame.variable_field(1).unwrap().unwrap(),
        &[1u8, 2, 3, 4, 5]
    );
    assert!(frame.variable_field(2).is_none());

    // payload / as_bytes bounds.
    assert_eq!(frame.as_bytes().len(), 65);
    assert_eq!(frame.payload().len(), 65 - 32);
}

#[test]
fn frame_parse_accepts_oversized_buffer_and_bounds_to_total_size() {
    // A shared-memory slot is larger than the frame; parse must bound to
    // total_size and ignore the trailing slack.
    let mut raw = imglike_frame();
    raw.extend_from_slice(&[0xEE; 100]); // slack
    let frame = Frame::parse(&raw).expect("parse");
    assert_eq!(frame.as_bytes().len(), 65, "must bound to total_size");
    assert_eq!(
        frame.variable_field(1).unwrap().unwrap(),
        &[1u8, 2, 3, 4, 5]
    );
}

#[test]
fn frame_empty_variable_fields_are_empty_not_error() {
    // Same Imglike shape but both entries (0, 0): fixed(8) + table(16), no
    // variable payload. total = 56, offset_table_offset = 40, count = 2.
    let mut f = header_bytes(0xABCD, 56, 40, 2, 0, 0);
    push_u32(&mut f, 0); // height
    push_u32(&mut f, 0); // width
    push_u32(&mut f, 0); // entry0 offset
    push_u32(&mut f, 0); // entry0 length
    push_u32(&mut f, 0); // entry1 offset
    push_u32(&mut f, 0); // entry1 length
    assert_eq!(f.len(), 56);

    let frame = Frame::parse(&f).expect("parse");
    assert_eq!(frame.variable_field(0).unwrap().unwrap(), b"");
    assert_eq!(frame.variable_field(1).unwrap().unwrap(), b"");
}

// ===========================================================================
// Frame — fixed-only and all-variable edges
// ===========================================================================

#[test]
fn frame_fixed_only_uses_offset_table_offset_boundary() {
    // Vec3-like: 3 x f64 = 24 fixed bytes, zero variable fields.
    // Production writer stamps offset_table_offset = 32 + 24 = 56, count = 0.
    let mut f = header_bytes(0x99, 56, 56, 0, 1, 2);
    push_u64(&mut f, 1.0f64.to_bits());
    push_u64(&mut f, 2.0f64.to_bits());
    push_u64(&mut f, 3.0f64.to_bits());
    assert_eq!(f.len(), 56);

    let frame = Frame::parse(&f).expect("parse");
    assert_eq!(frame.offset_entry_count(), 0);
    assert_eq!(frame.fixed_section().len(), 24);
    // The fixed section is the whole payload.
    assert_eq!(frame.fixed_section(), frame.payload());
    assert!(frame.offset_entry(0).is_none());
    assert!(frame.variable_field(0).is_none());
    assert_eq!(frame.offset_entries().count(), 0);
}

#[test]
fn frame_fixed_only_offset_table_offset_zero_convention() {
    // The minimal-envelope convention (service.rs / raw re-injection):
    // count == 0 AND offset_table_offset == 0. The fixed section must still be
    // the whole payload — offset_table_offset carries no meaning when count 0.
    let mut f = header_bytes(0x77, 44, 0, 0, 0, 0);
    push_u32(&mut f, 0xAAAA_AAAA);
    push_u32(&mut f, 0xBBBB_BBBB);
    push_u32(&mut f, 0xCCCC_CCCC); // 12 fixed bytes; total 44
    assert_eq!(f.len(), 44);

    let frame = Frame::parse(&f).expect("parse");
    assert_eq!(frame.fixed_section().len(), 12);
    assert_eq!(frame.fixed_section(), frame.payload());
}

#[test]
fn frame_all_variable_no_fixed_section() {
    // One dynamic field, no fixed section: fixed_size = 0.
    // offset_table_offset = 32, count = 1. Entry payload-rel offset = 8
    // (0 fixed + 8 table), length 3. Data at absolute [40..43]. total = 43.
    let mut f = header_bytes(0x55, 43, 32, 1, 0, 0);
    push_u32(&mut f, 8); // entry offset (payload-relative)
    push_u32(&mut f, 3); // entry length
    f.extend_from_slice(&[0xAA, 0xBB, 0xCC]);
    assert_eq!(f.len(), 43);

    let frame = Frame::parse(&f).expect("parse");
    assert!(frame.fixed_section().is_empty(), "no fixed fields");
    assert_eq!(
        frame.variable_field(0).unwrap().unwrap(),
        &[0xAA, 0xBB, 0xCC]
    );
}

// ===========================================================================
// Frame — adversarial / hostile input
// ===========================================================================

#[test]
fn frame_rejects_buffer_too_short_for_header() {
    assert_eq!(
        Frame::parse(&[0u8; 10]),
        Err(WireError::FrameTooShort { have: 10, need: 32 })
    );
}

#[test]
fn frame_rejects_invalid_total_size_below_header() {
    // total_size = 16 < 32.
    let f = header_bytes(0, 16, 0, 0, 0, 0);
    assert_eq!(
        Frame::parse(&f),
        Err(WireError::InvalidTotalSize {
            total_size: 16,
            min: 32
        })
    );
}

#[test]
fn frame_rejects_truncated_total_size() {
    // Header claims 100 bytes; only 40 provided.
    let mut f = header_bytes(0, 100, 40, 0, 0, 0);
    f.extend_from_slice(&[0u8; 8]); // buffer is 40 bytes total
    assert_eq!(f.len(), 40);
    assert_eq!(
        Frame::parse(&f),
        Err(WireError::TruncatedFrame {
            total_size: 100,
            available: 40
        })
    );
}

#[test]
fn frame_rejects_offset_table_past_end() {
    // count = 3 (24 table bytes) at offset 40, but total_size only 48 → the
    // table would run to 64 > 48.
    let mut f = header_bytes(0, 48, 40, 3, 0, 0);
    f.extend_from_slice(&[0u8; 16]); // pad to total 48
    assert_eq!(f.len(), 48);
    match Frame::parse(&f) {
        Err(WireError::OffsetTableOutOfBounds {
            offset,
            count,
            bytes,
            total_size,
        }) => {
            assert_eq!((offset, count, bytes, total_size), (40, 3, 24, 48));
        }
        other => panic!("expected OffsetTableOutOfBounds, got {other:?}"),
    }
}

#[test]
fn frame_rejects_offset_table_inside_header() {
    // count = 1 but offset_table_offset = 8 (inside the 32-byte header).
    let mut f = header_bytes(0, 48, 8, 1, 0, 0);
    f.extend_from_slice(&[0u8; 16]);
    assert_eq!(f.len(), 48);
    assert!(matches!(
        Frame::parse(&f),
        Err(WireError::OffsetTableOutOfBounds { offset: 8, .. })
    ));
}

#[test]
fn frame_rejects_hostile_variable_offset_past_end() {
    // Imglike shape, but entry0 claims offset 24 (payload-rel) length 100 —
    // absolute end 32+24+100 = 156 >> total 56 → out of bounds.
    let mut f = header_bytes(0, 56, 40, 2, 0, 0);
    push_u32(&mut f, 0); // height
    push_u32(&mut f, 0); // width
    push_u32(&mut f, 24); // entry0 offset
    push_u32(&mut f, 100); // entry0 length (hostile)
    push_u32(&mut f, 0); // entry1
    push_u32(&mut f, 0);
    assert_eq!(f.len(), 56);
    match Frame::parse(&f).unwrap().variable_field(0).unwrap() {
        Err(WireError::VariableFieldOutOfBounds {
            index,
            offset,
            length,
            total_size,
        }) => assert_eq!((index, offset, length, total_size), (0, 24, 100, 56)),
        other => panic!("expected VariableFieldOutOfBounds, got {other:?}"),
    }
}

#[test]
fn frame_rejects_hostile_variable_offset_near_u32_max_no_panic() {
    // Entry0 claims a payload-relative offset of u32::MAX-1. Adding the 32-byte
    // header would overflow `usize` on a 32-bit target; on any target it is far
    // out of bounds. Either way it must be a CLEAN VariableFieldOutOfBounds,
    // never a panic (the "never panics on adversarial bytes" contract).
    let mut f = header_bytes(0, 56, 40, 2, 0, 0);
    push_u32(&mut f, 0); // height
    push_u32(&mut f, 0); // width
    push_u32(&mut f, u32::MAX - 1); // entry0 offset (hostile)
    push_u32(&mut f, 8); // entry0 length
    push_u32(&mut f, 0); // entry1
    push_u32(&mut f, 0);
    assert_eq!(f.len(), 56);
    // The header + offset table are structurally valid — the hostile value is
    // in the ENTRY, caught only on access.
    let frame = Frame::parse(&f).expect("header/table are structurally valid");
    match frame.variable_field(0).unwrap() {
        Err(WireError::VariableFieldOutOfBounds {
            index,
            offset,
            length,
            total_size,
        }) => {
            assert_eq!(
                (index, offset, length, total_size),
                (0, (u32::MAX - 1) as usize, 8, 56)
            );
        }
        other => panic!("expected VariableFieldOutOfBounds, got {other:?}"),
    }
}

#[test]
fn frame_rejects_variable_offset_pointing_into_table() {
    // A non-empty entry whose offset points BELOW the variable-data floor
    // (into the fixed section / offset table) must be rejected — data_floor =
    // offset_table_offset(40) + 8*count(16) = 56 absolute; payload-rel floor =
    // 24. Entry offset 0 (payload-rel) → absolute 32 < 56.
    let mut f = header_bytes(0, 60, 40, 2, 0, 0);
    push_u32(&mut f, 0); // height
    push_u32(&mut f, 0); // width
    push_u32(&mut f, 0); // entry0 offset (payload-rel 0 → into fixed section)
    push_u32(&mut f, 4); // entry0 length
    push_u32(&mut f, 0); // entry1
    push_u32(&mut f, 0);
    f.extend_from_slice(&[0u8; 4]); // pad to total 60
    assert_eq!(f.len(), 60);
    assert!(matches!(
        Frame::parse(&f).unwrap().variable_field(0).unwrap(),
        Err(WireError::VariableFieldOutOfBounds { index: 0, .. })
    ));
}
