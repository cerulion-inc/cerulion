// SPDX-License-Identifier: AGPL-3.0-only
//! Byte-compatibility drift guard between `cerulion_core::wire` and the
//! permissive standalone `cerulion-wire` crate.
//!
//! `cerulion-wire` is a hand-maintained BYTE-EXACT mirror of
//! `cerulion_core/src/wire.rs`, kept dependency-free so closed-source apps can
//! decode Cerulion frames without linking AGPL `cerulion_core`. That mirror is
//! LOAD-BEARING: this test builds headers, offset-table entries, and full
//! frames with `cerulion_core`'s REAL writer primitives (`WireHeader::write_to_buf`,
//! `shm_runtime::write_offset_entry`) and re-parses them with `cerulion-wire`,
//! asserting byte-identity in BOTH directions. A change to the wire layout in
//! `cerulion_core` that the mirror does not track fails this test loudly.
//!
//! It ALSO cross-checks `cerulion-wire`'s schema-free structural view against
//! `cerulion_core`'s own schema-driven `FrameWalker` on the SAME frame bytes:
//! the two independent frame interpreters must agree.
//!
//! The final group (`production_*`) closes the loop against the REAL
//! writer/finalize path — `loan_proxy` → generated `<Name>Shm` setters →
//! `OutputProxy::Drop` over a per-test `TestTransport` (parallel-safe, crib
//! `frame_walker_production_test`), NOT a hand-stamped header — for the three
//! shapes most likely to drift silently to the sidecar: a fixed-only frame
//! (`Vector3`), an all-variable frame (`std_msgs/String`), and an EMPTY
//! variable field (`tf2_msgs/TFMessage` with `set_transforms_bytes(&[])` — the
//! `/tf` empty-transforms-array shape). Each asserts three-way agreement:
//! full-frame byte-identity + `cerulion-wire` `Frame::parse` slicing + the
//! `FrameWalker` decode, all against hand-written oracles.

use cerulion_core::codegen::frame_walker::{FrameValueKind, FrameWalker};
use cerulion_core::codegen::{parse_rosmsg, FieldDef, FieldType, MessageSchema};
use cerulion_core::shm_runtime::{read_offset_entry, write_offset_entry};
use cerulion_core::testing::TestTransport;
use cerulion_core::wire::{MaxSliceLen, OffsetEntry as CoreEntry, WireHeader as CoreHeader};
use cerulion_core::CerulionSubscriber;

use native_ros2_messages::geometry_msgs::Vector3;
use native_ros2_messages::std_msgs::String as RosString;
use native_ros2_messages::tf2_msgs::TFMessage;

use cerulion_wire::{Frame, OffsetEntry as WireEntry, WireHeader as WireHdr};

/// The struct sizes and the declared `SIZE` consts must be locked in lockstep.
#[test]
fn sizes_and_consts_are_locked() {
    assert_eq!(CoreHeader::SIZE, WireHdr::SIZE, "WireHeader::SIZE drift");
    assert_eq!(
        std::mem::size_of::<CoreHeader>(),
        std::mem::size_of::<WireHdr>(),
        "WireHeader size drift"
    );
    assert_eq!(
        std::mem::size_of::<CoreEntry>(),
        std::mem::size_of::<WireEntry>(),
        "OffsetEntry size drift"
    );
    assert_eq!(std::mem::size_of::<WireHdr>(), 32);
    assert_eq!(WireEntry::SIZE, 8);
}

/// The `#[repr(C)]` field offsets of the mirrored `WireHeader` must match
/// `cerulion_core`'s AND the fixed wire-contract offsets.
#[test]
fn header_field_offsets_match_core() {
    use std::mem::offset_of;
    assert_eq!(
        offset_of!(WireHdr, schema_hash),
        offset_of!(CoreHeader, schema_hash)
    );
    assert_eq!(
        offset_of!(WireHdr, total_size),
        offset_of!(CoreHeader, total_size)
    );
    assert_eq!(
        offset_of!(WireHdr, offset_table_offset),
        offset_of!(CoreHeader, offset_table_offset)
    );
    assert_eq!(
        offset_of!(WireHdr, offset_table_count),
        offset_of!(CoreHeader, offset_table_count)
    );
    assert_eq!(
        offset_of!(WireHdr, sequence),
        offset_of!(CoreHeader, sequence)
    );
    assert_eq!(
        offset_of!(WireHdr, timestamp_ns),
        offset_of!(CoreHeader, timestamp_ns)
    );

    // The absolute wire-contract offsets (little-endian, from frame start).
    assert_eq!(offset_of!(WireHdr, schema_hash), 0);
    assert_eq!(offset_of!(WireHdr, total_size), 8);
    assert_eq!(offset_of!(WireHdr, offset_table_offset), 12);
    assert_eq!(offset_of!(WireHdr, offset_table_count), 16);
    assert_eq!(offset_of!(WireHdr, sequence), 20);
    assert_eq!(offset_of!(WireHdr, timestamp_ns), 24);
}

/// A header serialized by `cerulion_core::WireHeader::write_to_buf` parses
/// field-identically with `cerulion-wire`, AND `cerulion-wire`'s encoder
/// reproduces the exact same 32 bytes (both directions).
#[test]
fn header_roundtrips_byte_identically_between_crates() {
    let core = CoreHeader {
        schema_hash: 0xFEED_FACE_1234_5678,
        total_size: 4096,
        offset_table_offset: 40,
        offset_table_count: 2,
        sequence: 99,
        timestamp_ns: 987_654_321,
    };
    let mut core_bytes = [0u8; CoreHeader::SIZE];
    core.write_to_buf(&mut core_bytes);

    // DECODE side: cerulion-wire parses core's bytes → every field equal.
    let w = WireHdr::parse(&core_bytes).expect("cerulion-wire parse");
    assert_eq!(w.schema_hash, core.schema_hash);
    assert_eq!(w.total_size, core.total_size);
    assert_eq!(w.offset_table_offset, core.offset_table_offset);
    assert_eq!(w.offset_table_count, core.offset_table_count);
    assert_eq!(w.sequence, core.sequence);
    assert_eq!(w.timestamp_ns, core.timestamp_ns);

    // ENCODE side: cerulion-wire encodes a header with the SAME hand-oracle
    // values → byte-identical to core's write_to_buf output.
    let w_hand = WireHdr {
        schema_hash: core.schema_hash,
        total_size: core.total_size,
        offset_table_offset: core.offset_table_offset,
        offset_table_count: core.offset_table_count,
        sequence: core.sequence,
        timestamp_ns: core.timestamp_ns,
    };
    assert_eq!(
        w_hand.to_bytes(),
        core_bytes,
        "cerulion-wire encode must byte-match core write_to_buf"
    );

    // And core reads cerulion-wire's encoding back to the identical header.
    let core_back = CoreHeader::read_from_buf(&w_hand.to_bytes()).expect("core read");
    assert_eq!(core_back, core);
}

/// An offset-table entry written by `cerulion_core::shm_runtime::write_offset_entry`
/// parses identically with `cerulion-wire`, and back again.
#[test]
fn offset_entry_roundtrips_byte_identically_between_crates() {
    let fixed_size = 8usize;
    let mut payload = vec![0u8; fixed_size + WireEntry::SIZE];
    // Core writes (offset=24, length=5) at variable-field index 0.
    write_offset_entry(&mut payload, fixed_size, 0, 24, 5);

    // Core reads it back.
    assert_eq!(read_offset_entry(&payload, fixed_size, 0), (24, 5));

    // cerulion-wire parses the same 8 table bytes.
    let entry = WireEntry::parse(&payload[fixed_size..]).expect("cerulion-wire parse");
    assert_eq!(entry, WireEntry::new(24, 5));

    // cerulion-wire encode reproduces the exact bytes core wrote.
    let mut w_bytes = [0u8; WireEntry::SIZE];
    entry.encode(&mut w_bytes).expect("cerulion-wire encode");
    assert_eq!(
        &w_bytes,
        &payload[fixed_size..fixed_size + WireEntry::SIZE],
        "cerulion-wire OffsetEntry encode must byte-match write_offset_entry"
    );
}

/// An Imglike message schema: fixed {height:u32, width:u32} + variable
/// {encoding:string, data:bytes} — the canonical fixed+variable shape.
fn imglike_schema() -> MessageSchema {
    let mut s = MessageSchema::new_in_package("Imglike", "msgs");
    s.add_field(FieldDef::new("height", FieldType::U32));
    s.add_field(FieldDef::new("width", FieldType::U32));
    s.add_field(FieldDef::new("encoding", FieldType::String));
    s.add_field(FieldDef::new("data", FieldType::Bytes));
    s
}

/// A FULL frame laid out by `cerulion_core`'s real serialization primitives is
/// sliced identically by `cerulion-wire`'s structural reader AND by
/// `cerulion_core`'s own schema-driven `FrameWalker` — the three-way agreement
/// pins the whole frame contract.
#[test]
fn full_frame_agrees_across_core_primitives_cerulion_wire_and_frame_walker() {
    let fixed_size = 8usize;
    let count = 2usize;
    let table_bytes = count * 8;
    let encoding: &[u8] = b"rgb8";
    let data: &[u8] = &[1, 2, 3, 4, 5];

    // Entry offsets are PAYLOAD-relative (from the first byte after the header).
    let enc_off = (fixed_size + table_bytes) as u32; // 24
    let data_off = enc_off + encoding.len() as u32; // 28

    // Build the payload with core's real primitives, exactly as the production
    // writer lays it out: [fixed section][offset table][variable payload].
    let mut payload = vec![0u8; fixed_size + table_bytes];
    payload[0..4].copy_from_slice(&480u32.to_le_bytes()); // height
    payload[4..8].copy_from_slice(&640u32.to_le_bytes()); // width
    write_offset_entry(&mut payload, fixed_size, 0, enc_off, encoding.len() as u32);
    write_offset_entry(&mut payload, fixed_size, 1, data_off, data.len() as u32);
    payload.extend_from_slice(encoding);
    payload.extend_from_slice(data);

    // A real schema hash so `walk_by_hash` resolves.
    let schema = imglike_schema();
    let schema_hash = schema.schema_hash();

    let total = (CoreHeader::SIZE + payload.len()) as u32;
    let core_hdr = CoreHeader {
        schema_hash,
        total_size: total,
        // Production stamps message-relative offset_table_offset = header + fixed.
        offset_table_offset: (CoreHeader::SIZE + fixed_size) as u32,
        offset_table_count: count as u32,
        sequence: 7,
        timestamp_ns: 12_345,
    };
    let mut frame = vec![0u8; CoreHeader::SIZE];
    core_hdr.write_to_buf(&mut frame);
    frame.extend_from_slice(&payload);

    // ---- cerulion-wire structural view ----
    let w = Frame::parse(&frame).expect("cerulion-wire Frame::parse");
    assert_eq!(w.header().schema_hash, schema_hash);
    assert_eq!(w.header().total_size, total);
    assert_eq!(w.header().offset_table_count, count as u32);
    assert_eq!(w.fixed_section(), &payload[..fixed_size]);
    assert_eq!(w.offset_entry_count(), count);
    assert_eq!(w.offset_entry(0), Some(WireEntry::new(enc_off, 4)));
    assert_eq!(w.offset_entry(1), Some(WireEntry::new(data_off, 5)));
    let w_enc = w.variable_field(0).expect("field 0").expect("in bounds");
    let w_data = w.variable_field(1).expect("field 1").expect("in bounds");
    assert_eq!(w_enc, encoding);
    assert_eq!(w_data, data);

    // ---- cerulion_core FrameWalker cross-check (independent interpreter) ----
    let (walker, warnings) = FrameWalker::new(vec![schema]);
    assert!(warnings.is_empty(), "walker warnings: {warnings:?}");
    let fv = walker
        .walk_by_hash(&frame)
        .expect("FrameWalker walk_by_hash");

    // Fixed fields.
    assert_eq!(fv.field("height"), Some(&FrameValueKind::U32(480)));
    assert_eq!(fv.field("width"), Some(&FrameValueKind::U32(640)));

    // Variable fields: the walker's decode must match cerulion-wire's raw
    // slices byte-for-byte.
    match fv.field("encoding") {
        Some(FrameValueKind::Str(s)) => assert_eq!(s.as_bytes(), w_enc),
        other => panic!("expected encoding Str, got {other:?}"),
    }
    match fv.field("data") {
        Some(FrameValueKind::Bytes(b)) => assert_eq!(*b, w_data),
        other => panic!("expected data Bytes, got {other:?}"),
    }
}

// ===========================================================================
// REAL writer/finalize path (loan_proxy -> setters -> OutputProxy::Drop).
//
// These build frames through the SAME code the production publisher runs, so
// the finalized `offset_table_offset`/`offset_table_count`/offset-table bytes
// are stamped by `cerulion_core`, NOT hand-written. Each shape is parsed by
// `cerulion-wire` AND `cerulion_core`'s `FrameWalker` and cross-checked against
// a hand oracle. Parallel-safe (per-test `TestTransport` SHM root) — crib of
// `frame_walker_production_test.rs`.
// ===========================================================================

/// A `FrameWalker` over every built-in ROS 2 schema (the exact `.msg` text the
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
        let mut frame = vec![0u8; CoreHeader::SIZE];
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

/// FIXED-ONLY real frame: a production `Vector3` (three f64, no variable
/// fields → `offset_table_count == 0`). `cerulion-wire` and `FrameWalker` must
/// agree with each other, the real bytes, and the hand oracle.
#[test]
fn production_fixed_only_frame_agrees_three_ways() {
    let tt = TestTransport::new();
    let mut publisher = tt.publisher("vec3", MaxSliceLen::const_new(256), 0);
    let sub = tt.subscriber("vec3");
    {
        let mut proxy = publisher.loan_proxy::<Vector3>().expect("loan Vector3");
        proxy.x = 1.5;
        proxy.y = -2.25;
        proxy.z = 3.125;
    }
    let frame = capture_last_frame(&sub);

    // cerulion-wire structural view.
    let w = Frame::parse(&frame).expect("cerulion-wire parse");
    assert_eq!(w.as_bytes(), frame.as_slice(), "byte-identity");
    assert_eq!(w.header().total_size as usize, frame.len());
    assert_eq!(
        w.offset_entry_count(),
        0,
        "fixed-only ⇒ zero variable fields"
    );
    assert!(w.variable_field(0).is_none());
    // Fixed-only ⇒ the whole payload is the fixed section (24 = 3 × f64).
    assert_eq!(w.fixed_section().len(), 24);
    assert_eq!(w.fixed_section(), w.payload());

    // FrameWalker decode == hand oracle.
    let fv = builtin_walker().walk_by_hash(&frame).expect("walk");
    assert_eq!(fv.schema_name, "geometry_msgs/Vector3");
    assert_eq!(fv.field("x"), Some(&FrameValueKind::F64(1.5)));
    assert_eq!(fv.field("y"), Some(&FrameValueKind::F64(-2.25)));
    assert_eq!(fv.field("z"), Some(&FrameValueKind::F64(3.125)));
}

/// ALL-VARIABLE real frame: a production `std_msgs/String` (`WIRE_FIXED_SIZE ==
/// 0`, one non-empty variable field). `cerulion-wire` slices the single field;
/// `FrameWalker` decodes it as a `Str`; both match the hand oracle.
#[test]
fn production_all_variable_frame_agrees_three_ways() {
    const TEXT: &str = "wire-lockstep";
    let tt = TestTransport::new();
    let mut publisher = tt.publisher("str", MaxSliceLen::const_new(256), 0);
    let sub = tt.subscriber("str");
    {
        let mut proxy = publisher.loan_proxy::<RosString>().expect("loan String");
        proxy.set_data(TEXT).expect("set_data");
    }
    let frame = capture_last_frame(&sub);

    // cerulion-wire structural view.
    let w = Frame::parse(&frame).expect("cerulion-wire parse");
    assert_eq!(w.as_bytes(), frame.as_slice(), "byte-identity");
    assert!(
        w.fixed_section().is_empty(),
        "all-variable ⇒ empty fixed section"
    );
    assert_eq!(w.offset_entry_count(), 1);
    let w_data = w.variable_field(0).expect("field 0").expect("in bounds");
    assert_eq!(
        w_data,
        TEXT.as_bytes(),
        "cerulion-wire slices the string body"
    );
    assert_eq!(
        w.offset_entry(0).expect("entry 0").length as usize,
        TEXT.len()
    );

    // FrameWalker decode == hand oracle == cerulion-wire's slice.
    let fv = builtin_walker().walk_by_hash(&frame).expect("walk");
    assert_eq!(fv.schema_name, "std_msgs/String");
    match fv.field("data") {
        Some(FrameValueKind::Str(s)) => {
            assert_eq!(*s, TEXT);
            assert_eq!(s.as_bytes(), w_data);
        }
        other => panic!("expected data Str, got {other:?}"),
    }
}

/// EMPTY variable field real frame: a production `tf2_msgs/TFMessage` with
/// `set_transforms_bytes(&[])` — the `/tf` empty-transforms-array shape. The
/// single variable field finalizes as an `(offset, 0)` entry; `cerulion-wire`
/// must slice it to an EMPTY slice (not an error, not a panic), and the
/// `FrameWalker` must decode the field present-but-empty.
#[test]
fn production_empty_variable_field_frame_agrees_three_ways() {
    let tt = TestTransport::new();
    let mut publisher = tt.publisher("tf", MaxSliceLen::const_new(256), 0);
    let sub = tt.subscriber("tf");
    {
        let mut proxy = publisher.loan_proxy::<TFMessage>().expect("loan TFMessage");
        proxy
            .set_transforms_bytes(&[])
            .expect("empty transforms idiom");
    }
    let frame = capture_last_frame(&sub);

    // cerulion-wire structural view.
    let w = Frame::parse(&frame).expect("cerulion-wire parse");
    assert_eq!(w.as_bytes(), frame.as_slice(), "byte-identity");
    assert!(
        w.fixed_section().is_empty(),
        "all-variable ⇒ empty fixed section"
    );
    assert_eq!(w.offset_entry_count(), 1);
    // THE empty-field pin: length-0 entry ⇒ empty slice, never an error/panic.
    assert_eq!(w.offset_entry(0).expect("entry 0").length, 0);
    let w_transforms = w.variable_field(0).expect("field 0").expect("empty is Ok");
    assert!(
        w_transforms.is_empty(),
        "empty variable field slices to &[]"
    );

    // FrameWalker decode: `transforms` is a DynamicArray<Nested-variable>.
    // The walker decodes that shape element-by-element, so an EMPTY blob is a
    // decoded ZERO-element array. restores BOTH assertions this
    // arm had lost: the variant is pinned EXACTLY (accepting
    // `NestedArrayOpaque`/`Bytes` as well would have let the empty-blob rule be
    // reverted with this test still green), and the walker's view of the field
    // is BYTE-COMPARED against cerulion-wire's slice — which the decoded
    // variant's `raw` makes expressible again.
    let fv = builtin_walker().walk_by_hash(&frame).expect("walk");
    assert_eq!(fv.schema_name, "tf2_msgs/TFMessage");
    match fv.field("transforms") {
        Some(FrameValueKind::NestedArray { elements, raw }) => {
            assert!(
                elements.is_empty(),
                "walker: empty transforms decodes to zero elements"
            );
            assert_eq!(
                *raw, w_transforms,
                "walker and cerulion-wire must see the SAME field bytes"
            );
            assert!(
                w_transforms.is_empty(),
                "cerulion-wire agrees the field is empty"
            );
        }
        other => panic!("expected an empty decoded transforms array, got {other:?}"),
    }
}
