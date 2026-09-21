// SPDX-License-Identifier: AGPL-3.0-only
//! The WIRE-LEGALITY oracle for top-level
//! GAP-FRAME placement.
//!
//! The borrow-window publish path emits frames whose top-level offset-table entries point at a
//! PAGE-ALIGNED, non-contiguous placement of a big variable field inside the
//! slot tail, with dead gap bytes inside `total_size` and fields placed out
//! of declaration order. That shape is legal wire by CONTRACT, not by
//! omission: `PayloadAudit::Frame` (frame_walker.rs) bounds each entry in
//! isolation (`off >= data_floor`, `off + len <= payload.len()`) and
//! deliberately requires no ordering / contiguity / exact accounting — and
//! this file is what pins that contract, so a future tightening of the
//! top-level audit fails HERE instead of silently breaking every
//! borrow-window frame. (The canonical ELEMENT audit stays strict — that is
//! `PayloadAudit::Element`'s job, untouched.)
//!
//! The fixture is the ONE stated layout in
//! [`cerulion_core::testing::gap_frame`]; the pins:
//!
//! * [`FrameWalker::walk_by_hash`] ACCEPTS the gap frame and decodes every
//!   field byte-exact against the hand oracle (fixed fields, the nested
//!   `header` sub-frame, the out-of-order `encoding`, the page-aligned
//!   `data`);
//! * the walker REFUSES an entry below `data_floor` (nonzero length at
//!   `data_floor - 1`, and the off-0-nonzero-len shape aliasing the fixed
//!   section) — the floor IS part of the pinned contract;
//! * the GENERATED typed accessors slice the same frame correctly over the
//!   real transport (`publish_raw` → `try_view::<Image>`).
//!
//! # The stated walker/accessor ASYMMETRY
//!
//! The generated accessors bounds-check only `off + len <= payload.len()` —
//! they do NOT enforce the `data_floor` (an accessor over a floor-violating
//! frame slices whatever the entry names, fixed-section bytes included).
//! That asymmetry is STATED here as a known property, deliberately not
//! pinned behaviorally: the walker's floor refusal is the contract, and
//! freezing the accessor's skip would block a future tightening of the
//! accessor side.
//!
//! What breaks these pins:
//! * tightening `PayloadAudit::Frame` to the Element ordering/contiguity
//!   rule fails `gap_frame_walks_by_hash_to_hand_written_values` (and the
//!   accessor arm's decode stays green — accessors never audited);
//! * deleting the `off >= data_floor` conjunct fails
//!   `entry_below_the_data_floor_is_refused`.
//!
//! Parallel-safe: hand-built frames + an isolated per-test SHM root
//! ([`TestTransport`]) for the accessor arm.

use cerulion_core::codegen::{parse_rosmsg, FrameValueKind, FrameWalker, MessageSchema, WalkError};
use cerulion_core::message::ShmMessage;
use cerulion_core::shm_runtime::write_offset_entry;
use cerulion_core::testing::gap_frame::{
    self, image_gap_frame, DATA_FLOOR, DATA_LEN, DATA_OFF, ENCODING, HEADER_FRAME_ID,
    HEADER_NANOSEC, HEADER_SEC, HEIGHT, IMAGE_VARIABLE_FIELD_COUNT, IMAGE_WIRE_FIXED_SIZE,
    IS_BIGENDIAN, STEP, WIDTH,
};
use cerulion_core::testing::TestTransport;
use cerulion_core::wire::{MaxSliceLen, WireHeader};
use native_ros2_messages::sensor_msgs::Image;

/// Walker over every built-in ROS 2 schema (the `frame_walker_production_test`
/// convention).
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

/// The fixture's layout constants must match the GENERATED type's wire shape —
/// the drift guard that keeps the hand-laid frame correct if Image's schema
/// ever changes.
fn assert_fixture_matches_generated_layout() {
    assert_eq!(Image::WIRE_FIXED_SIZE, IMAGE_WIRE_FIXED_SIZE);
    assert_eq!(Image::VARIABLE_FIELD_COUNT, IMAGE_VARIABLE_FIELD_COUNT);
}

/// The headline acceptance oracle: the gap frame — page-aligned big field,
/// out-of-declaration-order placement, dead gaps, trailing slack — walks by
/// its own header hash and every field decodes byte-exact to the hand values.
#[test]
fn gap_frame_walks_by_hash_to_hand_written_values() {
    assert_fixture_matches_generated_layout();
    let walker = builtin_walker();
    let frame = image_gap_frame(Image::SCHEMA_HASH, 3, 12_345);

    let fv = walker
        .walk_by_hash(&frame)
        .expect("the gap frame is legal wire and must walk by its own header hash");
    assert_eq!(fv.schema_name, "sensor_msgs/Image");

    // Fixed section.
    assert_eq!(fv.field("height"), Some(&FrameValueKind::U32(HEIGHT)));
    assert_eq!(fv.field("width"), Some(&FrameValueKind::U32(WIDTH)));
    assert_eq!(
        fv.field("is_bigendian"),
        Some(&FrameValueKind::U8(IS_BIGENDIAN))
    );
    assert_eq!(fv.field("step"), Some(&FrameValueKind::U32(STEP)));

    // The out-of-declaration-order `encoding` (declared 2nd, placed LAST).
    assert_eq!(fv.field("encoding"), Some(&FrameValueKind::Str(ENCODING)));

    // The page-aligned big field, byte-exact.
    assert_eq!(
        fv.field("data"),
        Some(&FrameValueKind::Bytes(&gap_frame::data_bytes()))
    );

    // The nested `header` sub-frame decodes through the gap placement.
    match fv.field("header") {
        Some(FrameValueKind::Nested(h)) => {
            assert_eq!(h.schema_name, "std_msgs/Header");
            assert_eq!(
                h.field("frame_id"),
                Some(&FrameValueKind::Str(HEADER_FRAME_ID))
            );
            match h.field("stamp") {
                Some(FrameValueKind::Nested(stamp)) => {
                    assert_eq!(stamp.schema_name, "builtin_interfaces/Time");
                    assert_eq!(stamp.field("sec"), Some(&FrameValueKind::I32(HEADER_SEC)));
                    assert_eq!(
                        stamp.field("nanosec"),
                        Some(&FrameValueKind::U32(HEADER_NANOSEC))
                    );
                }
                other => panic!("expected Nested stamp, got {other:?}"),
            }
        }
        other => panic!("expected Nested header, got {other:?}"),
    }
}

/// The refusal half of the pinned contract: an entry with a NONZERO length
/// below `data_floor` is refused as `VariableFieldOutOfBounds` naming the
/// field — in BOTH shapes: just under the floor (`data_floor - 1`, the
/// tightest boundary), and `off = 0` (aliasing the fixed section — NOT the
/// unwritten-field idiom, which requires `len == 0`).
#[test]
fn entry_below_the_data_floor_is_refused() {
    assert_fixture_matches_generated_layout();
    let walker = builtin_walker();

    for (why, off) in [
        ("one byte under the floor", (DATA_FLOOR - 1) as u32),
        ("aliasing the fixed section at offset 0", 0u32),
    ] {
        let mut frame = image_gap_frame(Image::SCHEMA_HASH, 0, 1);
        // Rewrite the `data` entry (declaration index 2) to point below the
        // floor, length untouched (nonzero).
        write_offset_entry(
            &mut frame[WireHeader::SIZE..],
            IMAGE_WIRE_FIXED_SIZE,
            2,
            off,
            DATA_LEN as u32,
        );
        match walker.walk_by_hash(&frame) {
            Err(WalkError::VariableFieldOutOfBounds { field, offset, .. }) => {
                assert_eq!(field, "data", "{why}: the refusal names the field");
                assert_eq!(offset, off as usize, "{why}: the refusal names the offset");
            }
            Err(other) => panic!("{why}: expected VariableFieldOutOfBounds, got {other:?}"),
            Ok(_) => panic!("{why}: an entry below the data floor must be refused"),
        }
    }
}

/// A zero-length entry stays the unwritten-field idiom whatever its offset —
/// the boundary the floor refusal must NOT swallow (a `(0, 0)` entry decodes
/// as an empty field, never an error).
#[test]
fn zero_length_entry_is_the_unwritten_idiom_not_a_floor_violation() {
    assert_fixture_matches_generated_layout();
    let walker = builtin_walker();
    let mut frame = image_gap_frame(Image::SCHEMA_HASH, 0, 1);
    write_offset_entry(
        &mut frame[WireHeader::SIZE..],
        IMAGE_WIRE_FIXED_SIZE,
        2,
        0,
        0,
    );
    let fv = walker
        .walk_by_hash(&frame)
        .expect("a (0, 0) entry is the unwritten idiom, not a corruption");
    assert_eq!(
        fv.field("data"),
        Some(&FrameValueKind::Bytes(&[])),
        "the unwritten field decodes as empty"
    );
    // The sibling gap-placed fields are untouched by the rewrite.
    assert_eq!(fv.field("encoding"), Some(&FrameValueKind::Str(ENCODING)));
}

/// The GENERATED typed accessors slice the same gap frame correctly over the
/// real transport: `publish_raw` carries the hand-built frame into SHM and
/// `try_view::<Image>` serves the generated `ImageShm` view — the third,
/// independent reader implementation (walker, accessors, and the raw
/// byte-fidelity suites all agree on the one stated layout).
#[test]
fn generated_accessors_slice_the_gap_frame_correctly() {
    assert_fixture_matches_generated_layout();
    let tt = TestTransport::new();
    let mut publisher = tt.publisher("gap/image", MaxSliceLen::const_new(64 * 1024), 0);
    let mut sub = tt.subscriber("gap/image");

    let frame = image_gap_frame(Image::SCHEMA_HASH, 0, 42);
    publisher.publish_raw(&frame).expect("publish gap frame");

    let observed = sub
        .try_view::<Image, _>(|view| {
            (
                view.height,
                view.width,
                view.is_bigendian,
                view.step,
                view.encoding().expect("encoding is valid UTF-8").to_owned(),
                view.data().to_vec(),
                view.header_bytes().to_vec(),
            )
        })
        .expect("try_view must not error")
        .expect("the gap frame must be delivered");

    assert_eq!(
        observed,
        (
            HEIGHT,
            WIDTH,
            IS_BIGENDIAN,
            STEP,
            ENCODING.to_owned(),
            gap_frame::data_bytes(),
            gap_frame::header_subframe(),
        ),
        "every generated accessor serves the hand-written value from the gap placement"
    );

    // Placement sanity, stated where a reader can see it (compile-time — the
    // operands are the fixture's consts): the served frame really is the gap
    // shape (big field page-aligned, encoding placed after it), so the
    // accessor pin above cannot be satisfied by a contiguous re-layout of the
    // fixture.
    const {
        assert!(DATA_OFF.is_multiple_of(4096));
        assert!(gap_frame::ENCODING_OFF > DATA_OFF + DATA_LEN);
    }
}
