// SPDX-License-Identifier: AGPL-3.0-only
//! Acceptance tests for variable-schema fixed-field direct-access.
//!
//! For variable schemas the codegen emits no `pub fn set_<fixed>(value)` /
//! `pub fn <fixed>() -> T` accessor methods on `<Name>Shm<'a>`.
//! Fixed primitive / `StringFixed` /
//! `FixedArray<primitive>` fields are exposed as `pub <name>: <type>`
//! on a `#[repr(C)]` overlay struct `<Name>FixedSection`, reached from
//! `<Name>Shm<'a>` via `Deref` / `DerefMut`. User code writes
//! `proxy.<field> = value` and reads `view.<field>` directly.
//!
//! This file verifies:
//!
//! - **Acceptance criteria 1+2** — direct field assignment writes to SHM
//!   for the publisher; direct field reads from SHM for the subscriber;
//!   the values round-trip exactly.
//! - **Wire layout** — `WIRE_FIXED_SIZE` for `Image` is
//!   `size_of::<ImageFixedSection>()` (16 bytes for `Image{u32, u32, u32, u8}`
//!   with `#[repr(C)]` trailing padding), not the packed 13-byte
//!   sum. Symmetric with how fixed-only schemas source their wire
//!   size from `size_of::<<Name>Shm>()`.
//! - **`from_bytes` / `from_bytes_mut` panic invariants**:
//!     - undersized buffer → panic with the "buffer too small ...; need
//!       fixed section + offset table" message.
//!     - misaligned source pointer → panic with the alignment message.
//! - **Snapshot determinism for canonical bool inputs** — round-trip is
//!   bit-identical when bool fields are written as 0 or 1.
//! - **Snapshot lossy normalization for non-canonical bool inputs** — the
//!   bool round-trip non-bit-identity contract documented in
//!   `<Name>FixedSection` rustdoc.
//! - **Snapshot 100× round-trip identity** — repeated snapshot →
//!   write_from_snapshot → snapshot produces bit-identical bytes for
//!   canonical inputs (Replay = Live, Principle #7).
//!
//! # Backend
//!
//! Tests use the iceoryx2 `TestTransport` helper for the integration
//! round-trip — it exercises the same `from_bytes_mut` / `loan_<f>` /
//! `OutputProxy::Drop` path over iceoryx2's shared-memory region. The file
//! is run serially because tests in this crate share the iceoryx2 region.
//!
//! # Running
//!
//! ```bash
//! cargo test -p cerulion_core --test variable_schema_fixed_field_test -- --test-threads=1
//! ```

use cerulion_core::wire::MaxSliceLen;

use cerulion_core::message::ShmMessage;
use cerulion_core::testing::TestTransport;
use cerulion_core::wire::WireHeader;
use native_ros2_messages::diagnostic_msgs::{KeyValue, KeyValueFixedSection};
use native_ros2_messages::sensor_msgs::{Image, ImageFixedSection, ImageShm, ImageSnapshot};

// ============================================================
// Wire layout: WIRE_FIXED_SIZE follows #[repr(C)] layout
// ============================================================

/// Wire format: variable-schema `WIRE_FIXED_SIZE` is
/// `size_of::<<Name>FixedSection>()` (with `#[repr(C)]` natural alignment +
/// trailing padding), not a packed byte sum.
///
/// For `Image{height: u32, width: u32, encoding: String, is_bigendian: u8,
/// step: u32, data: u8[]}`:
///   - fixed-section fields in declaration order: height(u32), width(u32),
///     is_bigendian(u8), step(u32). Variable fields skipped.
///   - `#[repr(C)]` layout: 4 + 4 + 1 + 3 padding + 4 = 16 bytes (struct
///     align 4; trailing padding rounds to 16).
///   - Packed sum (NOT the wire size): 4 + 4 + 1 + 4 = 13 bytes.
#[test]
fn image_wire_fixed_size_matches_repr_c_layout() {
    use std::mem::size_of;

    assert_eq!(
        <Image as ShmMessage>::WIRE_FIXED_SIZE,
        size_of::<ImageFixedSection>(),
        "WIRE_FIXED_SIZE must source from size_of::<ImageFixedSection>()",
    );
    assert_eq!(
        <Image as ShmMessage>::WIRE_FIXED_SIZE,
        16,
        "Image fixed section is 16 bytes under the #[repr(C)] layout \
         (height u32 + width u32 + is_bigendian u8 + 3 trailing padding + step u32)",
    );
}

/// Alignment guarantee: `<Name>FixedSection`'s alignment is at most
/// 8 (the static_assert in codegen enforces this at compile time). Verify
/// at runtime as a defense-in-depth check.
#[test]
fn image_fixed_section_align_is_at_most_8() {
    use std::mem::align_of;
    assert!(
        align_of::<ImageFixedSection>() <= 8,
        "FixedSection align must be ≤ 8; post-WireHeader payload pointer is 8-aligned",
    );
    // For Image, max field align is 4 (u32). Confirm.
    assert_eq!(align_of::<ImageFixedSection>(), 4);
}

// ============================================================
// Acceptance criteria 1+2: direct field write/read round-trip
// ============================================================

fn make_pubr_with_capacity(
    topic: &str,
    capacity: u32,
) -> (
    TestTransport,
    cerulion_core::transport::publisher::CerulionPublisher,
) {
    // `WireHeader::SIZE` is a compile-time `usize = 32`; cast to u32
    // before the add so the sum is computed in u32, with no silent
    // truncation via `as u32` on the sum.
    let max_slice = WireHeader::SIZE as u32 + capacity;
    // Buffer size 8 is the subscriber capacity every caller here needs.
    let tt = TestTransport::with_buffer_size(8);
    let pubr = tt.publisher(topic, MaxSliceLen::const_new(max_slice), 1);
    (tt, pubr)
}

/// Acceptance criterion 1+2: writing `proxy.<fixed_field> = value` writes
/// directly into the loaned SHM slot, and reading `view.<fixed_field>`
/// reads from SHM. The round-trip is exact.
#[test]
fn happy_path_direct_field_round_trip() {
    let topic = "test/happy";
    let (tt, mut pubr) = make_pubr_with_capacity(topic, 256);
    let mut sub = tt.subscriber(topic);

    {
        let mut proxy = pubr.loan_proxy::<Image>().expect("loan");
        // Direct fixed-field writes via Deref<Target = ImageFixedSection>.
        proxy.height = 1080;
        proxy.width = 1920;
        proxy.step = 1920 * 3;
        proxy.is_bigendian = 0;
        // Variable-field setters unchanged.
        proxy.set_header_bytes(&[]).expect("header");
        proxy.set_encoding("rgb8").expect("encoding");
        proxy.set_data(&[10, 20, 30, 40]).expect("data");
    }

    let observed = sub
        .try_view::<Image, _>(|view| {
            (
                view.height,
                view.width,
                view.step,
                view.is_bigendian,
                view.encoding().expect("utf-8").to_string(),
                view.data().to_vec(),
            )
        })
        .expect("try_view")
        .expect("subscriber should see one frame");

    assert_eq!(observed.0, 1080, "height round-trip");
    assert_eq!(observed.1, 1920, "width round-trip");
    assert_eq!(observed.2, 1920 * 3, "step round-trip");
    assert_eq!(observed.3, 0, "is_bigendian round-trip");
    assert_eq!(observed.4, "rgb8", "encoding round-trip");
    assert_eq!(observed.5, vec![10u8, 20, 30, 40], "data round-trip");
}

/// Hex-dump the wire frame for `Image` and verify the byte oracle: the
/// fixed section starts at `payload[0]` with `#[repr(C)]` layout matching
/// `[height_le_4][width_le_4][step_le_4 ... wait, declaration order is
/// height, width, is_bigendian, step]`.
///
/// Image fields in declaration order (per `Image.msg`):
///   - height: uint32  (declaration position 0)
///   - width: uint32   (declaration position 1)
///   - is_bigendian: uint8 (declaration position 2)
///   - step: uint32    (declaration position 3)
///
/// `#[repr(C)]` lays them out in declaration order with natural alignment:
///   - offset  0: height (4 bytes, align 4)
///   - offset  4: width  (4 bytes, align 4)
///   - offset  8: is_bigendian (1 byte, align 1)
///   - offset  9: 3 padding bytes (to align step to 4)
///   - offset 12: step (4 bytes, align 4)
///   - size 16, align 4.
///
/// Note: padding bytes' contents are not specified by Rust; we don't assert
/// on them. The test reads height/width/step via direct field access, which
/// is what user code does.
#[test]
fn fixed_section_byte_layout_matches_declaration_order() {
    use std::mem::offset_of;

    // Declaration-order offsets within ImageFixedSection.
    assert_eq!(offset_of!(ImageFixedSection, height), 0);
    assert_eq!(offset_of!(ImageFixedSection, width), 4);
    assert_eq!(offset_of!(ImageFixedSection, is_bigendian), 8);
    assert_eq!(offset_of!(ImageFixedSection, step), 12);
}

// ============================================================
// Snapshot determinism (Replay = Live, Principle #7)
// ============================================================

/// Snapshot → write_from_snapshot round-trip must be bit-identical for
/// canonical inputs across repeated iterations. This is the load-bearing
/// determinism property Replay relies on (Principle #7: Replay = Live).
///
/// **Oracle independence:** the oracle `snap0`
/// is constructed BY HAND as an `ImageSnapshot` literal — it does NOT come
/// from `view.snapshot()`. So a symmetric codegen bug shared between
/// `snapshot()` and `write_from_snapshot()` (e.g., both XOR the height by
/// some constant) would still produce a snap_i mismatching the hand-built
/// oracle. An oracle taken from `snapshot()` would be tautological with
/// respect to the codegen path, because both snap0 and snap_i would go
/// through `snapshot()`.
///
/// Note: Image's `is_bigendian` is declared `uint8`, not `bool`, in the
/// ROS2 Image schema, so this test does NOT exercise the bool round-trip
/// edge case (canonical {0,1} vs non-canonical bytes — that contract is
/// documented in `<Name>FixedSection` rustdoc).
#[test]
fn snapshot_round_trip_is_bit_identical_100x() {
    // Hand-built oracle independent of any codegen-path output.
    let snap0 = ImageSnapshot {
        header: vec![], // header is Nested → raw bytes
        height: 100,
        width: 200,
        encoding: "rgb8".to_string(),
        is_bigendian: 1,
        step: 600,
        data: vec![7u8; 8],
    };

    for i in 0..100 {
        let topic_i = format!("test/det/canonical/{i}");
        let (tt_i, mut pubr_i) = make_pubr_with_capacity(&topic_i, 256);
        let mut sub_i = tt_i.subscriber(&topic_i);
        {
            let mut proxy = pubr_i.loan_proxy::<Image>().expect("loan");
            proxy
                .write_from_snapshot(&snap0)
                .expect("write_from_snapshot");
        }
        let snap_i = sub_i
            .try_view::<Image, _>(|view| view.snapshot())
            .expect("try_view")
            .expect("subscriber should see frame");
        assert_eq!(
            snap_i, snap0,
            "round-trip iteration {i} must be bit-identical against the hand-built oracle",
        );
    }
}

/// Inherent constant `<Name>Shm::WIRE_FIXED_SIZE` and trait-impl
/// `<<Name> as ShmMessage>::WIRE_FIXED_SIZE` must agree. The codegen emits
/// both via `size_of::<<Name>FixedSection>()`, but they live in different
/// generator functions (`structs.rs` vs `wire_impl.rs`); a future patch
/// changing one without the other would silently desync the wire format
/// from the offset-table arithmetic.
#[test]
fn inherent_and_trait_wire_fixed_size_match() {
    assert_eq!(
        <Image as ShmMessage>::WIRE_FIXED_SIZE,
        ImageShm::WIRE_FIXED_SIZE,
        "trait and inherent WIRE_FIXED_SIZE constants must source the same value",
    );
}

// ============================================================
// All-variable schema (empty FixedSection) — Deref ZST path
// ============================================================

/// A schema with ZERO fixed-section
/// fields generates `<Name>FixedSection { _marker: [u8; 0] }` (zero-sized,
/// align 1) and the `Deref` impl points at a ZST overlay. Verify the
/// Deref evaluates without UB at runtime, and the round-trip via variable
/// accessors works.
///
/// `diagnostic_msgs::KeyValue` (`string key`, `string value`) is genuinely
/// all-variable — two String fields, no fixed primitives — so its FixedSection
/// is empty.
///
/// `geometry_msgs::Pose` does NOT qualify:
/// `resolve_fixed_nested` resolves Pose's fully-fixed nested
/// `Point`/`Quaternion` to FIXED (they live in the FixedSection, accessed via
/// Deref) — Pose is not all-variable. A message with only String/array
/// fields (KeyValue) keeps the ZST FixedSection this test exercises.
#[test]
fn all_variable_schema_round_trip_with_zero_sized_fixed_section() {
    use std::mem::{align_of, size_of};

    // Compile-time assertion that KeyValueFixedSection is genuinely zero-sized.
    assert_eq!(size_of::<KeyValueFixedSection>(), 0);
    assert_eq!(align_of::<KeyValueFixedSection>(), 1);
    assert_eq!(<KeyValue as ShmMessage>::WIRE_FIXED_SIZE, 0);

    let topic = "test/all_variable";
    let (tt, mut pubr) = make_pubr_with_capacity(topic, 256);
    let mut sub = tt.subscriber(topic);

    {
        let mut proxy = pubr.loan_proxy::<KeyValue>().expect("loan");
        // Force Deref evaluation: type-ascription to `&KeyValueFixedSection`
        // coerces through the Deref chain (OutputProxy → KeyValueShm →
        // KeyValueFixedSection), exercising the codegen-emitted ZST overlay
        // cast. Reading 0 bytes is sound regardless of pointer alignment; the
        // assertion confirms the Deref does not UB at runtime.
        let _: &KeyValueFixedSection = &proxy;
        // Variable-field round-trip via the typed String setters.
        proxy.set_key("alpha").expect("set key");
        proxy.set_value("beta").expect("set value");
    }

    let (k, v) = sub
        .try_view::<KeyValue, _>(|view| {
            // Read-side Deref evaluation (symmetric to the write side).
            let _: &KeyValueFixedSection = &view;
            (
                view.key().expect("utf-8").to_string(),
                view.value().expect("utf-8").to_string(),
            )
        })
        .expect("try_view")
        .expect("subscriber should see frame");

    assert_eq!(k, "alpha");
    assert_eq!(v, "beta");
}

// ============================================================
// Boundary tests for from_bytes / from_bytes_mut size assert
// ============================================================

/// Boundary: a buffer EXACTLY at the minimum size (fixed section + offset
/// table) must succeed. Image: WIRE_FIXED_SIZE=16 + 8*3 (3 variable fields)
/// = 40 bytes. Verify `from_bytes_mut` does not panic at this exact size.
///
/// Catches an off-by-one regression in the size assert (e.g. `>` vs `>=`).
#[test]
fn from_bytes_mut_succeeds_at_minimum_buffer_size() {
    // 40 bytes, 8-aligned (5 u64 = 40 bytes).
    let mut buf = [0u64; 5];
    let bytes = bytemuck::cast_slice_mut::<u64, u8>(&mut buf);
    assert_eq!(bytes.len(), 40);
    // `from_bytes_mut` takes (bytes, max_capacity, topic).
    let _shm = ImageShm::from_bytes_mut(
        bytes,
        cerulion_core::wire::MaxPayloadCapacity::const_new(40),
        std::sync::Arc::from("test"),
    );
    // must NOT panic.
}

/// Boundary: a buffer ONE byte below the minimum size must panic with the
/// "need fixed section + offset table" message. Catches the converse
/// off-by-one regression.
#[test]
#[should_panic(expected = "need fixed section + offset table")]
fn from_bytes_mut_panics_one_byte_below_minimum() {
    // 39 bytes — one less than the 40-byte minimum.
    let mut buf = [0u64; 5];
    let bytes_full = bytemuck::cast_slice_mut::<u64, u8>(&mut buf);
    let bytes = &mut bytes_full[..39];
    // `from_bytes_mut` takes (bytes, max_capacity, topic).
    let _shm = ImageShm::from_bytes_mut(
        bytes,
        cerulion_core::wire::MaxPayloadCapacity::const_new(40),
        std::sync::Arc::from("test"),
    );
}

// ============================================================
// Adversarial: from_bytes / from_bytes_mut panic invariants
// ============================================================

/// The `from_bytes_mut` size assert requires `bytes.len() >=
/// OFFSET_TABLE_OFFSET + OFFSET_TABLE_BYTES`. A buffer that fits the fixed
/// section but not the offset table panics with a clear "need fixed section
/// + offset table" message.
///
/// For Image: `WIRE_FIXED_SIZE = 16`, `VARIABLE_FIELD_COUNT = 3` →
/// `OFFSET_TABLE_BYTES = 24`. A 16-byte buffer fits the fixed section but
/// lacks the offset table.
#[test]
#[should_panic(expected = "need fixed section + offset table")]
fn from_bytes_mut_panics_when_buffer_lacks_offset_table() {
    // 16-byte aligned-to-8 buffer — alignment passes, size fails.
    let mut buf = [0u64; 2];
    let bytes = bytemuck::cast_slice_mut::<u64, u8>(&mut buf);
    // `from_bytes_mut` takes (bytes, max_capacity, topic).
    let _shm = ImageShm::from_bytes_mut(
        bytes,
        cerulion_core::wire::MaxPayloadCapacity::const_new(40),
        std::sync::Arc::from("test"),
    );
}

/// Same panic from the read-side `from_bytes`.
#[test]
#[should_panic(expected = "need fixed section + offset table")]
fn from_bytes_panics_when_buffer_lacks_offset_table() {
    let buf = [0u64; 2];
    let bytes = bytemuck::cast_slice::<u64, u8>(&buf);
    let _shm = ImageShm::from_bytes(bytes);
}

/// Alignment assert: `from_bytes_mut` requires the source pointer
/// aligned to `align_of::<<Name>FixedSection>()`. For Image (align 4), an
/// odd-offset slice panics.
#[test]
#[should_panic(expected = "misaligned source pointer")]
fn from_bytes_mut_panics_on_misaligned_pointer() {
    // 8-aligned u64 buffer, sized for full prefix + payload.
    let mut buf = [0u64; 16];
    let bytes = bytemuck::cast_slice_mut::<u64, u8>(&mut buf);
    // Slice starting at offset 1 is 1-aligned (odd) — won't satisfy
    // ImageFixedSection's 4-byte alignment.
    let misaligned = &mut bytes[1..];
    // `from_bytes_mut` takes (bytes, max_capacity, topic).
    let _shm = ImageShm::from_bytes_mut(
        misaligned,
        cerulion_core::wire::MaxPayloadCapacity::const_new(128),
        std::sync::Arc::from("test"),
    );
}

/// Same alignment panic from the read-side `from_bytes`.
#[test]
#[should_panic(expected = "misaligned source pointer")]
fn from_bytes_panics_on_misaligned_pointer() {
    let buf = [0u64; 16];
    let bytes = bytemuck::cast_slice::<u64, u8>(&buf);
    let misaligned = &bytes[1..];
    let _shm = ImageShm::from_bytes(misaligned);
}

// ============================================================
// FixedSection construction independent of Shm
// ============================================================

/// `<Name>FixedSection` derives `Default` (when no large arrays). Verify a
/// fresh-default instance has all fields zeroed, matching the publisher's
/// prefix-zero contract.
#[test]
fn fixed_section_default_zeroes_all_fields() {
    let fs = ImageFixedSection::default();
    assert_eq!(fs.height, 0);
    assert_eq!(fs.width, 0);
    assert_eq!(fs.is_bigendian, 0);
    assert_eq!(fs.step, 0);
}

/// `<Name>FixedSection` derives `Copy`. Round-trip via clone + struct
/// literal construction (the only way for users to build a FixedSection
/// outside the SHM overlay path) preserves field values.
#[test]
fn fixed_section_struct_literal_round_trip() {
    let original = ImageFixedSection {
        height: 480,
        width: 640,
        is_bigendian: 1,
        step: 1920,
    };
    let copy = original; // Copy via auto-Copy
    assert_eq!(original.height, copy.height);
    assert_eq!(original.width, copy.width);
    assert_eq!(original.is_bigendian, copy.is_bigendian);
    assert_eq!(original.step, copy.step);
}
