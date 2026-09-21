// SPDX-License-Identifier: AGPL-3.0-only
//! Codegen-emitted `fill_from_<f>` / `fill_from_<f>_bytes`
//! methods on `<Name>Shm`.
//!
//! These tests construct the `<Name>Shm` writer directly from a raw byte
//! buffer (no iceoryx2, no transport — pure codegen behavior). The
//! `cerulion_core::message::ShmMessage::build_writer(&mut bytes)` path
//! is the same one `OutputProxy::loan_proxy` calls internally, so the
//! cursor / offset-table state machine these tests exercise is the
//! identical one used in production.
//!
//! # What's tested
//!
//! - `fill_from_data` writes directly into the SHM region (pointer
//!   identity proof — no `Vec` indirection).
//! - Short-write truncation: producer returns `n < remaining`, the
//!   offset entry records `n`, and a subsequent `set_<other>` lands at
//!   `cursor_before + n`, not `cursor_before + remaining`. **#1 risk
//!   area — catches the cursor-bookkeeping bug.**
//! - Adversarial producer returns `written > dst.len()` → defensively
//!   clamped.
//! - Producer `Err` rewinds the cursor + leaves `mark_written` unset
//!   (the publish-gate canary that `OutputProxy::Drop` consults).
//! - Producer panic — same end state (mark_written unset, publish gated)
//!   via natural panic propagation, no `catch_unwind` needed.
//! - Typed-array `fill_from_<f><S: FillFrom<f32>>` on a primitive
//!   typed-array variable field.
//! - Re-write a variable field after writing another (orphaned-bytes
//!   semantics matching `set_<f>` behavior).
//! - Reserved-name collision: a schema field literally named
//!   `fill_from_foo` collides with the reserved accessor name —
//!   compile-time error via the L1707 `compile_error!` (covered by
//!   a separate `tests/ui/` test since trybuild snapshots
//!   live there).

use cerulion_core::message::ShmMessage;
use cerulion_core::transport::fill_from::SliceSource;
use cerulion_core::TransportError;
use native_ros2_messages::sensor_msgs::Image;

/// Buffer big enough for an Image with a few KiB of variable payload.
const BUF: usize = 8192;

/// Helper: a fresh zeroed buffer for `Image::build_writer`. The buffer
/// must be at least `Image::WIRE_FIXED_SIZE + 8 * 3` (fixed + offset
/// table) — `BUF` is plenty.
fn fresh_buf() -> Vec<u8> {
    vec![0u8; BUF]
}

// ============================================================
// Happy path: bytes-field fill_from writes directly to SHM
// ============================================================

#[test]
fn fill_from_data_writes_into_shm_region() {
    let mut bytes = fresh_buf();
    let payload_ptr_before = bytes.as_ptr() as usize;
    // `build_writer` now requires `max_capacity` + `topic`. These
    // tests exercise the steady-state path (writes fit in BUF), so the
    // ceiling equals the buffer size and overflow never fires.
    let mut writer = Image::build_writer(
        &mut bytes,
        cerulion_core::wire::MaxPayloadCapacity::const_new(BUF as u32),
        std::sync::Arc::from("test"),
    );

    // Required: write every variable field. Use minimal placeholders
    // so the producer's slice goes into the data field unambiguously.
    writer.set_header_bytes(&[]).expect("header empty");
    writer.set_encoding("g").expect("encoding");

    // Pointer-identity assertion: capture dst.as_ptr() inside the
    // producer and verify it points into the original buffer (not a
    // detached Vec).
    let mut dst_ptr_seen: usize = 0;
    writer
        .fill_from_data(|dst: &mut [u8]| {
            dst_ptr_seen = dst.as_ptr() as usize;
            dst[..4].copy_from_slice(b"abcd");
            Ok(4)
        })
        .expect("fill_from_data should succeed");

    // dst_ptr_seen must be INSIDE the original buffer.
    assert!(
        dst_ptr_seen >= payload_ptr_before && dst_ptr_seen < payload_ptr_before + BUF,
        "producer received slice OUTSIDE the original buffer — Vec indirection detected"
    );

    // Reader: data() returns &[u8] of length 4 with the producer-written bytes.
    let data = writer.data();
    assert_eq!(data, b"abcd", "data round-trip");
}

// ============================================================
// SliceSource as producer (proves blanket impl wiring)
// ============================================================

#[test]
fn fill_from_data_works_with_slice_source() {
    let mut bytes = fresh_buf();
    // `build_writer` now requires `max_capacity` + `topic`. These
    // tests exercise the steady-state path (writes fit in BUF), so the
    // ceiling equals the buffer size and overflow never fires.
    let mut writer = Image::build_writer(
        &mut bytes,
        cerulion_core::wire::MaxPayloadCapacity::const_new(BUF as u32),
        std::sync::Arc::from("test"),
    );
    writer.set_header_bytes(&[]).expect("");
    writer.set_encoding("g").expect("");
    writer
        .fill_from_data(SliceSource::new(b"hello"))
        .expect("fill_from with SliceSource");

    assert_eq!(writer.data(), b"hello");
}

// ============================================================
// #1 risk area: short-write truncation + subsequent field placement
// ============================================================
//
// This is the bug a naive cursor-bookkeeping implementation would let
// through silently: `fill_from` reserves `remaining` bytes, producer
// writes 3, and the cursor stays at `cursor_before + remaining` instead
// of being rewound to `cursor_before + 3`. The next `set_<other>` then
// lands at the wrong offset, leaving a `remaining - 3`-byte gap of
// garbage in the variable payload.

#[test]
fn fill_from_short_write_truncates_offset_entry_length() {
    let mut bytes = fresh_buf();
    // `build_writer` now requires `max_capacity` + `topic`. These
    // tests exercise the steady-state path (writes fit in BUF), so the
    // ceiling equals the buffer size and overflow never fires.
    let mut writer = Image::build_writer(
        &mut bytes,
        cerulion_core::wire::MaxPayloadCapacity::const_new(BUF as u32),
        std::sync::Arc::from("test"),
    );
    writer.set_header_bytes(&[]).expect("");
    writer.set_encoding("g").expect("");

    // Producer writes 3 bytes; the loan would have offered ~thousands.
    writer
        .fill_from_data(|dst: &mut [u8]| {
            dst[..3].copy_from_slice(b"abc");
            Ok(3)
        })
        .expect("");

    // Offset entry for data (variable index 2 in Image — header, encoding, data)
    // must record length=3, not the full remaining. The reader's
    // `data()` accessor reads through the offset entry, so if the
    // length were the full remaining the slice would be much larger
    // than 3.
    assert_eq!(writer.data().len(), 3, "data() returns exactly 3 bytes");
    assert_eq!(writer.data(), b"abc");
}

#[test]
fn fill_from_then_subsequent_field_lands_at_correct_cursor() {
    // The canary for the cursor-bookkeeping bug. Order:
    // 1. fill_from_header_bytes (complex variable) → 4 bytes
    // 2. fill_from_encoding → 2 bytes
    // 3. set_data → 5 bytes
    // After step 3, the cursor must be at:
    //   cursor_before_header + 4 (header) + 2 (encoding) + 5 (data) = +11
    // NOT at:
    //   cursor_before_header + remaining + remaining + 5
    // (which would also "work" data-wise because each field overwrites
    // the offset entry, but would silently fail the LATER check that
    // payload_wire_size matches the wire-format invariant).

    let mut bytes = fresh_buf();
    // `build_writer` now requires `max_capacity` + `topic`. These
    // tests exercise the steady-state path (writes fit in BUF), so the
    // ceiling equals the buffer size and overflow never fires.
    let mut writer = Image::build_writer(
        &mut bytes,
        cerulion_core::wire::MaxPayloadCapacity::const_new(BUF as u32),
        std::sync::Arc::from("test"),
    );
    let initial = Image::payload_wire_size(&writer);

    writer
        .fill_from_header_bytes(|dst: &mut [u8]| {
            dst[..4].copy_from_slice(&[1, 2, 3, 4]);
            Ok(4)
        })
        .expect("");
    let after_header = Image::payload_wire_size(&writer);
    assert_eq!(
        after_header - initial,
        4,
        "header_bytes write must advance cursor by exactly 4 bytes"
    );

    writer
        .fill_from_encoding(|dst: &mut [u8]| {
            dst[..2].copy_from_slice(b"AB");
            Ok(2)
        })
        .expect("");
    let after_encoding = Image::payload_wire_size(&writer);

    writer.set_data(b"hello").expect("");
    let after_data = Image::payload_wire_size(&writer);

    // Each subsequent step must extend payload_wire_size by EXACTLY the
    // number of bytes the producer wrote. A cursor-bookkeeping bug would
    // make `after_encoding - after_header > 2` (the encoding field gets
    // pushed past the header's reserved-but-unused tail).
    assert_eq!(
        after_encoding - after_header,
        2,
        "encoding write must advance cursor by exactly 2 bytes (header's reserved-but-unused tail must NOT be in the wire frame)"
    );
    assert_eq!(
        after_data - after_encoding,
        5,
        "data write must advance cursor by exactly 5 bytes"
    );

    // Read back: all three fields are correct.
    assert_eq!(writer.header_bytes(), &[1, 2, 3, 4]);
    assert_eq!(writer.encoding().expect("utf8"), "AB");
    assert_eq!(writer.data(), b"hello");
}

// ============================================================
// Producer error: rewinds cursor + leaves mark_written unset
// ============================================================

#[test]
fn fill_from_producer_err_rewinds_cursor() {
    let mut bytes = fresh_buf();
    // `build_writer` now requires `max_capacity` + `topic`. These
    // tests exercise the steady-state path (writes fit in BUF), so the
    // ceiling equals the buffer size and overflow never fires.
    let mut writer = Image::build_writer(
        &mut bytes,
        cerulion_core::wire::MaxPayloadCapacity::const_new(BUF as u32),
        std::sync::Arc::from("test"),
    );
    writer.set_header_bytes(&[]).expect("");
    writer.set_encoding("g").expect("");

    let cursor_before_data = Image::payload_wire_size(&writer);

    let err = writer
        .fill_from_data(|_dst: &mut [u8]| -> Result<usize, TransportError> {
            Err(TransportError::NodeError {
                node_id: "camera".into(),
                reason: "device disconnected".into(),
            })
        })
        .expect_err("producer Err should propagate");
    assert!(matches!(err, TransportError::NodeError { .. }));

    // Cursor unchanged: payload_wire_size matches pre-call.
    // (The Err path performs NO state mutation;
    // payload_wire_size is identical to the pre-fill_from value.)
    let cursor_after = Image::payload_wire_size(&writer);
    assert_eq!(
        cursor_after, cursor_before_data,
        "writer state must be unchanged on producer Err"
    );

    // `all_variables_written` must be false (data field never marked).
    assert!(
        !Image::all_variables_written(&writer),
        "mark_written should NOT have been set on Err"
    );
}

#[test]
fn fill_from_err_then_successful_set_lands_at_pre_err_cursor() {
    // After Err, the writer state is bit-for-bit unchanged from
    // before the call. A subsequent
    // `set_data(b"recovered")` then writes at the original
    // cursor — same end state as if fill_from was never called.
    let mut bytes = fresh_buf();
    // `build_writer` now requires `max_capacity` + `topic`. These
    // tests exercise the steady-state path (writes fit in BUF), so the
    // ceiling equals the buffer size and overflow never fires.
    let mut writer = Image::build_writer(
        &mut bytes,
        cerulion_core::wire::MaxPayloadCapacity::const_new(BUF as u32),
        std::sync::Arc::from("test"),
    );
    writer.set_header_bytes(&[]).expect("");
    writer.set_encoding("g").expect("");

    let cursor_before = Image::payload_wire_size(&writer);

    let _ = writer.fill_from_data(|_dst: &mut [u8]| -> Result<usize, TransportError> {
        Err(TransportError::NodeError {
            node_id: "n".into(),
            reason: "r".into(),
        })
    });

    // Between failure and recovery: payload_wire_size MUST equal
    // cursor_before because the Err path performs NO state mutation
    // (defers everything to Ok arm). A buggy implementation that did
    // partial mutation + rewind would still expose drift on
    // payload_wire_size or on a subsequent set_data position.
    let cursor_after_err = Image::payload_wire_size(&writer);
    assert_eq!(
        cursor_after_err, cursor_before,
        "writer state must be untouched immediately after Err (no mutation, no rewind needed)"
    );

    writer.set_data(b"recovered").expect("recovery set");
    let cursor_after = Image::payload_wire_size(&writer);

    // After Err (no-mutation) + set_data(9 bytes), cursor advanced
    // by exactly 9 from the original.
    assert_eq!(
        cursor_after - cursor_before,
        9,
        "set_data after fill_from Err must land at the pre-fill cursor"
    );
    assert_eq!(writer.data(), b"recovered");
    assert!(Image::all_variables_written(&writer));
}

// ============================================================
// Producer panic: mark_written stays unset, publish gate holds
// ============================================================

#[test]
fn fill_from_producer_panic_leaves_state_untouched() {
    // Panic safety
    // is structural — state mutations are deferred to the
    // Ok arm, so panic propagation through the producer call leaves
    // the writer state BIT-FOR-BIT identical to pre-call. No
    // catch_unwind needed inside the emitted method; we use it here
    // in the TEST only to keep the process alive for assertions.
    let mut bytes = fresh_buf();
    // `build_writer` now requires `max_capacity` + `topic`. These
    // tests exercise the steady-state path (writes fit in BUF), so the
    // ceiling equals the buffer size and overflow never fires.
    let mut writer = Image::build_writer(
        &mut bytes,
        cerulion_core::wire::MaxPayloadCapacity::const_new(BUF as u32),
        std::sync::Arc::from("test"),
    );
    writer.set_header_bytes(&[]).expect("");
    writer.set_encoding("g").expect("");

    let cursor_before_data = Image::payload_wire_size(&writer);

    let result = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
        let _ = writer.fill_from_data(|_dst: &mut [u8]| -> Result<usize, TransportError> {
            panic!("camera driver panicked mid-fill");
        });
    }));
    assert!(result.is_err(), "panic should propagate from fill_from");

    // 1. mark_written for the data field MUST be false (publish gate
    //    refuses partial frames).
    assert!(
        !Image::all_variables_written(&writer),
        "panic during producer must NOT mark the field as written"
    );

    // 2. Cursor unchanged — state mutations are deferred to Ok arm,
    //    so panic leaves the cursor exactly where it was before.
    //    Same end state as Err.
    let cursor_after_panic = Image::payload_wire_size(&writer);
    assert_eq!(
        cursor_after_panic, cursor_before_data,
        "panic during fill_from must leave cursor untouched (no state mutation before producer call)"
    );
}

// ============================================================
// Critical regression: fill_from(Err) after a
// successful prior write must NOT corrupt the prior write's
// offset entry (would otherwise let Drop publish a frame with
// stale field_starts + offset pointing at garbage)
// ============================================================

#[test]
fn fill_from_err_after_successful_set_preserves_prior_data() {
    // The bug this pins: a previous
    // `set_data(b"good")` set field_starts[2]=cursor, offset_entry[2]
    // =(cursor, 4), mark_written[2]=true. A subsequent
    // `fill_from_data(Err)` would OVERWRITE field_starts[2] and
    // offset_entry[2] during the loan-reservation phase before the
    // producer call. mark_written[2] was already true and stayed
    // true (Err arm didn't clear it). OutputProxy::Drop sees
    // all_variables_written=true and publishes a frame whose
    // offset_entry[2] points at the failed loan's reservation
    // region instead of the original "good" bytes — silent data
    // corruption.
    //
    // Contract: fill_from defers ALL state mutations to the Ok arm.
    // Err leaves writer state bit-for-bit unchanged from before
    // the call.
    let mut bytes = fresh_buf();
    // `build_writer` now requires `max_capacity` + `topic`. These
    // tests exercise the steady-state path (writes fit in BUF), so the
    // ceiling equals the buffer size and overflow never fires.
    let mut writer = Image::build_writer(
        &mut bytes,
        cerulion_core::wire::MaxPayloadCapacity::const_new(BUF as u32),
        std::sync::Arc::from("test"),
    );
    writer.set_header_bytes(&[]).expect("");
    writer.set_encoding("g").expect("");

    // First successful write — sets the load-bearing prior state.
    writer.set_data(b"good_data").expect("first write");
    let good_size = Image::payload_wire_size(&writer);
    assert_eq!(writer.data(), b"good_data");
    assert!(Image::all_variables_written(&writer));

    // Failed fill_from on the same field — must NOT corrupt
    // anything.
    let err = writer
        .fill_from_data(|_buf: &mut [u8]| -> Result<usize, TransportError> {
            Err(TransportError::NodeError {
                node_id: "camera".into(),
                reason: "device disconnected".into(),
            })
        })
        .expect_err("producer Err must propagate");
    assert!(matches!(err, TransportError::NodeError { .. }));

    // An eager-mutation bug would make writer.data() return garbage
    // bytes from the loan region (or an arbitrary slice that
    // overlaps subsequent writes).
    assert_eq!(
        writer.data(),
        b"good_data",
        "fill_from Err MUST preserve the prior successful write's offset entry"
    );
    assert_eq!(
        Image::payload_wire_size(&writer),
        good_size,
        "fill_from Err MUST leave cursor unchanged from before the call"
    );
    assert!(
        Image::all_variables_written(&writer),
        "publish gate must still be open — the prior good write's mark_written bit survives"
    );
}

#[test]
fn fill_from_panic_after_successful_set_preserves_prior_data() {
    // Same scenario as the Err test but with panic. Panic propagates
    // before the Ok arm runs, so state is also untouched. No
    // catch_unwind inside the emitted method.
    let mut bytes = fresh_buf();
    // `build_writer` now requires `max_capacity` + `topic`. These
    // tests exercise the steady-state path (writes fit in BUF), so the
    // ceiling equals the buffer size and overflow never fires.
    let mut writer = Image::build_writer(
        &mut bytes,
        cerulion_core::wire::MaxPayloadCapacity::const_new(BUF as u32),
        std::sync::Arc::from("test"),
    );
    writer.set_header_bytes(&[]).expect("");
    writer.set_encoding("g").expect("");
    writer.set_data(b"good_data").expect("first write");
    let good_size = Image::payload_wire_size(&writer);

    let result = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
        let _ = writer.fill_from_data(|_buf: &mut [u8]| -> Result<usize, TransportError> {
            panic!("camera driver panicked");
        });
    }));
    assert!(result.is_err());

    assert_eq!(
        writer.data(),
        b"good_data",
        "panic during fill_from MUST preserve the prior successful write"
    );
    assert_eq!(
        Image::payload_wire_size(&writer),
        good_size,
        "panic during fill_from MUST leave cursor unchanged"
    );
}

// ============================================================
// Adversarial producer: returns written > dst.len() → clamped
// ============================================================

// The codegen emits `tracing::warn!` when it clamps
// a lying producer's reported `written > n_elements`. This test pins
// that the warn fires with the expected structured fields. Uses
// `tracing-test`'s `no-env-filter` capture (the warn fires from
// codegen-emitted code inside `native_ros2_messages::Image`, not the
// test crate, so the default EnvFilter would silently drop it).
#[tracing_test::traced_test]
#[test]
fn fill_from_lying_producer_emits_tracing_warn() {
    let mut bytes = fresh_buf();
    // `build_writer` now requires `max_capacity` + `topic`. These
    // tests exercise the steady-state path (writes fit in BUF), so the
    // ceiling equals the buffer size and overflow never fires.
    let mut writer = Image::build_writer(
        &mut bytes,
        cerulion_core::wire::MaxPayloadCapacity::const_new(BUF as u32),
        std::sync::Arc::from("test"),
    );
    writer.set_header_bytes(&[]).expect("");
    writer.set_encoding("g").expect("");

    struct LyingSource;
    impl cerulion_core::transport::fill_from::FillFrom for LyingSource {
        fn fill_from(&mut self, dst: &mut [u8]) -> Result<usize, TransportError> {
            dst[..2].copy_from_slice(b"xy");
            Ok(usize::MAX) // BUG: claims to write usize::MAX into a much smaller dst
        }
    }

    writer.fill_from_data(LyingSource).expect("clamp succeeds");

    // The codegen MUST emit a tracing::warn! when the clamp triggers.
    // Match the structured fields that pin the diagnostic — semantic
    // anchor, not exact format.
    assert!(
        logs_contain("FillFrom producer returned written > dst.len()"),
        "tracing::warn! must fire when the clamp triggers"
    );
    assert!(
        logs_contain("field=\"data\""),
        "tracing::warn! must include the field name as a structured field"
    );
    assert!(
        logs_contain("loan_capacity="),
        "tracing::warn! must include the loan_capacity structured field"
    );
}

#[tracing_test::traced_test]
#[test]
fn fill_from_honest_producer_does_not_emit_warn() {
    // Mutation-twin of the warn test: a codegen that emitted the warn
    // unconditionally (on every fill_from call) would still pass a
    // happy-path round-trip — so this test MUST use traced_test +
    // NEGATIVE logs_contain assertion to actually catch that mutation.
    //
    // Without traced_test and the
    // negative assertion this test cannot see that mutation. With
    // the assertion below, a buggy codegen that emitted the warn on
    // every clamp-branch-or-not would FAIL this test.
    let mut bytes = fresh_buf();
    // `build_writer` now requires `max_capacity` + `topic`. These
    // tests exercise the steady-state path (writes fit in BUF), so the
    // ceiling equals the buffer size and overflow never fires.
    let mut writer = Image::build_writer(
        &mut bytes,
        cerulion_core::wire::MaxPayloadCapacity::const_new(BUF as u32),
        std::sync::Arc::from("test"),
    );
    writer.set_header_bytes(&[]).expect("");
    writer.set_encoding("g").expect("");
    writer
        .fill_from_data(|buf: &mut [u8]| {
            buf[..3].copy_from_slice(b"abc");
            Ok(3)
        })
        .expect("in-range producer succeeds");
    assert_eq!(writer.data(), b"abc");

    // The load-bearing assertion: warn must NOT fire when the
    // producer is in range (`written <= dst.len()`). Catches a codegen
    // mutation that emits the warn unconditionally.
    assert!(
        !logs_contain("FillFrom producer returned written > dst.len()"),
        "tracing::warn! must NOT fire when the producer's written count is in range (the clamp branch should be taken only when written > n_elements)"
    );
}

#[test]
fn fill_from_lying_producer_is_clamped_to_remaining() {
    let mut bytes = fresh_buf();
    // `build_writer` now requires `max_capacity` + `topic`. These
    // tests exercise the steady-state path (writes fit in BUF), so the
    // ceiling equals the buffer size and overflow never fires.
    let mut writer = Image::build_writer(
        &mut bytes,
        cerulion_core::wire::MaxPayloadCapacity::const_new(BUF as u32),
        std::sync::Arc::from("test"),
    );
    writer.set_header_bytes(&[]).expect("");
    writer.set_encoding("g").expect("");

    // The closure blanket's debug_assert would fire before the codegen
    // clamp; use a direct `FillFrom` impl to exercise the codegen-side
    // clamp instead.
    struct LyingSource;
    impl cerulion_core::transport::fill_from::FillFrom for LyingSource {
        fn fill_from(&mut self, dst: &mut [u8]) -> Result<usize, TransportError> {
            // Write 2 bytes, claim usize::MAX.
            dst[..2].copy_from_slice(b"xy");
            Ok(usize::MAX)
        }
    }

    writer.fill_from_data(LyingSource).expect("clamp succeeds");

    // Clamped to remaining = capacity − cursor. The offset entry length
    // == remaining, not usize::MAX. Reader sees the full remaining as
    // "data" (most of it zeros from the buffer init).
    let data = writer.data();
    assert!(
        data.len() <= BUF,
        "clamped length must fit in the buffer; got {} bytes",
        data.len()
    );
    assert_eq!(&data[..2], b"xy", "the bytes the producer wrote round-trip");
    assert!(Image::all_variables_written(&writer));
}

// ============================================================
// Zero-byte producer (Ok(0) → empty field, mark_written set)
// ============================================================

#[test]
fn fill_from_zero_byte_producer_marks_written_with_empty() {
    let mut bytes = fresh_buf();
    // `build_writer` now requires `max_capacity` + `topic`. These
    // tests exercise the steady-state path (writes fit in BUF), so the
    // ceiling equals the buffer size and overflow never fires.
    let mut writer = Image::build_writer(
        &mut bytes,
        cerulion_core::wire::MaxPayloadCapacity::const_new(BUF as u32),
        std::sync::Arc::from("test"),
    );
    writer.set_header_bytes(&[]).expect("");
    writer.set_encoding("g").expect("");

    writer
        .fill_from_data(|_dst: &mut [u8]| Ok(0))
        .expect("zero-byte producer ok");

    assert_eq!(writer.data().len(), 0, "empty data");
    // mark_written must fire even for zero-byte writes (matches
    // set_<f>(&[]) behavior).
    assert!(Image::all_variables_written(&writer));
}

// ============================================================
// Typed array: fill_from_<f><S: FillFrom<f32>> on DynArray<f32>
// ============================================================
//
// Imu has angular_velocity_covariance / linear_acceleration_covariance
// as fixed arrays, but Imu's variable side is just header_bytes. We need
// a schema with DynArray<primitive non-u8> for this test. Looking at
// native_ros2_messages: sensor_msgs/Joy has float32[] axes and int32[]
// buttons.

#[test]
fn fill_from_typed_f32_array_writes_typed_elements() {
    use native_ros2_messages::sensor_msgs::Joy;

    let mut bytes = vec![0u8; BUF];
    let mut writer = Joy::build_writer(
        &mut bytes,
        cerulion_core::wire::MaxPayloadCapacity::const_new(BUF as u32),
        std::sync::Arc::from("test"),
    );

    writer.set_header_bytes(&[]).expect("");

    // Producer writes 3 f32 values into the axes field.
    writer
        .fill_from_axes(|dst: &mut [f32]| {
            dst[0] = 1.5;
            dst[1] = 2.5;
            dst[2] = 3.5;
            Ok(3)
        })
        .expect("fill_from_axes");

    writer.set_buttons(&[10i32, 20, 30]).expect("set buttons");

    assert_eq!(writer.axes(), &[1.5f32, 2.5, 3.5]);
    assert_eq!(writer.buttons(), &[10i32, 20, 30]);
}

// ============================================================
// Buffer exhaustion before fill_from (regression test)
// ============================================================
//
// When prior variable writes leave
// `state.cursor` near `self.len`, the typed-array alignment bump can
// push `cursor_aligned` past `self.len`, and the raw slice index
// `&mut payload[cursor_aligned..cursor_aligned + 0]` panics with OOB
// (Rust requires the start index <= len even for empty ranges).
//
// The guard: bounds-check `cursor_aligned > self.len` before any slice
// indexing or state mutation, and return
// `TransportError::ProxyBufferTooSmall` — matching `loan_<f>`'s
// contract. The test below pins the contract by reproducing that
// case.

#[test]
fn fill_from_typed_spills_when_alignment_bump_overflows_buffer() {
    use native_ros2_messages::sensor_msgs::Joy;

    // Overflow-redirect update: when cursor_aligned > self.len, fill_from triggers
    // a spill to a heap fallback (if max_capacity allows). With BUF as
    // the ceiling, the spill succeeds and the producer is called with
    // dst sized to the remaining post-spill capacity.
    //
    // BEFORE the overflow redirect: this case returned ProxyBufferTooSmall.
    // AFTER it: this case spills + Ok.
    //
    // The PayloadTooLarge boundary case (cursor_aligned > max_capacity)
    // is covered by separate tests.
    let mut bytes = vec![0u8; 199];
    let mut writer = Joy::build_writer(
        &mut bytes,
        cerulion_core::wire::MaxPayloadCapacity::const_new(BUF as u32),
        std::sync::Arc::from("test"),
    );

    let cursor_now = Joy::payload_wire_size(&writer);
    let pad_size = 197usize.saturating_sub(cursor_now);
    writer
        .set_header_bytes(&vec![0u8; pad_size])
        .expect("filler header");

    // cursor is ~197; align_up(197, 4) = 200; 200 > 199 = self.len →
    // ensure_capacity_for(1, 200) triggers spill (200 + 1 < BUF=8192).
    // Producer is called with the post-spill remaining bytes; Ok(0)
    // commits an empty buttons array.
    writer
        .fill_from_buttons(|_dst: &mut [i32]| Ok(0))
        .expect("fill_from must spill (not Err) when within max_capacity");
    // After spill, the writer is in spilled state.
    assert!(
        writer.has_overflow(),
        "fill_from past self.len within max_capacity must spill"
    );
}

#[test]
fn fill_from_typed_returns_payload_too_large_when_alignment_bump_past_max_capacity() {
    use native_ros2_messages::sensor_msgs::Joy;

    // Tight max_capacity: 199 (= self.len). cursor_aligned = 200 > 199 →
    // ensure_capacity_for(1, 200) → required_end = 201 > max_cap = 199 →
    // PayloadTooLarge.
    let mut bytes = vec![0u8; 199];
    let mut writer = Joy::build_writer(
        &mut bytes,
        cerulion_core::wire::MaxPayloadCapacity::const_new(199u32),
        std::sync::Arc::from("test"),
    );

    let cursor_now = Joy::payload_wire_size(&writer);
    let pad_size = 197usize.saturating_sub(cursor_now);
    writer
        .set_header_bytes(&vec![0u8; pad_size])
        .expect("filler header");

    let err = writer
        .fill_from_buttons(|_dst: &mut [i32]| Ok(0))
        .expect_err("must fail when post-alignment cursor exceeds max_capacity");
    assert!(
        matches!(err, TransportError::PayloadTooLarge { .. }),
        "expected PayloadTooLarge, got: {err:?}"
    );

    // mark_written for buttons MUST be false (state was not mutated).
    assert!(
        !Joy::all_variables_written(&writer),
        "PayloadTooLarge path must NOT mark the field as written"
    );
    // Writer must NOT be in spilled state — ensure_capacity_for
    // returned Err BEFORE allocating the spill buffer.
    assert!(
        !writer.has_overflow(),
        "PayloadTooLarge must not leave the writer in spilled state"
    );
}

#[test]
fn fill_from_typed_at_exact_capacity_boundary_succeeds_with_zero_elements() {
    // Boundary-value pair for the OOB regression test above: when
    // `cursor_aligned == self.len` exactly, the bounds-check (strict
    // `>`) must let the call through. Empty slice is loaned; producer
    // returns Ok(0); the field is marked written.
    //
    // Pins the `>` vs `>=` mutation on the bounds-check: a buggy
    // `if cursor_aligned >= self.len` would erroneously return Err
    // here, but should succeed.
    use native_ros2_messages::sensor_msgs::Joy;

    let mut bytes = vec![0u8; 200];
    let mut writer = Joy::build_writer(
        &mut bytes,
        cerulion_core::wire::MaxPayloadCapacity::const_new(BUF as u32),
        std::sync::Arc::from("test"),
    );

    // Fill header_bytes so cursor lands at exactly 200 (= self.len) when
    // align_up(cursor, 4) is taken. We want cursor at 200 (already aligned)
    // so cursor_aligned = 200 = self.len.
    let cursor_now = Joy::payload_wire_size(&writer);
    let pad_size = 200usize.saturating_sub(cursor_now);
    writer
        .set_header_bytes(&vec![0u8; pad_size])
        .expect("filler header");

    writer.set_axes(&[]).expect("");

    // Now cursor is at 200 = self.len. align_up(200, 4) = 200. Equal,
    // not greater, so bounds-check must NOT fire.
    writer
        .fill_from_buttons(|_dst: &mut [i32]| Ok(0))
        .expect("fill_from at exact capacity must succeed with empty loan");

    // All 3 fields marked written; readable as empty.
    assert!(Joy::all_variables_written(&writer));
    assert_eq!(writer.buttons().len(), 0);
}

#[test]
fn fill_from_typed_i32_array_writes_typed_elements() {
    use native_ros2_messages::sensor_msgs::Joy;

    let mut bytes = vec![0u8; BUF];
    let mut writer = Joy::build_writer(
        &mut bytes,
        cerulion_core::wire::MaxPayloadCapacity::const_new(BUF as u32),
        std::sync::Arc::from("test"),
    );

    writer.set_header_bytes(&[]).expect("");
    writer.set_axes(&[0.0f32]).expect("");

    // i32 path: alignment honored at loan time; producer writes typed
    // ints into the typed slice.
    writer
        .fill_from_buttons(|dst: &mut [i32]| {
            dst[0] = -7;
            dst[1] = 42;
            Ok(2)
        })
        .expect("fill_from_buttons");

    assert_eq!(writer.buttons(), &[-7i32, 42]);
}

// ============================================================
// Complex variable: fill_from_<f>_bytes
// ============================================================

#[test]
fn fill_from_header_bytes_for_complex_variable_field() {
    // Image's header is a Nested type — emits header_bytes / set_header_bytes
    // / fill_from_header_bytes.
    let mut bytes = fresh_buf();
    // `build_writer` now requires `max_capacity` + `topic`. These
    // tests exercise the steady-state path (writes fit in BUF), so the
    // ceiling equals the buffer size and overflow never fires.
    let mut writer = Image::build_writer(
        &mut bytes,
        cerulion_core::wire::MaxPayloadCapacity::const_new(BUF as u32),
        std::sync::Arc::from("test"),
    );

    let cursor_before = Image::payload_wire_size(&writer);
    writer
        .fill_from_header_bytes(|dst: &mut [u8]| {
            dst[..6].copy_from_slice(b"raw_h!");
            Ok(6)
        })
        .expect("");
    let cursor_after = Image::payload_wire_size(&writer);

    // Verify the _bytes path advances cursor by exactly the producer's
    // written count (parity with non-_bytes path tested in test #4).
    assert_eq!(
        cursor_after - cursor_before,
        6,
        "fill_from_<f>_bytes advances cursor by exactly the producer's written count"
    );

    writer.set_encoding("g").expect("");
    writer.set_data(&[]).expect("");

    assert_eq!(writer.header_bytes(), b"raw_h!");
    assert!(Image::all_variables_written(&writer));
}

// ============================================================
// Field-not-at-tail safety: rewriting a non-tail field doesn't
// disturb other fields' offset entries
// ============================================================
//
// Pins the "orphaned-bytes semantics" the
// module doc describes. The contract
// being pinned: if you've written field A then field B, and then
// you re-write field A (now non-tail), the new bytes land past B's
// region (orphaning A's previous bytes), but B's offset entry is
// UNDISTURBED — readers of B continue to see the original bytes.

#[test]
fn rewriting_a_non_tail_variable_field_does_not_disturb_other_fields() {
    let mut bytes = fresh_buf();
    // `build_writer` now requires `max_capacity` + `topic`. These
    // tests exercise the steady-state path (writes fit in BUF), so the
    // ceiling equals the buffer size and overflow never fires.
    let mut writer = Image::build_writer(
        &mut bytes,
        cerulion_core::wire::MaxPayloadCapacity::const_new(BUF as u32),
        std::sync::Arc::from("test"),
    );
    writer.set_header_bytes(&[]).expect("");

    // Step 1: write encoding ("AB"), then data ("xyz"). Encoding is now non-tail.
    writer
        .fill_from_encoding(|dst: &mut [u8]| {
            dst[..2].copy_from_slice(b"AB");
            Ok(2)
        })
        .expect("");
    writer
        .fill_from_data(|dst: &mut [u8]| {
            dst[..3].copy_from_slice(b"xyz");
            Ok(3)
        })
        .expect("");

    // Read-back checkpoint: both fields visible as written.
    assert_eq!(writer.encoding().expect(""), "AB");
    assert_eq!(writer.data(), b"xyz");

    let cursor_before_rewrite = Image::payload_wire_size(&writer);

    // Step 2: re-write encoding to "C" (non-tail re-write). The new
    // bytes must land past `data`'s region, advancing the cursor.
    writer
        .fill_from_encoding(|dst: &mut [u8]| {
            dst[..1].copy_from_slice(b"C");
            Ok(1)
        })
        .expect("");

    let cursor_after_rewrite = Image::payload_wire_size(&writer);
    assert!(
        cursor_after_rewrite > cursor_before_rewrite,
        "non-tail rewrite must advance cursor (orphaned-bytes semantics)"
    );

    // Step 3: critical assertion — data() still returns "xyz". The
    // non-tail rewrite of encoding must NOT have stomped data's
    // offset entry.
    assert_eq!(
        writer.encoding().expect(""),
        "C",
        "rewritten encoding visible"
    );
    assert_eq!(
        writer.data(),
        b"xyz",
        "data offset entry MUST survive non-tail rewrite of encoding"
    );
}

// ============================================================
// Determinism: same producer input across runs produces byte-identical
// SHM frames (Principle #7 — Replay = Live)
// ============================================================

#[test]
fn fill_from_is_deterministic_across_runs() {
    fn build_one() -> Vec<u8> {
        let mut bytes = fresh_buf();
        // `build_writer` now requires `max_capacity` + `topic`. These
        // tests exercise the steady-state path (writes fit in BUF), so the
        // ceiling equals the buffer size and overflow never fires.
        let mut writer = Image::build_writer(
            &mut bytes,
            cerulion_core::wire::MaxPayloadCapacity::const_new(BUF as u32),
            std::sync::Arc::from("test"),
        );
        writer.height = 1;
        writer.width = 2;
        writer.step = 3;
        writer.is_bigendian = 0;
        writer
            .fill_from_header_bytes(SliceSource::new(b"hdr"))
            .expect("");
        writer
            .fill_from_encoding(|dst: &mut [u8]| {
                dst[..3].copy_from_slice(b"rgb");
                Ok(3)
            })
            .expect("");
        writer
            .fill_from_data(SliceSource::new(&[10u8, 20, 30, 40, 50]))
            .expect("");
        // Drop writer; bytes contains the SHM frame.
        bytes
    }

    let a = build_one();
    let b = build_one();
    assert_eq!(
        a, b,
        "two identical fill_from sequences must produce byte-identical SHM payloads"
    );
}
