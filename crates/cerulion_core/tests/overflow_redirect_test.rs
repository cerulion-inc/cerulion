// SPDX-License-Identifier: AGPL-3.0-only
//! In-tick overflow redirect for variable-field setters.
//!
//! Pure codegen-level tests of the spill helper (`ensure_capacity_for`)
//! and the writer's overflow accessors (`has_overflow`,
//! `overflow_view_bytes`, `take_overflow`). The publisher / Drop
//! integration is tested in `overflow_redirect_e2e_test.rs` (serial).
//!
//! # What's tested
//!
//! - **Steady-state path** (`cursor + bytes_needed <= self.len`):
//!   `ensure_capacity_for` is a no-op; `has_overflow == false`.
//! - **Spill happy path**: setter that would overflow the loan succeeds
//!   via heap fallback; `has_overflow == true` post-spill;
//!   `overflow_view_bytes` returns Some.
//! - **`PayloadTooLarge` boundary**: setter that exceeds `max_capacity`
//!   returns `PayloadTooLarge`; no spill happened.
//! - **Bytes preserved across spill**: fixed-section + offset table +
//!   any prior writes survive the spill copy intact.
//! - **Subsequent writes go to heap**: after spill, additional setter
//!   calls write into the heap buffer (not the loan slice).
//! - **`take_overflow` returns the Box<[u64]>**: explicit ownership
//!   transfer for `OutputProxy::Drop`'s re-loan path.
//! - **8-byte alignment of spill**: the heap `Vec<u64>` reinterpret is
//!   8-byte aligned for sound multi-byte typed loans (e.g. `f64`).

use cerulion_core::message::ShmMessage;
use cerulion_core::TransportError;
use native_ros2_messages::sensor_msgs::Image;
use native_ros2_messages::std_msgs::Float64MultiArray;

/// Helper: 8-aligned buffer of `n` bytes for direct `from_bytes_mut`.
fn aligned_buf(n_bytes: usize) -> Vec<u64> {
    let n_u64 = n_bytes.div_ceil(8);
    vec![0u64; n_u64]
}

/// View an aligned u64 buffer as `&mut [u8]` of exactly `n_bytes` bytes.
fn aligned_view(buf: &mut [u64], n_bytes: usize) -> &mut [u8] {
    let ptr = buf.as_mut_ptr() as *mut u8;
    let total_bytes = buf.len() * 8;
    assert!(n_bytes <= total_bytes);
    // SAFETY: u64 alignment >= u8; the slice covers `n_bytes` valid
    // zero-initialised bytes within the Vec<u64>.
    unsafe { std::slice::from_raw_parts_mut(ptr, n_bytes) }
}

#[test]
fn steady_state_no_spill_when_loan_fits() {
    let mut buf = aligned_buf(8192);
    let bytes = aligned_view(&mut buf, 8192);
    let mut writer = Image::build_writer(
        bytes,
        cerulion_core::wire::MaxPayloadCapacity::const_new(65536),
        std::sync::Arc::from("test/no_spill"),
    );

    // Required: write all three variable fields. Total writes < 8192 bytes.
    writer.set_header_bytes(b"hdr").expect("header set");
    writer.set_encoding("rgb8").expect("encoding set");
    writer.set_data(&[1u8; 256]).expect("data set");

    assert!(
        !writer.has_overflow(),
        "steady-state path must NOT spill — loan was 8 KiB, writes were ~270 bytes"
    );
    assert!(
        writer.overflow_view_bytes().is_none(),
        "overflow_view_bytes must be None when no spill occurred"
    );
}

#[test]
fn spill_fires_when_setter_would_overflow_loan() {
    // Small loan (1 KiB), large max_capacity (1 MiB), payload spike to 64 KiB.
    let mut buf = aligned_buf(1024);
    let bytes = aligned_view(&mut buf, 1024);
    let mut writer = Image::build_writer(
        bytes,
        cerulion_core::wire::MaxPayloadCapacity::const_new(1024 * 1024),
        std::sync::Arc::from("test/spill"),
    );

    writer.set_header_bytes(b"hdr").expect("header set");
    writer.set_encoding("rgb8").expect("encoding set");
    // 64 KiB exceeds the 1 KiB loan but fits in the 1 MiB ceiling.
    let big = vec![0xabu8; 64 * 1024];
    writer
        .set_data(&big)
        .expect("data set must succeed via spill");

    assert!(
        writer.has_overflow(),
        "spill must fire when a setter would overflow the loan but fits in max_capacity"
    );
    let view = writer.overflow_view_bytes().expect("overflow view present");
    // View covers fixed section + offset table + variable payload up to cursor.
    // The cursor is past header (3 bytes) + encoding (4 bytes) + data (64 KiB)
    // plus the fixed section + offset table.
    assert!(view.len() > 64 * 1024, "view should include the spike data");
}

#[test]
fn payload_too_large_returns_err_no_spill() {
    // Tight max_capacity: 4 KiB. Loan: 1 KiB. Spike: 16 KiB > max_capacity.
    let mut buf = aligned_buf(1024);
    let bytes = aligned_view(&mut buf, 1024);
    let mut writer = Image::build_writer(
        bytes,
        cerulion_core::wire::MaxPayloadCapacity::const_new(4096),
        std::sync::Arc::from("test/payload_too_large"),
    );

    writer.set_header_bytes(b"hdr").expect("header set");
    writer.set_encoding("rgb8").expect("encoding set");
    // 16 KiB > 4 KiB max_capacity → PayloadTooLarge, no spill.
    let huge = vec![0xcdu8; 16 * 1024];
    let err = writer
        .set_data(&huge)
        .expect_err("set_data with payload > max_capacity must fail");
    match err {
        TransportError::PayloadTooLarge {
            topic,
            requested,
            max,
        } => {
            assert_eq!(topic, "test/payload_too_large");
            assert!(
                requested >= 16 * 1024,
                "requested must include the 16 KiB write (got {})",
                requested
            );
            assert_eq!(max, 4096);
        }
        other => panic!("expected PayloadTooLarge, got {:?}", other),
    }
    assert!(
        !writer.has_overflow(),
        "PayloadTooLarge must NOT leave the writer in spilled state"
    );
}

#[test]
fn bytes_preserved_across_spill() {
    // Loan = 256 bytes (tight), ceiling = 64 KiB, writes that spill mid-way.
    let mut buf = aligned_buf(256);
    let bytes = aligned_view(&mut buf, 256);
    let mut writer = Image::build_writer(
        bytes,
        cerulion_core::wire::MaxPayloadCapacity::const_new(64 * 1024),
        std::sync::Arc::from("test/preserved"),
    );

    // Image FixedSection: height/width/step (u32×3 = 12 bytes) + offset table
    // (8 × 3 = 24 bytes). Cursor starts at 12 + 24 = 36.
    // First two var fields are tiny; third spills.
    writer.set_header_bytes(b"hello").expect("");
    writer.set_encoding("xyz").expect("");
    // 4 KiB write — definitely spills past the 256-byte loan.
    let big = vec![0x42u8; 4096];
    writer.set_data(&big).expect("data spill");

    assert!(writer.has_overflow());
    let view = writer.overflow_view_bytes().expect("");
    // "hello" was the first variable write. It lands at cursor_start =
    // WIRE_FIXED_SIZE + OFFSET_TABLE_BYTES (= 8 * 3 = 24).
    let cursor_start = <Image as ShmMessage>::WIRE_FIXED_SIZE + 8 * 3;
    assert_eq!(
        &view[cursor_start..cursor_start + 5],
        b"hello",
        "first variable write must survive the spill memcpy at WIRE_FIXED_SIZE+OFFSET_TABLE_BYTES"
    );
    // "xyz" (3 bytes) lands right after "hello" at cursor_start + 5.
    assert_eq!(
        &view[cursor_start + 5..cursor_start + 8],
        b"xyz",
        "second variable write must survive the spill memcpy"
    );

    // Verify the offset table
    // bytes also survive the spill memcpy. The offset table is at
    // [WIRE_FIXED_SIZE..WIRE_FIXED_SIZE + 8*N] with 8 bytes per variable
    // field encoding (cursor_at_write_time: u32, byte_length: u32). If
    // the spill failed to copy these bytes, subscribers would mis-decode
    // every variable field — exactly the kind of corruption a payload-
    // only test would miss.
    let table_off = <Image as ShmMessage>::WIRE_FIXED_SIZE;
    // Field 0 (header): wrote 5 bytes ("hello") starting at cursor_start
    let (off0, len0) = cerulion_core::shm_runtime::read_offset_entry(view, table_off, 0);
    assert_eq!(off0 as usize, cursor_start, "header offset preserved");
    assert_eq!(len0, 5, "header length preserved");
    // Field 1 (encoding): wrote 3 bytes ("xyz") starting at cursor_start + 5
    let (off1, len1) = cerulion_core::shm_runtime::read_offset_entry(view, table_off, 1);
    assert_eq!(off1 as usize, cursor_start + 5, "encoding offset preserved");
    assert_eq!(len1, 3, "encoding length preserved");
    // Field 2 (data): wrote 4096 bytes (the spike) at cursor_start + 8
    let (off2, len2) = cerulion_core::shm_runtime::read_offset_entry(view, table_off, 2);
    assert_eq!(off2 as usize, cursor_start + 8, "data offset preserved");
    assert_eq!(len2, 4096, "data length preserved");
}

#[test]
fn take_overflow_returns_vec_u64() {
    let mut buf = aligned_buf(1024);
    let bytes = aligned_view(&mut buf, 1024);
    let mut writer = Image::build_writer(
        bytes,
        cerulion_core::wire::MaxPayloadCapacity::const_new(16 * 1024),
        std::sync::Arc::from("test/take"),
    );

    writer.set_header_bytes(b"hdr").expect("");
    writer.set_encoding("rgb8").expect("");
    writer.set_data(&vec![0u8; 8192]).expect("data spill");

    assert!(writer.has_overflow());
    let taken = writer.take_overflow().expect("take_overflow returns Some");
    // 16 KiB ceiling rounded up to u64 multiple = 16384 / 8 = 2048 u64s.
    assert_eq!(
        taken.len(),
        16 * 1024 / 8,
        "Box<[u64]> length matches u64-rounded max_capacity"
    );
    // After take, the field is None.
    assert!(writer.take_overflow().is_none());
}

#[test]
fn spill_buffer_is_8_byte_aligned_for_typed_loans() {
    // Float64MultiArray has a `data: float64[]` variable field. Spill, then
    // verify the spill buffer's underlying ptr is 8-byte aligned (sound for
    // `&mut [f64]` reinterpret).
    let mut buf = aligned_buf(1024);
    let bytes = aligned_view(&mut buf, 1024);
    let mut writer = Float64MultiArray::build_writer(
        bytes,
        cerulion_core::wire::MaxPayloadCapacity::const_new(64 * 1024),
        std::sync::Arc::from("test/aligned"),
    );

    // Float64MultiArray's variable fields: `layout.dim` (complex) and `data`.
    // We need to write any required var fields to make all_variables_written
    // pass — but for THIS test we only care that the spill buffer is aligned.
    // Trigger a spill via the typed `data` setter.
    let big_data: Vec<f64> = (0..2000).map(|i| i as f64).collect(); // 16 KiB
    writer.set_data(&big_data).expect("data spill");

    assert!(writer.has_overflow());
    let taken = writer.take_overflow().expect("");
    let ptr_addr = taken.as_ptr() as usize;
    assert_eq!(
        ptr_addr % 8,
        0,
        "spill buffer must be 8-byte aligned (Vec<u64> guarantee)"
    );
}

#[test]
fn reader_side_misuse_setter_call_attributes_to_read_only_topic() {
    // Pin that a `&mut self` setter call on a reader-constructed
    // `<Name>Shm` produces an error attributed to `topic: "<read-only>"`
    // rather than `topic: ""`. The reader-side defaults
    // (`max_capacity: 0, topic: "<read-only>"`) are the
    // sentinels: max_capacity=0 makes any `required_end > 0`
    // immediately PayloadTooLarge (no allocation attempt), and topic=
    // "<read-only>" identifies the misuse source to the operator.
    //
    // This test exercises the path because `Reader<'a> == Writer<'a>`
    // by GAT identity for variable schemas — the value-form Shm returned
    // by `build_reader` exposes the same `&mut self` setter methods as
    // the writer-side. The misuse is unsound (writes through a
    // pointer that originally aliased immutable bytes) but the error
    // message must attribute it correctly for debuggability.
    // Buf sized only for fixed section + offset table (no room for any
    // variable payload). `set_data` with even 1 byte must overflow and
    // route through `ensure_capacity_for` which checks max_capacity.
    // 8-aligned (via aligned_buf helper) so from_bytes' alignment
    // assert passes.
    let mut buf = aligned_buf(40);
    let bytes = aligned_view(&mut buf, 40);
    // build_reader returns a value-form `<Image as ShmMessage>::Reader<'_>`
    // which equals `<Image as ShmMessage>::Writer<'_>` for variable
    // schemas. The setter is callable via &mut on the value-form binding.
    let mut reader = Image::build_reader(bytes);
    // set_data triggers `ensure_capacity_for(N, cursor)` which sees
    // `required_end > self.len (40)` AND `required_end > max_cap (0)` →
    // returns PayloadTooLarge with topic="<read-only>" attribution.
    // With a reader-side max_cap of u32::MAX this would fall through to
    // spill_to_overflow attempting a 4 GiB allocation (DoS surface).
    let result = reader.set_data(&[1u8; 16]);
    match result {
        Err(TransportError::PayloadTooLarge { topic, max, .. }) => {
            assert_eq!(
                topic, "<read-only>",
                "misuse via reader-side Shm must be attributed to <read-only>, \
                 not the empty string (operator must see the root cause)"
            );
            assert_eq!(
                max, 0,
                "reader-side max_capacity=0 forces immediate PayloadTooLarge \
                 without an allocation attempt (the DoS guard)"
            );
        }
        other => panic!(
            "expected PayloadTooLarge with topic=<read-only>, got: {:?}",
            other
        ),
    }
}

#[test]
fn alignment_bump_past_self_len_does_not_oob_read_during_spill() {
    // Regression test:
    // a `loan_<f>` typed setter that advanced `self.state.cursor =
    // cursor_aligned` BEFORE calling `ensure_capacity_for` would, when the
    // alignment bump pushed cursor_aligned past `self.len` and the
    // spill triggered, let `spill_to_overflow`'s memcpy use the corrupted
    // cursor (`written_so_far = self.state.cursor`) as the copy length,
    // reading `(cursor_aligned - self.len)` bytes past the original
    // loan boundary into the spill buffer. Silent OOB read (UB).
    //
    // Instead, cursor advancement is DEFERRED until after
    // ensure_capacity_for / spill completes. `written_so_far` only
    // reaches pre-bump cursor (always <= self.len) so the memcpy is
    // bounded by the loan.
    //
    // This test triggers the precise pathology: a buffer sized so that
    // a Float64MultiArray's `set_data` typed setter has cursor_aligned
    // > self.len (alignment bump alone overflows the loan) but the
    // spill (max_capacity > cursor_aligned + bytes_needed) recovers.

    // Float64MultiArray: layout = MultiArrayLayout (variable), data = float64[].
    // For the alignment bump to push past self.len, we need cursor to be
    // < self.len but cursor_aligned (rounded up to 8) > self.len.
    // E.g. cursor = self.len - 3, alignment to 8 = cursor + 5 > self.len.
    //
    // Compute: WIRE_FIXED_SIZE + OFFSET_TABLE_BYTES (8 * VARIABLE_FIELD_COUNT).
    let fixed_plus_offset = <Float64MultiArray as ShmMessage>::WIRE_FIXED_SIZE
        + 8 * <Float64MultiArray as ShmMessage>::VARIABLE_FIELD_COUNT;
    // Set self.len to a value where after writing the first variable
    // field (layout, empty) cursor = fixed_plus_offset = N (already 8-aligned),
    // then add 5 to self.len so cursor + 5 = self.len and cursor + 8
    // (aligned for f64) > self.len.
    //
    // Easier: self.len = fixed_plus_offset + 5 → cursor_aligned for next
    // 8-byte write = fixed_plus_offset (already 8-aligned) → no spill
    // unless write exceeds self.len. To force alignment-only spill:
    //   1. Write 3 bytes of header-bytes first (cursor → fixed_plus_offset + 3).
    //   2. Loan_data(1) with elem_size=8: cursor_aligned =
    //      ((fixed_plus_offset + 3) + 7) & !7 = fixed_plus_offset + 8.
    //   3. If self.len = fixed_plus_offset + 5, cursor_aligned (= fixed_plus_offset + 8)
    //      > self.len → spill (the OOB shape).
    let buf_len = fixed_plus_offset + 5;
    let mut buf = aligned_buf(buf_len);
    let bytes = aligned_view(&mut buf, buf_len);
    // max_capacity large enough so spill rescues; reader-side topic gives
    // us a writer.
    let mut writer = Float64MultiArray::build_writer(
        bytes,
        cerulion_core::wire::MaxPayloadCapacity::const_new(64 * 1024),
        std::sync::Arc::from("test/align_spill"),
    );

    // Write `layout` (complex variable; first field) with 3 bytes → cursor
    // ends at fixed_plus_offset + 3.
    writer.set_layout_bytes(&[1u8, 2, 3]).expect("layout");

    // Now set_data with 1 f64: alignment bump from
    // (fixed_plus_offset + 3) to (fixed_plus_offset + 8) > self.len.
    // With an early cursor advance spill_to_overflow would OOB-read 3 bytes past self.ptr.
    // With the deferred advance the spill happens cleanly, writes succeed.
    writer
        .set_data(&[42.5f64])
        .expect("set_data with alignment-spill must succeed via heap fallback");

    assert!(
        writer.has_overflow(),
        "alignment-bump-only overflow must trigger spill"
    );
    // Verify data field round-trips byte-correct (would corrupt if the
    // OOB-read poisoned the spill buffer).
    let data_view = writer.data();
    assert_eq!(data_view.len(), 1);
    assert!(
        (data_view[0] - 42.5).abs() < 1e-9,
        "f64 round-trip via spill must be byte-correct"
    );
}

#[test]
fn fill_from_cursor_aligned_equals_max_capacity_spills_then_zero_elements() {
    // Regression test for the alignment-only spill path:
    // a fill_from that passed `bytes_needed=1` to ensure_capacity_for
    // there would, when `cursor_aligned ==
    // max_cap` exactly (alignment bump lands on the absolute ceiling),
    // see `required_end = max_cap + 1 > max_cap` and raise a spurious
    // `PayloadTooLarge` — but the correct semantic is "spill cleanly
    // and let the producer write 0 bytes" (mirrors the
    // `cursor_aligned == self.len` zero-elements contract).
    //
    // `bytes_needed=0` makes `required_end = cursor_aligned
    // == max_cap` → spill succeeds → `remaining_bytes = 0` → producer
    // gets empty dst → Ok with 0 elements written.
    //
    // Construction: Float64MultiArray with state.cursor positioned so
    // that the alignment bump for f64 (elem_size=8) takes cursor_aligned
    // exactly to max_capacity (8-aligned).
    let fixed_plus_offset = <Float64MultiArray as ShmMessage>::WIRE_FIXED_SIZE
        + 8 * <Float64MultiArray as ShmMessage>::VARIABLE_FIELD_COUNT;

    // Choose max_capacity 8-aligned + > fixed_plus_offset + 1.
    let max_capacity: u32 = ((fixed_plus_offset as u32) + 64) & !7;

    // We need state.cursor in [max_capacity - 7, max_capacity - 1] so that
    // (state.cursor + 7) & !7 == max_capacity. Pick state.cursor =
    // max_capacity - 7.
    let target_cursor = (max_capacity - 7) as usize;
    // Layout (first variable field) bytes to write to reach target_cursor:
    // state.cursor starts at fixed_plus_offset.
    let layout_bytes_to_write = target_cursor - fixed_plus_offset;

    // Buffer must accommodate self.len < max_capacity (forces spill path).
    // bytes.len() = target_cursor (so layout write exactly fills the loan).
    let buf_len = target_cursor;
    let mut buf = aligned_buf(buf_len);
    let bytes = aligned_view(&mut buf, buf_len);
    let mut writer = Float64MultiArray::build_writer(
        bytes,
        cerulion_core::wire::MaxPayloadCapacity::const_new(max_capacity),
        std::sync::Arc::from("test/align_eq_max"),
    );

    // Write `layout` complex bytes to advance state.cursor to target_cursor.
    let layout_filler = vec![0u8; layout_bytes_to_write];
    writer
        .set_layout_bytes(&layout_filler)
        .expect("layout filler");

    // Sanity: state.cursor is now target_cursor (= max_capacity - 7),
    // self.len = target_cursor, so cursor_aligned for f64 will bump
    // cursor up by 7 to max_capacity. cursor_aligned > self.len (the
    // strict check fires); ensure_capacity_for(0, max_capacity) →
    // required_end = max_capacity ≤ max_capacity → spill.
    let mut producer_called = false;
    let mut dst_len_seen = usize::MAX;
    writer
        .fill_from_data(|dst: &mut [f64]| {
            producer_called = true;
            dst_len_seen = dst.len();
            Ok(0)
        })
        .expect(
            "fill_from at cursor_aligned == max_capacity boundary must spill cleanly \
             (not return spurious PayloadTooLarge from a `bytes_needed=1`)",
        );

    assert!(producer_called, "producer must be called");
    assert_eq!(
        dst_len_seen, 0,
        "producer's dst slice must be empty (0 elements) — \
         remaining_bytes = self.len - cursor_aligned = max_cap - max_cap = 0"
    );
    assert!(
        writer.has_overflow(),
        "spill must have fired (alignment bump pushed cursor past self.len)"
    );
}

#[test]
fn allocation_failed_when_spill_fault_armed() {
    // Constructing an `AllocationFailed` variant by hand and asserting on its
    // `Display` text would leave the actual codegen `try_reserve_exact`
    // failure path uncovered. The fault-injection helper
    // `cerulion_core::spill_fault_injection::arm()` reaches it.
    //
    // This test exercises the actual `spill_to_overflow`
    // codegen path: arm the fault, trigger a setter that would
    // spill, and assert the `AllocationFailed` Err surfaces with
    // the correct topic + requested-bytes count.
    //
    // Complements the e2e test
    // `e2e_spill_allocation_failed_surfaces_to_setter_call`
    // (which exercises the full pipeline over a `TestTransport`
    // iceoryx2 SHM root)
    // by pinning the codegen-emitted gate at the unit-test level
    // — no transport, no Drop, just the setter Err.
    use cerulion_core::spill_fault_injection;

    let mut buf = aligned_buf(1024);
    let bytes = aligned_view(&mut buf, 1024);
    let mut writer = Image::build_writer(
        bytes,
        cerulion_core::wire::MaxPayloadCapacity::const_new(64 * 1024),
        std::sync::Arc::from("test/spill_oom"),
    );

    writer.set_header_bytes(b"hdr").expect("header set");
    writer.set_encoding("rgb8").expect("encoding set");

    // Arm the fault. The next setter that would spill returns
    // `AllocationFailed`. Without the arm, the 8 KiB write would
    // spill happily (1 KiB loan → 8 KiB write fits in 64 KiB ceiling).
    spill_fault_injection::arm();

    let big = vec![0xeeu8; 8 * 1024];
    let err = writer
        .set_data(&big)
        .expect_err("set_data with spill fault armed must fail");
    match err {
        TransportError::AllocationFailed { topic, requested } => {
            assert_eq!(
                topic, "test/spill_oom",
                "AllocationFailed must carry the writer's topic"
            );
            assert_eq!(
                requested,
                64 * 1024,
                "AllocationFailed.requested must reflect max_capacity \
                 (the spill would allocate the full ceiling)"
            );
        }
        other => panic!("expected AllocationFailed, got {other:?}"),
    }

    // The fault-injection check runs at the TOP of spill_to_overflow,
    // BEFORE try_reserve_exact + into_boxed_slice. So when armed, the
    // function returns Err without ever constructing the spill buffer —
    // writer state is unchanged from before set_data was called.
    // (Fire-once consume + recovery is verified end-to-end in
    // `e2e_spill_allocation_failed_surfaces_to_setter_call`, which
    // runs a subsequent setter after the armed Err.)
    assert!(
        !writer.has_overflow(),
        "no spill happened because fault fired BEFORE try_reserve_exact"
    );
}
