//! Misaligned payload receive (bytemuck alignment regression).
//!
//! The transport receive path reads the 32-byte `WireHeader` off an
//! incoming payload `&[u8]` whose backing address is NOT guaranteed to
//! be 8-aligned (iceoryx2 hands back shared-memory slices; see the
//! `read_from_buf` rationale in `wire.rs`). The receive path therefore
//! uses the alignment-safe readers:
//!   - `WireHeader::read_from_buf` — field-by-field `u64::from_le_bytes`,
//!     decodes regardless of alignment (subscriber.rs:988/1763/1878/1937).
//!   - `WireHeader::from_bytes` — returns `None` on a misaligned pointer
//!     instead of forming a misaligned `&Self`.
//!
//! REGRESSION GUARDED: a naive `bytemuck::from_bytes::<T>(&payload)`
//! reinterpret (T align == 8) PANICS / is UB on a misaligned `&[u8]`.
//!
//! WHY THIS IS A REAL ORACLE, NOT A TAUTOLOGY: each test feeds a
//! *genuinely* misaligned slice (an odd byte offset into a `Vec<u8>`,
//! asserted `% 8 != 0`) and then proves with `bytemuck` ITSELF that an
//! align-8 reinterpret of that exact slice fails (`try_from_bytes` →
//! `Err`; `from_bytes` → panic). If the receive path were ever swapped
//! to a bytemuck-style reinterpret, these tests would panic / mis-decode;
//! they pass only because the current readers are alignment-tolerant.

use bytemuck::PodCastError;
use cerulion_core::wire::WireHeader;

/// Distinctive header so a correct decode is unambiguous (no zeroed field
/// could pass by accident).
fn sample_header() -> WireHeader {
    WireHeader {
        schema_hash: 0xDEAD_BEEF_CAFE_F00D,
        total_size: 1234,
        offset_table_offset: 32,
        offset_table_count: 3,
        sequence: 42,
        timestamp_ns: 0x0102_0304_0506_0708,
    }
}

/// Serialize `header` into a buffer whose 32-byte window starts at a
/// deliberately non-8-aligned address. Returns the owning `Vec` and the
/// start offset of the misaligned window.
fn misaligned_wire_buffer(header: &WireHeader) -> (Vec<u8>, usize) {
    // Reserve 8 bytes of slack so we can slide the window to an address
    // that is not a multiple of 8. One of any 8 consecutive addresses is
    // 8-aligned; pick the first offset (1..=8) that is NOT.
    let mut buf = vec![0u8; WireHeader::SIZE + 8];
    let base = buf.as_ptr() as usize;
    let off = (1..=8)
        .find(|o| !(base + o).is_multiple_of(8))
        .expect("one of 8 consecutive offsets must be non-8-aligned");
    header.write_to_buf(&mut buf[off..off + WireHeader::SIZE]);
    (buf, off)
}

#[test]
fn read_from_buf_decodes_misaligned_payload_without_panic() {
    let header = sample_header();
    let (buf, off) = misaligned_wire_buffer(&header);
    let payload = &buf[off..off + WireHeader::SIZE];

    // The slice really is misaligned for an 8-aligned read.
    assert!(
        !(payload.as_ptr() as usize).is_multiple_of(8),
        "test fixture failed to produce a misaligned slice"
    );

    // ORACLE: a naive `bytemuck::from_bytes::<u64>` (align 8) over
    // this exact slice would panic. Prove the misalignment is real and
    // bytemuck-fatal via the non-panicking sibling.
    assert_eq!(
        bytemuck::try_from_bytes::<u64>(&payload[0..8]),
        Err(PodCastError::TargetAlignmentGreaterAndInputNotAligned),
        "slice must be misaligned enough that a bytemuck reinterpret rejects it"
    );

    // The actual receive-path reader: alignment-safe, correct decode.
    let decoded = WireHeader::read_from_buf(payload)
        .expect("read_from_buf must decode a misaligned payload, not None/panic");
    assert_eq!(decoded, header, "misaligned decode must be byte-correct");
}

#[test]
fn from_bytes_rejects_misaligned_pointer_gracefully() {
    let header = sample_header();
    let (buf, off) = misaligned_wire_buffer(&header);
    let payload = &buf[off..off + WireHeader::SIZE];
    assert!(!(payload.as_ptr() as usize).is_multiple_of(8));

    // `from_bytes` forms a `&WireHeader` (align 8) only when aligned;
    // on a misaligned pointer it must return `None`, never UB/panic.
    assert!(
        WireHeader::from_bytes(payload).is_none(),
        "from_bytes must reject a misaligned pointer with None"
    );

    // Sanity: the same bytes at an aligned address DO decode via from_bytes,
    // so the None above is the alignment guard firing, not bad data. Build the
    // aligned window deterministically (complement of `misaligned_wire_buffer`):
    // slack the alloc by 8 and pick the offset where `base + o` IS a multiple
    // of 8, so this positive control always runs — never silently skipped.
    let mut aligned_buf = [0u8; WireHeader::SIZE + 8];
    let base = aligned_buf.as_ptr() as usize;
    let aoff = (0..=8)
        .find(|o| (base + o).is_multiple_of(8))
        .expect("one of 8 consecutive offsets must be 8-aligned");
    header.write_to_buf(&mut aligned_buf[aoff..aoff + WireHeader::SIZE]);
    let aligned = &aligned_buf[aoff..aoff + WireHeader::SIZE];
    assert!(
        (aligned.as_ptr() as usize).is_multiple_of(8),
        "test fixture failed to produce an aligned slice"
    );
    let view = WireHeader::from_bytes(aligned).expect("aligned buffer decodes");
    assert_eq!(*view, header, "aligned decode must be byte-correct");
}
