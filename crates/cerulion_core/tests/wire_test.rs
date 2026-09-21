// SPDX-License-Identifier: AGPL-3.0-only
//! Integration tests for wire format.
//!
//! These tests verify the wire format structures and utilities work correctly
//! across module boundaries.

use cerulion_core::wire::{align8, fnv1a_hash, OffsetEntry, WireError, WireHeader};
use std::mem::{align_of, size_of};

/// 8-byte-aligned test buffer. A bare `[u8; N]` has align 1 — natively it
/// usually lands 8-aligned by stack-layout luck, but `WireHeader::from_bytes`
/// documents `None` for unaligned input, and Miri deliberately misaligns
/// align-1 allocations (the Miri job caught two tests relying on that luck).
#[repr(align(8))]
struct AlignedBuf<const N: usize>([u8; N]);

/// Test that WireHeader is exactly 32 bytes.
#[test]
fn test_wire_header_size() {
    assert_eq!(
        size_of::<WireHeader>(),
        32,
        "WireHeader must be exactly 32 bytes"
    );
    assert_eq!(WireHeader::SIZE, 32, "WireHeader::SIZE must match size_of");
}

/// Test that WireHeader has proper alignment.
#[test]
fn test_wire_header_alignment() {
    assert_eq!(
        align_of::<WireHeader>(),
        8,
        "WireHeader must be 8-byte aligned"
    );
}

/// Test that OffsetEntry is exactly 8 bytes.
#[test]
fn test_offset_entry_size() {
    assert_eq!(
        size_of::<OffsetEntry>(),
        8,
        "OffsetEntry must be exactly 8 bytes"
    );
}

/// Test WireHeader field layout matches expected offsets.
#[test]
fn test_wire_header_field_offsets() {
    use std::mem::offset_of;

    // Verify field offsets for C ABI compatibility
    assert_eq!(offset_of!(WireHeader, schema_hash), 0);
    assert_eq!(offset_of!(WireHeader, total_size), 8);
    assert_eq!(offset_of!(WireHeader, offset_table_offset), 12);
    assert_eq!(offset_of!(WireHeader, offset_table_count), 16);
    assert_eq!(offset_of!(WireHeader, sequence), 20);
    assert_eq!(offset_of!(WireHeader, timestamp_ns), 24);
}

/// Test align8 utility function.
#[test]
fn test_align8_correctness() {
    // Edge cases
    assert_eq!(align8(0), 0);
    assert_eq!(align8(1), 8);
    assert_eq!(align8(7), 8);
    assert_eq!(align8(8), 8);
    assert_eq!(align8(9), 16);

    // Larger values
    assert_eq!(align8(100), 104);
    assert_eq!(align8(1000), 1000); // Already aligned
    assert_eq!(align8(1001), 1008);
}

/// Test FNV-1a hash is deterministic.
#[test]
fn test_fnv1a_deterministic() {
    let test_strings = ["Image", "JointState", "LaserScan", "Header", ""];

    for s in test_strings {
        let hash1 = fnv1a_hash(s.as_bytes());
        let hash2 = fnv1a_hash(s.as_bytes());
        assert_eq!(hash1, hash2, "Hash should be deterministic for '{}'", s);
    }
}

/// Test FNV-1a produces different hashes for different inputs.
#[test]
fn test_fnv1a_collision_resistant() {
    let hashes: Vec<_> = ["Image", "JointState", "LaserScan", "Header", "Pose"]
        .iter()
        .map(|s| fnv1a_hash(s.as_bytes()))
        .collect();

    // Check all pairs are different
    for i in 0..hashes.len() {
        for j in i + 1..hashes.len() {
            assert_ne!(hashes[i], hashes[j], "Hashes should be unique");
        }
    }
}

/// Test WireHeader creation and accessors.
#[test]
fn test_wire_header_creation() {
    let hash = fnv1a_hash(b"TestMessage");
    let header = WireHeader::new(hash, 42, 1000000000);

    assert_eq!(header.schema_hash, hash);
    assert_eq!(header.sequence, 42);
    assert_eq!(header.timestamp_ns, 1000000000);
    assert_eq!(header.total_size, 0); // Not set yet
    assert_eq!(header.offset_table_offset, 0);
    assert_eq!(header.offset_table_count, 0);
}

/// Test WireHeader::with_schema convenience constructor.
#[test]
fn test_wire_header_with_schema() {
    let hash = fnv1a_hash(b"Image");
    let header = WireHeader::with_schema(hash);

    assert_eq!(header.schema_hash, hash);
    assert_eq!(header.sequence, 0);
    assert_eq!(header.timestamp_ns, 0);
}

/// Test schema validation succeeds for matching hash.
#[test]
fn test_schema_validation_success() {
    let hash = fnv1a_hash(b"Image");
    let header = WireHeader::with_schema(hash);

    assert!(header.validate_schema(hash, "Image").is_ok());
}

/// Test schema validation fails for mismatched hash.
#[test]
fn test_schema_validation_mismatch() {
    let image_hash = fnv1a_hash(b"Image");
    let laser_hash = fnv1a_hash(b"LaserScan");
    let header = WireHeader::with_schema(image_hash);

    let result = header.validate_schema(laser_hash, "LaserScan");
    assert!(result.is_err());

    match result.unwrap_err() {
        WireError::SchemaMismatch {
            expected_name,
            expected_hash,
            actual_hash,
        } => {
            assert_eq!(expected_name, "LaserScan");
            assert_eq!(expected_hash, laser_hash);
            assert_eq!(actual_hash, image_hash);
        }
        _ => panic!("Expected SchemaMismatch error"),
    }
}

/// Test WireHeader as_bytes roundtrip.
#[test]
fn test_wire_header_bytes_roundtrip() {
    let hash = fnv1a_hash(b"TestMsg");
    let original = WireHeader::new(hash, 123, 9876543210);

    let bytes = original.as_bytes();
    assert_eq!(bytes.len(), WireHeader::SIZE);

    let mut aligned_buf = AlignedBuf::<64>([0u8; 64]);
    aligned_buf.0[..32].copy_from_slice(bytes);

    let parsed = WireHeader::from_bytes(&aligned_buf.0).expect("Should parse");
    assert_eq!(parsed.schema_hash, original.schema_hash);
    assert_eq!(parsed.sequence, original.sequence);
    assert_eq!(parsed.timestamp_ns, original.timestamp_ns);
}

/// Test WireHeader from_bytes rejects too-small buffers.
#[test]
fn test_wire_header_from_bytes_too_small() {
    let small_buf = [0u8; 16];
    assert!(WireHeader::from_bytes(&small_buf).is_none());
}

/// Test OffsetEntry creation.
#[test]
fn test_offset_entry_creation() {
    let entry = OffsetEntry::new(100, 500);
    assert_eq!(entry.offset, 100);
    assert_eq!(entry.length, 500);
}

/// Test WireError display formatting.
#[test]
fn test_wire_error_display() {
    let err = WireError::BufferTooSmall {
        required: 100,
        available: 50,
    };
    let msg = err.to_string();
    assert!(msg.contains("100"));
    assert!(msg.contains("50"));

    let err = WireError::InvalidOffset {
        offset: 1000,
        max: 500,
    };
    let msg = err.to_string();
    assert!(msg.contains("1000"));
    assert!(msg.contains("500"));
}

// NOTE: the `schema_hash!` macro (name-only FNV-1a) was deleted —
// it codified the premise the layout-sensitive recipe removes (hash = fnv1a(name)). Its only
// consumers were tests of itself. The schema hash is now the layout-
// sensitive `codegen::MessageSchema::schema_hash` recipe.

// === Edge Case Tests ===

/// Test WireHeader from_bytes with exactly 32 bytes (boundary condition).
#[test]
fn test_wire_header_exact_size_boundary() {
    let hash = fnv1a_hash(b"TestMsg");
    let original = WireHeader::new(hash, 42, 123456789);

    // Buffer of exactly 32 bytes (boundary) with guaranteed 8-byte alignment.
    let mut buf = AlignedBuf::<32>([0u8; 32]);
    buf.0.copy_from_slice(original.as_bytes());

    let parsed = WireHeader::from_bytes(&buf.0);
    assert!(parsed.is_some(), "Should parse buffer of exactly 32 bytes");

    let header = parsed.unwrap();
    assert_eq!(header.schema_hash, hash);
    assert_eq!(header.sequence, 42);
}

/// from_bytes documents `None` for non-8-aligned buffers (the
/// alignment-safe path is `read_from_buf`). Pin that contract with a
/// deliberately misaligned view into an aligned allocation.
#[test]
fn test_wire_header_from_bytes_rejects_unaligned() {
    let hash = fnv1a_hash(b"TestMsg");
    let original = WireHeader::new(hash, 42, 123456789);

    let mut buf = AlignedBuf::<40>([0u8; 40]);
    buf.0[1..33].copy_from_slice(original.as_bytes());

    assert!(
        WireHeader::from_bytes(&buf.0[1..33]).is_none(),
        "from_bytes must reject a non-8-aligned buffer"
    );

    // The alignment-safe path handles the same bytes fine.
    let header = WireHeader::read_from_buf(&buf.0[1..33])
        .expect("read_from_buf must accept an unaligned 32-byte buffer");
    assert_eq!(header.schema_hash, hash);
    assert_eq!(header.sequence, 42);
}

/// Test WireHeader with zero values (edge case).
#[test]
fn test_wire_header_zero_values() {
    let header = WireHeader::new(0, 0, 0);

    assert_eq!(header.schema_hash, 0);
    assert_eq!(header.sequence, 0);
    assert_eq!(header.timestamp_ns, 0);
    assert_eq!(header.total_size, 0);
    assert_eq!(header.offset_table_offset, 0);
    assert_eq!(header.offset_table_count, 0);
}

/// Test WireHeader with maximum u64 values (edge case).
#[test]
fn test_wire_header_max_values() {
    let header = WireHeader::new(u64::MAX, u32::MAX, u64::MAX);

    assert_eq!(header.schema_hash, u64::MAX);
    assert_eq!(header.sequence, u32::MAX);
    assert_eq!(header.timestamp_ns, u64::MAX);
}

/// Test OffsetEntry with zero values.
#[test]
fn test_offset_entry_zero_values() {
    let entry = OffsetEntry::new(0, 0);
    assert_eq!(entry.offset, 0);
    assert_eq!(entry.length, 0);
}

/// Test OffsetEntry with maximum u32 values.
#[test]
fn test_offset_entry_max_values() {
    let entry = OffsetEntry::new(u32::MAX, u32::MAX);
    assert_eq!(entry.offset, u32::MAX);
    assert_eq!(entry.length, u32::MAX);
}

/// Test large offset table simulation (many variable fields).
#[test]
fn test_large_offset_table() {
    // Simulate a message with 256 variable fields
    let num_fields: u32 = 256;
    let entries: Vec<OffsetEntry> = (0..num_fields)
        .map(|i| OffsetEntry::new(i * 100, 100))
        .collect();

    // Verify all entries are correct
    assert_eq!(entries.len(), 256);
    for (i, entry) in entries.iter().enumerate() {
        assert_eq!(entry.offset, (i * 100) as u32);
        assert_eq!(entry.length, 100);
    }

    // Verify total size calculation
    let table_size = num_fields * (size_of::<OffsetEntry>() as u32);
    assert_eq!(table_size, 256 * 8);
}

/// Test FNV-1a hash with empty input.
#[test]
fn test_fnv1a_empty_input() {
    let hash = fnv1a_hash(b"");
    // FNV-1a offset basis for empty string
    assert_eq!(hash, 0xcbf29ce484222325);
}

/// Test FNV-1a hash with long input.
#[test]
fn test_fnv1a_long_input() {
    // Test with a long schema name (unusual but valid)
    let long_name = "a".repeat(1000);
    let hash1 = fnv1a_hash(long_name.as_bytes());
    let hash2 = fnv1a_hash(long_name.as_bytes());

    // Should be deterministic
    assert_eq!(hash1, hash2);

    // Should be different from short names
    let short_hash = fnv1a_hash(b"a");
    assert_ne!(hash1, short_hash);
}

/// Test align8 with maximum usize value doesn't overflow.
#[test]
fn test_align8_large_values() {
    // Values that are already aligned
    assert_eq!(align8(usize::MAX - 7), usize::MAX - 7);

    // Large but not max values
    assert_eq!(align8(1_000_000_000), 1_000_000_000); // Already aligned
    assert_eq!(align8(1_000_000_001), 1_000_000_008);
}

// === Property-based tests (proptest) ===

use proptest::prelude::*;

proptest! {
    /// Roundtrip: arbitrary WireHeader fields survive write_to_buf → read_from_buf.
    #[test]
    fn prop_wire_header_roundtrip(
        schema_hash in any::<u64>(),
        total_size in any::<u32>(),
        offset_table_offset in any::<u32>(),
        offset_table_count in any::<u32>(),
        sequence in any::<u32>(),
        timestamp_ns in any::<u64>(),
    ) {
        let header = WireHeader {
            schema_hash,
            total_size,
            offset_table_offset,
            offset_table_count,
            sequence,
            timestamp_ns,
        };
        let mut buf = [0u8; WireHeader::SIZE];
        header.write_to_buf(&mut buf);
        let parsed = WireHeader::read_from_buf(&buf).expect("should parse any valid header");
        prop_assert_eq!(parsed.schema_hash, schema_hash);
        prop_assert_eq!(parsed.total_size, total_size);
        prop_assert_eq!(parsed.offset_table_offset, offset_table_offset);
        prop_assert_eq!(parsed.offset_table_count, offset_table_count);
        prop_assert_eq!(parsed.sequence, sequence);
        prop_assert_eq!(parsed.timestamp_ns, timestamp_ns);
    }

    /// No-panic: arbitrary 32 bytes should never cause read_from_buf to panic.
    #[test]
    fn prop_wire_header_no_panic_on_arbitrary_bytes(
        bytes in prop::collection::vec(any::<u8>(), 32..=32),
    ) {
        // Should return Some (any 32 bytes are "valid" structurally) or None, but never panic
        let _ = WireHeader::read_from_buf(&bytes);
    }

    /// No-panic: buffers shorter than 32 bytes should return None, never panic.
    #[test]
    fn prop_wire_header_short_buffer_no_panic(
        bytes in prop::collection::vec(any::<u8>(), 0..32),
    ) {
        let result = WireHeader::read_from_buf(&bytes);
        prop_assert!(result.is_none(), "short buffer should return None");
    }
}

/// Test WireError variants are all distinct.
#[test]
fn test_wire_error_variants() {
    let errors = [
        WireError::BufferTooSmall {
            required: 100,
            available: 50,
        },
        WireError::InvalidOffset {
            offset: 1000,
            max: 500,
        },
        WireError::SchemaMismatch {
            expected_name: "Test".to_string(),
            expected_hash: 123,
            actual_hash: 456,
        },
    ];

    // Each error should have a unique string representation
    let messages: Vec<String> = errors.iter().map(|e| e.to_string()).collect();
    for i in 0..messages.len() {
        for j in i + 1..messages.len() {
            assert_ne!(messages[i], messages[j], "Error messages should be unique");
        }
    }
}
