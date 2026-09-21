//! Follow-up: mirror of `shm_invariants_overflow.rs`
//! but pinning the `VARIABLE_FIELD_COUNT` arm of the
//! `_SHM_INVARIANTS` assertion.
//!
//! `_SHM_INVARIANTS` checks
//! `WireHeader::SIZE + WIRE_FIXED_SIZE + 8 * VARIABLE_FIELD_COUNT
//!  <= u32::MAX as usize`.
//! The other UI test covers the `WIRE_FIXED_SIZE = usize::MAX` path;
//! this test covers `VARIABLE_FIELD_COUNT = usize::MAX`. Both ends
//! must fail compilation so that a future refactor of the assert
//! (e.g. dropping `.saturating_add(table)`) cannot silently lose
//! the variable-field-overflow protection.

use cerulion_core::message::ShmMessage;
use cerulion_core::wire::MaxSliceLen;

/// Hand-written pathological schema: VARIABLE_FIELD_COUNT at the
/// usize ceiling. The `8 * VARIABLE_FIELD_COUNT` offset-table size
/// overflows u32 by orders of magnitude; `_SHM_INVARIANTS` must
/// reject this at const-eval (via `saturating_mul` + `saturating_add`
/// → comparison against `u32::MAX as usize` fails).
struct Pathological;

impl ShmMessage for Pathological {
    type Reader<'a> = &'a [u8];
    type Writer<'a> = &'a mut [u8];
    const WIRE_FIXED_SIZE: usize = 0;
    const VARIABLE_FIELD_COUNT: usize = usize::MAX;
    const MAX_SLICE_LEN: Option<MaxSliceLen> = None;
    const SCHEMA_HASH: u64 = 0;
    fn build_reader(bytes: &[u8]) -> Self::Reader<'_> {
        bytes
    }
    fn build_writer<'a>(bytes: &'a mut [u8], _max_capacity: cerulion_core::wire::MaxPayloadCapacity, _topic: std::sync::Arc<str>) -> Self::Writer<'a> {
        bytes
    }
    fn payload_wire_size(_writer: &Self::Writer<'_>) -> usize {
        0
    }
    fn all_variables_written(_writer: &Self::Writer<'_>) -> bool {
        true
    }
}

const _: () = <Pathological as ShmMessage>::_SHM_INVARIANTS;

fn main() {}
