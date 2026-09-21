// SPDX-License-Identifier: AGPL-3.0-only
//! Runtime helpers for codegen-emitted SHM-backed message types.
//!
//! Generated `<Name>Shm<'a>` structs for variable-length schemas embed a
//! [`WriterState`] tracking the in-progress write cursor, a per-field
//! "written" bitset, and the per-field starting offset within the SHM
//! payload. The state lives in this crate (rather than being re-emitted
//! per schema) so the codegen output stays compact and the type is
//! independently testable.
//!
//! # Wire layout
//!
//! ```text
//! [WireHeader (32 bytes)][fixed_section][offset_table][variable_payload]
//! ```
//!
//! `<Name>Shm<'a>` borrows the bytes *after* the WireHeader as
//! `payload: &'a mut [u8]`. The fixed section starts at `payload[0]`,
//! the offset table at `payload[WIRE_FIXED_SIZE]`, and the variable
//! payload immediately after the offset table. Cursor is the running
//! write position into the variable payload region; on construction it
//! is initialised to `WIRE_FIXED_SIZE + 8 * VARIABLE_FIELD_COUNT` so
//! the first variable write lands just past the (zero-initialised)
//! offset table.
//!
//! # Random-order writes
//!
//! Variable-field setters may be called in any order; the offset table
//! records each field's starting offset and length, so reads do not
//! depend on declaration order. The `cursor` only advances forward —
//! re-writing a field with `set_/loan_` allocates a fresh region at the
//! tail and updates the offset-table slot to point at it (the previous
//! region becomes dead bytes). See `conversation.md:140`.

/// Runtime per-tick state for a variable-length SHM-backed message writer.
///
/// `N` is the schema's variable-field count (one bit in `written` per
/// declared variable field, one entry in `field_starts` per declared
/// variable field). For schemas with no variable fields, `N == 0` and
/// `field_starts` is a zero-sized array.
///
/// All fields are `pub` so codegen-emitted methods can read and write
/// them directly without going through accessors. The struct is
/// `Copy + Clone` so it lives entirely on the stack.
#[derive(Debug, Clone, Copy)]
pub struct WriterState<const N: usize> {
    /// Forward-only write cursor into the SHM payload (in bytes from
    /// `payload[0]`). Starts just past the offset table on construction
    /// and only ever advances.
    pub cursor: u32,

    /// One bit per variable field: bit `i` is set iff field index `i`
    /// has been written at least once during this tick. Used by
    /// `<Name>Shm::all_variables_written()` for the `OutputProxy::Drop`
    /// missing-field check.
    ///
    /// Limits the schema to 64 variable fields; this is far above any
    /// realistic ROS2 message (the largest in `native_ros2_messages`
    /// is `Marker` with 7 variable fields).
    pub written: u64,

    /// Per-field starting offset (in bytes from `payload[0]`) into the
    /// variable payload region. Indexed by variable-field declaration
    /// order. Only entries with the corresponding bit set in `written`
    /// are meaningful.
    pub field_starts: [u32; N],
}

impl<const N: usize> WriterState<N> {
    /// Create a fresh writer state with the cursor positioned just past
    /// the offset table.
    ///
    /// `wire_fixed_size` is the size of the fixed section in bytes;
    /// the cursor lands at `wire_fixed_size + 8 * N` so the first
    /// variable write goes into the variable payload region.
    #[inline]
    pub const fn new(wire_fixed_size: u32) -> Self {
        Self {
            cursor: wire_fixed_size + (8 * N as u32),
            written: 0,
            field_starts: [0u32; N],
        }
    }

    /// Returns true iff every declared variable field has been written
    /// at least once during this tick.
    ///
    /// Used by `OutputProxy::Drop` to gate the publish.
    #[inline]
    pub const fn all_written(&self) -> bool {
        // For N == 64 the mask is `u64::MAX`; for smaller N we want the
        // low N bits. `1u64 << N` would overflow at N == 64, so handle
        // that case explicitly.
        let mask: u64 = if N >= 64 { u64::MAX } else { (1u64 << N) - 1 };
        (self.written & mask) == mask
    }

    /// Mark variable field `idx` as written and record its starting offset.
    ///
    /// Caller is responsible for keeping `field_starts[idx]` consistent
    /// with the actual data location (set / loan / push update it before
    /// calling this).
    #[inline]
    pub fn mark_written(&mut self, idx: usize) {
        debug_assert!(idx < N, "field index out of range");
        self.written |= 1u64 << idx;
    }

    /// Returns true iff variable field `idx` has been written.
    #[inline]
    pub const fn is_written(&self, idx: usize) -> bool {
        debug_assert!(idx < N, "field index out of range");
        (self.written & (1u64 << idx)) != 0
    }
}

/// Read an `(offset, length)` pair from the offset table.
///
/// `payload` is the post-header bytes; `wire_fixed_size` is the fixed
/// section size; `idx` is the variable-field index. Returns
/// `(offset_in_payload, length_bytes)`. Both are `u32` per
/// [`crate::wire::OffsetEntry`].
///
/// Returns `(0, 0)` if `payload` is too short or the slot looks
/// uninitialised — variable-field readers fall back to an empty slice
/// in that case rather than panicking, so a partially-constructed
/// frame still parses.
#[inline]
pub fn read_offset_entry(payload: &[u8], wire_fixed_size: usize, idx: usize) -> (u32, u32) {
    let entry_start = wire_fixed_size + 8 * idx;
    if payload.len() < entry_start + 8 {
        return (0, 0);
    }
    let offset = u32::from_le_bytes(payload[entry_start..entry_start + 4].try_into().unwrap());
    let length = u32::from_le_bytes(
        payload[entry_start + 4..entry_start + 8]
            .try_into()
            .unwrap(),
    );
    (offset, length)
}

/// Write an `(offset, length)` pair into the offset table.
///
/// Inverse of [`read_offset_entry`]. Caller must ensure `payload` is
/// large enough; in practice the codegen-emitted setters guarantee
/// this because the offset table's bytes are reserved on writer
/// construction (cursor starts past the table).
#[inline]
pub fn write_offset_entry(
    payload: &mut [u8],
    wire_fixed_size: usize,
    idx: usize,
    offset: u32,
    length: u32,
) {
    let entry_start = wire_fixed_size + 8 * idx;
    payload[entry_start..entry_start + 4].copy_from_slice(&offset.to_le_bytes());
    payload[entry_start + 4..entry_start + 8].copy_from_slice(&length.to_le_bytes());
}

// ---------------------------------------------------------------------------
// Nested-writer sugar: staged scratch for complex-nested
// variable fields
// ---------------------------------------------------------------------------

/// Staging state for ONE complex-nested variable field of a generated
/// `<Name>Shm<'a>` writer (the `self.image.header.frame_id =
/// ...` sugar).
///
/// The parent writer cannot hold the nested `<Target>Shm<'_>` VIEW across
/// accessor calls (it would be self-referential); instead it holds this
/// NON-GENERIC record — an owned 8-aligned scratch buffer plus the nested
/// writer's persisted `WriterState` fields — and the generated
/// `__cer_with_nested_<f>` accessor reconstructs the view fresh per call
/// via the target's `__cer_staging_resume` constructor, then harvests the
/// state back through `__cer_staging_export`. Non-generic on purpose: the
/// PARENT's codegen knows the target schema only by NAME (the proc-macro
/// registry gap), so it cannot name `WriterState<N>`; `field_starts` is a
/// heap `Vec` sized once (at first touch) from the target's
/// `VARIABLE_FIELD_COUNT` const.
///
/// Growth strategy: the scratch starts small (the generated accessor
/// passes `max(<Target>Shm fixed+table floor, STAGED_SCRATCH_FLOOR_BYTES)`)
/// while the nested view's `max_capacity` is the PARENT's full capacity —
/// so an over-floor nested write triggers the target's EXISTING overflow
/// spill (into the nested writer's own heap buffer, state preserved), and
/// the accessor harvests that spill buffer as the NEW scratch after the
/// closure returns. One battle-tested growth path, no bespoke realloc.
#[derive(Debug)]
pub struct StagedNested {
    /// 8-aligned scratch backing the staged nested payload
    /// (`[fixed_section][offset_table][variable payload…]` of the TARGET
    /// schema — no WireHeader; this is exactly the byte shape
    /// `set_<f>_bytes` expects at flush).
    pub scratch: Vec<u64>,
    /// Persisted nested-writer cursor (bytes from `scratch[0]`).
    pub cursor: u32,
    /// Persisted nested-writer written bitset.
    pub written: u64,
    /// Persisted nested-writer per-field start offsets
    /// (len == the target schema's `VARIABLE_FIELD_COUNT`).
    pub field_starts: Vec<u32>,
    /// Recursive staging persistence: the TARGET schema's own
    /// staged complex-nested children, keyed by the child's raw field name.
    /// A transient nested view (`__cer_staging_resume`) starts with empty
    /// staging slots; a grandchild write (`self.out.mid.deep.leaf = …`)
    /// stages into the VIEW's slot, which would die when the view drops —
    /// so the embedding accessor drains the view's slots into this vec on
    /// exit (`__cer_staging_take_children`) and restores them into the next
    /// view (`__cer_staging_restore_children`); the flush recurses through
    /// them (`__cer_flush_staged` per level). `StagedNested` is non-generic,
    /// so the self-containment is legal; nesting depth is bounded by the
    /// schema graph (resolved schemas cannot be cyclic).
    pub children: Vec<(String, StagedNested)>,
}

/// Floor for the initial staged-scratch size (bytes). Covers typical
/// `Header`-class nested payloads (a frame-id string + slack) without ever
/// touching the growth path; bigger payloads grow via the target's overflow
/// spill (see [`StagedNested`] docs).
pub const STAGED_SCRATCH_FLOOR_BYTES: usize = 256;

impl StagedNested {
    /// Fallibly allocate a zeroed staged scratch of at least `min_bytes`
    /// (rounded up to whole `u64`s, floored at
    /// [`STAGED_SCRATCH_FLOOR_BYTES`]) plus the persisted-state vectors for
    /// a target with `var_count` variable fields.
    ///
    /// `initial_cursor` is the target's `WIRE_FIXED_SIZE + 8 * var_count`
    /// (the same position `WriterState::new` computes — the parent passes
    /// it from the target's consts since it cannot instantiate the
    /// generic). Returns `None` on allocation failure (the generated
    /// caller maps it to `TransportError::AllocationFailed` with topic
    /// attribution).
    pub fn try_new(min_bytes: usize, var_count: usize, initial_cursor: u32) -> Option<Self> {
        let bytes = min_bytes.max(STAGED_SCRATCH_FLOOR_BYTES);
        let words = bytes.div_ceil(8);
        let mut scratch: Vec<u64> = Vec::new();
        if scratch.try_reserve_exact(words).is_err() {
            return None;
        }
        scratch.resize(words, 0u64);
        let mut field_starts: Vec<u32> = Vec::new();
        if field_starts.try_reserve_exact(var_count).is_err() {
            return None;
        }
        field_starts.resize(var_count, 0u32);
        Some(Self {
            scratch,
            cursor: initial_cursor,
            written: 0,
            field_starts,
            children: Vec::new(),
        })
    }

    /// Logical byte length of the scratch (== `scratch.len() * 8`; the
    /// round-up slack is usable payload — the nested writer's own
    /// `max_capacity` bound is what limits real writes).
    #[inline]
    pub fn len_bytes(&self) -> usize {
        self.scratch.len() * 8
    }
}

/// View a `[u64]` buffer as mutable bytes.
///
/// Used by generated staging code to hand a [`StagedNested`] scratch to the
/// target's `__cer_staging_resume` constructor. That constructor asserts
/// alignment to the TARGET fixed-section's own `align_of` — not a fixed
/// 8-byte bound. The `u64` backing (align 8) satisfies it for every current
/// schema because the largest primitive field alignment is 8 (`u64`/`f64`);
/// a future over-aligned fixed section (align > 8) would fail the resume
/// assert and needs a wider scratch backing, not a doc change.
///
/// SAFETY (encapsulated): every byte of an initialized `[u64]` is an
/// initialized `u8`; alignment 8 ≥ 1; length math cannot overflow (the
/// slice already exists).
#[inline]
pub fn u64s_as_bytes_mut(words: &mut [u64]) -> &mut [u8] {
    // SAFETY: see doc comment — initialized, aligned, in-bounds.
    unsafe { std::slice::from_raw_parts_mut(words.as_mut_ptr() as *mut u8, words.len() * 8) }
}

/// View a `[u64]` buffer as shared bytes (the flush-side read twin of
/// [`u64s_as_bytes_mut`]).
#[inline]
pub fn u64s_as_bytes(words: &[u64]) -> &[u8] {
    // SAFETY: see `u64s_as_bytes_mut` — initialized, aligned, in-bounds.
    unsafe { std::slice::from_raw_parts(words.as_ptr() as *const u8, words.len() * 8) }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn writer_state_new_positions_cursor_past_offset_table() {
        let s: WriterState<3> = WriterState::new(16);
        // 16 (fixed) + 3 * 8 (offset table) = 40
        assert_eq!(s.cursor, 40);
        assert_eq!(s.written, 0);
        assert_eq!(s.field_starts, [0u32; 3]);
    }

    #[test]
    fn writer_state_zero_variable_fields_handles_empty_array() {
        let s: WriterState<0> = WriterState::new(8);
        assert_eq!(s.cursor, 8);
        assert!(s.all_written()); // vacuously true
    }

    #[test]
    fn writer_state_mark_and_check() {
        let mut s: WriterState<3> = WriterState::new(0);
        assert!(!s.all_written());
        s.mark_written(0);
        s.mark_written(2);
        assert!(s.is_written(0));
        assert!(!s.is_written(1));
        assert!(s.is_written(2));
        assert!(!s.all_written());
        s.mark_written(1);
        assert!(s.all_written());
    }

    #[test]
    fn writer_state_64_fields_mask() {
        let mut s: WriterState<64> = WriterState::new(0);
        assert!(!s.all_written());
        s.written = u64::MAX;
        assert!(s.all_written());
    }

    #[test]
    fn offset_entry_roundtrip() {
        let mut payload = vec![0u8; 64];
        write_offset_entry(&mut payload, 16, 0, 40, 100);
        write_offset_entry(&mut payload, 16, 1, 140, 50);
        assert_eq!(read_offset_entry(&payload, 16, 0), (40, 100));
        assert_eq!(read_offset_entry(&payload, 16, 1), (140, 50));
    }

    #[test]
    fn read_offset_entry_short_buffer_returns_zero() {
        let payload = vec![0u8; 4];
        assert_eq!(read_offset_entry(&payload, 16, 0), (0, 0));
    }

    // ---- StagedNested + byte-view helpers ----

    #[test]
    fn staged_nested_try_new_floors_and_rounds() {
        // Below the floor → floored at 256 bytes = 32 words.
        let s = StagedNested::try_new(10, 3, 40).expect("alloc");
        assert_eq!(s.scratch.len(), 32);
        assert_eq!(s.len_bytes(), 256);
        assert_eq!(s.cursor, 40);
        assert_eq!(s.written, 0);
        assert_eq!(s.field_starts, vec![0u32; 3]);
        // Above the floor → rounded UP to whole u64s (1001 → 126 words).
        let s = StagedNested::try_new(1001, 0, 0).expect("alloc");
        assert_eq!(s.scratch.len(), 126);
        assert_eq!(s.len_bytes(), 1008);
        assert!(s.field_starts.is_empty());
    }

    #[test]
    fn u64_byte_views_are_zeroed_aligned_and_writable() {
        let mut s = StagedNested::try_new(16, 1, 16).expect("alloc");
        {
            let bytes = u64s_as_bytes_mut(&mut s.scratch);
            assert_eq!(bytes.len(), 256);
            assert!(bytes.iter().all(|&b| b == 0), "fresh scratch is zeroed");
            assert_eq!(bytes.as_ptr() as usize % 8, 0, "8-aligned backing");
            bytes[5] = 0xAB;
        }
        let bytes = u64s_as_bytes(&s.scratch);
        assert_eq!(bytes[5], 0xAB, "read view sees the write");
    }
}
