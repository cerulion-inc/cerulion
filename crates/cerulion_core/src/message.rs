// SPDX-License-Identifier: AGPL-3.0-only
//! Message traits for zero-copy pub/sub.
//!
//! `ShmMessage` is the single trait used by the SHM-backed pub/sub
//! path. It carries the GATs (`Reader<'a>`, `Writer<'a>`)
//! that bind a schema to its codegen-emitted SHM-backed accessor type.
//!
//! Every codegen-emitted schema is a unit marker `<Name>` carrying
//! `impl ShmMessage`, plus the SHM-backed `<Name>Shm[<'a>]` accessor type.
//! (The earlier `Message` / `FixedMessage` traits and the heap-owned message
//! structs they fed were deleted.)
//!
//! A node author meets this trait only as the bound on a port field's type:
//! `#[output] image: Image` works because `Image: ShmMessage`. Nothing here
//! is called from node code.
//!
//! # Wire format
//!
//! Every `ShmMessage` impl agrees on the unified offset-table layout:
//!
//! ```text
//! [WireHeader (32 bytes)][fixed_section][offset_table][variable_payload]
//! ```
//!
//! For fixed schemas, `WIRE_FIXED_SIZE == size of fixed_section` and the
//! offset table is empty. For variable schemas, the offset table has one
//! entry per variable-length field (a string, a dynamic array, a
//! variable-size nested message), indexed by schema field index. Whether a
//! field is variable-length follows from its type; there is no attribute.

use std::sync::{Arc, OnceLock};

use crate::error::TransportError;
use crate::wire::{MaxSliceLen, WireHeader};

/// Sentinel topic for reader-constructed `<Name>Shm`
/// values. Used as the default `topic` in codegen-emitted `from_bytes`
/// (read-side). Cached once via `OnceLock` so per-call `Image::build_reader`
/// is one `Arc::clone` (atomic refcount bump), not a heap allocation —
/// preserving the zero-alloc invariant on the subscriber hot path.
///
/// Reader-misuse via macros is structurally unreachable (`InputView<T>`
/// exposes only `&Self::Reader<'a>`, no `DerefMut`), so this sentinel
/// only surfaces for test code that calls `T::build_reader` directly.
/// When such code accidentally invokes a `&mut self` setter, the
/// resulting `PayloadTooLarge { topic: "<read-only>", max: 0 }` error
/// attributes the misuse to the reader-side construction path.
pub fn read_only_topic() -> Arc<str> {
    static READ_ONLY: OnceLock<Arc<str>> = OnceLock::new();
    READ_ONLY.get_or_init(|| Arc::from("<read-only>")).clone()
}

/// SHM-backed message trait carrying the `Reader<'a>`/`Writer<'a>` GATs.
///
/// Codegen emits a unit marker `pub struct <Name>;` per schema and
/// implements `ShmMessage` on that marker. The actual SHM-backed accessor
/// type is `<<Name> as ShmMessage>::Writer<'a>` (which equals
/// `<<Name> as ShmMessage>::Reader<'a>` — one type does both).
///
/// `OutputProxy<T>` and `InputView<T>` are generic over the marker `T`,
/// derefing to `T::Writer<'a>` and `T::Reader<'a>` respectively.
pub trait ShmMessage: Sized {
    /// Schema hash for validation. Layout-sensitive: FNV-1a 64 over the
    /// schema name, the wire fixed-section size, and every field's name and
    /// canonical type in declaration order (the recipe is
    /// `codegen::MessageSchema::schema_hash`). It is NOT a hash of the name
    /// alone, so two ends that disagree on a message's fields disagree on
    /// this value and the frame is refused rather than misread.
    const SCHEMA_HASH: u64;

    /// Number of variable-length fields in this schema.
    ///
    /// `0` for fixed schemas (offset table omitted from the wire frame).
    const VARIABLE_FIELD_COUNT: usize;

    /// Size of the fixed section in bytes (excludes WireHeader, offset table,
    /// and variable payload).
    ///
    /// For fixed schemas this is the entire payload.
    const WIRE_FIXED_SIZE: usize;

    /// Schema-default maximum SHM slot size (header + payload), if known.
    ///
    /// Codegen emits a concrete `Some(n)` per schema. Hand-written
    /// `impl ShmMessage` blocks should leave this as `None` unless the
    /// schema has a publicly documented bound.
    ///
    /// Codegen-only contract: the constant exists on
    /// every generated marker and, since chunk A2, the runtime consults
    /// it as TIER-2 of the `max_slice_len` resolver — the wiring is macro
    /// → `OutputMeta::max_slice_len_default` → `runtime.rs`'s
    /// `resolve_max_slice_len` → `tracing::warn!` + tier-3 fallback when
    /// it is `None` and no YAML override is given (see `graph::runtime`).
    /// The paired `DEFAULT_MAX_SLICE_LEN` bump was 16 MiB in chunk A2,
    /// later raised to 128 MiB in the launch tier bump. Users
    /// never read this const directly.
    ///
    /// Codegen emission contract:
    /// - Fixed schemas: `NonZeroUsize::new(WireHeader::SIZE + WIRE_FIXED_SIZE)`
    ///   (provably correct upper bound — every published frame is
    ///   exactly this size; sum is always > 0 because `WireHeader::SIZE`
    ///   is 32).
    /// - Variable schemas in `native_ros2_messages/`: per-schema
    ///   budget from `variable_schema_max_slice_len` (5 tiers,
    ///   16 KiB to 128 MiB later; every in-repo schema listed).
    /// - User-defined codegen schemas: 128 MiB user-defined catch-all
    ///   so unfamiliar schemas reserve enough SHM by default.
    /// - Hand-written `impl ShmMessage` blocks: default `None` so
    ///   runtime tier-3 fallback fires.
    ///
    /// The type is `Option<MaxSliceLen>` (not `Option<usize>`, and
    /// tighter than `Option<NonZero…>`), so `Some(0)` is
    /// unrepresentable — the
    /// `MaxSliceLen` newtype encodes BOTH the upper bound
    /// (`u32::MAX`, the wire format's `WireHeader::total_size`
    /// ceiling) AND the lower bound (`>= WireHeader::SIZE = 32`,
    /// so the slot can hold at least the header). Codegen emits
    /// `MaxSliceLen::const_new(<expr>)` which panics at const-eval
    /// if the bound is violated — pathological schemas fail to
    /// build. The runtime tier-2 lookup unwraps via `.get()` for
    /// a `u32` directly; no floor / ceiling check needed (both are
    /// type-system properties).
    const MAX_SLICE_LEN: Option<MaxSliceLen> = None;

    /// Monomorphization-time invariants assertion.
    ///
    /// Referenced once from `loan_proxy<T>` (via
    /// `let _: () = <T as ShmMessage>::_SHM_INVARIANTS;`) to force
    /// const-evaluation at every call site. A pathological hand-
    /// written `impl ShmMessage` (e.g. `WIRE_FIXED_SIZE = usize::MAX`)
    /// fails to compile when used, not at definition.
    ///
    /// The assertion: `WireHeader::SIZE + WIRE_FIXED_SIZE + 8 *
    /// VARIABLE_FIELD_COUNT <= u32::MAX` (the `min_required` upper
    /// bound used in `loan_proxy`'s buffer math). For codegen-emitted
    /// impls this is trivially satisfied (largest ROS2 fixed schema
    /// is well under MB-scale); for hand-written impls this is a
    /// documented precondition.
    const _SHM_INVARIANTS: () = {
        // `saturating_*` is const-stable since 1.47; `assert!` in
        // const context since 1.57. No need for `Option::and_then`
        // (still const-unstable).
        let header = WireHeader::SIZE;
        let fixed = Self::WIRE_FIXED_SIZE;
        let table = Self::VARIABLE_FIELD_COUNT.saturating_mul(8);
        let total = header.saturating_add(fixed).saturating_add(table);
        assert!(
            total <= u32::MAX as usize,
            "ShmMessage impl violates wire-format invariant: \
             WireHeader::SIZE + WIRE_FIXED_SIZE + 8 * VARIABLE_FIELD_COUNT must fit in u32",
        );
    };

    /// SHM-backed reader/writer type. Same Rust type does both.
    ///
    /// `&Reader<'a>` reads, `&mut Reader<'a>` writes. Lifetime `'a` is
    /// bound to the iceoryx2 sample held by `InputView<'a, Self>` or
    /// `OutputProxy<'a, Self>`.
    type Reader<'a>
    where
        Self: 'a;

    /// SHM-backed writer type — same as `Reader<'a>`.
    type Writer<'a>
    where
        Self: 'a;

    /// Construct a `Reader<'a>` over an immutable byte slice (the SHM payload
    /// region after the 32-byte `WireHeader`).
    ///
    /// Used by `CerulionSubscriber::try_view`
    /// to wrap the inbound iceoryx2 sample's payload in the schema-specific
    /// reader type. Caller must ensure the slice covers at least the payload
    /// for this schema (validated upstream via `WireHeader::total_size`).
    ///
    /// For fixed schemas this returns `&'a <Name>Shm` (zero-cost cast). For
    /// variable schemas this returns the opaque `<Name>Shm<'a>` value that
    /// owns its own writer-state cursor (initialised but unused on the read
    /// side — only `&self` accessors are public).
    fn build_reader(bytes: &[u8]) -> Self::Reader<'_>;

    /// Construct a `Writer<'a>` over a mutable byte slice (the SHM payload
    /// region after the 32-byte `WireHeader`).
    ///
    /// Used by `CerulionPublisher::loan_proxy`
    /// to wrap the outbound iceoryx2 sample's payload. For fixed schemas this
    /// returns `&'a mut <Name>Shm`. For variable schemas this returns
    /// `<Name>Shm<'a>` with the cursor positioned just past the offset table
    /// and the offset-table region zero-initialised.
    ///
    /// # `max_capacity` and `topic`
    ///
    /// `max_capacity` is the configured `max_slice_len` ceiling MINUS the
    /// 32-byte `WireHeader` (i.e., the absolute upper bound on the
    /// post-header payload). Variable schemas store it so a setter that
    /// detects `cursor + bytes_needed > self.len` (current loan) can decide
    /// between two paths: (a) `cursor + bytes_needed >
    /// max_capacity` → `Err(PayloadTooLarge)` (no rescue possible); (b)
    /// otherwise, spill writes to a lazily-allocated heap buffer. Fixed
    /// schemas ignore the value — they cannot overflow because their wire
    /// size is constant.
    ///
    /// `topic` is the publisher's topic name as an `Arc<str>`. Variable
    /// schemas store it as a struct field so codegen-emitted overflow
    /// errors (`PayloadTooLarge` / `AllocationFailed`) can be attributed
    /// to the originating topic. Fixed schemas ignore it.
    ///
    /// **Zero-allocation contract**: using `Arc<str>` (vs. an owned
    /// `String` or borrowed `&'a str`) preserves the zero-alloc
    /// invariant on the steady-state publish path. Each loan does one
    /// `Arc::clone` (atomic refcount bump, no allocation); the publisher
    /// allocates the Arc ONCE at construction time. Only the error path
    /// allocates (`self.topic.to_string()` inside `PayloadTooLarge`
    /// construction), which is rare.
    ///
    /// Previously this was `&'a str` and `loan_proxy`
    /// used a raw-pointer reborrow `unsafe` block to detach the borrow
    /// from `&mut self`. With `Arc<str>` the `unsafe` is gone — the Arc
    /// is owned (not borrowed) so the borrow checker is satisfied
    /// without lifetime gymnastics.
    fn build_writer<'a>(
        bytes: &'a mut [u8],
        max_capacity: crate::wire::MaxPayloadCapacity,
        topic: Arc<str>,
    ) -> Self::Writer<'a>;

    /// Wire size of the payload region (excludes the 32-byte `WireHeader`).
    ///
    /// Used by `OutputProxy::Drop` to finalise `WireHeader::total_size` from
    /// the writer's runtime state. For fixed schemas this is always
    /// `WIRE_FIXED_SIZE`. For variable schemas this is the writer's current
    /// cursor (which already includes the fixed section, the offset table,
    /// and any variable payload that was written).
    fn payload_wire_size(writer: &Self::Writer<'_>) -> usize;

    /// True iff every declared variable field has been written.
    ///
    /// For fixed schemas (`VARIABLE_FIELD_COUNT == 0`) this is vacuously true.
    /// For variable schemas this delegates to `<Name>Shm::all_variables_written()`
    /// which consults the per-tick `WriterState::written` bitset.
    ///
    /// Used by `OutputProxy::Drop` to gate the publish.
    fn all_variables_written(writer: &Self::Writer<'_>) -> bool;

    /// True iff this writer spilled to a heap fallback buffer.
    ///
    /// Variable schemas override this to return `writer.has_overflow()`.
    /// Fixed schemas keep the default `false` — they cannot overflow
    /// because their wire size is constant.
    ///
    /// `OutputProxy::Drop` consults this to decide between the
    /// steady-state send path (the loaned SHM sample is sent) and the
    /// overflow re-loan path (a fresh sample is loaned and the heap
    /// bytes are memcpied in).
    fn has_overflow(_writer: &Self::Writer<'_>) -> bool {
        false
    }

    /// View the spill buffer's bytes `[0..payload_wire_size]`.
    ///
    /// Returns `None` for fixed schemas (cannot overflow) and for
    /// variable schemas that did not spill this tick.
    ///
    /// `OutputProxy::Drop` reads the returned slice to memcpy the
    /// payload into the re-loaned sample. The byte view's length equals
    /// the writer's current cursor, which is also `payload_wire_size`.
    fn overflow_view_bytes<'w>(_writer: &'w Self::Writer<'_>) -> Option<&'w [u8]> {
        None
    }

    /// Flush any STAGED complex-nested
    /// field writes into the writer's real payload.
    ///
    /// Variable schemas with complex-nested fields stage
    /// `self.<port>.<nested>.<leaf> = …` / `with_<nested>(|n| …)` writes in
    /// per-field heap scratch; `OutputProxy::Drop` calls this BEFORE the
    /// `all_variables_written` gate so each touched staged field lands via
    /// the schema's own `set_<f>_bytes` (marking it written). Untouched
    /// staged fields flush nothing — the write-all-variable-fields discard
    /// rule is unchanged for them. A flush failure (capacity, incomplete
    /// staged child, invariant violation) is handled by a DEDICATED step-0
    /// branch in `OutputProxy::Drop` that logs the error and early-returns
    /// — skipping publish BEFORE the `all_variables_written` gate ever runs
    /// (same loud-discard CLASS as a missing field, but its own control
    /// path: a flush `Err` never falls through INTO the gate). Never a
    /// silent partial frame.
    ///
    /// Default: no-op `Ok(())` — fixed schemas and variable schemas without
    /// complex-nested fields have nothing to stage.
    fn flush_staged_nested(_writer: &mut Self::Writer<'_>) -> Result<(), TransportError> {
        Ok(())
    }
}
