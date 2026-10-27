// SPDX-License-Identifier: AGPL-3.0-only
//! Wire format for zero-copy message passing.
//!
//! # Message Layout
//!
//! ```text
//! ┌─────────────────────────────────────────────────────────────┐
//! │                    WireHeader (40 bytes)                    │
//! ├─────────────────────────────────────────────────────────────┤
//! │                    Fixed Fields Section                     │
//! │              (primitives, fixed arrays inline)              │
//! ├─────────────────────────────────────────────────────────────┤
//! │                    Offset Table                             │
//! │         (one entry per variable-length field)               │
//! ├─────────────────────────────────────────────────────────────┤
//! │                    Variable Data Section                    │
//! │         (strings, dynamic arrays, nested messages)          │
//! └─────────────────────────────────────────────────────────────┘
//! ```

use std::mem::size_of;

/// The version of the [`WireHeader`] layout this build reads and writes.
///
/// Every place a frame crosses a process or a file states it: the zenoh
/// demand handshake, the network catalog, the `cerulion-netd` banner, the
/// iceoryx2 service attribute and the bag schema descriptor. Where this build
/// can see the other side state a different version, it refuses loudly; a
/// recording at version 1 is instead upgraded on read. The checks run on this
/// build's side only: a release older than these fields states nothing (an
/// iceoryx2 service without the attribute is accepted with a warning) and
/// cannot refuse this build, so one Cerulion release per machine is the
/// supported configuration. Version 1 is the 32-byte header without
/// [`WireHeader::step_tag`]; version 2 is the 40-byte header with it. A
/// version 1 frame read from a recording is converted with
/// [`upgrade_v1_frame_into`].
pub const WIRE_HEADER_VERSION: u16 = 2;

/// The size of a version 1 [`WireHeader`]: every field before
/// [`WireHeader::step_tag`].
pub const V1_HEADER_SIZE: usize = 32;

/// Which producer step a frame belongs to: the producer's rank in the top 16
/// bits, and the step number plus one in the low 48 bits, so the all-zero tag
/// means "not tagged" and can never be mistaken for step 0 of rank 0.
///
/// The largest step that fits is `2^48 - 2` ([`StepTag::MAX_STEP`]);
/// [`StepTag::new`] refuses anything larger instead of wrapping into the rank
/// bits. Every frame this release publishes is [`StepTag::UNTAGGED`].
#[repr(transparent)]
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default)]
pub struct StepTag(u64);

/// [`StepTag::new`] was given a step past [`StepTag::MAX_STEP`].
#[derive(Debug, Clone, Copy, PartialEq, Eq, thiserror::Error)]
#[error("step {step} does not fit a step tag (the largest is {max})", max = StepTag::MAX_STEP)]
pub struct StepTagOverflow {
    /// The step that did not fit.
    pub step: u64,
}

impl StepTag {
    /// The tag of a frame no step was recorded for (all bits zero).
    pub const UNTAGGED: Self = Self(0);

    /// Bits 0 to 47: the step number plus one.
    const STEP_BITS: u32 = 48;
    const STEP_MASK: u64 = (1u64 << Self::STEP_BITS) - 1;

    /// The largest step a tag can carry: `2^48 - 2`, because the field stores
    /// the step plus one and `2^48 - 1` is its largest value.
    pub const MAX_STEP: u64 = Self::STEP_MASK - 1;

    /// Tag step `step` of the producer at `rank`. Refuses a step past
    /// [`Self::MAX_STEP`].
    pub const fn new(rank: u16, step: u64) -> Result<Self, StepTagOverflow> {
        if step > Self::MAX_STEP {
            return Err(StepTagOverflow { step });
        }
        Ok(Self(((rank as u64) << Self::STEP_BITS) | (step + 1)))
    }

    /// The tag exactly as it rides the wire. Every `u64` is accepted: a value
    /// whose step field (bits 0 to 47) is zero, such as `rank << 48`, names a
    /// rank but no step, and [`Self::step`] reads it as `None`, the same as
    /// [`Self::UNTAGGED`]; [`Self::is_untagged`] is true only for all zeros.
    pub const fn from_raw(raw: u64) -> Self {
        Self(raw)
    }

    /// The tag's wire value.
    pub const fn raw(self) -> u64 {
        self.0
    }

    /// The producer rank (bits 48 to 63).
    pub const fn rank(self) -> u16 {
        (self.0 >> Self::STEP_BITS) as u16
    }

    /// The step, or `None` when the step field is zero (an untagged frame).
    pub const fn step(self) -> Option<u64> {
        match self.0 & Self::STEP_MASK {
            0 => None,
            plus_one => Some(plus_one - 1),
        }
    }

    /// True for [`Self::UNTAGGED`].
    pub const fn is_untagged(self) -> bool {
        self.0 == 0
    }
}

/// Wire message header - fixed 40 bytes.
///
/// Minimal header for zero-copy messaging. No magic/version/flags - iceoryx2
/// handles transport integrity. The layout version is stated out of band,
/// wherever frames cross a process or a file ([`WIRE_HEADER_VERSION`]).
///
/// All multi-byte fields are little-endian.
#[repr(C, align(8))]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct WireHeader {
    /// Layout-sensitive schema hash for type validation.
    ///
    /// FNV-1a 64 over a length-prefixed byte stream of the schema name,
    /// the wire fixed-section size, and every field's name + canonical
    /// type string in declaration order — see
    /// `codegen::MessageSchema::schema_hash` for the canonical recipe.
    /// Own-field layout changes (rename / reorder / retype / add /
    /// remove) change the hash. For a nested field it depends on how the
    /// nested schema resolved:
    ///
    /// - A FIXED-size nested schema is inlined in the parent's fixed section,
    ///   so its own full `schema_hash` is folded into the parent. ANY change
    ///   inside it changes the parent's hash.
    /// - A VARIABLE-size nested schema rides the offset table and
    ///   contributes only its package-qualified name. A layout change inside
    ///   it does NOT change the parent's hash.
    pub schema_hash: u64,

    /// Total message size in bytes (including header)
    pub total_size: u32,

    /// Byte offset from message start to offset table
    pub offset_table_offset: u32,

    /// Number of variable-length fields (offset table entries)
    pub offset_table_count: u32,

    /// Message sequence number (for debugging/replay)
    pub sequence: u32,

    /// Timestamp in nanoseconds (set by publisher)
    pub timestamp_ns: u64,

    /// The producer step this frame belongs to ([`StepTag`]), at byte 32.
    pub step_tag: StepTag,
}

impl WireHeader {
    /// Header size in bytes.
    pub const SIZE: usize = 40;

    /// Create a new, untagged header for the given schema.
    pub fn new(schema_hash: u64, sequence: u32, timestamp_ns: u64) -> Self {
        Self {
            schema_hash,
            total_size: 0,
            offset_table_offset: 0,
            offset_table_count: 0,
            sequence,
            timestamp_ns,
            step_tag: StepTag::UNTAGGED,
        }
    }

    /// Create a header with just the schema hash (other fields zeroed).
    pub fn with_schema(schema_hash: u64) -> Self {
        Self::new(schema_hash, 0, 0)
    }

    /// Validate that the schema hash matches expected.
    pub fn validate_schema(
        &self,
        expected_hash: u64,
        expected_name: &str,
    ) -> Result<(), WireError> {
        if self.schema_hash != expected_hash {
            return Err(WireError::SchemaMismatch {
                expected_name: expected_name.to_string(),
                expected_hash,
                actual_hash: self.schema_hash,
            });
        }
        Ok(())
    }

    /// Convert header to bytes.
    #[inline]
    pub fn as_bytes(&self) -> &[u8] {
        unsafe { std::slice::from_raw_parts(self as *const Self as *const u8, Self::SIZE) }
    }

    /// Parse header from bytes.
    ///
    /// Returns None if buffer is too small or not 8-byte aligned.
    /// For unaligned buffers (e.g., iceoryx2 shared memory), use [`read_from_buf`](Self::read_from_buf).
    #[inline]
    pub fn from_bytes(bytes: &[u8]) -> Option<&Self> {
        if bytes.len() < Self::SIZE {
            return None;
        }
        // Safety: WireHeader is repr(C) with known alignment
        if !(bytes.as_ptr() as usize).is_multiple_of(8) {
            // Unaligned - need to copy
            return None;
        }
        Some(unsafe { &*(bytes.as_ptr() as *const Self) })
    }

    /// Write header to buffer using field-by-field serialization (alignment-safe).
    ///
    /// Does NOT require 8-byte alignment. Writes exactly [`SIZE`](Self::SIZE) (40) bytes.
    /// Use this for iceoryx2 shared memory buffers where alignment is not guaranteed.
    ///
    /// # Panics
    ///
    /// Panics if `buf.len() < Self::SIZE`.
    #[inline]
    pub fn write_to_buf(&self, buf: &mut [u8]) {
        buf[0..8].copy_from_slice(&self.schema_hash.to_le_bytes());
        buf[8..12].copy_from_slice(&self.total_size.to_le_bytes());
        buf[12..16].copy_from_slice(&self.offset_table_offset.to_le_bytes());
        buf[16..20].copy_from_slice(&self.offset_table_count.to_le_bytes());
        buf[20..24].copy_from_slice(&self.sequence.to_le_bytes());
        buf[24..32].copy_from_slice(&self.timestamp_ns.to_le_bytes());
        buf[32..40].copy_from_slice(&self.step_tag.raw().to_le_bytes());
    }

    /// Read header from buffer using field-by-field deserialization (alignment-safe).
    ///
    /// Does NOT require 8-byte alignment. Reads exactly [`SIZE`](Self::SIZE) (40) bytes.
    /// Returns an owned `WireHeader` (40 bytes on stack).
    ///
    /// Returns `None` if buffer is too small.
    #[inline]
    pub fn read_from_buf(buf: &[u8]) -> Option<Self> {
        if buf.len() < Self::SIZE {
            return None;
        }
        // Counted only where a test can read it. A read path that parses the
        // same forty bytes twice costs nothing a behaviour test can see,
        // so the count is the only way to pin "parsed once per frame".
        #[cfg(any(test, feature = "test-helpers"))]
        HEADER_PARSES.fetch_add(1, core::sync::atomic::Ordering::Relaxed);
        Some(Self {
            schema_hash: u64::from_le_bytes(buf[0..8].try_into().unwrap()),
            total_size: u32::from_le_bytes(buf[8..12].try_into().unwrap()),
            offset_table_offset: u32::from_le_bytes(buf[12..16].try_into().unwrap()),
            offset_table_count: u32::from_le_bytes(buf[16..20].try_into().unwrap()),
            sequence: u32::from_le_bytes(buf[20..24].try_into().unwrap()),
            timestamp_ns: u64::from_le_bytes(buf[24..32].try_into().unwrap()),
            step_tag: StepTag::from_raw(u64::from_le_bytes(buf[32..40].try_into().unwrap())),
        })
    }
}

/// Why [`upgrade_v1_frame_into`] could not convert a frame.
#[derive(Debug, Clone, Copy, PartialEq, Eq, thiserror::Error)]
pub enum FrameUpgradeError {
    /// The source is shorter than a version 1 header.
    #[error("a version 1 frame is at least {V1_HEADER_SIZE} bytes, this one is {len}")]
    TooShort {
        /// The source length.
        len: usize,
    },
    /// The source's `total_size` is larger than the source, or smaller than a
    /// version 1 header, so no whole version 1 frame starts it.
    #[error("the version 1 frame declares total_size {declared} but holds {len} bytes")]
    SizeMismatch {
        /// The declared `total_size`.
        declared: u32,
        /// The source length.
        len: usize,
    },
    /// The upgraded frame would not fit a `u32` `total_size`.
    #[error("the upgraded frame would exceed the u32 total_size")]
    TooLarge,
    /// The destination buffer cannot hold the upgraded frame.
    #[error("the destination holds {available} bytes, the upgraded frame needs {required}")]
    DestinationTooSmall {
        /// Bytes the upgraded frame needs.
        required: usize,
        /// Bytes the destination has.
        available: usize,
    },
}

/// Convert one version 1 frame (a 32-byte header, no step tag) into the
/// current layout in `dst`, returning the upgraded length (`src.len() + 8`).
///
/// The header grows by eight bytes: the six version 1 fields keep their
/// offsets, the step tag is [`StepTag::UNTAGGED`], `total_size` grows by 8 and
/// `offset_table_offset` (measured from the start of the frame) grows by 8
/// when it points past the version 1 header. The body (fixed section, offset
/// table, variable data) is copied byte for byte: offset table entries are
/// measured from the start of the body, so none of them moves.
///
/// A source longer than its declared `total_size` (trailing bytes after the
/// frame, as a loaned slot can carry) is upgraded the same way: the frame is
/// `src[..total_size]`, and the bytes after it are copied after the upgraded
/// frame unchanged, so the output is still `src.len() + 8` long and its
/// `total_size` names the upgraded frame alone. Refused (the caller decides
/// what to do with the bytes) when the source is shorter than a version 1
/// header, or declares a `total_size` past its end or below the header size.
pub fn upgrade_v1_frame_into(src: &[u8], dst: &mut [u8]) -> Result<usize, FrameUpgradeError> {
    if src.len() < V1_HEADER_SIZE {
        return Err(FrameUpgradeError::TooShort { len: src.len() });
    }
    let declared = u32::from_le_bytes(src[8..12].try_into().unwrap());
    if (declared as usize) > src.len() || (declared as usize) < V1_HEADER_SIZE {
        return Err(FrameUpgradeError::SizeMismatch {
            declared,
            len: src.len(),
        });
    }
    let grow = WireHeader::SIZE - V1_HEADER_SIZE;
    let required = src.len() + grow;
    // `declared <= src.len()`, so a `required` that fits a u32 bounds this too.
    u32::try_from(required).map_err(|_| FrameUpgradeError::TooLarge)?;
    let total_size = declared + grow as u32;
    if dst.len() < required {
        return Err(FrameUpgradeError::DestinationTooSmall {
            required,
            available: dst.len(),
        });
    }
    let offset_table_offset = u32::from_le_bytes(src[12..16].try_into().unwrap());
    let upgraded_offset = if offset_table_offset as usize >= V1_HEADER_SIZE {
        offset_table_offset + grow as u32
    } else {
        offset_table_offset
    };
    let header = WireHeader {
        schema_hash: u64::from_le_bytes(src[0..8].try_into().unwrap()),
        total_size,
        offset_table_offset: upgraded_offset,
        offset_table_count: u32::from_le_bytes(src[16..20].try_into().unwrap()),
        sequence: u32::from_le_bytes(src[20..24].try_into().unwrap()),
        timestamp_ns: u64::from_le_bytes(src[24..32].try_into().unwrap()),
        step_tag: StepTag::UNTAGGED,
    };
    header.write_to_buf(&mut dst[..WireHeader::SIZE]);
    // The body and any trailing bytes after the declared frame, in order.
    dst[WireHeader::SIZE..required].copy_from_slice(&src[V1_HEADER_SIZE..]);
    Ok(required)
}

/// [`upgrade_v1_frame_into`] into a new buffer.
// hot-path-alloc-ok-fn: recording read path, one buffer per legacy frame, never live delivery.
pub fn upgrade_v1_frame(src: &[u8]) -> Result<Vec<u8>, FrameUpgradeError> {
    let mut out = vec![0u8; src.len().saturating_add(WireHeader::SIZE - V1_HEADER_SIZE)];
    let n = upgrade_v1_frame_into(src, &mut out)?;
    out.truncate(n);
    Ok(out)
}

/// Wire-format-bounded byte count for publisher slot sizing.
///
/// Encodes the dual invariant on `max_slice_len`:
///
/// 1. **Upper bound `u32::MAX`**: the wire format's
///    `WireHeader::total_size` field is `u32`, so a slot larger than
///    4 GiB cannot be represented on the wire.
/// 2. **Lower bound `>= WireHeader::SIZE`**: a slot must hold at least
///    the 40-byte header. Smaller values would fail at first publish
///    via `BufferTooSmall`.
///
/// Construction is fallible (`try_new`) — `None` for invalid inputs
/// (`0`, `< WireHeader::SIZE`, by-construction-impossible
/// `> u32::MAX`). The `const_new` variant panics at const-eval for
/// codegen and trait const initializers; it gives the same compile-
/// time guarantee but in a context where `?` / `Option`-handling is
/// awkward.
///
/// The lower bound is `>= WireHeader::SIZE` (40): a slot must be
/// large enough to hold the 40-byte wire header. An empty payload
/// (zero fixed fields, zero variable fields, e.g. `std_msgs::Empty`)
/// is exactly 40 bytes and is valid.
///
/// This newtype stands in for three
/// runtime guards (a resolver `u32::try_from(...).unwrap_or(u32::MAX)`
/// clamp, a `>= WireHeader::SIZE` floor check in the resolver's tier-2
/// branch, and publisher constructor `debug_assert!`s) — all
/// invariants are type-system properties at construction.
#[repr(transparent)]
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct MaxSliceLen(::std::num::NonZeroU32);

impl MaxSliceLen {
    /// Fallible constructor. Returns `None` if `n < WireHeader::SIZE`
    /// (40): a slot must be large enough to hold the wire header.
    /// `n == WireHeader::SIZE` is valid (header-only / empty payload).
    /// `n > u32::MAX` is impossible by type.
    pub const fn try_new(n: u32) -> Option<Self> {
        // `WireHeader::SIZE` is 40 (`usize`); the `as u32` is
        // compile-time-known and lossless.
        if n < WireHeader::SIZE as u32 {
            return None;
        }
        // `NonZeroU32::new` is const fn (since 1.47); guaranteed Some
        // because we just checked `n >= 40 > 0`, but we still use the
        // safe constructor for clarity.
        match ::std::num::NonZeroU32::new(n) {
            Some(nz) => Some(MaxSliceLen(nz)),
            None => None,
        }
    }

    /// `const fn` constructor that **panics at const-eval** if the
    /// argument is invalid. Use in `const` contexts (codegen-emitted
    /// trait const initializers, test fixtures) where `Option`
    /// handling is awkward and a runtime panic is the wrong shape.
    ///
    /// Building cerulion with a pathological `MaxSliceLen::const_new(8)`
    /// inside a `const` produces a compile error, not a runtime panic.
    pub const fn const_new(n: u32) -> Self {
        match Self::try_new(n) {
            Some(v) => v,
            None => panic!("MaxSliceLen::const_new requires n >= WireHeader::SIZE (40)",),
        }
    }

    /// Returns the underlying `u32` value.
    pub const fn get(self) -> u32 {
        self.0.get()
    }
}

impl ::std::fmt::Display for MaxSliceLen {
    fn fmt(&self, f: &mut ::std::fmt::Formatter<'_>) -> ::std::fmt::Result {
        ::std::fmt::Display::fmt(&self.0.get(), f)
    }
}

/// Maximum payload region size.
///
/// A `MaxPayloadCapacity` is the upper bound on the variable-payload
/// region inside a `<Name>Shm` writer — i.e., the part of the loaned
/// SHM slot AFTER the 40-byte `WireHeader`. It governs the
/// `ensure_capacity_for` / `spill_to_overflow` decision: setter writes
/// past `max_capacity` materialise as `PayloadTooLarge`, writes between
/// the current loan `self.len` and `max_capacity` trigger an in-tick
/// spill to a heap fallback.
///
/// Before this newtype, the underlying `u32` was passed bare through
/// `build_writer` / `from_bytes_mut`. Two distinct invariants lived
/// implicitly:
/// 1. Publisher path: `max_capacity == max_slice_len - WireHeader::SIZE`
///    (a derivation, not a free value).
/// 2. Reader path: `max_capacity == 0` (sentinel — no setter calls
///    allowed; if invoked anyway, `PayloadTooLarge` attributes the
///    misuse via the sentinel `<read-only>` topic).
///
/// The newtype makes the construction explicit:
/// - `MaxPayloadCapacity::from_max_slice_len(MaxSliceLen)` produces the
///   publisher-path derivation. The `MaxSliceLen` type-level floor
///   (`>= WireHeader::SIZE`) is preserved by the subtraction.
/// - `MaxPayloadCapacity::ZERO` produces the reader-side sentinel
///   (named to surface intent at the call site).
/// - `MaxPayloadCapacity::const_new(u32)` for test fixtures that need
///   to pin specific values (the `u32` cap is intrinsic to the type).
///
/// Like `MaxSliceLen`, this is `#[repr(transparent)]` over `u32` so it
/// has no runtime overhead.
#[repr(transparent)]
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct MaxPayloadCapacity(u32);

impl MaxPayloadCapacity {
    /// Reader-side sentinel: no payload write capacity. Used as the
    /// default in `<Name>Shm::from_bytes` (read-only constructor) so
    /// any setter accidentally invoked through a reader handle returns
    /// `PayloadTooLarge { topic: "<read-only>", max: 0, requested: ... }`
    /// rather than silently writing into a borrowed buffer the caller
    /// did not intend to mutate.
    pub const ZERO: Self = Self(0);

    /// Derive `max_capacity` from a `MaxSliceLen` by subtracting the
    /// 40-byte `WireHeader`. `MaxSliceLen`'s type-level floor
    /// (`>= WireHeader::SIZE`) makes the subtraction lossless — the
    /// resulting `u32` is `>= 0` for every constructible `MaxSliceLen`.
    /// The output range is `[0, u32::MAX - WireHeader::SIZE]`, which
    /// exactly matches the upper bound enforced by `try_new` /
    /// `const_new` below — so any value produced by this constructor
    /// is also constructible via the validating constructors.
    pub const fn from_max_slice_len(max_slice_len: MaxSliceLen) -> Self {
        // `WireHeader::SIZE` is 40; the `as u32` is compile-time-known.
        // `max_slice_len.get() >= WireHeader::SIZE` by `MaxSliceLen::try_new`,
        // so this subtraction never wraps.
        Self(max_slice_len.get() - WireHeader::SIZE as u32)
    }

    /// Fallible constructor. Returns `None` if `n > u32::MAX -
    /// WireHeader::SIZE` (40), i.e., values that could not have been
    /// produced by `from_max_slice_len` from any constructible
    /// `MaxSliceLen`. Mirrors `MaxSliceLen::try_new`'s bound-enforcement
    /// pattern.
    ///
    /// The bound is the post-header-subtraction max:
    ///   `MaxSliceLen::try_new(u32::MAX)` is valid → produces
    ///   `MaxPayloadCapacity = u32::MAX - 32` via `from_max_slice_len`.
    /// Values above that are pathological — only reachable through
    /// bypass (raw `Self(n)` construction is private; `try_new` +
    /// `const_new` are the only constructors).
    pub const fn try_new(n: u32) -> Option<Self> {
        // `WireHeader::SIZE` is 40; the `as u32` is compile-time-known.
        if n > u32::MAX - WireHeader::SIZE as u32 {
            return None;
        }
        Some(Self(n))
    }

    /// `const fn` constructor that **panics at const-eval** if `n`
    /// exceeds the type's upper bound (`u32::MAX - WireHeader::SIZE`).
    /// Use in `const` contexts (codegen-emitted trait const
    /// initializers, test fixtures) where `Option` handling is
    /// awkward and a runtime panic is the wrong shape.
    ///
    /// Building cerulion with a pathological
    /// `MaxPayloadCapacity::const_new(u32::MAX)` inside a `const`
    /// produces a compile error, not a runtime panic — mirroring
    /// `MaxSliceLen::const_new`'s floor-panic behavior.
    pub const fn const_new(n: u32) -> Self {
        match Self::try_new(n) {
            Some(v) => v,
            None => panic!(
                "MaxPayloadCapacity::const_new requires n <= u32::MAX - WireHeader::SIZE \
                 (40), values above this are unreachable from any constructible MaxSliceLen"
            ),
        }
    }

    /// Returns the underlying `u32` value.
    pub const fn get(self) -> u32 {
        self.0
    }
}

impl ::std::fmt::Display for MaxPayloadCapacity {
    fn fmt(&self, f: &mut ::std::fmt::Formatter<'_>) -> ::std::fmt::Result {
        ::std::fmt::Display::fmt(&self.0, f)
    }
}

/// Offset table entry for variable-length fields.
///
/// Each variable field gets one entry describing where its data lives.
#[repr(C)]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct OffsetEntry {
    /// Byte offset of the field's data measured from the start of the
    /// POST-HEADER payload (`payload[0]`, i.e. immediately after the
    /// 40-byte `WireHeader`), NOT from the start of the variable data
    /// section. This is the convention the production writer stamps
    /// (`shm_runtime::write_offset_entry` callers pass payload-relative
    /// cursors) and every reader assumes (`read_offset_entry` consumers,
    /// the frame walker).
    pub offset: u32,

    /// Length in bytes
    pub length: u32,
}

impl OffsetEntry {
    /// Size of one offset-table entry on the wire (const-asserted below).
    pub const SIZE: usize = 8;

    /// Create a new offset entry.
    pub fn new(offset: u32, length: u32) -> Self {
        Self { offset, length }
    }
}

/// The bytes of a frame that PRECEDE its variable data, for a schema whose
/// fixed section is `fixed_size` bytes and whose offset table has
/// `offset_table_entries` entries: the 40-byte [`WireHeader`], the fixed
/// section, then one [`OffsetEntry`] per variable field (the layout the
/// publisher writes — `total_size = WireHeader::SIZE + payload.len()`, the
/// payload being fixed section + offset table + variable data) — i.e.
/// `WireHeader::SIZE + WireLayout::data_floor()`, for a layout. `None` when
/// the sum overflows `usize`.
///
/// This is the ONE wire-representability rule every size fold consults
/// (the CLI engine's schema surfaces, the recorder's schema corpus): a
/// schema is representable iff even its EMPTY frame — this prefix and not a
/// single variable byte — fits the `u32` `total_size`. A ceiling stated on
/// the fixed section alone (`fixed_size <= u32::MAX`) accepts a schema whose
/// smallest frame is `u32::MAX + 32` bytes, which no publisher can emit.
pub const fn frame_prefix_size(fixed_size: usize, offset_table_entries: usize) -> Option<usize> {
    let Some(table) = offset_table_entries.checked_mul(OffsetEntry::SIZE) else {
        return None;
    };
    let Some(payload) = fixed_size.checked_add(table) else {
        return None;
    };
    payload.checked_add(WireHeader::SIZE)
}

/// True when NO frame of this shape can declare its `total_size`: the
/// prefix alone ([`frame_prefix_size`]) exceeds `u32::MAX`, or overflows
/// `usize` computing it.
pub const fn frame_prefix_exceeds_wire(fixed_size: usize, offset_table_entries: usize) -> bool {
    match frame_prefix_size(fixed_size, offset_table_entries) {
        Some(prefix) => prefix > u32::MAX as usize,
        None => true,
    }
}

/// The largest fixed section a frame carrying `offset_table_entries`
/// variable fields can declare: `u32::MAX − WireHeader::SIZE −
/// OffsetEntry::SIZE · entries`, saturating at 0 (an offset table that
/// alone fills the wire leaves no room for a fixed section). The boundary
/// [`frame_prefix_exceeds_wire`] accepts at and refuses one byte past.
pub const fn max_wire_fixed_size(offset_table_entries: usize) -> usize {
    let table = offset_table_entries.saturating_mul(OffsetEntry::SIZE);
    (u32::MAX as usize)
        .saturating_sub(WireHeader::SIZE)
        .saturating_sub(table)
}

/// Wire format errors.
#[derive(Debug, Clone, PartialEq, Eq, thiserror::Error)]
pub enum WireError {
    /// Schema hash mismatch - includes expected schema name for debugging
    #[error("schema mismatch: expected '{expected_name}' (0x{expected_hash:016X}), got 0x{actual_hash:016X}")]
    SchemaMismatch {
        expected_name: String,
        expected_hash: u64,
        actual_hash: u64,
    },

    /// Buffer too small for message
    #[error("buffer too small: need {required} bytes, have {available}")]
    BufferTooSmall { required: usize, available: usize },

    /// Invalid offset in offset table
    #[error("invalid offset: {offset} exceeds max {max}")]
    InvalidOffset { offset: u32, max: u32 },

    /// String field bytes failed UTF-8 validation.
    ///
    /// Variable-string accessors do not silently fall back to `""` on
    /// decode failure: the public API returns
    /// `Result<&str, WireError>` so users see the failure instead of
    /// reading empty strings off mangled SHM frames.
    #[error("invalid UTF-8 in variable-string field at offset {offset}, length {length}")]
    InvalidUtf8 { offset: u32, length: u32 },
}

/// Align a value up to the next 8-byte boundary.
#[inline]
pub const fn align8(size: usize) -> usize {
    (size + 7) & !7
}

/// FNV-1a 64-bit hash primitive.
///
/// This provides a fast, deterministic hash. The schema hash in
/// `WireHeader.schema_hash` is NOT `fnv1a_hash(schema_name)` — it is the
/// layout-sensitive recipe implemented in
/// `crate::codegen::MessageSchema::schema_hash`, which feeds a
/// length-prefixed byte stream (name + wire fixed size + per-field
/// name/type strings) through this primitive.
pub const fn fnv1a_hash(bytes: &[u8]) -> u64 {
    fnv1a_fold(FNV_OFFSET, bytes)
}

/// Fold `bytes` into a running FNV-1a 64 `hash`, in a `const` context.
/// Folding chunks one after another equals [`fnv1a_hash`] over their
/// concatenation when the first fold starts from [`FNV_OFFSET`].
pub(crate) const fn fnv1a_fold(mut hash: u64, bytes: &[u8]) -> u64 {
    let mut i = 0;
    while i < bytes.len() {
        hash ^= bytes[i] as u64;
        hash = hash.wrapping_mul(FNV_PRIME);
        i += 1;
    }
    hash
}

/// FNV-1a 64-bit offset basis (initial hash state).
///
/// `pub(crate)` so `crate::state::StateShape` folds `STATE_SHAPE` through the
/// SAME constants rather than carrying a second copy — one definition of the
/// hash, exactly as `MessageSchema::schema_hash` already relies on.
pub(crate) const FNV_OFFSET: u64 = 0xcbf29ce484222325;
/// FNV-1a 64-bit prime multiplier. `pub(crate)` for the same reason as
/// [`FNV_OFFSET`].
pub(crate) const FNV_PRIME: u64 = 0x100000001b3;

/// Streaming FNV-1a 64-bit accumulator.
///
/// Feeds bytes incrementally so callers building a length-prefixed byte
/// stream (e.g. [`crate::codegen::MessageSchema::schema_hash`]) can hash
/// without first assembling the whole stream into a `Vec<u8>`. The result
/// of feeding chunks `c0, c1, ..., cn` then calling [`Self::finish`] is
/// byte-for-byte identical to `fnv1a_hash(&[c0, c1, ..., cn].concat())` —
/// FNV-1a is a pure left-fold over the byte sequence, so only the order of
/// bytes matters, not how they are chunked.
pub struct Fnv1aHasher {
    hash: u64,
}

impl Fnv1aHasher {
    /// Create a hasher seeded with the FNV-1a offset basis.
    #[inline]
    pub fn new() -> Self {
        Self { hash: FNV_OFFSET }
    }

    /// Fold `bytes` into the running hash.
    #[inline]
    pub fn feed(&mut self, bytes: &[u8]) {
        for &b in bytes {
            self.hash ^= b as u64;
            self.hash = self.hash.wrapping_mul(FNV_PRIME);
        }
    }

    /// Consume the accumulated state and return the 64-bit hash.
    #[inline]
    pub fn finish(self) -> u64 {
        self.hash
    }
}

impl Default for Fnv1aHasher {
    fn default() -> Self {
        Self::new()
    }
}

// Compile-time size assertions
const _: () = {
    assert!(size_of::<WireHeader>() == 40, "WireHeader must be 40 bytes");
    assert!(size_of::<WireHeader>() == WireHeader::SIZE);
    assert!(std::mem::offset_of!(WireHeader, timestamp_ns) == 24);
    assert!(std::mem::offset_of!(WireHeader, step_tag) == V1_HEADER_SIZE);
    assert!(size_of::<StepTag>() == 8);
    assert!(size_of::<OffsetEntry>() == 8, "OffsetEntry must be 8 bytes");
    assert!(
        size_of::<OffsetEntry>() == OffsetEntry::SIZE,
        "OffsetEntry::SIZE must equal its wire size"
    );
    // The zero-entry frame ceiling and `MaxPayloadCapacity`'s upper bound are
    // ONE number — the largest payload a `u32`-sized slot can hold — stated
    // twice (a schema's smallest frame vs a writer's largest payload); this
    // pins the two statements to each other.
    assert!(max_wire_fixed_size(0) == u32::MAX as usize - WireHeader::SIZE);
    assert!(MaxPayloadCapacity::try_new(max_wire_fixed_size(0) as u32).is_some());
    assert!(MaxPayloadCapacity::try_new(max_wire_fixed_size(0) as u32 + 1).is_none());
};

// ---------------------------------------------------------------------------
// This module's half of the ABI LAYOUT PIN (see `crate::abi_layout`).
//
// `abi_pin_struct!` expands to an exhaustive destructuring pattern with no `..`
// rest pattern, so adding or removing a field of one of these structs is a
// COMPILE ERROR naming the struct and the field; it also measures
// size/align/`offset_of!`, which `crate::abi_layout` compares against the
// snapshot table keyed to `CERULION_ABI_VERSION`. `abi_pin_enum!` does the
// same for a variant set (an enum carries no stable field offsets).
// ---------------------------------------------------------------------------
/// How many times [`WireHeader::read_from_buf`] has parsed a header.
///
/// Test-visible only. A read path that parses the same thirty-two bytes two or
/// three times per frame is invisible to every behaviour test, because the extra
/// parses change no answer and only cost time. This counter is what lets a test
/// assert the rule instead of the result: ONE parse per frame delivered.
#[cfg(any(test, feature = "test-helpers"))]
static HEADER_PARSES: core::sync::atomic::AtomicU64 = core::sync::atomic::AtomicU64::new(0);

/// Read the header parse count: how many times
/// [`WireHeader::read_from_buf`] has parsed a header since the last reset.
#[cfg(any(test, feature = "test-helpers"))]
pub fn header_parse_count() -> u64 {
    HEADER_PARSES.load(core::sync::atomic::Ordering::Relaxed)
}

/// Zero the header parse count, so a test counts only its own frames.
#[cfg(any(test, feature = "test-helpers"))]
pub fn reset_header_parse_count() {
    HEADER_PARSES.store(0, core::sync::atomic::Ordering::Relaxed);
}

#[cfg(test)]
pub(crate) fn abi_layout_pins() -> Vec<crate::abi_layout::MeasuredStruct> {
    use crate::abi_layout::abi_pin_struct;
    vec![abi_pin_struct!(MaxSliceLen(len @ 0))]
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_wire_header_size() {
        assert_eq!(size_of::<WireHeader>(), 40);
    }

    #[test]
    fn test_offset_entry_size() {
        assert_eq!(size_of::<OffsetEntry>(), 8);
    }

    #[test]
    fn test_header_schema_validation_success() {
        let hash = fnv1a_hash(b"Image");
        let header = WireHeader::with_schema(hash);
        assert!(header.validate_schema(hash, "Image").is_ok());
    }

    #[test]
    fn test_header_schema_validation_mismatch() {
        let header = WireHeader::with_schema(fnv1a_hash(b"Image"));
        let err = header
            .validate_schema(fnv1a_hash(b"LaserScan"), "LaserScan")
            .unwrap_err();

        match err {
            WireError::SchemaMismatch {
                expected_name,
                expected_hash: _,
                actual_hash: _,
            } => {
                assert_eq!(expected_name, "LaserScan");
            }
            _ => panic!("expected SchemaMismatch error"),
        }
    }

    #[test]
    fn test_schema_mismatch_error_message() {
        let err = WireError::SchemaMismatch {
            expected_name: "Image".to_string(),
            expected_hash: 0x1234,
            actual_hash: 0x5678,
        };
        let msg = err.to_string();
        assert!(msg.contains("Image"));
        assert!(msg.contains("0x0000000000001234"));
        assert!(msg.contains("0x0000000000005678"));
    }

    #[test]
    fn test_align8() {
        assert_eq!(align8(0), 0);
        assert_eq!(align8(1), 8);
        assert_eq!(align8(7), 8);
        assert_eq!(align8(8), 8);
        assert_eq!(align8(9), 16);
        assert_eq!(align8(32), 32);
    }

    #[test]
    fn test_fnv1a_hash_deterministic() {
        let hash1 = fnv1a_hash(b"Image");
        let hash2 = fnv1a_hash(b"Image");
        assert_eq!(hash1, hash2);
    }

    #[test]
    fn test_fnv1a_hash_different_inputs() {
        let hash1 = fnv1a_hash(b"Image");
        let hash2 = fnv1a_hash(b"LaserScan");
        assert_ne!(hash1, hash2);
    }

    /// The streaming `Fnv1aHasher` MUST produce a byte-for-byte identical
    /// result to the one-shot `fnv1a_hash` over the concatenated stream,
    /// regardless of how the bytes are chunked. This is the invariant that
    /// lets `MessageSchema::schema_hash()` drop its intermediate `Vec<u8>`
    /// without changing any wire-contract hash value.
    #[test]
    fn test_fnv1a_hasher_streaming_matches_oneshot() {
        let chunks: &[&[u8]] = &[b"Image", b"", b"\x05\x00\x00", b"frame_id", b"string"];
        let concatenated: Vec<u8> = chunks.concat();

        let mut h = Fnv1aHasher::new();
        for c in chunks {
            h.feed(c);
        }
        assert_eq!(
            h.finish(),
            fnv1a_hash(&concatenated),
            "streaming feed of chunks must equal one-shot hash of the concatenation"
        );

        // Different chunk boundaries over the SAME bytes → same hash.
        let mut h2 = Fnv1aHasher::new();
        h2.feed(&concatenated[..3]);
        h2.feed(&concatenated[3..]);
        assert_eq!(h2.finish(), fnv1a_hash(&concatenated));

        // Empty stream → the FNV offset basis.
        assert_eq!(Fnv1aHasher::new().finish(), fnv1a_hash(b""));
    }

    #[test]
    fn test_write_to_buf_read_from_buf_roundtrip() {
        let header = WireHeader {
            schema_hash: 0xDEADBEEFCAFEBABE,
            total_size: 1024,
            offset_table_offset: 40,
            offset_table_count: 3,
            sequence: 42,
            timestamp_ns: 1_000_000_000,
            step_tag: StepTag::new(3, 9).unwrap(),
        };

        let mut buf = [0u8; WireHeader::SIZE];
        header.write_to_buf(&mut buf);

        let read_back = WireHeader::read_from_buf(&buf).expect("should parse");
        assert_eq!(read_back, header);
    }

    #[test]
    fn test_write_to_buf_unaligned() {
        // Simulate unaligned buffer by offsetting into a larger allocation
        let header = WireHeader::new(fnv1a_hash(b"Test"), 7, 999_999);

        let mut backing = [0u8; WireHeader::SIZE + 3];
        // Write at offset 1 (unaligned)
        header.write_to_buf(&mut backing[1..1 + WireHeader::SIZE]);
        let read_back =
            WireHeader::read_from_buf(&backing[1..1 + WireHeader::SIZE]).expect("should parse");
        assert_eq!(read_back, header);
    }

    #[test]
    fn test_read_from_buf_too_small() {
        let buf = [0u8; 39]; // One byte short
        assert!(WireHeader::read_from_buf(&buf).is_none());
    }

    #[test]
    fn test_write_to_buf_matches_as_bytes() {
        let header = WireHeader {
            schema_hash: 0x1234567890ABCDEF,
            total_size: 512,
            offset_table_offset: 64,
            offset_table_count: 2,
            sequence: 10,
            timestamp_ns: 5_000_000_000,
            step_tag: StepTag::UNTAGGED,
        };

        // Compare alignment-safe write with direct as_bytes
        let mut buf = [0u8; WireHeader::SIZE];
        header.write_to_buf(&mut buf);

        let direct_bytes = header.as_bytes();
        assert_eq!(&buf, direct_bytes);
    }

    // MaxPayloadCapacity newtype tests.
    #[test]
    fn test_max_payload_capacity_zero_sentinel() {
        // The reader-side default sentinel must be exactly 0 — any setter
        // accidentally invoked through a reader handle gets
        // `PayloadTooLarge { max: 0, ... }` (no allocation attempt).
        assert_eq!(MaxPayloadCapacity::ZERO.get(), 0);
    }

    #[test]
    fn test_max_payload_capacity_const_new_roundtrip() {
        // const_new accepts values in [0, u32::MAX - WireHeader::SIZE].
        // .get() returns the same value.
        for value in [0u32, 1, 32, 256, 65536, u32::MAX - WireHeader::SIZE as u32] {
            let cap = MaxPayloadCapacity::const_new(value);
            assert_eq!(
                cap.get(),
                value,
                "const_new({value}).get() must equal the input",
            );
        }
    }

    #[test]
    fn test_max_payload_capacity_try_new_accepts_within_bound() {
        // try_new returns Some for the full valid range
        // [0, u32::MAX - WireHeader::SIZE].
        let upper = u32::MAX - WireHeader::SIZE as u32;
        for value in [0u32, 1, 32, 256, 65536, upper] {
            assert_eq!(
                MaxPayloadCapacity::try_new(value).map(|c| c.get()),
                Some(value),
                "try_new({value}) must Some-construct within bound",
            );
        }
    }

    #[test]
    fn test_max_payload_capacity_try_new_rejects_above_bound() {
        // try_new returns None for values above the upper bound.
        // Bound: u32::MAX - WireHeader::SIZE (= u32::MAX - 32).
        // Boundary + above must reject.
        let above_bound = u32::MAX - WireHeader::SIZE as u32 + 1; // first invalid
        assert_eq!(MaxPayloadCapacity::try_new(above_bound), None);
        // u32::MAX is the loudest pathological value — must reject.
        assert_eq!(MaxPayloadCapacity::try_new(u32::MAX), None);
    }

    #[test]
    fn test_max_payload_capacity_const_new_compiles_at_const_eval() {
        // Pin that `const_new` is usable in const context. A future
        // change converting `const fn` to `fn` (e.g. via a non-const
        // helper) would break this assertion at compile time.
        const _UPPER: MaxPayloadCapacity =
            MaxPayloadCapacity::const_new(u32::MAX - WireHeader::SIZE as u32);
        const _ZERO_VIA_CONST_NEW: MaxPayloadCapacity = MaxPayloadCapacity::const_new(0);
        // Sanity-check both are well-formed.
        assert_eq!(_UPPER.get(), u32::MAX - WireHeader::SIZE as u32);
        assert_eq!(_ZERO_VIA_CONST_NEW.get(), 0);
    }

    #[test]
    fn test_max_payload_capacity_from_max_slice_len_subtracts_header() {
        // Publisher derivation: payload region = max_slice_len - WireHeader::SIZE.
        // Cover the lower bound (=40, payload region = 0) and a typical value.
        let min = MaxSliceLen::const_new(WireHeader::SIZE as u32);
        assert_eq!(
            MaxPayloadCapacity::from_max_slice_len(min).get(),
            0,
            "min MaxSliceLen (40) must yield zero payload capacity",
        );

        let big = MaxSliceLen::const_new(1024);
        assert_eq!(
            MaxPayloadCapacity::from_max_slice_len(big).get(),
            1024 - WireHeader::SIZE as u32,
            "from_max_slice_len must subtract WireHeader::SIZE",
        );

        // Top of the range — must not overflow.
        let top = MaxSliceLen::const_new(u32::MAX);
        assert_eq!(
            MaxPayloadCapacity::from_max_slice_len(top).get(),
            u32::MAX - WireHeader::SIZE as u32,
            "from_max_slice_len(u32::MAX) must not wrap",
        );
    }

    #[test]
    fn test_max_payload_capacity_repr_transparent_to_u32() {
        // Size + alignment must match u32 so the type is zero-cost vs the
        // bare u32 that preceded it. `#[repr(transparent)]` guarantees
        // this; the assertion pins the layout against accidental
        // wrapping (e.g. adding a phantom marker would break FFI).
        assert_eq!(
            size_of::<MaxPayloadCapacity>(),
            size_of::<u32>(),
            "MaxPayloadCapacity must be repr(transparent) over u32",
        );
        assert_eq!(
            std::mem::align_of::<MaxPayloadCapacity>(),
            std::mem::align_of::<u32>(),
            "MaxPayloadCapacity alignment must match u32",
        );
    }

    #[test]
    fn test_max_payload_capacity_display() {
        assert_eq!(format!("{}", MaxPayloadCapacity::const_new(1024)), "1024");
        assert_eq!(format!("{}", MaxPayloadCapacity::ZERO), "0");
    }

    /// The frame prefix is header + fixed section + offset table, and the
    /// wire ceiling is a BOTH-SIDES boundary on it: the largest fixed section
    /// [`max_wire_fixed_size`] names is accepted and one byte past it is
    /// refused — for a schema with no variable field (the offset table is
    /// empty, the ceiling is `u32::MAX - 40`) and for one with a single
    /// variable field (one 8-byte entry, the ceiling is `u32::MAX - 48`).
    /// A fixed section of exactly `u32::MAX` bytes (a header-blind ceiling)
    /// is refused on both shapes: its smallest frame is `u32::MAX + 40`.
    #[test]
    fn frame_prefix_ceiling_is_a_both_sides_boundary_that_counts_header_and_table() {
        const U32_MAX: usize = u32::MAX as usize;
        // No variable field: header + fixed section (hand literals, not the
        // function's own arithmetic).
        assert_eq!(max_wire_fixed_size(0), 4_294_967_255);
        assert_eq!(
            frame_prefix_size(U32_MAX - 40, 0),
            Some(U32_MAX),
            "at the ceiling the prefix IS u32::MAX"
        );
        assert!(
            !frame_prefix_exceeds_wire(U32_MAX - 40, 0),
            "at the ceiling: accepted"
        );
        assert!(
            frame_prefix_exceeds_wire(U32_MAX - 39, 0),
            "one byte past: refused"
        );
        // One variable field: header + fixed section + one offset entry.
        assert_eq!(max_wire_fixed_size(1), 4_294_967_247);
        assert!(
            !frame_prefix_exceeds_wire(U32_MAX - 48, 1),
            "at the ceiling: accepted"
        );
        assert!(
            frame_prefix_exceeds_wire(U32_MAX - 47, 1),
            "one byte past: refused"
        );
        // A header-blind ceiling (`fixed_size <= u32::MAX`) admits a frame no
        // publisher can emit.
        assert!(frame_prefix_exceeds_wire(U32_MAX, 0));
        assert!(frame_prefix_exceeds_wire(U32_MAX, 1));
        // An offset table alone can exhaust the wire; the fixed ceiling then
        // saturates at 0 and every non-empty fixed section is refused.
        let table_only = 536_870_906; // (u32::MAX - 40) / 8, remainder 7
        assert_eq!(max_wire_fixed_size(table_only), 7);
        assert_eq!(max_wire_fixed_size(table_only + 1), 0);
        assert!(frame_prefix_exceeds_wire(1, table_only + 1));
        // Arithmetic overflow computing the prefix is a refusal, never a wrap.
        assert_eq!(frame_prefix_size(usize::MAX, 0), None);
        assert!(frame_prefix_exceeds_wire(usize::MAX, 0));
        assert!(frame_prefix_exceeds_wire(0, usize::MAX));
        assert_eq!(max_wire_fixed_size(usize::MAX), 0, "saturates, never wraps");
        // The empty frame (`std_msgs/Empty`) is exactly the header.
        assert_eq!(frame_prefix_size(0, 0), Some(WireHeader::SIZE));
    }
    /// The header byte layout, against hand-written bytes: the six version 1
    /// fields at their version 1 offsets, then the step tag at byte 32.
    #[test]
    fn the_header_layout_is_the_version_1_fields_then_the_step_tag_at_byte_32() {
        let header = WireHeader {
            schema_hash: 0x0102_0304_0506_0708,
            total_size: 0x1112_1314,
            offset_table_offset: 0x2122_2324,
            offset_table_count: 0x3132_3334,
            sequence: 0x4142_4344,
            timestamp_ns: 0x5152_5354_5556_5758,
            step_tag: StepTag::from_raw(0x6162_6364_6566_6768),
        };
        let expected: [u8; 40] = [
            0x08, 0x07, 0x06, 0x05, 0x04, 0x03, 0x02, 0x01, // schema_hash
            0x14, 0x13, 0x12, 0x11, // total_size
            0x24, 0x23, 0x22, 0x21, // offset_table_offset
            0x34, 0x33, 0x32, 0x31, // offset_table_count
            0x44, 0x43, 0x42, 0x41, // sequence
            0x58, 0x57, 0x56, 0x55, 0x54, 0x53, 0x52, 0x51, // timestamp_ns
            0x68, 0x67, 0x66, 0x65, 0x64, 0x63, 0x62, 0x61, // step_tag
        ];
        let mut buf = [0u8; 40];
        header.write_to_buf(&mut buf);
        assert_eq!(buf, expected);
        assert_eq!(header.as_bytes(), &expected[..]);
        assert_eq!(WireHeader::read_from_buf(&expected), Some(header));
        assert_eq!(WireHeader::SIZE, 40);
        assert_eq!(V1_HEADER_SIZE, 32);
        assert_eq!(WIRE_HEADER_VERSION, 2);
        // A fresh header is untagged.
        assert_eq!(WireHeader::new(1, 2, 3).step_tag, StepTag::UNTAGGED);
    }

    /// The step tag packs the rank into bits 48 to 63 and the step plus one
    /// into bits 0 to 47, so zero is "untagged" and never step 0 of rank 0.
    #[test]
    fn a_step_tag_packs_rank_and_step_plus_one_and_zero_means_untagged() {
        assert_eq!(StepTag::UNTAGGED.raw(), 0);
        assert!(StepTag::UNTAGGED.is_untagged());
        assert_eq!(StepTag::UNTAGGED.step(), None);
        assert_eq!(StepTag::UNTAGGED.rank(), 0);
        assert_eq!(StepTag::default(), StepTag::UNTAGGED);

        let first = StepTag::new(0, 0).unwrap();
        assert_eq!(first.raw(), 0x0000_0000_0000_0001);
        assert!(!first.is_untagged());
        assert_eq!(first.step(), Some(0));

        let t = StepTag::new(0x0102, 0x0000_0304_0506).unwrap();
        assert_eq!(t.raw(), 0x0102_0000_0304_0507);
        assert_eq!(t.rank(), 0x0102);
        assert_eq!(t.step(), Some(0x0000_0304_0506));

        // The ends of both fields.
        assert_eq!(StepTag::MAX_STEP, 0x0000_FFFF_FFFF_FFFE);
        let top = StepTag::new(u16::MAX, StepTag::MAX_STEP).unwrap();
        assert_eq!(top.raw(), 0xFFFF_FFFF_FFFF_FFFF);
        assert_eq!(top.rank(), u16::MAX);
        assert_eq!(top.step(), Some(StepTag::MAX_STEP));
    }

    /// A step that does not fit is refused, never wrapped into the rank bits.
    #[test]
    fn a_step_past_the_largest_is_refused() {
        assert_eq!(
            StepTag::new(1, 0x0000_FFFF_FFFF_FFFF),
            Err(StepTagOverflow {
                step: 0x0000_FFFF_FFFF_FFFF
            })
        );
        assert_eq!(
            StepTag::new(0, u64::MAX),
            Err(StepTagOverflow { step: u64::MAX })
        );
        // A raw value whose step field is zero carries no step, whatever its rank.
        assert_eq!(StepTag::from_raw(0x0005_0000_0000_0000).step(), None);
        assert_eq!(StepTag::from_raw(0x0005_0000_0000_0000).rank(), 5);
    }

    /// A version 1 frame upgrades to the 40-byte layout: hand-written bytes in,
    /// hand-written bytes out. The header grows by 8 (untagged step tag),
    /// `total_size` and `offset_table_offset` grow by 8, the body is copied
    /// unchanged (offset entries are body-relative).
    #[test]
    fn a_version_1_frame_upgrades_with_an_untagged_step_tag_and_its_body_unchanged() {
        // Version 1: header (32) + fixed u32 (4) + pad (4) + one offset entry
        // (8) + 3 bytes "abc" = 51 bytes. offset_table_offset = 32 + 8 = 40.
        let v1: [u8; 51] = [
            0xEF, 0xCD, 0xAB, 0x89, 0x67, 0x45, 0x23, 0x01, // schema_hash
            51, 0, 0, 0, // total_size
            40, 0, 0, 0, // offset_table_offset
            1, 0, 0, 0, // offset_table_count
            7, 0, 0, 0, // sequence
            0x00, 0xCA, 0x9A, 0x3B, 0, 0, 0, 0, // timestamp_ns = 1e9
            0x2A, 0, 0, 0, 0, 0, 0, 0, // fixed u32 = 42, pad
            16, 0, 0, 0, 3, 0, 0, 0, // offset entry: body offset 16, len 3
            b'a', b'b', b'c',
        ];
        let want: [u8; 59] = [
            0xEF, 0xCD, 0xAB, 0x89, 0x67, 0x45, 0x23, 0x01, // schema_hash
            59, 0, 0, 0, // total_size + 8
            48, 0, 0, 0, // offset_table_offset + 8
            1, 0, 0, 0, // offset_table_count
            7, 0, 0, 0, // sequence
            0x00, 0xCA, 0x9A, 0x3B, 0, 0, 0, 0, // timestamp_ns
            0, 0, 0, 0, 0, 0, 0, 0, // step_tag: untagged
            0x2A, 0, 0, 0, 0, 0, 0, 0, // body, unchanged
            16, 0, 0, 0, 3, 0, 0, 0, b'a', b'b', b'c',
        ];
        let mut dst = [0xFFu8; 64];
        assert_eq!(upgrade_v1_frame_into(&v1, &mut dst), Ok(59));
        assert_eq!(&dst[..59], &want[..]);
        assert_eq!(&dst[59..], &[0xFF; 5], "nothing past the frame is written");
        assert_eq!(upgrade_v1_frame(&v1).unwrap(), want.to_vec());

        // A header-only version 1 frame (an empty schema) becomes a header-only
        // frame; its offset_table_offset of 0 stays 0.
        let mut empty = [0u8; 32];
        empty[8] = 32;
        let up = upgrade_v1_frame(&empty).unwrap();
        assert_eq!(up.len(), 40);
        let h = WireHeader::read_from_buf(&up).unwrap();
        assert_eq!(h.total_size, 40);
        assert_eq!(h.offset_table_offset, 0);
        assert_eq!(h.step_tag, StepTag::UNTAGGED);
    }

    /// A source carrying bytes after its declared frame upgrades the frame and
    /// keeps the trailing bytes after it: hand-written bytes in and out.
    #[test]
    fn a_version_1_frame_with_trailing_bytes_upgrades_and_keeps_them_after_the_frame() {
        // A header-only version 1 frame (total_size 32) followed by 4 bytes.
        let mut v1 = [0u8; 36];
        v1[0] = 0x11; // schema_hash low byte
        v1[8] = 32; // total_size
        v1[20] = 9; // sequence
        v1[32..].copy_from_slice(&[0xA1, 0xA2, 0xA3, 0xA4]);
        let want: [u8; 44] = [
            0x11, 0, 0, 0, 0, 0, 0, 0, // schema_hash
            40, 0, 0, 0, // total_size: the upgraded frame alone
            0, 0, 0, 0, // offset_table_offset: 0 stays 0
            0, 0, 0, 0, // offset_table_count
            9, 0, 0, 0, // sequence
            0, 0, 0, 0, 0, 0, 0, 0, // timestamp_ns
            0, 0, 0, 0, 0, 0, 0, 0, // step_tag: untagged
            0xA1, 0xA2, 0xA3, 0xA4, // the trailing bytes, after the frame
        ];
        assert_eq!(upgrade_v1_frame(&v1).unwrap(), want.to_vec());
    }

    /// Every way the source or destination can be wrong is refused.
    #[test]
    fn a_frame_that_is_not_one_whole_version_1_frame_is_refused() {
        let mut dst = [0u8; 128];
        assert_eq!(
            upgrade_v1_frame_into(&[0u8; 31], &mut dst),
            Err(FrameUpgradeError::TooShort { len: 31 })
        );
        let mut frame = [0u8; 40];
        frame[8] = 41; // declares 41, holds 40
        assert_eq!(
            upgrade_v1_frame_into(&frame, &mut dst),
            Err(FrameUpgradeError::SizeMismatch {
                declared: 41,
                len: 40
            })
        );
        frame[8] = 31; // declares less than a version 1 header
        assert_eq!(
            upgrade_v1_frame_into(&frame, &mut dst),
            Err(FrameUpgradeError::SizeMismatch {
                declared: 31,
                len: 40
            })
        );
        frame[8] = 40;
        assert_eq!(
            upgrade_v1_frame_into(&frame, &mut [0u8; 47]),
            Err(FrameUpgradeError::DestinationTooSmall {
                required: 48,
                available: 47
            })
        );
        assert_eq!(upgrade_v1_frame_into(&frame, &mut [0u8; 48]), Ok(48));
    }
}
