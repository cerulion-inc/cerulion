// SPDX-License-Identifier: AGPL-3.0-only
//! rosidl introspection → Cerulion wire format type bridge.
//!
//! The rmw layer hands us type-erased C messages (`const void *`) plus a
//! rosidl typesupport handle. Through the INTROSPECTION typesupport we
//! see the C struct's exact shape (field names/types/offsets) at
//! runtime, build the equivalent Cerulion [`MessageSchema`], and compute
//! the [`WireLayout`] — the SAME layout `native_ros2_messages`' generated
//! structs use (proven equivalent for all 220 vendored schemas in
//! `layout_equivalence_test.rs`). ROS 2 nodes on `rmw_cerulion` and
//! native Cerulion nodes therefore share topics BYTE-COMPATIBLY:
//! matching FQN schema hashes, matching fixed-section offsets, matching
//! variable-payload encodings.
//!
//! # Zero-copy scope
//!
//! - **Recursively fixed messages** (`Pose`, `Twist`, `Transform`, …):
//!   the rosidl C struct layout and the Cerulion fixed section are
//!   computed by the same `#[repr(C)]` algorithm over the same fields —
//!   when the bridge VERIFIES they coincide field-by-field
//!   ([`BridgedMessage::can_loan`]), loaned-message publish/take hand
//!   rclcpp a pointer straight into the SHM slot. TRUE zero-copy.
//! - **Variable messages** (anything with strings/sequences): ONE
//!   flatten copy from the scattered C representation DIRECTLY into
//!   the SHM loan (`frame_size` pre-pass → uninit loan →
//!   `flatten_into_uninit`; complex fields add small intermediate
//!   buffers), and one unflatten on take. No payload-sized heap
//!   allocation, no second memcpy, no CDR, no serialization step —
//!   beating `rmw_iceoryx`'s serialize fallback.
//!
//! # Variable-payload encodings (canonical v1)
//!
//! Simple variable fields match the native generated accessors exactly
//! (INTEROP-critical):
//! - `string`  → raw UTF-8 bytes (native readers `str::from_utf8`).
//! - `T[]` of primitives → raw little-endian element bytes, cursor
//!   aligned to the element alignment (native readers cast to `&[T]`).
//!
//! Complex variable fields use the CANONICAL Cerulion encodings, built
//! through `cerulion_core::codegen::element_codec` — the same routine
//! the `ros2 attach` / `dds_bridge` CDR codec calls, and the shape
//! `FrameWalker` decodes:
//! - nested VARIABLE message → the headerless sub-frame
//!   `[fixed, repr(C) padded][8N offset table][variable payloads]`,
//!   mirroring the top-level frame's payload (see
//!   `encode_message_payload`).
//! - sequence/array of FIXED nested → back-to-back fixed structs
//!   (stride = padded size).
//! - sequence/array of VARIABLE nested → `u32 count` + per element
//!   `u32 len` + element payload encoding.
//! - sequence/array of strings → `u32 count` + per element `u32 len` +
//!   UTF-8 bytes.
//!
//! # Memory contract on take (unflatten)
//!
//! rcl hands `rmw_take` an INITIALIZED message (rosidl `init` ran:
//! strings/sequences are empty-but-valid). The bridge assigns through
//! raw `libc` malloc/free — byte-compatible with rosidl's default
//! allocator, so rosidl `fini` frees what we allocate and vice versa.

use std::collections::BTreeMap;
use std::os::raw::c_void;

use cerulion_core::codegen::layout::{LayoutResolver, VariableFieldLayout, WireLayout};
use cerulion_core::codegen::{
    CanonicalBodyBuilder, CanonicalBodyReader, FieldDef, FieldType, MessageSchema,
};
use cerulion_core::wire::WireHeader;

use crate::ffi;

/// Introspection field-type ids (field_types.h numeric contract —
/// IDENTICAL numeric values for the C and C++ introspection variants).
///
/// PUBLIC so that hand-built introspection fixtures in the integration tests
/// can be DRIFT-GUARDED against this table instead of re-typing the numbers
/// by hand. A re-typed number is close to invisible: a canonical-element
/// fixture declaring `ROS_TYPE_INT32 = 6`, which is `BOOLEAN`, is something nothing
/// can see — `Time` is recursively fixed (copied wholesale, so no byte
/// moves) and a VARIABLE nested field contributes only its qualified NAME to
/// the parent's schema hash, so neither the byte oracles nor the hash gate
/// is sensitive to it.
pub mod ros_type {
    pub const FLOAT: u8 = 1;
    pub const DOUBLE: u8 = 2;
    pub const LONG_DOUBLE: u8 = 3;
    pub const CHAR: u8 = 4;
    pub const WCHAR: u8 = 5;
    pub const BOOLEAN: u8 = 6;
    pub const OCTET: u8 = 7;
    pub const UINT8: u8 = 8;
    pub const INT8: u8 = 9;
    pub const UINT16: u8 = 10;
    pub const INT16: u8 = 11;
    pub const UINT32: u8 = 12;
    pub const INT32: u8 = 13;
    pub const UINT64: u8 = 14;
    pub const INT64: u8 = 15;
    pub const STRING: u8 = 16;
    pub const WSTRING: u8 = 17;
    pub const MESSAGE: u8 = 18;
}

/// Local mirror of rosidl's `ROSIDL_RUNTIME_C_MSG_INIT_ALL`
/// (= 0, full initialization — declared defaults applied). Local because
/// the deployment bindgen allowlist (`build.rs`) carries the
/// `rosidl_runtime_c__message_initialization` TYPE (matched by
/// `allowlist_type("rosidl_.*")`) but not the `ROSIDL_RUNTIME_C_MSG_INIT_*`
/// enum constants (no matching `allowlist_var`), while the vendored
/// bindings carry both — a literal keeps both binding flavors compiling.
const ROSIDL_MSG_INIT_ALL: ffi::rosidl_runtime_c__message_initialization = 0;

/// rosidl C string: `{ data: *mut c_char, size, capacity }`.
/// Layout-identical across distros (rosidl_runtime_c/string.h).
#[repr(C)]
struct RosString {
    data: *mut u8,
    size: usize,
    capacity: usize,
}

/// rosidl C sequence header: `{ data: *mut T, size, capacity }`. This is
/// the whole struct of every generated MESSAGE sequence on every era
/// (rosidl_generator_c's msg__struct template never grew), and the common
/// prefix of every other sequence kind. Primitive and string sequences
/// (the `ROSIDL_RUNTIME_C__PRIMITIVE_SEQUENCE` macro, which `string.h`
/// uses too) append two flags from Lyrical on; see [`RosPrimitiveSequence`].
#[repr(C)]
struct RosSequence {
    data: *mut u8,
    size: usize,
    capacity: usize,
}

/// Lyrical and Rolling primitive or string sequence: the header plus
/// `is_rosidl_buffer` (when set, `data` points at a `rosidl::Buffer<T>`
/// object rather than at `T` elements) and `owns_rosidl_buffer`. A
/// Buffer-backed instance is never freed, forged, filled, or read as bytes
/// by this bridge; every path that would refuses the frame instead (see
/// [`seq_is_rosidl_buffer`]). Message sequences never carry the flags, so
/// this mirror is only ever laid over a non-message member's field.
#[cfg(cerulion_has_is_rosidl_buffer)]
#[repr(C)]
struct RosPrimitiveSequence {
    data: *mut u8,
    size: usize,
    capacity: usize,
    is_rosidl_buffer: bool,
    owns_rosidl_buffer: bool,
}

// The hand-written mirrors must be exactly the generated structs on every
// era: a mirror that lags (or leads) its era reads or writes the wrong
// bytes of every sequence field. The primitive and string sequences are
// pinned to their bindgen twins on every era; on the era that grew them,
// the message sequence is pinned to a bindgen message sequence to prove it
// did NOT grow.
#[cfg(cerulion_has_is_rosidl_buffer)]
const _: () = assert!(
    std::mem::size_of::<RosPrimitiveSequence>()
        == std::mem::size_of::<ffi::rosidl_runtime_c__uint8__Sequence>()
        && std::mem::size_of::<RosPrimitiveSequence>()
            == std::mem::size_of::<ffi::rosidl_runtime_c__String__Sequence>()
        && std::mem::size_of::<RosSequence>()
            == std::mem::size_of::<ffi::rosidl_runtime_c__type_description__Field__Sequence>(),
    "sequence mirrors must match their bindgen twins (Lyrical layout)"
);
#[cfg(not(cerulion_has_is_rosidl_buffer))]
const _: () = assert!(
    std::mem::size_of::<RosSequence>()
        == std::mem::size_of::<ffi::rosidl_runtime_c__uint8__Sequence>()
        && std::mem::size_of::<RosSequence>()
            == std::mem::size_of::<ffi::rosidl_runtime_c__String__Sequence>(),
    "sequence mirror must match its bindgen twins (one shape before Lyrical)"
);

/// Is this primitive-or-string sequence INSTANCE Buffer-backed (Lyrical
/// and Rolling: `data` points at a `rosidl::Buffer<T>` object, not at
/// elements)? `seq` MUST be a primitive or string sequence: only those
/// carry the flag (a message sequence is 24 bytes and reading a flag past
/// it would read the next field). Never true before Lyrical.
unsafe fn prim_seq_is_rosidl_buffer(seq: *const RosSequence) -> bool {
    #[cfg(cerulion_has_is_rosidl_buffer)]
    {
        (*(seq as *const RosPrimitiveSequence)).is_rosidl_buffer
    }
    #[cfg(not(cerulion_has_is_rosidl_buffer))]
    {
        let _ = seq;
        false
    }
}

/// Member-aware form of [`prim_seq_is_rosidl_buffer`]: a message sequence
/// has no flag and is never Buffer-backed.
unsafe fn seq_is_rosidl_buffer(
    member: &ffi::rosidl_typesupport_introspection_c__MessageMember,
    seq: *const RosSequence,
) -> bool {
    member.type_id_ != ros_type::MESSAGE && prim_seq_is_rosidl_buffer(seq)
}

/// Bridge-level errors. Every variant names the offending type/field.
/// Registration variants (`NoIntrospection`/`UnsupportedFieldType`/
/// `Resolution`/`LayoutMismatch`) surface at create_publisher/
/// subscription time; `Encode` surfaces at publish time for corrupt
/// message VALUES.
#[derive(Debug)]
pub enum BridgeError {
    /// The typesupport handle didn't yield introspection data.
    NoIntrospection,
    /// A field uses a type the bridge does not support.
    UnsupportedFieldType {
        message: String,
        field: String,
        type_id: u8,
    },
    /// Schema-set resolution produced warnings (must be loud).
    Resolution(Vec<String>),
    /// The computed layout disagrees with the C struct in a way that
    /// breaks the bridge's invariants (bug guard — should be unreachable
    /// because both run the same repr(C) algorithm).
    LayoutMismatch { message: String, detail: String },
    /// A message VALUE could not be encoded at publish time (corrupt
    /// sequence header: element-count × stride overflow, or total frame
    /// size beyond [`MAX_FRAME_BYTES`]). Never silent — the publish
    /// returns an error instead of panicking or emitting a torn frame.
    Encode { message: String, detail: String },
}

impl std::fmt::Display for BridgeError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::NoIntrospection => write!(f, "typesupport has no introspection data"),
            Self::UnsupportedFieldType {
                message,
                field,
                type_id,
            } => write!(
                f,
                "{message}.{field}: unsupported introspection type id {type_id} \
                 (long double / wchar / wstring are not bridged)"
            ),
            Self::Resolution(warnings) => {
                write!(f, "schema resolution warnings: {}", warnings.join("; "))
            }
            Self::LayoutMismatch { message, detail } => {
                write!(f, "layout mismatch for {message}: {detail}")
            }
            Self::Encode { message, detail } => {
                write!(f, "encode failed for {message}: {detail}")
            }
        }
    }
}

/// Hard ceiling on a single encoded wire frame (1 GiB). A corrupt C
/// sequence header (garbage `size` field) must produce an encode ERROR,
/// not an OOM abort or a multi-gigabyte allocation.
pub const MAX_FRAME_BYTES: usize = 1 << 30;

/// Detail string for the bounds-checked-cursor failure class: the write
/// pass needed a different byte count than the `frame_size` pre-pass
/// sized (a sequence/string grew or shrank between the two reads —
/// the TOCTOU window). The publish fails gracefully; no torn or
/// out-of-bounds write is possible.
pub(crate) const ERR_FRAME_SIZE_CHANGED: &str =
    "frame size changed during encode (message mutated concurrently?)";

/// Bounds-checked write cursor over a caller-provided frame buffer
/// (flatten-into-loan). This is the SHM-direct alternative to an
/// append-mode heap `Vec` flatten: instead of growing a heap buffer and
/// memcpying it into the loan, the encode writes straight into the
/// loaned slot through this cursor.
///
/// # Safety posture
///
/// A two-pass (size pass + unchecked copy pass) design can
/// write out of bounds if a sequence grows between passes. The cursor
/// makes that structurally impossible:
///
/// - EVERY write checks remaining capacity and returns
///   [`ERR_FRAME_SIZE_CHANGED`] instead of writing past the buffer.
/// - Variable lengths are re-read from the C message during the write
///   pass; a divergence from the pre-pass size surfaces as `Err`
///   (capacity exhausted, or [`Self::position`] ≠ buffer length at the
///   end), never as OOB or torn output.
/// - The buffer may be UNINITIALIZED ([`MaybeUninit<u8>`]): the cursor
///   only ever writes concrete bytes, and `len` is the watermark of
///   initialized bytes — `[0, len)` is always fully initialized
///   (construction zeroes the head region; appends initialize as they
///   advance). On success the whole buffer is initialized.
pub(crate) struct FrameCursor<'a> {
    buf: &'a mut [std::mem::MaybeUninit<u8>],
    /// Initialized-bytes watermark: `buf[..len]` is initialized.
    len: usize,
}

impl<'a> FrameCursor<'a> {
    /// Create the cursor and deterministically ZERO the head region
    /// `[0, head_len)` (WireHeader + fixed section + offset table) —
    /// the field ops then overwrite real data in place, leaving
    /// repr(C)-style gaps zeroed (Principle #7 determinism).
    pub(crate) fn new(
        buf: &'a mut [std::mem::MaybeUninit<u8>],
        head_len: usize,
    ) -> Result<Self, &'static str> {
        if head_len > buf.len() {
            return Err(ERR_FRAME_SIZE_CHANGED);
        }
        // SAFETY: in-bounds (checked above); zeroing initializes
        // [0, head_len).
        unsafe { std::ptr::write_bytes(buf.as_mut_ptr() as *mut u8, 0, head_len) };
        Ok(Self { buf, len: head_len })
    }

    /// Bytes written (initialized) so far.
    pub(crate) fn position(&self) -> usize {
        self.len
    }

    /// Append `bytes` at the watermark.
    pub(crate) fn append(&mut self, bytes: &[u8]) -> Result<(), &'static str> {
        if bytes.len() > self.buf.len() - self.len {
            return Err(ERR_FRAME_SIZE_CHANGED);
        }
        // SAFETY: in-bounds (checked above); source and destination
        // cannot overlap (the frame buffer is exclusively borrowed).
        unsafe {
            std::ptr::copy_nonoverlapping(
                bytes.as_ptr(),
                (self.buf.as_mut_ptr() as *mut u8).add(self.len),
                bytes.len(),
            );
        }
        self.len += bytes.len();
        Ok(())
    }

    /// Append `n` zero bytes (alignment padding / fill-in-place
    /// regions), returning the zeroed span for callers that overwrite
    /// it element-wise (the `vector<bool>` fetch path).
    pub(crate) fn append_zeroed(&mut self, n: usize) -> Result<&mut [u8], &'static str> {
        if n > self.buf.len() - self.len {
            return Err(ERR_FRAME_SIZE_CHANGED);
        }
        let start = self.len;
        // SAFETY: in-bounds; zeroing initializes [start, start + n).
        unsafe { std::ptr::write_bytes((self.buf.as_mut_ptr() as *mut u8).add(start), 0, n) };
        self.len += n;
        // SAFETY: just initialized.
        Ok(unsafe {
            std::slice::from_raw_parts_mut((self.buf.as_mut_ptr() as *mut u8).add(start), n)
        })
    }

    /// Overwrite ALREADY-INITIALIZED bytes at `[off, off + bytes.len())`
    /// (header stamping, fixed-section field copies, offset-table
    /// entries — all inside the pre-zeroed head).
    pub(crate) fn write_initialized_at(
        &mut self,
        off: usize,
        bytes: &[u8],
    ) -> Result<(), &'static str> {
        let end = off.checked_add(bytes.len()).ok_or(ERR_FRAME_SIZE_CHANGED)?;
        if end > self.len {
            return Err(ERR_FRAME_SIZE_CHANGED);
        }
        // SAFETY: in-bounds within the initialized region.
        unsafe {
            std::ptr::copy_nonoverlapping(
                bytes.as_ptr(),
                (self.buf.as_mut_ptr() as *mut u8).add(off),
                bytes.len(),
            );
        }
        Ok(())
    }

    /// Zero ALREADY-INITIALIZED bytes at `[off, off + n)` (repr(C)
    /// padding ranges inside a fixed-section field span).
    pub(crate) fn zero_initialized_at(&mut self, off: usize, n: usize) -> Result<(), &'static str> {
        let end = off.checked_add(n).ok_or(ERR_FRAME_SIZE_CHANGED)?;
        if end > self.len {
            return Err(ERR_FRAME_SIZE_CHANGED);
        }
        // SAFETY: in-bounds within the initialized region.
        unsafe { std::ptr::write_bytes((self.buf.as_mut_ptr() as *mut u8).add(off), 0, n) };
        Ok(())
    }

    /// Require the watermark to sit EXACTLY at the end of the buffer —
    /// the proof obligation for `assume_init` on an uninit SHM loan:
    /// `Ok` means every byte of the buffer was initialized by this
    /// cursor.
    pub(crate) fn require_full(&self) -> Result<(), &'static str> {
        if self.len == self.buf.len() {
            Ok(())
        } else {
            Err(ERR_FRAME_SIZE_CHANGED)
        }
    }
}

/// Append one variable entry through the cursor: align the payload
/// cursor (zero padding), record the `(offset, len)` table entry in the
/// pre-zeroed head, append the bytes. The cursor's bounds checks
/// stand in for a heap-append `MAX_FRAME_BYTES` gate (the buffer is
/// sized by the capped pre-pass).
pub(crate) fn cursor_append_var(
    cur: &mut FrameCursor<'_>,
    table_base: usize,
    var_idx: usize,
    bytes: &[u8],
    align: usize,
) -> Result<(), &'static str> {
    let offset = cursor_align_var(cur, align)?;
    cursor_write_var_entry(cur, table_base, var_idx, offset, bytes.len())?;
    cur.append(bytes)
}

/// Bytes before the variable-data region: `WireHeader` + fixed section +
/// offset table. The SINGLE source of this formula for both bridges'
/// size and write passes (the two must agree byte-for-byte, else
/// `require_full` rejects the frame).
pub(crate) fn frame_head_len(layout: &WireLayout) -> usize {
    WireHeader::SIZE + layout.fixed_size + layout.offset_table_bytes()
}

/// Align the payload-relative cursor to `align` (zero-padding) and
/// return the payload-relative offset the next variable entry starts at.
pub(crate) fn cursor_align_var(
    cur: &mut FrameCursor<'_>,
    align: usize,
) -> Result<usize, &'static str> {
    let payload_base = WireHeader::SIZE;
    // The cursor never sits below the header once flatten has started
    // (FrameCursor::new pre-fills the head); guard the subtraction.
    debug_assert!(
        cur.position() >= payload_base,
        "cursor must be past the WireHeader before a variable entry"
    );
    let mut offset = cur.position() - payload_base;
    if align > 1 {
        let aligned = align_up(offset, align);
        cur.append_zeroed(aligned - offset)?;
        offset = aligned;
    }
    Ok(offset)
}

/// Write one offset-table entry (`offset` is payload-relative — what
/// the read side and the native readers expect).
pub(crate) fn cursor_write_var_entry(
    cur: &mut FrameCursor<'_>,
    table_base: usize,
    var_idx: usize,
    offset: usize,
    len: usize,
) -> Result<(), &'static str> {
    // offset/len fit u32 because the frame is MAX_FRAME_BYTES-capped
    // (< 4 GiB) by the size pre-pass; guard the truncation regardless.
    debug_assert!(
        offset <= u32::MAX as usize && len <= u32::MAX as usize,
        "offset-table entry exceeds u32 (frame > 4 GiB?)"
    );
    let mut entry = [0u8; 8];
    entry[..4].copy_from_slice(&(offset as u32).to_le_bytes());
    entry[4..].copy_from_slice(&(len as u32).to_le_bytes());
    cur.write_initialized_at(table_base + var_idx * 8, &entry)
}

/// Size-pass twin of [`cursor_append_var`]: advance `size` by the
/// alignment padding + `byte_len`, enforcing [`MAX_FRAME_BYTES`].
pub(crate) fn add_var_size(
    size: usize,
    byte_len: usize,
    align: usize,
) -> Result<usize, &'static str> {
    let payload_base = WireHeader::SIZE;
    // `size` always includes the head (callers seed it with head_len);
    // guard the subtraction against a future caller that doesn't.
    debug_assert!(size >= payload_base, "var size must include the WireHeader");
    let mut cursor = size - payload_base;
    if align > 1 {
        cursor = align_up(cursor, align);
    }
    payload_base
        .checked_add(cursor)
        .and_then(|n| n.checked_add(byte_len))
        .filter(|&n| n <= MAX_FRAME_BYTES)
        .ok_or("frame exceeds MAX_FRAME_BYTES")
}

/// One flatten/unflatten operation for a top-level field.
#[derive(Debug)]
enum FieldOp {
    /// Fixed-section field: memcpy between C struct and wire fixed
    /// section. (C `bool` is 1 byte 0/1 — identical to the canonical
    /// SHM storage, so fixed regions copy verbatim.)
    FixedCopy {
        c_offset: usize,
        wire_offset: usize,
        size: usize,
        /// repr(C) padding byte-ranges WITHIN this field's span
        /// (offset, len — relative to the field start). C-side padding
        /// is uninitialized memory; copying it verbatim makes frames
        /// nondeterministic (Principle #7) and leaks process memory
        /// into SHM. Flatten zeroes these after the copy.
        /// Empty for primitives/primitive arrays.
        pad_ranges: Vec<(usize, usize)>,
    },
    /// `string` field → variable entry: raw UTF-8.
    String { c_offset: usize, var_idx: usize },
    /// Primitive sequence (`T[]`/bounded) → variable entry: raw LE
    /// element bytes, offset aligned to element alignment. Planned only
    /// for primitive element types (see [`plan_field_op`]), never for a
    /// message sequence, so every arm may read the Lyrical instance flag
    /// through [`prim_seq_is_rosidl_buffer`] in bounds, and every arm
    /// that reads the header does: a Buffer-backed instance is refused.
    PrimSeq {
        c_offset: usize,
        var_idx: usize,
        elem_size: usize,
        /// On the FORGED loaned take this member is not
        /// copied — its rosidl `{data, size, capacity}` header is aimed at
        /// the held SHM sample's bytes (see [`is_forgeable_sequence`]).
        /// The copying take (`unflatten`) ignores the flag.
        forge: bool,
    },
    // NOTE: an inline primitive array (`T[N]`) the wire classifies as
    // variable cannot happen — fixed arrays of primitives are fixed
    // section by construction.
    /// Complex variable field (nested variable message, or any
    /// array/sequence of nested/string). `member_index` points back into
    /// the introspection members for recursive encoding.
    Complex {
        c_offset: usize,
        var_idx: usize,
        member_index: usize,
    },
}

/// Wire layouts for every transitively nested message type, keyed by its
/// rosidl introspection `MessageMembers` POINTER.
///
/// The bridge builds a `LayoutResolver` over every reachable nested
/// schema and RETAINS it, not only the root layout — without it
/// the nested encoder would have no offsets to write to and would need a packed format
/// instead. Retaining it costs one registration-time walk and makes the nested
/// body encoder use the SAME canonical layout the top level does.
///
/// Keyed by pointer rather than qualified name so the per-element hot path is
/// an integer map lookup with no `format!` per element. rosidl introspection
/// data is `'static` per type, so the pointer is a stable identity.
#[derive(Debug, Default)]
pub struct NestedLayouts {
    by_members: BTreeMap<usize, WireLayout>,
}

impl NestedLayouts {
    /// The layout for a nested type, by its introspection pointer.
    #[inline]
    pub fn get(
        &self,
        members: *const ffi::rosidl_typesupport_introspection_c__MessageMembers,
    ) -> Option<&WireLayout> {
        self.by_members.get(&(members as usize))
    }

    /// The layout for a nested C++ introspection type. Same map, different
    /// pointer type — the two typesupport ABIs never share a process-wide
    /// pointer space in one `BridgedMessage`/`CppBridgedMessage`, and each
    /// bridge only ever looks up pointers it registered itself.
    #[inline]
    pub fn get_cpp<T>(&self, members: *const T) -> Option<&WireLayout> {
        self.by_members.get(&(members as usize))
    }

    /// Register a nested type's layout under its introspection pointer.
    /// Called only from the two bridges' registration paths.
    #[inline]
    pub fn insert(&mut self, members_ptr: usize, layout: WireLayout) {
        self.by_members.insert(members_ptr, layout);
    }

    /// Number of distinct introspection types carrying a layout.
    #[inline]
    pub fn len(&self) -> usize {
        self.by_members.len()
    }

    /// True when no nested type was registered.
    #[inline]
    pub fn is_empty(&self) -> bool {
        self.by_members.is_empty()
    }
}

// SAFETY: the keys are `usize` copies of pointers into rosidl's immutable,
// process-lifetime static data; nothing is dereferenced through the map.
unsafe impl Send for NestedLayouts {}
unsafe impl Sync for NestedLayouts {}

/// A message type registered with the bridge.
pub struct BridgedMessage {
    /// Qualified Cerulion name ("geometry_msgs/Pose").
    pub qualified_name: String,
    /// Wire layout (hash, fixed offsets, variable order).
    pub layout: WireLayout,
    /// `size_of` the rosidl C struct.
    pub c_size: usize,
    /// True when the C struct IS the wire fixed section byte-for-byte —
    /// the loaned-message zero-copy path is enabled. Verified
    /// field-by-field at registration, never assumed.
    pub can_loan: bool,
    /// Root introspection members (borrowed from the typesupport's
    /// static data — 'static for the process lifetime by rosidl
    /// contract).
    members: *const ffi::rosidl_typesupport_introspection_c__MessageMembers,
    /// Flatten/unflatten plan for top-level fields, declaration order.
    ops: Vec<FieldOp>,
    /// Wire layouts for every transitively nested type, so a nested
    /// / element body is encoded in the CANONICAL `[fixed][table][var]` shape
    /// instead of a bespoke packed one.
    nested_layouts: NestedLayouts,
    /// Repr(C) padding byte-ranges of the WHOLE loanable struct
    /// (offset, len — relative to the payload / fixed-section start):
    /// intra-field padding, inter-field alignment gaps and tail padding.
    /// Computed once at registration (the same walk as
    /// `zero_struct_padding`, hoisted so publish does no per-frame walk or
    /// allocation); EMPTY unless [`Self::can_loan`]. The loaned publish
    /// path zeroes these ranges before send — the caller's C-side writes
    /// can deposit process-memory garbage in padding (a whole-struct
    /// assignment copies padding bytes), and frames must be deterministic
    /// (Principle #7) and must not leak process memory into SHM.
    loan_pad_ranges: Vec<(usize, usize)>,
    /// How many top-level members the FORGED loaned take
    /// aims at the held SHM sample instead of copying (the
    /// [`FieldOp::PrimSeq`] ops with `forge == true`). Zero when the type
    /// is not eligible for the shadow take at all — see
    /// [`Self::can_loan_take`].
    forge_count: usize,
}

// SAFETY: `members` points at rosidl's static typesupport data, which is
// immutable and process-lifetime; the bridge only reads through it.
unsafe impl Send for BridgedMessage {}
unsafe impl Sync for BridgedMessage {}

impl BridgedMessage {
    /// Build the bridge for a message typesupport's INTROSPECTION data.
    ///
    /// # Safety
    /// `members` must point at valid, process-lifetime introspection
    /// data (the rosidl typesupport contract).
    pub unsafe fn new(
        members: *const ffi::rosidl_typesupport_introspection_c__MessageMembers,
    ) -> Result<Self, BridgeError> {
        if members.is_null() {
            return Err(BridgeError::NoIntrospection);
        }

        // 1. Collect the schema set: this message + every transitively
        //    reachable nested message type.
        let mut schemas: BTreeMap<String, MessageSchema> = BTreeMap::new();
        let mut type_ptrs: BTreeMap<usize, String> = BTreeMap::new();
        collect_schemas(members, &mut schemas, &mut type_ptrs)?;
        let root_qualified = qualified_name_of(members)?;

        // 2. Resolve + compute the wire layout (the SAME pipeline the
        //    native codegen runs at build time).
        let schema_vec: Vec<MessageSchema> = schemas.into_values().collect();
        let (mut resolver, warnings) = LayoutResolver::new(schema_vec);
        if !warnings.is_empty() {
            return Err(BridgeError::Resolution(warnings));
        }
        let layout = resolver
            .layout_of(&root_qualified)
            .ok_or(BridgeError::NoIntrospection)?;

        // 2b. RETAIN a layout per nested introspection type instead
        //     of discarding the resolver. This is what lets the nested/element
        //     body encoder write the canonical `[fixed][table][var]` shape.
        let mut nested_layouts = NestedLayouts::default();
        for (ptr, qname) in &type_ptrs {
            let l = resolver
                .layout_of(qname)
                .ok_or(BridgeError::NoIntrospection)?;
            // Verify the SAME fixed/variable lockstep + size agreement the
            // root is checked for below. Without this, a per-element encode
            // would index `layout.fixed_fields` against a differently-ordered
            // introspection walk and silently misplace bytes.
            verify_nested_lockstep(
                *ptr as *const ffi::rosidl_typesupport_introspection_c__MessageMembers,
                qname,
                &l,
            )?;
            nested_layouts.insert(*ptr, l);
        }

        // 3. Build the per-field plan and verify offsets.
        let m = &*members;
        let c_size = m.size_of_;
        let member_slice = std::slice::from_raw_parts(m.members_, m.member_count_ as usize);

        let mut ops = Vec::with_capacity(member_slice.len());
        let mut fixed_iter = layout.fixed_fields.iter().peekable();
        let mut var_idx = 0usize;
        let mut loanable = layout.is_fixed() && layout.fixed_size == c_size;
        let mut forge_count = 0usize;

        for (i, member) in member_slice.iter().enumerate() {
            let fname = ffi::cstr(member.name_).unwrap_or("<field>").to_string();
            let c_offset = member.offset_ as usize;
            let is_fixed_array =
                member.is_array_ && member.array_size_ > 0 && !member.is_upper_bound_;

            let classified_variable = is_variable_member(member);
            if classified_variable {
                // Which variable entry is this? Declaration order match —
                // a mismatch means the layout and introspection disagree
                // about the schema and NOTHING downstream is trustworthy.
                let v = layout.variable_fields.get(var_idx).ok_or_else(|| {
                    BridgeError::LayoutMismatch {
                        message: root_qualified.clone(),
                        detail: format!("variable field '{fname}' missing from layout"),
                    }
                })?;
                if v.name != fname {
                    return Err(BridgeError::LayoutMismatch {
                        message: root_qualified.clone(),
                        detail: format!(
                            "variable-field order mismatch: layout '{}' vs introspection '{fname}'",
                            v.name
                        ),
                    });
                }
                let forge = is_forgeable_sequence(member, &v.field_type);
                let op = plan_variable_op(c_offset, var_idx, i, v, forge);
                if matches!(op, FieldOp::PrimSeq { forge: true, .. }) {
                    forge_count += 1;
                }
                ops.push(op);
                var_idx += 1;
                loanable = false;
            } else {
                // Fixed-section field: wire offset from the layout.
                let fl = fixed_iter
                    .next()
                    .ok_or_else(|| BridgeError::LayoutMismatch {
                        message: root_qualified.clone(),
                        detail: format!("fixed field '{fname}' missing from layout"),
                    })?;
                if fl.name != fname {
                    return Err(BridgeError::LayoutMismatch {
                        message: root_qualified.clone(),
                        detail: format!(
                            "fixed-field order mismatch: layout '{}' vs introspection '{fname}'",
                            fl.name
                        ),
                    });
                }
                // The loan fast-path requires the C offset to EQUAL the
                // wire offset (same repr(C) algorithm ⇒ should always
                // hold; verified, never assumed).
                if fl.offset != c_offset {
                    loanable = false;
                }
                let _ = is_fixed_array;
                // Size disagreement would memcpy OOB from the C struct
                // AND misplace pad_ranges (zeroing real data) — hard
                // error, same posture as the order check above.
                let c_field_size = fixed_member_size(member);
                if fl.size != c_field_size {
                    return Err(BridgeError::LayoutMismatch {
                        message: root_qualified.clone(),
                        detail: format!(
                            "fixed-field size mismatch for '{fname}': layout {} vs C {}",
                            fl.size, c_field_size
                        ),
                    });
                }
                let size = fl.size;
                // Padding map: ranges within this span not
                // covered by real (leaf-primitive) data.
                let mut runs = Vec::new();
                member_data_runs(member, 0, &mut runs);
                let pad_ranges = padding_ranges(&mut runs, size);
                ops.push(FieldOp::FixedCopy {
                    c_offset,
                    wire_offset: fl.offset,
                    size,
                    pad_ranges,
                });
            }
        }

        // Whole-struct padding map for the loaned publish path
        // (see the `loan_pad_ranges` field doc). Only meaningful — and
        // only computed — when the type is loanable.
        let loan_pad_ranges = if loanable {
            let mut runs = Vec::new();
            for nm in member_slice {
                member_data_runs(nm, nm.offset_ as usize, &mut runs);
            }
            padding_ranges(&mut runs, c_size)
        } else {
            Vec::new()
        };

        // The shadow take constructs and destroys rmw-owned
        // message objects through the typesupport's OWN init/fini — a C
        // typesupport missing either cannot host a shadow (its strings and
        // nested containers would never be freed on the matching allocator),
        // so the type keeps the copying take. Real rosidl typesupports always
        // carry both; this is a hand-built-fixture / drift guard.
        if forge_count > 0 && (m.init_function.is_none() || m.fini_function.is_none()) {
            tracing::debug!(
                schema = %root_qualified,
                "typesupport lacks init_function/fini_function; forged loaned take disabled \
                 for this type (copying take instead)"
            );
            forge_count = 0;
            for op in &mut ops {
                if let FieldOp::PrimSeq { forge, .. } = op {
                    *forge = false;
                }
            }
        }

        Ok(Self {
            qualified_name: root_qualified,
            layout,
            c_size,
            can_loan: loanable,
            members,
            ops,
            nested_layouts,
            loan_pad_ranges,
            forge_count,
        })
    }

    /// Wire schema hash (FNV-1a of the qualified name — identical to the
    /// native generated `SCHEMA_HASH`).
    pub fn schema_hash(&self) -> u64 {
        self.layout.schema_hash
    }

    /// Can `rmw_take_loaned_message` serve this type
    /// WITHOUT a payload copy? Either the whole message is the wire fixed
    /// section ([`Self::can_loan`] — the pointer handed out aims straight
    /// into SHM), or at least one top-level member is a FORGEABLE primitive
    /// sequence (`is_forgeable_sequence`, private — the rule is documented there) whose bytes stay in the held SHM
    /// sample while the rmw-owned SHADOW message carries the small copied
    /// remainder (fixed fields, strings, nested messages). The publish-side
    /// rule stays [`Self::can_loan`]: a borrow must hand out a whole
    /// SHM-resident struct, which only a fixed type is.
    pub fn can_loan_take(&self) -> bool {
        self.can_loan || self.forge_count > 0
    }

    /// Number of top-level members the forged take aims at SHM (zero on a
    /// fixed type — its take needs no shadow at all).
    pub fn forged_sequence_count(&self) -> usize {
        self.forge_count
    }

    /// Construct one rmw-owned SHADOW message on the heap
    /// — `c_size` bytes at `SHADOW_ALIGN` (16), zeroed, then the typesupport's
    /// `init_function` with `ALL` (the same two-step the loaned borrow uses:
    /// rosidl's C `__init` writes only defaulted members, so the zero
    /// baseline is load-bearing). `None` only on allocation failure. The
    /// object is destroyed by [`Self::destroy_shadow`], never by rosidl's
    /// `__destroy`.
    ///
    /// # Safety
    /// [`Self::can_loan_take`] must hold with a nonzero
    /// [`Self::forged_sequence_count`] (which guarantees the typesupport has
    /// an `init_function` and a `fini_function`).
    pub unsafe fn new_shadow(&self) -> Option<*mut c_void> {
        let layout = shadow_layout(self.c_size);
        // hot-path-alloc-ok: shadows are built at most once per borrow-budget
        // slot per subscription and then RECYCLED; a steady-state take never
        // reaches this.
        let ptr = std::alloc::alloc_zeroed(layout);
        if ptr.is_null() {
            return None;
        }
        if let Some(init) = (*self.members).init_function {
            init(ptr as *mut c_void, ROSIDL_MSG_INIT_ALL);
        }
        Some(ptr as *mut c_void)
    }

    /// Destroy a shadow from [`Self::new_shadow`]: un-forge (so the
    /// typesupport's `fini_function` sees empty sequences and frees nothing
    /// that lives in shared memory), `fini`, deallocate.
    ///
    /// `forged` is the mask of the last take served through this shadow
    /// ([`ForgeOutcome::forged`]; `0` for a never-forged or already un-forged
    /// shadow) — members outside it hold copies `fini` must free.
    ///
    /// # Safety
    /// `shadow` must come from [`Self::new_shadow`] on this bridge and must
    /// not be used afterwards.
    pub unsafe fn destroy_shadow(&self, shadow: *mut c_void, forged: u64) {
        self.unforge(shadow, forged);
        if let Some(fini) = (*self.members).fini_function {
            fini(shadow);
        }
        std::alloc::dealloc(shadow as *mut u8, shadow_layout(self.c_size));
    }

    /// The FORGED take: unflatten `payload` (the post-`WireHeader` bytes of
    /// a HELD SHM sample) into `shadow` — fixed members, strings and nested
    /// messages are COPIED exactly as [`Self::unflatten`] copies them, while
    /// every forgeable primitive sequence is FORGED: its rosidl
    /// `{data, size, capacity}` header is aimed at the entry's bytes inside
    /// `payload` with `capacity == size`. No payload byte moves.
    ///
    /// The PLACEMENT rule (`forge_placement`, private): an entry is forged only at
    /// or above the frame's data floor (`WireLayout::data_floor`, the rule
    /// the `FrameWalker` enforces). An entry BELOW it aliases the fixed
    /// section or the offset table, so that member is COPIED instead —
    /// exactly the bytes the copying take serves — and counted in
    /// [`ForgeOutcome::below_floor`]; the frame still succeeds. An EMPTY
    /// entry is always forged to the empty header (nothing to alias).
    ///
    /// All-or-nothing for the ALIASES: `Err` (malformed frame — bad entry, a
    /// length that is not a multiple of the element size, a forged pointer
    /// that is not element-aligned) leaves the shadow UN-FORGED (`forged ==
    /// 0`). It carries the partial outcome because COPIES made before the
    /// failure (below-floor members) still live in the shadow: a nonzero
    /// `below_floor` on the `Err` means the caller must RETIRE the shadow
    /// (its `fini` frees them) rather than recycle it.
    ///
    /// # Safety
    /// `shadow` must come from [`Self::new_shadow`]; `payload` must be the
    /// held sample's own bytes and the sample must outlive every use of the
    /// forged members (the caller un-forges via [`Self::unforge`] with the
    /// returned mask BEFORE releasing the sample).
    pub unsafe fn unflatten_forged(
        &self,
        payload: &[u8],
        shadow: *mut c_void,
    ) -> Result<ForgeOutcome, ForgeOutcome> {
        if payload.len() < self.layout.fixed_size + self.layout.offset_table_bytes() {
            tracing::warn!(
                message = %self.qualified_name,
                payload_len = payload.len(),
                "wire payload too short for fixed section + offset table"
            );
            return Err(ForgeOutcome::default());
        }
        let table_base = self.layout.fixed_size;
        let data_floor = self.layout.data_floor();
        let mut outcome = ForgeOutcome::default();
        let mut forge_idx = 0usize;
        for op in &self.ops {
            let ok = match op {
                FieldOp::FixedCopy {
                    c_offset,
                    wire_offset,
                    size,
                    pad_ranges: _,
                } => {
                    std::ptr::copy_nonoverlapping(
                        payload.as_ptr().add(*wire_offset),
                        (shadow as *mut u8).add(*c_offset),
                        *size,
                    );
                    true
                }
                FieldOp::String { c_offset, var_idx } => {
                    match read_var_entry(payload, table_base, *var_idx) {
                        Some(bytes) => {
                            assign_ros_string(shadow.add(*c_offset) as *mut RosString, bytes)
                        }
                        None => false,
                    }
                }
                FieldOp::PrimSeq {
                    c_offset,
                    var_idx,
                    elem_size,
                    forge: false,
                } => match read_var_entry(payload, table_base, *var_idx) {
                    Some(bytes) if bytes.len() % elem_size == 0 => assign_prim_sequence(
                        shadow.add(*c_offset) as *mut RosSequence,
                        bytes,
                        *elem_size,
                    ),
                    _ => false,
                },
                FieldOp::PrimSeq {
                    c_offset,
                    var_idx,
                    elem_size,
                    forge: true,
                } => {
                    let bit = forge_idx;
                    forge_idx += 1;
                    let seq = shadow.add(*c_offset) as *mut RosSequence;
                    match read_var_entry(payload, table_base, *var_idx) {
                        // An empty entry has nothing to alias: the empty header,
                        // recorded as forged so the un-forge stays uniform.
                        Some([]) => {
                            *seq = EMPTY_ROS_SEQUENCE;
                            outcome.forged |= mask_bit(bit);
                            true
                        }
                        Some(bytes) if bytes.len().is_multiple_of(*elem_size) => {
                            match forge_placement(payload, bytes, *elem_size, data_floor, bit) {
                                ForgePlacement::Forge => {
                                    let seq = &mut *seq;
                                    seq.data = bytes.as_ptr() as *mut u8;
                                    seq.size = bytes.len() / elem_size;
                                    seq.capacity = seq.size;
                                    outcome.forged |= mask_bit(bit);
                                    true
                                }
                                ForgePlacement::Copy { below_floor } => {
                                    outcome.below_floor += usize::from(below_floor);
                                    assign_prim_sequence(seq, bytes, *elem_size)
                                }
                                ForgePlacement::Malformed => false,
                            }
                        }
                        _ => false,
                    }
                }
                FieldOp::Complex {
                    c_offset,
                    var_idx,
                    member_index,
                } => match read_var_entry(payload, table_base, *var_idx) {
                    Some(bytes) => decode_complex(
                        &self.nested_layouts,
                        self.member(*member_index),
                        bytes,
                        shadow.add(*c_offset),
                    ),
                    None => false,
                },
            };
            if !ok {
                // All-or-nothing for the ALIASES: nothing forged survives.
                // COPIES made before the failure (below-floor members) do
                // survive, and the `Err` carries their count so the caller
                // retires the shadow instead of recycling it — a recycled
                // shadow's next forge would overwrite that header and leak
                // the buffer.
                self.unforge(shadow, outcome.forged);
                outcome.forged = 0;
                warn_bad_entry(&self.qualified_name, op_var_idx(op));
                return Err(outcome);
            }
        }
        Ok(outcome)
    }

    /// Re-point every sequence header in `forged` (a [`ForgeOutcome::forged`]
    /// mask) at nothing — `{NULL, 0, 0}`, the rosidl empty state — so nothing
    /// in the shadow references the sample any more. Members OUTSIDE the mask
    /// are untouched: they hold copies the typesupport's `fini` owns and
    /// frees. Idempotent; a zero mask is a no-op. MUST run before the sample
    /// it aliased is released and before [`Self::destroy_shadow`] (whose
    /// `fini` would otherwise `free()` a shared-memory address).
    ///
    /// # Safety
    /// `shadow` must come from [`Self::new_shadow`] on this bridge.
    pub unsafe fn unforge(&self, shadow: *mut c_void, forged: u64) {
        let mut forge_idx = 0usize;
        for op in &self.ops {
            if let FieldOp::PrimSeq {
                c_offset,
                forge: true,
                ..
            } = op
            {
                if mask_has(forged, forge_idx) {
                    *(shadow.add(*c_offset) as *mut RosSequence) = EMPTY_ROS_SEQUENCE;
                }
                forge_idx += 1;
            }
        }
    }

    /// True when every forgeable sequence header of `shadow` selected by
    /// `mask` is the un-forged `{NULL, 0, 0}` (`u64::MAX` selects them all —
    /// the state [`Self::new_shadow`] starts from). The observable the
    /// take-side tests pin the un-forge-before-release contract on.
    ///
    /// # Safety
    /// `shadow` must come from [`Self::new_shadow`] on this bridge.
    pub unsafe fn forged_members_are_empty(&self, shadow: *const c_void, mask: u64) -> bool {
        let mut forge_idx = 0usize;
        self.ops.iter().all(|op| match op {
            FieldOp::PrimSeq {
                c_offset,
                forge: true,
                ..
            } => {
                let selected = mask_has(mask, forge_idx);
                forge_idx += 1;
                if !selected {
                    return true;
                }
                let seq = &*(shadow.add(*c_offset) as *const RosSequence);
                seq.data.is_null() && seq.size == 0 && seq.capacity == 0
            }
            _ => true,
        })
    }

    /// Adopt-take (`--adopt-take`): copy the members selected by
    /// `forged` (a [`ForgeOutcome::forged`] mask) from the wire into `msg` —
    /// exactly the bytes [`Self::unflatten`] would serve for them. The
    /// registration-failure rollback runs this AFTER [`Self::unforge`], so
    /// every masked header is EMPTY and `assign_prim_sequence` frees
    /// nothing pre-existing. Deliberately NOT a re-run of the whole
    /// `unflatten` over the partially-populated message: only the masked
    /// `PrimSeq` copy arm executes, so no string/sequence member is
    /// double-assigned. Returns `false` on allocation failure or an
    /// unreadable entry (both unreachable for a payload a successful
    /// `unflatten_forged` just walked — kept as a loud refusal anyway).
    ///
    /// # Safety
    /// `msg` must be a valid, initialized message of this bridged type
    /// whose masked members are currently EMPTY (un-forged); `payload` must
    /// be the same post-header bytes the preceding `unflatten_forged`
    /// walked.
    pub unsafe fn copy_forged_members(
        &self,
        payload: &[u8],
        msg: *mut c_void,
        forged: u64,
    ) -> bool {
        let table_base = self.layout.fixed_size;
        let mut forge_idx = 0usize;
        for op in &self.ops {
            if let FieldOp::PrimSeq {
                c_offset,
                var_idx,
                elem_size,
                forge: true,
            } = op
            {
                let bit = forge_idx;
                forge_idx += 1;
                if !mask_has(forged, bit) {
                    continue;
                }
                let seq = msg.add(*c_offset) as *mut RosSequence;
                let ok = match read_var_entry(payload, table_base, *var_idx) {
                    Some(bytes) if bytes.len().is_multiple_of(*elem_size) => {
                        assign_prim_sequence(seq, bytes, *elem_size)
                    }
                    _ => false,
                };
                if !ok {
                    return warn_bad_entry(&self.qualified_name, *var_idx);
                }
            }
        }
        true
    }

    /// Adopt-take: the exact `{address, len}` byte range each FORGED
    /// member of `msg` aims at — read back from the container headers
    /// [`Self::unflatten_forged`] just wrote, because the pointer the app
    /// will eventually `free()` is the pointer that must be registered
    /// (the hook's granularity contract: exact per-field sub-ranges, never
    /// the enclosing sample). Members whose forged header is EMPTY (an
    /// empty wire entry) alias nothing, so their free is already a rosidl
    /// no-op and they are skipped.
    ///
    /// # Safety
    /// `msg` must be a valid message of this bridged type whose members in
    /// `forged` were just forged by [`Self::unflatten_forged`].
    pub unsafe fn forged_entry_ranges(
        &self,
        msg: *const c_void,
        forged: u64,
        out: &mut Vec<(usize, usize)>,
    ) {
        let mut forge_idx = 0usize;
        for op in &self.ops {
            if let FieldOp::PrimSeq {
                c_offset,
                elem_size,
                forge: true,
                ..
            } = op
            {
                let bit = forge_idx;
                forge_idx += 1;
                if !mask_has(forged, bit) {
                    continue;
                }
                let seq = std::ptr::read_unaligned(msg.add(*c_offset) as *const RosSequence);
                if !seq.data.is_null() && seq.size > 0 {
                    out.push((seq.data as usize, seq.size * *elem_size));
                }
            }
        }
    }

    /// Can this frame's VARIABLE entries be resolved at
    /// all, WITHOUT writing anything?
    ///
    /// The adopted take runs this before its first write to the caller's
    /// message. Both decodes below (`unflatten` and `unflatten_forged`) walk
    /// the ops writing member by member and bail at the first unreadable
    /// entry, which leaves a caller that legally reuses one message across
    /// takes holding a CHIMERA — some members from the new frame, the rest
    /// from the old one — on a call that reported nothing taken. Answering
    /// the same question first, read-only, means a malformed frame is
    /// refused with the caller's message BYTE-UNTOUCHED.
    ///
    /// Totality: this closes the ENTRY class (an
    /// offset-table entry that does not resolve, and a primitive sequence
    /// whose length is not a whole number of elements) for every member of
    /// the type. Three failure classes remain and still fail mid-decode,
    /// exactly as the plain copying take does: an ALLOCATION failure inside
    /// a copy arm; a malformed BODY inside a nested member (its top-level
    /// entry resolves, its interior is only checked as it is decoded); and a
    /// forged entry whose start address is not element-aligned
    /// (`ForgePlacement::Malformed`), which is arm-dependent — the copying
    /// decode serves such an entry rather than failing, so refusing it here
    /// would DROP a frame the over-retention arm can deliver.
    ///
    /// Read-only and allocation-free: the per-entry decision is the shared
    /// pure [`crate::take_gate::var_entry_decodable`], so the C and C++
    /// bridges cannot drift apart on what "malformed" means.
    ///
    /// `Err` carries the offending member's `var_idx` and the verdict, so
    /// the refusal line names WHICH member and WHY — the decode's own
    /// failure arm logs `var_idx` through `warn_bad_entry`, and a gate that
    /// answered a bare `bool` would front-run it and leave the operator
    /// with strictly less.
    pub fn frame_entries_readable(
        &self,
        payload: &[u8],
    ) -> Result<(), (usize, crate::take_gate::EntryVerdict)> {
        if payload.len() < self.layout.fixed_size + self.layout.offset_table_bytes() {
            // Too short for the fixed section plus the table: there is no
            // entry to blame, so the whole frame is reported as entry 0's
            // out-of-bounds — which is what it is.
            return Err((0, crate::take_gate::EntryVerdict::EntryOutOfBounds));
        }
        let table_base = self.layout.fixed_size;
        for op in &self.ops {
            let (var_idx, elem_size) = match op {
                // A fixed span reads inside the region the length check
                // above already covered.
                FieldOp::FixedCopy { .. } => continue,
                FieldOp::String { var_idx, .. } | FieldOp::Complex { var_idx, .. } => {
                    (*var_idx, None)
                }
                FieldOp::PrimSeq {
                    var_idx, elem_size, ..
                } => (*var_idx, Some(*elem_size)),
            };
            let entry = read_var_entry(payload, table_base, var_idx);
            let verdict = crate::take_gate::var_entry_decodable(entry, elem_size);
            if !verdict.is_decodable() {
                return Err((var_idx, verdict));
            }
        }
        Ok(())
    }

    /// Adopt-take: free + EMPTY every forge-flagged member's existing
    /// storage in a CALLER-owned message, BEFORE an adopting
    /// `unflatten_forged` runs over it. The plain `unflatten` frees a
    /// reused message's previous allocation inside its copy arms; the forge
    /// arm OVERWRITES the header instead, so without this pre-pass a caller
    /// that legally reuses one initialized message across takes would leak
    /// the previous take's buffer — and, worse, a previously-FORGED header
    /// would lose the only pointer that can ever release its sample,
    /// pinning a borrow slot forever. Freeing here is correct for both
    /// shapes: a heap buffer goes to the allocator, and a previously-forged
    /// SHM pointer routes through the preloaded hook's interposed `free` to
    /// a release (this helper is only reachable under the adopt grant,
    /// which requires the preload).
    ///
    /// # Safety
    /// `msg` must be a valid, initialized message of this bridged type.
    pub unsafe fn release_forgeable_members(&self, msg: *mut c_void) {
        for op in &self.ops {
            if let FieldOp::PrimSeq {
                c_offset,
                forge: true,
                ..
            } = op
            {
                let seq = &mut *(msg.add(*c_offset) as *mut RosSequence);
                // Memory safety (shared with
                // the C++ twin): the release is gated on the sequence's
                // ownership state — a non-zero capacity — never on a
                // non-null `data`. This bridge mints the counter-example
                // itself: the forge writes `capacity = size`, so an EMPTY
                // forged entry leaves a non-null SHM address behind a
                // capacity of 0. A zero-length range registers nothing
                // with the hook, so freeing that pointer reaches the real
                // `free` with a shared-memory address. An empty `/scan`
                // published twice reaches it. `capacity == 0` owns
                // nothing, whatever `data` says.
                // A Buffer-backed instance is not this bridge's to free or
                // reset (the forge never produces one; a user's is left as
                // is). A PrimSeq op is a primitive sequence by construction
                // (`is_forgeable_sequence`), so the flag read is in bounds.
                if prim_seq_is_rosidl_buffer(seq) {
                    continue;
                }
                if seq.capacity > 0 && !seq.data.is_null() {
                    libc_free(seq.data);
                }
                *seq = EMPTY_ROS_SEQUENCE;
            }
        }
    }

    /// Repr(C) padding byte-ranges of the loanable struct
    /// (relative to the payload start). Empty unless [`Self::can_loan`].
    pub fn loan_pad_ranges(&self) -> &[(usize, usize)] {
        &self.loan_pad_ranges
    }

    /// Bring a freshly-loaned SHM payload slot to the state a
    /// ROS message is REQUIRED to start in — rosidl defaults, not zeros.
    ///
    /// rclcpp's `LoanedMessage` never placement-news `MessageT` on the
    /// loaned branch (jazzy `loaned_message.hpp`: `static_cast` only;
    /// only the heap fallback runs `new (message_) MessageT()`), and its
    /// destructor never runs `~MessageT` — so whatever this function
    /// leaves in the slot IS the message's initial state. A loaned
    /// `geometry_msgs/Quaternion` published untouched must read
    /// `w == 1.0`, exactly as it does on every other rmw.
    ///
    /// C semantics: the payload is zeroed FIRST, then the typesupport's
    /// `init_function` runs with `ROSIDL_RUNTIME_C_MSG_INIT_ALL`. The
    /// zero pass is load-bearing, not belt-and-suspenders: rosidl's C
    /// generator emits assignments only for members that NEED init
    /// (declared defaults, strings, nested containers — rosidl#477);
    /// default-less primitives are left untouched on the documented
    /// assumption that callers hand it zero-allocated memory (`__create`
    /// uses `zero_allocate`). We are that caller. (The C++ twin skips
    /// the memset — its ALL constructor writes every member.)
    ///
    /// A typesupport with NO `init_function` keeps the zeroed baseline
    /// and leaves a breadcrumb — never silent divergence: a
    /// default-carrying type would read zeros where every other rmw
    /// serves its declared defaults.
    ///
    /// Allocation safety: only reachable behind [`Self::can_loan`],
    /// which requires `layout.is_fixed() && fixed_size == c_size` with
    /// per-field C==wire offsets — a recursively-FIXED, all-primitive
    /// layout with no strings, sequences or variable nesting anywhere.
    /// For such a type `__init` writes primitive defaults in place and
    /// allocates NOTHING. If a future `can_loan` widening ever admits a
    /// pointer-bearing type, `init_function` would heap-allocate
    /// containers INTO SHARED MEMORY (their pointers meaningless in
    /// every other process) — the borrow path's
    /// `debug_assert!(layout.is_fixed())` is the tripwire.
    ///
    /// # Safety
    /// `payload` must point at a writable region of at least
    /// `self.c_size` bytes (the loaned slot's payload region).
    pub unsafe fn init_loaned_payload(&self, payload: *mut c_void) {
        std::ptr::write_bytes(payload as *mut u8, 0, self.c_size);
        match (*self.members).init_function {
            Some(init) => init(payload, ROSIDL_MSG_INIT_ALL),
            None => tracing::debug!(
                schema = %self.qualified_name,
                "typesupport has no init_function; loaned message keeps the zeroed baseline (declared field defaults will NOT be applied)"
            ),
        }
    }

    /// Size pre-pass: exact wire-frame byte count for this C message,
    /// using the SAME hostile-count validation the encode path runs
    /// (count × stride overflow / [`MAX_FRAME_BYTES`] caps, corrupt
    /// string sizes). Never dereferences sequence data — corrupt
    /// headers are rejected BEFORE any loan or allocation is sized
    /// from them.
    ///
    /// The pre-pass answer is advisory, not trusted: `flatten_into*`
    /// re-reads every variable length during the write pass through a
    /// bounds-checked cursor, so a message mutated between the two
    /// passes produces a graceful [`BridgeError::Encode`] — never an
    /// out-of-bounds write or a torn frame (the TOCTOU posture).
    ///
    /// # Safety
    /// `c_msg` must point at a valid message of this bridged type.
    pub unsafe fn frame_size(&self, c_msg: *const c_void) -> Result<usize, BridgeError> {
        let mut size = self.head_len();
        for op in &self.ops {
            match op {
                FieldOp::FixedCopy { .. } => {}
                FieldOp::String { c_offset, .. } => {
                    let s = &*(c_msg.add(*c_offset) as *const RosString);
                    let bytes = ros_string_bytes(s).map_err(|d| self.encode_err(d))?;
                    size = add_var_size(size, bytes.len(), 1).map_err(|d| self.encode_err(d))?;
                }
                FieldOp::PrimSeq {
                    c_offset,
                    elem_size,
                    ..
                } => {
                    let seq = &*(c_msg.add(*c_offset) as *const RosSequence);
                    // A Buffer-backed instance (Lyrical, Rolling) keeps a
                    // Buffer object behind `data`, not elements: it is not
                    // readable as bytes, so the frame is refused here and
                    // in the write pass alike.
                    if prim_seq_is_rosidl_buffer(seq) {
                        return Err(self.encode_err(
                            "sequence instance is a rosidl Buffer, not readable as bytes",
                        ));
                    }
                    let byte_len = seq
                        .size
                        .checked_mul(*elem_size)
                        .filter(|&n| n <= MAX_FRAME_BYTES)
                        .ok_or_else(|| {
                            self.encode_err(
                                "sequence length × element size overflows or exceeds cap",
                            )
                        })?;
                    size =
                        add_var_size(size, byte_len, *elem_size).map_err(|d| self.encode_err(d))?;
                }
                FieldOp::Complex {
                    c_offset,
                    member_index,
                    ..
                } => {
                    // Complex fields are small (Headers, string arrays);
                    // sizing them by encoding into a scratch buffer keeps
                    // ONE codec (no size-only twin that could drift).
                    let member = self.member(*member_index);
                    let mut buf = Vec::new();
                    encode_complex(&self.nested_layouts, member, c_msg.add(*c_offset), &mut buf)
                        .map_err(|d| self.encode_err(d))?;
                    size = add_var_size(size, buf.len(), 1).map_err(|d| self.encode_err(d))?;
                }
            }
        }
        Ok(size)
    }

    /// `WireHeader + fixed section + offset table` — the pre-zeroed
    /// frame head every encode starts from.
    fn head_len(&self) -> usize {
        frame_head_len(&self.layout)
    }

    /// Flatten a C message DIRECTLY into `out` (initialized buffer
    /// form — tests and non-SHM callers). `out` must be EXACTLY
    /// [`Self::frame_size`] bytes; any mismatch — undersized, oversized,
    /// or the message mutating between passes — returns `Err` without
    /// ever writing out of bounds. See [`Self::flatten_into_uninit`].
    ///
    /// # Safety
    /// `c_msg` must point at a valid message of this bridged type.
    pub unsafe fn flatten_into(
        &self,
        c_msg: *const c_void,
        sequence: u32,
        timestamp_ns: u64,
        out: &mut [u8],
    ) -> Result<usize, BridgeError> {
        // SAFETY: viewing initialized bytes as MaybeUninit is sound
        // here because the cursor NEVER de-initializes — it only writes
        // concrete bytes.
        let uninit = std::slice::from_raw_parts_mut(
            out.as_mut_ptr() as *mut std::mem::MaybeUninit<u8>,
            out.len(),
        );
        self.flatten_into_uninit(c_msg, sequence, timestamp_ns, uninit)
    }

    /// Flatten a C message DIRECTLY into a possibly-uninitialized
    /// buffer (the SHM-loan publish path: `frame_size` → uninit loan →
    /// `flatten_into_uninit` → send). On `Ok(n)`:
    ///
    /// - `n == out.len()` (the cursor ended EXACTLY at the pre-pass
    ///   size), and
    /// - EVERY byte of `out` was initialized by the encode (head region
    ///   pre-zeroed, repr(C) padding zeroed, alignment gaps
    ///   zeroed) — the proof obligation for `assume_init` on the loan.
    ///
    /// On `Err` the buffer may be partially written but NEVER beyond
    /// its bounds; the caller must drop the loan, not send it.
    ///
    /// `sequence`/`timestamp_ns` stamp the WireHeader (the publisher's
    /// counters/clock — deterministic under replay).
    ///
    /// # Safety
    /// `c_msg` must point at a valid message of this bridged type.
    pub unsafe fn flatten_into_uninit(
        &self,
        c_msg: *const c_void,
        sequence: u32,
        timestamp_ns: u64,
        out: &mut [std::mem::MaybeUninit<u8>],
    ) -> Result<usize, BridgeError> {
        let payload_base = WireHeader::SIZE;
        let table_base = payload_base + self.layout.fixed_size;
        let head_len = frame_head_len(&self.layout);
        let total = out.len();
        let mut cur = FrameCursor::new(out, head_len).map_err(|d| self.encode_err(d))?;

        for op in &self.ops {
            match op {
                FieldOp::FixedCopy {
                    c_offset,
                    wire_offset,
                    size,
                    pad_ranges,
                } => {
                    // In-bounds by construction: wire_offset + size ≤
                    // fixed_size (layout invariant), all inside the
                    // pre-zeroed head — but the cursor re-checks anyway.
                    let src =
                        std::slice::from_raw_parts((c_msg as *const u8).add(*c_offset), *size);
                    cur.write_initialized_at(payload_base + wire_offset, src)
                        .map_err(|d| self.encode_err(d))?;
                    // repr(C) padding is uninitialized on the C side
                    // — zero it for byte-identical replay frames.
                    for &(off, len) in pad_ranges {
                        cur.zero_initialized_at(payload_base + wire_offset + off, len)
                            .map_err(|d| self.encode_err(d))?;
                    }
                }
                FieldOp::String { c_offset, var_idx } => {
                    let s = &*(c_msg.add(*c_offset) as *const RosString);
                    // Single materialization of (ptr, len) — no re-read
                    // within the write pass.
                    let bytes = ros_string_bytes(s).map_err(|d| self.encode_err(d))?;
                    cursor_append_var(&mut cur, table_base, *var_idx, bytes, 1)
                        .map_err(|d| self.encode_err(d))?;
                }
                FieldOp::PrimSeq {
                    c_offset,
                    var_idx,
                    elem_size,
                    forge: _,
                } => {
                    let seq = &*(c_msg.add(*c_offset) as *const RosSequence);
                    if prim_seq_is_rosidl_buffer(seq) {
                        return Err(self.encode_err(
                            "sequence instance is a rosidl Buffer, not readable as bytes",
                        ));
                    }
                    let count = seq.size; // single read in the write pass
                                          // Bound BEFORE forming the slice — `from_raw_parts`
                                          // with a corrupt huge length is UB by itself.
                    let byte_len = count
                        .checked_mul(*elem_size)
                        .filter(|&n| n <= MAX_FRAME_BYTES)
                        .ok_or_else(|| {
                            self.encode_err(
                                "sequence length × element size overflows or exceeds cap",
                            )
                        })?;
                    let bytes = if byte_len == 0 {
                        &[][..]
                    } else if seq.data.is_null() {
                        // Null data + nonzero size: corrupt header. A
                        // SIGSEGV is not a panic — ffi_guard can't save
                        // us; error BEFORE the deref.
                        return Err(self.encode_err("sequence has null data with nonzero size"));
                    } else {
                        std::slice::from_raw_parts(seq.data, byte_len)
                    };
                    cursor_append_var(&mut cur, table_base, *var_idx, bytes, *elem_size)
                        .map_err(|d| self.encode_err(d))?;
                }
                FieldOp::Complex {
                    c_offset,
                    var_idx,
                    member_index,
                } => {
                    let member = self.member(*member_index);
                    let mut buf = Vec::new();
                    encode_complex(&self.nested_layouts, member, c_msg.add(*c_offset), &mut buf)
                        .map_err(|detail| self.encode_err(detail))?;
                    cursor_append_var(&mut cur, table_base, *var_idx, &buf, 1)
                        .map_err(|d| self.encode_err(d))?;
                }
            }
        }

        // The cursor must land EXACTLY on the pre-pass size — short
        // (message shrank) and long (message grew; already an Err
        // above) both fail the publish instead of sending a frame
        // whose header lies about its length or whose tail is
        // uninitialized.
        cur.require_full().map_err(|d| self.encode_err(d))?;

        let header = WireHeader {
            schema_hash: self.layout.schema_hash,
            total_size: total as u32,
            offset_table_offset: self.layout.fixed_size as u32,
            offset_table_count: self.layout.variable_fields.len() as u32,
            sequence,
            timestamp_ns,
        };
        let mut head = [0u8; WireHeader::SIZE];
        header.write_to_buf(&mut head);
        cur.write_initialized_at(0, &head)
            .map_err(|d| self.encode_err(d))?;
        Ok(total)
    }

    /// Flatten a C message into a complete heap wire frame — a thin
    /// wrapper over [`Self::frame_size`] + [`Self::flatten_into_uninit`]
    /// (ONE encode code path; the byte-identity tests exercise the same
    /// cursor the SHM-loan publish path uses). Used by the service
    /// layer and TRANSIENT_LOCAL publishers (which need owned frame
    /// bytes for history).
    ///
    /// # Safety
    /// `c_msg` must point at a valid message of this bridged type.
    pub unsafe fn flatten(
        &self,
        c_msg: *const c_void,
        sequence: u32,
        timestamp_ns: u64,
    ) -> Result<Vec<u8>, BridgeError> {
        let size = self.frame_size(c_msg)?;
        let mut frame: Vec<u8> = Vec::new();
        frame
            .try_reserve_exact(size)
            .map_err(|_| self.encode_err("frame allocation failed"))?;
        let n = self.flatten_into_uninit(
            c_msg,
            sequence,
            timestamp_ns,
            &mut frame.spare_capacity_mut()[..size],
        )?;
        debug_assert_eq!(n, size);
        // SAFETY: flatten_into_uninit Ok(n) guarantees bytes [0, n)
        // are initialized and n == size ≤ capacity.
        frame.set_len(n);
        Ok(frame)
    }

    fn encode_err(&self, detail: &str) -> BridgeError {
        BridgeError::Encode {
            message: self.qualified_name.clone(),
            detail: detail.to_string(),
        }
    }

    /// Flatten WITHOUT the WireHeader — the post-header payload bytes
    /// (`[fixed][offset table][variable payload]`). Used by the service
    /// layer, which frames payloads itself (envelope + header).
    ///
    /// # Safety
    /// `c_msg` must point at a valid message of this bridged type.
    pub unsafe fn flatten_payload_only(
        &self,
        c_msg: *const c_void,
    ) -> Result<Vec<u8>, BridgeError> {
        let mut frame = self.flatten(c_msg, 0, 0)?;
        // In-place header strip: a
        // `frame[SIZE..].to_vec()` would pay a SECOND full-payload heap
        // alloc + memcpy per service request/response just to drop 32
        // bytes; `drain` is a memmove within the existing allocation.
        frame.drain(..WireHeader::SIZE);
        Ok(frame)
    }

    /// Unflatten a received wire payload (post-WireHeader bytes) into an
    /// INITIALIZED C message.
    ///
    /// Returns false (with a warning) on malformed frames — the message
    /// is left in a valid (possibly partially-assigned) state.
    ///
    /// # Safety
    /// `c_msg` must point at a valid, initialized message of this
    /// bridged type (the rmw_take contract).
    pub unsafe fn unflatten(&self, payload: &[u8], c_msg: *mut c_void) -> bool {
        if payload.len() < self.layout.fixed_size + self.layout.offset_table_bytes() {
            tracing::warn!(
                message = %self.qualified_name,
                payload_len = payload.len(),
                "wire payload too short for fixed section + offset table"
            );
            return false;
        }
        let table_base = self.layout.fixed_size;

        for op in &self.ops {
            match op {
                FieldOp::FixedCopy {
                    c_offset,
                    wire_offset,
                    size,
                    // Decode direction: copying zeroed wire padding INTO
                    // the C struct's padding bytes is harmless.
                    pad_ranges: _,
                } => {
                    std::ptr::copy_nonoverlapping(
                        payload.as_ptr().add(*wire_offset),
                        (c_msg as *mut u8).add(*c_offset),
                        *size,
                    );
                }
                FieldOp::String { c_offset, var_idx } => {
                    let Some(bytes) = read_var_entry(payload, table_base, *var_idx) else {
                        return warn_bad_entry(&self.qualified_name, *var_idx);
                    };
                    if !assign_ros_string(c_msg.add(*c_offset) as *mut RosString, bytes) {
                        return warn_bad_entry(&self.qualified_name, *var_idx);
                    }
                }
                FieldOp::PrimSeq {
                    c_offset,
                    var_idx,
                    elem_size,
                    forge: _,
                } => {
                    let Some(bytes) = read_var_entry(payload, table_base, *var_idx) else {
                        return warn_bad_entry(&self.qualified_name, *var_idx);
                    };
                    if bytes.len() % elem_size != 0 {
                        return warn_bad_entry(&self.qualified_name, *var_idx);
                    }
                    if !assign_prim_sequence(
                        c_msg.add(*c_offset) as *mut RosSequence,
                        bytes,
                        *elem_size,
                    ) {
                        return warn_bad_entry(&self.qualified_name, *var_idx);
                    }
                }
                FieldOp::Complex {
                    c_offset,
                    var_idx,
                    member_index,
                } => {
                    let Some(bytes) = read_var_entry(payload, table_base, *var_idx) else {
                        return warn_bad_entry(&self.qualified_name, *var_idx);
                    };
                    let member = self.member(*member_index);
                    if !decode_complex(&self.nested_layouts, member, bytes, c_msg.add(*c_offset)) {
                        return warn_bad_entry(&self.qualified_name, *var_idx);
                    }
                }
            }
        }
        true
    }

    fn member(&self, index: usize) -> &ffi::rosidl_typesupport_introspection_c__MessageMember {
        // SAFETY: index was minted from the same members slice at
        // registration; rosidl data is static.
        unsafe {
            &std::slice::from_raw_parts(
                (*self.members).members_,
                (*self.members).member_count_ as usize,
            )[index]
        }
    }
}

// =====================================================================
// Schema collection (introspection → MessageSchema set)
// =====================================================================

/// "geometry_msgs__msg" + "Pose" → package "geometry_msgs", name "Pose".
unsafe fn qualified_name_of(
    members: *const ffi::rosidl_typesupport_introspection_c__MessageMembers,
) -> Result<String, BridgeError> {
    let m = &*members;
    let ns = ffi::cstr(m.message_namespace_).ok_or(BridgeError::NoIntrospection)?;
    let name = ffi::cstr(m.message_name_).ok_or(BridgeError::NoIntrospection)?;
    let package = ns.split("__").next().unwrap_or(ns);
    Ok(format!("{package}/{name}"))
}

unsafe fn collect_schemas(
    members: *const ffi::rosidl_typesupport_introspection_c__MessageMembers,
    out: &mut BTreeMap<String, MessageSchema>,
    ptrs: &mut BTreeMap<usize, String>,
) -> Result<(), BridgeError> {
    let m = &*members;
    let ns = ffi::cstr(m.message_namespace_).ok_or(BridgeError::NoIntrospection)?;
    let name = ffi::cstr(m.message_name_).ok_or(BridgeError::NoIntrospection)?;
    let package = ns.split("__").next().unwrap_or(ns).to_string();
    let qualified = format!("{package}/{name}");

    // Recursion is deduped by introspection POINTER, not by
    // qualified NAME. The bridges resolve a nested type's `WireLayout` by
    // pointer (the rosidl data is 'static per type, so the pointer is a
    // stable identity with no per-element string build), and two DISTINCT
    // typesupport instances can carry the same qualified name — under a
    // name-dedupe the second instance's children would never be walked, so
    // they would have no layout entry and their bodies could not be encoded.
    if ptrs.insert(members as usize, qualified.clone()).is_some() {
        return Ok(());
    }
    if out.contains_key(&qualified) {
        // Same type, different typesupport instance: the pointer is newly
        // recorded above, but the schema is already collected. Still recurse
        // (below) so THIS instance's children get pointer entries too.
        return collect_nested_schemas(member_slice_of(m), out, ptrs);
    }

    let mut schema = MessageSchema::new_in_package(name, &package);
    let member_slice = std::slice::from_raw_parts(m.members_, m.member_count_ as usize);
    for member in member_slice {
        let fname = ffi::cstr(member.name_)
            .ok_or(BridgeError::NoIntrospection)?
            .to_string();
        let base = base_field_type(member, &qualified, &fname)?;
        let ft = if member.is_array_ {
            if member.array_size_ > 0 && !member.is_upper_bound_ {
                FieldType::FixedArray {
                    element_type: Box::new(base),
                    length: member.array_size_,
                }
            } else {
                FieldType::DynamicArray {
                    element_type: Box::new(base),
                }
            }
        } else {
            base
        };
        schema.add_field(FieldDef::new(fname, ft));
    }
    out.insert(qualified, schema);

    collect_nested_schemas(member_slice, out, ptrs)
}

/// Borrow a `MessageMembers`' member slice.
unsafe fn member_slice_of(
    m: &ffi::rosidl_typesupport_introspection_c__MessageMembers,
) -> &'static [ffi::rosidl_typesupport_introspection_c__MessageMember] {
    std::slice::from_raw_parts(m.members_, m.member_count_ as usize)
}

/// Recurse into every nested message type of `member_slice`.
unsafe fn collect_nested_schemas(
    member_slice: &[ffi::rosidl_typesupport_introspection_c__MessageMember],
    out: &mut BTreeMap<String, MessageSchema>,
    ptrs: &mut BTreeMap<usize, String>,
) -> Result<(), BridgeError> {
    for member in member_slice {
        if member.type_id_ == ros_type::MESSAGE {
            let nested_ts = member.members_;
            if nested_ts.is_null() {
                return Err(BridgeError::NoIntrospection);
            }
            let nested_members =
                (*nested_ts).data as *const ffi::rosidl_typesupport_introspection_c__MessageMembers;
            collect_schemas(nested_members, out, ptrs)?;
        }
    }
    Ok(())
}

unsafe fn base_field_type(
    member: &ffi::rosidl_typesupport_introspection_c__MessageMember,
    message: &str,
    field: &str,
) -> Result<FieldType, BridgeError> {
    Ok(match member.type_id_ {
        ros_type::FLOAT => FieldType::F32,
        ros_type::DOUBLE => FieldType::F64,
        ros_type::CHAR => FieldType::U8,
        ros_type::BOOLEAN => FieldType::Bool,
        ros_type::OCTET | ros_type::UINT8 => FieldType::U8,
        ros_type::INT8 => FieldType::I8,
        ros_type::UINT16 => FieldType::U16,
        ros_type::INT16 => FieldType::I16,
        ros_type::UINT32 => FieldType::U32,
        ros_type::INT32 => FieldType::I32,
        ros_type::UINT64 => FieldType::U64,
        ros_type::INT64 => FieldType::I64,
        ros_type::STRING => FieldType::String,
        ros_type::MESSAGE => {
            let nested = (*member.members_).data
                as *const ffi::rosidl_typesupport_introspection_c__MessageMembers;
            let nm = &*nested;
            let ns = ffi::cstr(nm.message_namespace_).ok_or(BridgeError::NoIntrospection)?;
            let nname = ffi::cstr(nm.message_name_).ok_or(BridgeError::NoIntrospection)?;
            FieldType::Nested {
                schema_name: nname.to_string(),
                package: Some(ns.split("__").next().unwrap_or(ns).to_string()),
                fixed: None,
            }
        }
        other @ (ros_type::LONG_DOUBLE | ros_type::WCHAR | ros_type::WSTRING) => {
            return Err(BridgeError::UnsupportedFieldType {
                message: message.to_string(),
                field: field.to_string(),
                type_id: other,
            })
        }
        other => {
            return Err(BridgeError::UnsupportedFieldType {
                message: message.to_string(),
                field: field.to_string(),
                type_id: other,
            })
        }
    })
}

/// Mirror of the wire classification for an introspection member.
///
/// MUST recurse into nested message fixedness exactly like the layout
/// side does: a direct (non-array) VARIABLE nested member — e.g. the
/// `std_msgs/Header` in every stamped MoveIt message — is a variable
/// field. A non-recursive form would classify it FIXED, desyncing
/// the fixed/variable iterators in `BridgedMessage::new` and failing
/// registration with LayoutMismatch for ALL Header-bearing messages.
unsafe fn is_variable_member(
    member: &ffi::rosidl_typesupport_introspection_c__MessageMember,
) -> bool {
    is_variable_member_recursive(member)
}

// =====================================================================
// Variable-op planning
// =====================================================================

fn plan_variable_op(
    c_offset: usize,
    var_idx: usize,
    member_index: usize,
    layout_field: &VariableFieldLayout,
    forge: bool,
) -> FieldOp {
    let prim_seq = |elem_size: usize| FieldOp::PrimSeq {
        c_offset,
        var_idx,
        elem_size,
        forge,
    };
    match &layout_field.field_type {
        FieldType::String => FieldOp::String { c_offset, var_idx },
        FieldType::Bytes => prim_seq(1),
        FieldType::DynamicArray { element_type } => match element_type.as_ref() {
            FieldType::Bool | FieldType::I8 | FieldType::U8 => prim_seq(1),
            FieldType::I16 | FieldType::U16 => prim_seq(2),
            FieldType::I32 | FieldType::U32 | FieldType::F32 => prim_seq(4),
            FieldType::I64 | FieldType::U64 | FieldType::F64 => prim_seq(8),
            _ => FieldOp::Complex {
                c_offset,
                var_idx,
                member_index,
            },
        },
        _ => FieldOp::Complex {
            c_offset,
            var_idx,
            member_index,
        },
    }
}

/// The offset-table index a top-level variable op reads (fixed ops have
/// none — reported as `usize::MAX` on the diagnostic line, which they never
/// reach because a fixed memcpy cannot fail).
fn op_var_idx(op: &FieldOp) -> usize {
    match op {
        FieldOp::FixedCopy { .. } => usize::MAX,
        FieldOp::String { var_idx, .. }
        | FieldOp::PrimSeq { var_idx, .. }
        | FieldOp::Complex { var_idx, .. } => *var_idx,
    }
}

/// May the forged loaned take aim this C member's rosidl
/// sequence header at the held SHM sample instead of copying?
///
/// Exactly the UNBOUNDED, non-`bool`, default-less primitive sequence — the
/// `uint8[]` / `float32[]` payload class (`Image.data`, `PointCloud2.data`,
/// `CompressedImage.data`, `LaserScan.ranges`). Each exclusion is a
/// distinct reason, not conservatism:
///
/// - a BOUNDED sequence (`is_upper_bound_`) carries a capacity contract the
///   wire does not validate; a fixed array (`array_size_ > 0`) is fixed
///   section already and never reaches this;
/// - `bool[]` stays copied so the C and C++ bridges agree on the eligible
///   set (`std::vector<bool>` is bit-packed and can never be forged);
/// - a member with a rosidl DEFAULT VALUE starts life owning a heap
///   allocation the forge would overwrite and the un-forge would drop on the
///   floor — copied instead, leaking nothing;
/// - strings and nested-message arrays are not primitive sequences at all:
///   their wire shape is not the C object (the `Complex`/`String` ops).
///
/// `field_type` is the wire's view of the same member; both agree by the
/// registration lockstep, and the primitive check reads the wire side so a
/// `Bytes`-classified member is admitted exactly like `U8`.
unsafe fn is_forgeable_sequence(
    member: &ffi::rosidl_typesupport_introspection_c__MessageMember,
    field_type: &FieldType,
) -> bool {
    // Lyrical and Rolling: a member the typesupport marks as a rosidl
    // Buffer (`uint8[]`) may hold a Buffer object behind `data`; the forge
    // writes a plain element triplet, so it never touches such a member.
    #[cfg(cerulion_has_is_rosidl_buffer)]
    if member.is_rosidl_buffer_ {
        return false;
    }
    let element_is_forgeable = match field_type {
        FieldType::Bytes => true,
        FieldType::DynamicArray { element_type } => matches!(
            element_type.as_ref(),
            FieldType::I8
                | FieldType::U8
                | FieldType::I16
                | FieldType::U16
                | FieldType::I32
                | FieldType::U32
                | FieldType::F32
                | FieldType::I64
                | FieldType::U64
                | FieldType::F64
        ),
        _ => false,
    };
    element_is_forgeable
        && member.is_array_
        && member.array_size_ == 0
        && !member.is_upper_bound_
        && member.default_value_.is_null()
        && is_primitive_type(member.type_id_)
        && member.type_id_ != ros_type::BOOLEAN
}

/// What one forged take did to a shadow — returned by
/// `BridgedMessage::unflatten_forged` / `CppBridgedMessage::unflatten_forged`
/// and recorded on the shadow, because the UN-FORGE must know which members
/// alias the sample (re-point to nothing) and which hold a COPY the
/// typesupport's `fini` owns (leave alone).
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct ForgeOutcome {
    /// Bit `i` set ⇔ the `i`-th forgeable member (in declaration order) was
    /// FORGED — its header aims into the held sample. At most
    /// [`FORGE_MASK_BITS`] members can be forged; any beyond are copied.
    pub forged: u64,
    /// Forgeable members that were COPIED instead because their offset-table
    /// entry sits BELOW the frame's data floor (`WireLayout::data_floor` —
    /// inside the fixed section or the table). Served with the same bytes the
    /// copying take serves; reported through the fallback latch.
    pub below_floor: usize,
}

/// Width of [`ForgeOutcome::forged`]: forgeable members past this index are
/// always copied (no real ROS type has more than a handful).
pub const FORGE_MASK_BITS: usize = 64;

/// Bit test over a [`ForgeOutcome::forged`] mask, overflow-safe.
#[inline]
pub(crate) fn mask_has(forged: u64, forge_idx: usize) -> bool {
    forge_idx < FORGE_MASK_BITS && forged & (1u64 << forge_idx) != 0
}

/// The mask bit for forgeable member `forge_idx` (`0` past the mask width —
/// such a member is never forged, see [`forge_placement`]).
#[inline]
pub(crate) fn mask_bit(forge_idx: usize) -> u64 {
    if forge_idx < FORGE_MASK_BITS {
        1u64 << forge_idx
    } else {
        0
    }
}

/// Where a NON-EMPTY forgeable entry's bytes may be served from.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) enum ForgePlacement {
    /// At or above the data floor and element-aligned: alias the sample.
    Forge,
    /// Copy into the shadow instead. `below_floor` says why: the entry sits
    /// below the data floor (counted + latched), or — rarely — the member is
    /// past [`FORGE_MASK_BITS`] (not counted; a static property of the type).
    Copy { below_floor: bool },
    /// The bytes cannot be addressed as elements at all (not element-aligned):
    /// the frame is malformed.
    Malformed,
}

/// Decide a forgeable entry's placement. `bytes` is the entry's slice INSIDE
/// `payload` (a whole number of elements, non-empty — the caller checks both
/// first), `data_floor` is `WireLayout::data_floor()`.
///
/// The PLACEMENT rule comes first, the alignment rule second, and the two are
/// different kinds of fact. The floor is the wire's own rule (the
/// `FrameWalker` refuses an entry below it): forging such an entry would hand
/// a C++ `std::vector` header/table bytes as its elements — silent wrong data
/// — so the take COPIES the member instead, serving exactly what the copying
/// `rmw_take` serves for the same frame. Alignment is the WRITER's contract
/// (every Cerulion producer aligns an entry to its element size); a
/// misaligned entry is a corrupt frame and is refused, since a `float*` that
/// is not 4-aligned cannot be handed to C at all. The frame is WIRE input (a
/// bag player can put any same-hash frame on the topic), so both are checked,
/// never trusted.
pub(crate) fn forge_placement(
    payload: &[u8],
    bytes: &[u8],
    elem_size: usize,
    data_floor: usize,
    forge_idx: usize,
) -> ForgePlacement {
    debug_assert!(!bytes.is_empty() && bytes.len().is_multiple_of(elem_size));
    let off = bytes.as_ptr() as usize - payload.as_ptr() as usize;
    if off < data_floor {
        return ForgePlacement::Copy { below_floor: true };
    }
    if forge_idx >= FORGE_MASK_BITS {
        return ForgePlacement::Copy { below_floor: false };
    }
    if !(bytes.as_ptr() as usize).is_multiple_of(elem_size) {
        return ForgePlacement::Malformed;
    }
    ForgePlacement::Forge
}

/// Over-aligned home for a shadow message: every ROS primitive (and every
/// `std::string`/`std::vector` on the C++ side) needs at most 8, and 16
/// keeps a placement-new'd C++ object correctly aligned on every target the
/// crate builds for.
pub(crate) const SHADOW_ALIGN: usize = 16;

/// The heap layout of one shadow of `c_size` bytes (never zero-sized:
/// `alloc` forbids it, and a message struct is never empty).
pub(crate) fn shadow_layout(c_size: usize) -> std::alloc::Layout {
    std::alloc::Layout::from_size_align(c_size.max(1), SHADOW_ALIGN)
        .expect("shadow layout: size rounds up to a multiple of 16 within isize::MAX")
}

/// The rosidl empty sequence — what a forged member is re-pointed at on
/// un-forge, and what rosidl's `fini` treats as "nothing to free".
const EMPTY_ROS_SEQUENCE: RosSequence = RosSequence {
    data: std::ptr::null_mut(),
    size: 0,
    capacity: 0,
};

/// The rosidl empty string state `fini` treats as "nothing to free" — what
/// the borrow seal re-points an in-slot string header at (its bytes were
/// already copied into the frame; the slot storage is not allocator
/// memory).
const EMPTY_ROS_STRING: RosString = RosString {
    data: std::ptr::null_mut(),
    size: 0,
    capacity: 0,
};

// =====================================================================
// Complex-field encoding (canonical v1 — see module docs)
// =====================================================================

unsafe fn nested_members_of(
    member: &ffi::rosidl_typesupport_introspection_c__MessageMember,
) -> *const ffi::rosidl_typesupport_introspection_c__MessageMembers {
    (*member.members_).data as *const ffi::rosidl_typesupport_introspection_c__MessageMembers
}

/// Verify a NESTED type's introspection walk agrees with its
/// `WireLayout`, exactly as `BridgedMessage::new` does for the root.
///
/// The canonical nested encoder writes fixed member `k` at
/// `layout.fixed_fields[k].offset` and pushes variable payloads in
/// declaration order. Both indexings are only sound if the introspection
/// partition (`is_variable_member_recursive`) and the layout partition
/// (`FieldType::is_variable`) agree field-for-field, and if each fixed
/// member's C size equals the layout's. A mismatch means the two disagree
/// about the schema and NOTHING downstream is trustworthy, so this is a hard
/// registration-time error — never a per-element surprise.
///
/// Note the ROOT check additionally requires `fl.offset == c_offset` for the
/// zero-copy loan fast path; a nested body has no such requirement, because
/// it is always re-packed field-by-field from the C offsets into the wire
/// offsets.
unsafe fn verify_nested_lockstep(
    members: *const ffi::rosidl_typesupport_introspection_c__MessageMembers,
    qname: &str,
    layout: &WireLayout,
) -> Result<(), BridgeError> {
    let m = &*members;
    let member_slice = std::slice::from_raw_parts(m.members_, m.member_count_ as usize);
    let mut fixed_idx = 0usize;
    let mut var_idx = 0usize;
    for member in member_slice {
        let fname = ffi::cstr(member.name_).unwrap_or("<field>").to_string();
        if is_variable_member_recursive(member) {
            let v =
                layout
                    .variable_fields
                    .get(var_idx)
                    .ok_or_else(|| BridgeError::LayoutMismatch {
                        message: qname.to_string(),
                        detail: format!("variable field '{fname}' missing from layout"),
                    })?;
            if v.name != fname {
                return Err(BridgeError::LayoutMismatch {
                    message: qname.to_string(),
                    detail: format!(
                        "variable-field order mismatch: layout '{}' vs introspection '{fname}'",
                        v.name
                    ),
                });
            }
            var_idx += 1;
        } else {
            let fl =
                layout
                    .fixed_fields
                    .get(fixed_idx)
                    .ok_or_else(|| BridgeError::LayoutMismatch {
                        message: qname.to_string(),
                        detail: format!("fixed field '{fname}' missing from layout"),
                    })?;
            if fl.name != fname {
                return Err(BridgeError::LayoutMismatch {
                    message: qname.to_string(),
                    detail: format!(
                        "fixed-field order mismatch: layout '{}' vs introspection '{fname}'",
                        fl.name
                    ),
                });
            }
            let c_field_size = fixed_member_size(member);
            if fl.size != c_field_size {
                return Err(BridgeError::LayoutMismatch {
                    message: qname.to_string(),
                    detail: format!(
                        "fixed-field size mismatch for '{fname}': layout {} vs C {}",
                        fl.size, c_field_size
                    ),
                });
            }
            fixed_idx += 1;
        }
    }
    if fixed_idx != layout.fixed_fields.len() || var_idx != layout.variable_fields.len() {
        return Err(BridgeError::LayoutMismatch {
            message: qname.to_string(),
            detail: format!(
                "field-count mismatch: introspection {fixed_idx} fixed / {var_idx} variable vs \
                 layout {} fixed / {} variable",
                layout.fixed_fields.len(),
                layout.variable_fields.len()
            ),
        });
    }
    Ok(())
}

/// Encode one complex field value (nested message / sequence of nested /
/// sequence of strings / fixed array thereof) into `out`.
///
/// Errors (`&'static str` detail, wrapped into [`BridgeError::Encode`]
/// by the caller) instead of panicking/OOM-ing on corrupt C sequence
/// headers; enforces [`MAX_FRAME_BYTES`] as it grows.
unsafe fn encode_complex(
    layouts: &NestedLayouts,
    member: &ffi::rosidl_typesupport_introspection_c__MessageMember,
    field_ptr: *const c_void,
    out: &mut Vec<u8>,
) -> Result<(), &'static str> {
    if member.is_array_ {
        let (count, elem_base, stride) = sequence_view(member, field_ptr)?;
        check_seq_bound(count, stride)?;
        if count > 0 && elem_base.is_null() {
            return Err("sequence has null data with nonzero size");
        }
        if member.type_id_ == ros_type::STRING {
            out.extend_from_slice(&(count as u32).to_le_bytes());
            for i in 0..count {
                let s = &*(elem_base.add(i * stride) as *const RosString);
                let bytes = ros_string_bytes(s)?;
                check_out_cap(out, 4 + bytes.len())?;
                out.extend_from_slice(&(bytes.len() as u32).to_le_bytes());
                out.extend_from_slice(bytes);
            }
        } else {
            // sequence/array of MESSAGE.
            let nested = nested_members_of(member);
            if nested_is_fixed_for_encoding(nested) {
                // Back-to-back fixed structs (stride = padded size; the
                // C struct IS the wire fixed section for fixed types).
                for i in 0..count {
                    encode_message_payload(
                        layouts,
                        nested,
                        elem_base.add(i * stride) as *const c_void,
                        out,
                    )?;
                }
            } else {
                out.extend_from_slice(&(count as u32).to_le_bytes());
                for i in 0..count {
                    let mut elem = Vec::new();
                    encode_message_payload(
                        layouts,
                        nested,
                        elem_base.add(i * stride) as *const c_void,
                        &mut elem,
                    )?;
                    check_out_cap(out, 4 + elem.len())?;
                    out.extend_from_slice(&(elem.len() as u32).to_le_bytes());
                    out.extend_from_slice(&elem);
                }
            }
        }
        Ok(())
    } else {
        // Single nested VARIABLE message (single FIXED nested lives in
        // the fixed section, never here).
        let nested = nested_members_of(member);
        encode_message_payload(layouts, nested, field_ptr, out)
    }
}

/// Reject sequence headers whose count × stride overflows or exceeds
/// the frame cap — corrupt C memory, not a real message.
pub(crate) fn check_seq_bound(count: usize, stride: usize) -> Result<(), &'static str> {
    match count.checked_mul(stride.max(1)) {
        Some(n) if n <= MAX_FRAME_BYTES => Ok(()),
        _ => Err("sequence count × stride overflows or exceeds MAX_FRAME_BYTES"),
    }
}

pub(crate) fn check_out_cap(out: &[u8], add: usize) -> Result<(), &'static str> {
    match out.len().checked_add(add) {
        Some(n) if n <= MAX_FRAME_BYTES => Ok(()),
        _ => Err("encoded complex field exceeds MAX_FRAME_BYTES"),
    }
}

/// Whether a nested type's canonical encoding is the raw C struct
/// (recursively fixed: no strings/sequences anywhere).
unsafe fn nested_is_fixed_for_encoding(
    members: *const ffi::rosidl_typesupport_introspection_c__MessageMembers,
) -> bool {
    let m = &*members;
    let member_slice = std::slice::from_raw_parts(m.members_, m.member_count_ as usize);
    for member in member_slice {
        if member.type_id_ == ros_type::STRING || member.type_id_ == ros_type::WSTRING {
            return false;
        }
        if member.is_array_ && (member.array_size_ == 0 || member.is_upper_bound_) {
            return false;
        }
        if member.type_id_ == ros_type::MESSAGE
            && !nested_is_fixed_for_encoding(nested_members_of(member))
        {
            return false;
        }
    }
    true
}

/// Encode a message VALUE as its canonical Cerulion body.
///
/// - A recursively-FIXED message is its raw `repr(C)` struct (padding zeroed
///   for byte-determinism) — the C struct IS the wire fixed section.
/// - A VARIABLE message is the canonical headerless sub-frame
///   `[fixed][offset table][variable payloads]`, built by
///   [`CanonicalBodyBuilder`] — the SAME routine `CdrCodec` uses, and the
///   same shape as the top-level frame that contains it.
///
/// The variable case writes the canonical framing, never a bespoke
/// `[packed fixed, NO alignment padding][u32 count-of-variable-members][per
/// variable: u32 len + payload]`, which no other component in the system
/// could read: `FrameWalker` would refuse it (the count `1` sits where an offset
/// belongs and lands below the body's data floor), so an rmw robot's
/// `nav_msgs/Path` / `tf2_msgs/TFMessage` / `Detection*Array` would stay opaque
/// to viz and `topic echo`.
unsafe fn encode_message_payload(
    layouts: &NestedLayouts,
    members: *const ffi::rosidl_typesupport_introspection_c__MessageMembers,
    msg: *const c_void,
    out: &mut Vec<u8>,
) -> Result<(), &'static str> {
    if nested_is_fixed_for_encoding(members) {
        let m = &*members;
        let start = out.len();
        check_out_cap(out, m.size_of_)?;
        out.resize(start + m.size_of_, 0);
        std::ptr::copy_nonoverlapping(msg as *const u8, out.as_mut_ptr().add(start), m.size_of_);
        // Zero repr(C) padding inside the copied struct.
        zero_struct_padding(members, &mut out[start..start + m.size_of_]);
        return Ok(());
    }

    // Variable nested message: the canonical headerless sub-frame.
    let m = &*members;
    let member_slice = std::slice::from_raw_parts(m.members_, m.member_count_ as usize);
    let layout = layouts
        .get(members)
        .ok_or("no wire layout registered for a nested message type")?;

    let mut builder = CanonicalBodyBuilder::new(layout);

    // Fixed members: copy each from its C offset to its WIRE offset. The two
    // may legitimately differ (the wire fixed section excludes the C sequence
    // headers of variable members), which is exactly why the layout drives
    // the destination — packing them back-to-back at a
    // running cursor with no alignment would leave nothing able to read the result.
    // `verify_nested_lockstep` proved this indexing sound at registration.
    {
        let fixed = builder.fixed_mut();
        let mut fixed_idx = 0usize;
        for member in member_slice {
            if is_variable_member_recursive(member) {
                continue;
            }
            let fl = layout
                .fixed_fields
                .get(fixed_idx)
                .ok_or("fixed-field index past the layout (registration lockstep violated)")?;
            let size = fixed_member_size(member);
            if fl
                .offset
                .checked_add(size)
                .is_none_or(|end| end > fixed.len())
            {
                return Err("fixed field does not fit the wire fixed section");
            }
            std::ptr::copy_nonoverlapping(
                (msg as *const u8).add(member.offset_ as usize),
                fixed.as_mut_ptr().add(fl.offset),
                size,
            );
            // repr(C) padding inside a nested-message member is
            // uninitialized on the C side — zero it for byte-identical
            // frames (Principle #7).
            if member.type_id_ == ros_type::MESSAGE {
                let mut runs = Vec::new();
                member_data_runs(member, 0, &mut runs);
                for (off, len) in padding_ranges(&mut runs, size) {
                    fixed[fl.offset + off..fl.offset + off + len].fill(0);
                }
            }
            fixed_idx += 1;
        }
    }

    // Variable members, in declaration order — entry i of the offset table.
    for member in member_slice {
        if !is_variable_member_recursive(member) {
            continue;
        }
        let field_ptr = msg.add(member.offset_ as usize);
        let mut buf = Vec::new();
        if member.type_id_ == ros_type::STRING && !member.is_array_ {
            let s = &*(field_ptr as *const RosString);
            buf.extend_from_slice(ros_string_bytes(s)?);
        } else if member.type_id_ == ros_type::MESSAGE && !member.is_array_ {
            encode_message_payload(layouts, nested_members_of(member), field_ptr, &mut buf)?;
        } else if member.is_array_ && is_primitive_type(member.type_id_) {
            let (count, base, stride) = sequence_view(member, field_ptr)?;
            check_seq_bound(count, stride)?;
            if count > 0 && base.is_null() {
                return Err("sequence has null data with nonzero size");
            }
            let byte_len = count * stride;
            buf.resize(byte_len, 0);
            if byte_len > 0 {
                std::ptr::copy_nonoverlapping(base, buf.as_mut_ptr(), byte_len);
            }
        } else {
            encode_complex(layouts, member, field_ptr, &mut buf)?;
        }
        builder.push_variable(buf).map_err(|e| e.as_str())?;
    }

    let body = builder.finish().map_err(|e| e.as_str())?;
    check_out_cap(out, body.len())?;
    out.extend_from_slice(&body);
    Ok(())
}

/// Variable-ness for the NESTED encoding (recursive: nested fixed
/// messages are fixed).
unsafe fn is_variable_member_recursive(
    member: &ffi::rosidl_typesupport_introspection_c__MessageMember,
) -> bool {
    if member.type_id_ == ros_type::STRING || member.type_id_ == ros_type::WSTRING {
        return true;
    }
    if member.is_array_ && (member.array_size_ == 0 || member.is_upper_bound_) {
        return true;
    }
    if member.type_id_ == ros_type::MESSAGE {
        let nested = nested_members_of(member);
        let fixed = nested_is_fixed_for_encoding(nested);
        if member.is_array_ {
            // Fixed array of fixed nested = fixed.
            return !fixed;
        }
        return !fixed;
    }
    false
}

/// Byte size of a fixed member in the C struct (primitive, fixed array
/// of primitives, fixed nested, fixed array of fixed nested).
unsafe fn fixed_member_size(
    member: &ffi::rosidl_typesupport_introspection_c__MessageMember,
) -> usize {
    let base = if member.type_id_ == ros_type::MESSAGE {
        (*nested_members_of(member)).size_of_
    } else {
        primitive_size(member.type_id_)
    };
    if member.is_array_ {
        base * member.array_size_
    } else {
        base
    }
}

/// Collect the byte runs within a FIXED member's span holding real
/// (leaf-primitive) data, recursing through nested structs and tiling
/// across inline arrays. Bytes not covered are repr(C) padding whose
/// C-side content is uninitialized.
unsafe fn member_data_runs(
    member: &ffi::rosidl_typesupport_introspection_c__MessageMember,
    base: usize,
    runs: &mut Vec<(usize, usize)>,
) {
    if member.type_id_ != ros_type::MESSAGE {
        // Primitives and inline primitive arrays are contiguous data —
        // no internal padding.
        runs.push((base, fixed_member_size(member)));
        return;
    }
    let nested = nested_members_of(member);
    let m = &*nested;
    let stride = m.size_of_;
    let count = if member.is_array_ {
        member.array_size_
    } else {
        1
    };
    let nested_slice = std::slice::from_raw_parts(m.members_, m.member_count_ as usize);
    for i in 0..count {
        for nm in nested_slice {
            member_data_runs(nm, base + i * stride + nm.offset_ as usize, runs);
        }
    }
}

/// Complement of (sorted, merged) `runs` within `[0, span)`.
pub(crate) fn padding_ranges(runs: &mut [(usize, usize)], span: usize) -> Vec<(usize, usize)> {
    runs.sort_unstable();
    let mut pads = Vec::new();
    let mut cur = 0usize;
    for &(off, len) in runs.iter() {
        if off > cur {
            pads.push((cur, off - cur));
        }
        cur = cur.max(off + len);
    }
    if span > cur {
        pads.push((cur, span - cur));
    }
    pads
}

/// Zero the repr(C) padding bytes of one struct value in place
/// (`buf` = exactly the struct's bytes).
unsafe fn zero_struct_padding(
    members: *const ffi::rosidl_typesupport_introspection_c__MessageMembers,
    buf: &mut [u8],
) {
    let m = &*members;
    let member_slice = std::slice::from_raw_parts(m.members_, m.member_count_ as usize);
    let mut runs = Vec::new();
    for nm in member_slice {
        member_data_runs(nm, nm.offset_ as usize, &mut runs);
    }
    for (off, len) in padding_ranges(&mut runs, buf.len()) {
        buf[off..off + len].fill(0);
    }
}

pub(crate) fn primitive_size(type_id: u8) -> usize {
    match type_id {
        ros_type::BOOLEAN | ros_type::OCTET | ros_type::UINT8 | ros_type::INT8 | ros_type::CHAR => {
            1
        }
        ros_type::UINT16 | ros_type::INT16 => 2,
        ros_type::FLOAT | ros_type::UINT32 | ros_type::INT32 => 4,
        ros_type::DOUBLE | ros_type::UINT64 | ros_type::INT64 => 8,
        _ => 0,
    }
}

pub(crate) fn is_primitive_type(type_id: u8) -> bool {
    primitive_size(type_id) > 0
}

/// View a sequence (or inline fixed array) field: (count, base, stride).
unsafe fn sequence_view(
    member: &ffi::rosidl_typesupport_introspection_c__MessageMember,
    field_ptr: *const c_void,
) -> Result<(usize, *const u8, usize), &'static str> {
    let stride = if member.type_id_ == ros_type::MESSAGE {
        (*nested_members_of(member)).size_of_
    } else if member.type_id_ == ros_type::STRING {
        std::mem::size_of::<RosString>()
    } else {
        primitive_size(member.type_id_)
    };
    if member.array_size_ > 0 && !member.is_upper_bound_ {
        // Inline fixed array.
        Ok((member.array_size_, field_ptr as *const u8, stride))
    } else {
        let seq = &*(field_ptr as *const RosSequence);
        // A Buffer-backed instance holds a Buffer object behind `data`, not
        // elements: it cannot be read as bytes, so the frame is refused.
        if seq_is_rosidl_buffer(member, seq) {
            return Err("sequence instance is a rosidl Buffer, not readable as bytes");
        }
        Ok((seq.size, seq.data as *const u8, stride))
    }
}

// =====================================================================
// Complex-field decoding (inverse of encode_complex)
// =====================================================================

unsafe fn decode_complex(
    layouts: &NestedLayouts,
    member: &ffi::rosidl_typesupport_introspection_c__MessageMember,
    bytes: &[u8],
    field_ptr: *mut c_void,
) -> bool {
    if member.is_array_ {
        if member.type_id_ == ros_type::STRING {
            let Some((count, mut rest)) = read_u32_prefix(bytes) else {
                return false;
            };
            // Each element needs ≥ 4 bytes (its length prefix) — a count
            // larger than that is a corrupt frame; reject BEFORE calloc
            // so a hostile count can't drive a huge allocation.
            if count > rest.len() / 4 {
                return false;
            }
            let Some(base) = prepare_sequence(member, field_ptr, count) else {
                return false;
            };
            for i in 0..count {
                let Some((len, after)) = read_u32_prefix(rest) else {
                    return false;
                };
                if after.len() < len {
                    return false;
                }
                if !assign_ros_string(
                    base.add(i * std::mem::size_of::<RosString>()) as *mut RosString,
                    &after[..len],
                ) {
                    return false;
                }
                rest = &after[len..];
            }
            true
        } else {
            let nested = nested_members_of(member);
            if nested_is_fixed_for_encoding(nested) {
                let stride = (*nested).size_of_;
                if stride == 0 || !bytes.len().is_multiple_of(stride) {
                    return false;
                }
                let count = bytes.len() / stride;
                let Some(base) = prepare_sequence(member, field_ptr, count) else {
                    return false;
                };
                std::ptr::copy_nonoverlapping(bytes.as_ptr(), base, bytes.len());
                true
            } else {
                let Some((count, mut rest)) = read_u32_prefix(bytes) else {
                    return false;
                };
                // Same hostile-count guard as the string path: each
                // element carries a 4-byte length prefix.
                if count > rest.len() / 4 {
                    return false;
                }
                let stride = (*nested).size_of_;
                let Some(base) = prepare_sequence(member, field_ptr, count) else {
                    return false;
                };
                for i in 0..count {
                    let Some((len, after)) = read_u32_prefix(rest) else {
                        return false;
                    };
                    if after.len() < len {
                        return false;
                    }
                    if !decode_message_payload(
                        layouts,
                        nested,
                        &after[..len],
                        base.add(i * stride) as *mut c_void,
                    ) {
                        return false;
                    }
                    rest = &after[len..];
                }
                true
            }
        }
    } else {
        decode_message_payload(layouts, nested_members_of(member), bytes, field_ptr)
    }
}

/// Decode a canonical Cerulion body into a C message value — the exact
/// inverse of [`encode_message_payload`], reading through the SHARED
/// [`CanonicalBodyReader`].
unsafe fn decode_message_payload(
    layouts: &NestedLayouts,
    members: *const ffi::rosidl_typesupport_introspection_c__MessageMembers,
    bytes: &[u8],
    msg: *mut c_void,
) -> bool {
    if nested_is_fixed_for_encoding(members) {
        let m = &*members;
        if bytes.len() != m.size_of_ {
            return false;
        }
        std::ptr::copy_nonoverlapping(bytes.as_ptr(), msg as *mut u8, m.size_of_);
        return true;
    }

    let m = &*members;
    let member_slice = std::slice::from_raw_parts(m.members_, m.member_count_ as usize);
    let Some(layout) = layouts.get(members) else {
        return false;
    };
    let Ok(reader) = CanonicalBodyReader::new(layout, bytes) else {
        return false;
    };

    // Fixed members: read each from its WIRE offset into its C offset.
    let fixed = reader.fixed();
    let mut fixed_idx = 0usize;
    for member in member_slice {
        if is_variable_member_recursive(member) {
            continue;
        }
        let Some(fl) = layout.fixed_fields.get(fixed_idx) else {
            return false;
        };
        let size = fixed_member_size(member);
        match fl.offset.checked_add(size) {
            Some(end) if end <= fixed.len() => {}
            _ => return false,
        }
        std::ptr::copy_nonoverlapping(
            fixed.as_ptr().add(fl.offset),
            (msg as *mut u8).add(member.offset_ as usize),
            size,
        );
        fixed_idx += 1;
    }

    // Variable members, in declaration order — entry i of the offset table.
    let mut var_idx = 0usize;
    for member in member_slice {
        if !is_variable_member_recursive(member) {
            continue;
        }
        let Some(chunk) = reader.variable(var_idx) else {
            return false;
        };
        var_idx += 1;
        let field_ptr = (msg as *mut u8).add(member.offset_ as usize) as *mut c_void;
        let ok = if member.type_id_ == ros_type::STRING && !member.is_array_ {
            assign_ros_string(field_ptr as *mut RosString, chunk)
        } else if member.type_id_ == ros_type::MESSAGE && !member.is_array_ {
            decode_message_payload(layouts, nested_members_of(member), chunk, field_ptr)
        } else if member.is_array_ && is_primitive_type(member.type_id_) {
            let elem = primitive_size(member.type_id_);
            if chunk.len() % elem != 0 {
                false
            } else {
                assign_prim_sequence(field_ptr as *mut RosSequence, chunk, elem)
            }
        } else {
            decode_complex(layouts, member, chunk, field_ptr)
        };
        if !ok {
            return false;
        }
    }
    true
}

/// Point a sequence field at freshly-allocated zeroed storage for
/// `count` elements (frees any prior data — allocator-compatible with
/// rosidl's default). For inline fixed arrays returns the array base
/// when `count` matches.
unsafe fn prepare_sequence(
    member: &ffi::rosidl_typesupport_introspection_c__MessageMember,
    field_ptr: *mut c_void,
    count: usize,
) -> Option<*mut u8> {
    let stride = if member.type_id_ == ros_type::MESSAGE {
        (*nested_members_of(member)).size_of_
    } else if member.type_id_ == ros_type::STRING {
        std::mem::size_of::<RosString>()
    } else {
        primitive_size(member.type_id_)
    };
    if member.array_size_ > 0 && !member.is_upper_bound_ {
        if count != member.array_size_ {
            return None;
        }
        return Some(field_ptr as *mut u8);
    }
    let seq = &mut *(field_ptr as *mut RosSequence);
    // A Buffer-backed instance cannot be filled by this bridge: refuse the
    // frame rather than free a Buffer object and overwrite its pointer.
    if seq_is_rosidl_buffer(member, seq) {
        return None;
    }
    // NOTE: prior element contents (strings/nested sequences) were
    // allocated by rosidl init or a previous take; freeing just the
    // backing array would leak their internals. rmw_take's contract is
    // a FRESHLY-initialized message (sequences empty), so in-spec calls
    // never hit a non-empty sequence here. Defensive: free the array
    // itself, accept the (in-spec unreachable) internal leak rather
    // than corrupting memory by guessing element layouts.
    if !seq.data.is_null() {
        libc_free(seq.data);
    }
    // Reset BEFORE any fallible step — bailing below must never leave a
    // dangling pointer + stale size for rosidl fini to double-free
    // (rmw_deserialize reuses messages, so the free branch above is
    // reachable).
    seq.data = std::ptr::null_mut();
    seq.size = 0;
    seq.capacity = 0;
    let byte_len = count.checked_mul(stride)?;
    let data = if byte_len == 0 {
        std::ptr::null_mut()
    } else {
        let p = libc_calloc(byte_len);
        if p.is_null() {
            return None;
        }
        p
    };
    seq.data = data;
    seq.size = count;
    seq.capacity = count;
    Some(data)
}

// =====================================================================
// rosidl string/sequence assignment (libc-allocator compatible)
// =====================================================================

/// Caps `size` at MAX_FRAME_BYTES BEFORE forming the slice — a corrupt
/// `RosString.size` is the same UB hazard class as the PrimSeq path
/// (`from_raw_parts` with a huge length is UB by itself). A capped-out
/// string fails the frame-size gate in `append_var`/`check_out_cap`
/// instead of invoking UB here.
unsafe fn ros_string_bytes<'a>(s: &RosString) -> Result<&'a [u8], &'static str> {
    if s.data.is_null() || s.size == 0 {
        Ok(&[])
    } else if s.size > MAX_FRAME_BYTES {
        Err("string size exceeds MAX_FRAME_BYTES (corrupt header)")
    } else {
        Ok(std::slice::from_raw_parts(s.data, s.size))
    }
}

/// Returns false on allocation failure (field left empty-but-valid) —
/// callers MUST drop the frame rather than deliver a silently-truncated
/// message.
unsafe fn assign_ros_string(s: *mut RosString, bytes: &[u8]) -> bool {
    let s = &mut *s;
    if !s.data.is_null() {
        libc_free(s.data);
    }
    let mem = libc_calloc(bytes.len() + 1);
    if mem.is_null() {
        s.data = std::ptr::null_mut();
        s.size = 0;
        s.capacity = 0;
        return false;
    }
    std::ptr::copy_nonoverlapping(bytes.as_ptr(), mem, bytes.len());
    s.data = mem;
    s.size = bytes.len();
    s.capacity = bytes.len() + 1;
    true
}

/// Returns false on allocation failure — see [`assign_ros_string`].
unsafe fn assign_prim_sequence(seq: *mut RosSequence, bytes: &[u8], _elem_size: usize) -> bool {
    let seq = &mut *seq;
    // A Buffer-backed instance is refused, never freed or overwritten
    // (every caller hands a primitive sequence, so the flag read is in
    // bounds).
    if prim_seq_is_rosidl_buffer(seq) {
        return false;
    }
    if !seq.data.is_null() {
        libc_free(seq.data);
    }
    if bytes.is_empty() {
        seq.data = std::ptr::null_mut();
        seq.size = 0;
        seq.capacity = 0;
        return true;
    }
    // Uninitialized alloc: the copy below overwrites ALL
    // `bytes.len()` bytes, so a calloc zero-fill would be immediately
    // clobbered -- that redundant `rep stos` measured ~18% of the RX decode
    // in a perf profile. The empty-bytes case already returned
    // above, and the null-check below still guards OOM.
    let mem = libc_malloc(bytes.len());
    if mem.is_null() {
        seq.data = std::ptr::null_mut();
        seq.size = 0;
        seq.capacity = 0;
        return false;
    }
    std::ptr::copy_nonoverlapping(bytes.as_ptr(), mem, bytes.len());
    seq.data = mem;
    seq.size = bytes.len() / _elem_size;
    seq.capacity = seq.size;
    true
}

// Thin libc shims (rosidl's default allocator is malloc/free — staying
// on the same allocator keeps rosidl fini() compatible with memory we
// allocate and vice versa).
extern "C" {
    fn calloc(nmemb: usize, size: usize) -> *mut c_void;
    fn malloc(size: usize) -> *mut c_void;
    fn free(ptr: *mut c_void);
}
unsafe fn libc_calloc(len: usize) -> *mut u8 {
    calloc(len, 1) as *mut u8
}
/// Uninitialized allocation (mirrors [`libc_calloc`] but WITHOUT the
/// zero-fill). Use only when EVERY byte is written before it can be read
/// -- the `assign_prim_sequence` copy overwrites the whole allocation.
unsafe fn libc_malloc(len: usize) -> *mut u8 {
    malloc(len) as *mut u8
}
// The C-side `release_forgeable_members` frees a rosidl sequence buffer
// here — the rcutils default allocator's pair. The C++ twin does NOT use
// this: a `std::vector` buffer is released through the
// shim's `::operator delete`, the pair it was allocated with.
pub(crate) unsafe fn libc_free(ptr: *mut u8) {
    free(ptr as *mut c_void);
}

// =====================================================================
// Frame helpers
// =====================================================================

pub(crate) fn read_var_entry(payload: &[u8], table_base: usize, idx: usize) -> Option<&[u8]> {
    let entry = table_base + idx * 8;
    if payload.len() < entry + 8 {
        return None;
    }
    let off = u32::from_le_bytes(payload[entry..entry + 4].try_into().ok()?) as usize;
    let len = u32::from_le_bytes(payload[entry + 4..entry + 8].try_into().ok()?) as usize;
    payload.get(off..off.checked_add(len)?)
}

pub(crate) fn read_u32_prefix(bytes: &[u8]) -> Option<(usize, &[u8])> {
    if bytes.len() < 4 {
        return None;
    }
    let v = u32::from_le_bytes(bytes[..4].try_into().ok()?) as usize;
    Some((v, &bytes[4..]))
}

pub(crate) fn warn_bad_entry(message: &str, var_idx: usize) -> bool {
    tracing::warn!(
        message = %message,
        var_idx,
        "malformed variable entry in wire frame, dropping message"
    );
    false
}

#[inline]
pub(crate) fn align_up(value: usize, align: usize) -> usize {
    (value + align - 1) & !(align - 1)
}

// =====================================================================
// The borrow-window SEAL — publish-side zero-copy for
// unbounded primitive-sequence types
// =====================================================================
//
// `rmw_borrow_loaned_message` for a windowed type loans ONE slot shaped
//
//   [WireHeader 32][C struct: c_size][zeroed remnant][window TAIL …]
//                  ^payload 0        ^data_floor     ^tail_off
//
// hands rclcpp the C struct pointer (constructed in the slot by the
// typesupport's `init_function`, the construct-in-slot rule), and arms the
// heap hook's borrow window over `[tail_off, payload_len)`, so
// the stock fill's `std::vector`/rosidl-sequence storage bump-allocates
// straight into the tail. At publish the SEAL turns the filled struct
// into a legal wire frame IN PLACE:
//
// - a forgeable primitive sequence whose storage the window ADOPTED
//   (an exact address-range test against the window's `[base, cursor)`)
//   is published ZERO-COPY: its offset-table entry points at the bytes
//   where the fill left them — the gap-frame shape, pinned
//   legal by the wire-legality oracle suite;
// - every other variable member (strings, nested messages, non-forgeable
//   sequences, and any forgeable member whose storage ESCAPED the window
//   — another thread, growth past the tail, a foreign allocator) is
//   COPIED into the tail AT OR ABOVE the window's bump cursor. The
//   cursor bound is load-bearing, not a placement nicety: every byte
//   BELOW the cursor was handed to some allocation on the filling
//   thread and may be a LIVE object the caller still holds (an
//   incidental `std::string` built during the fill) — writing there is
//   memory corruption, the one outcome this design forbids. Bytes between
//   the last adoption and the cursor therefore ship VERBATIM as dead
//   gap bytes (never zeroed, for the same reason), bounded by the
//   `max_gap_bytes` refusal below.
//
// The seal runs entirely AFTER the caller disarmed the window (the
// window extent arrives as plain numbers — base + bisected cursor), so
// its own heap allocations (complex-member scratch encodes, the head
// buffer) can never bump into a live window. It is plan-then-commit:
// every refusal — copies that do not fit, gap bytes over budget, a
// corrupt container header — is decided BEFORE the first mutating write,
// so a refused seal leaves the struct fully intact for the caller's
// exact-size copy-loan fallback. That fallback is the ALWAYS-SOUND spine:
// nothing in this path can make a publish fail that would have succeeded
// under the plain copy path.

/// Alignment of the window tail base within the slot payload (matches the
/// glibc `malloc` alignment the hook's bump serves, so a clean single-fill
/// lands at EXACTLY `tail_off` with zero gap).
pub const BORROW_TAIL_ALIGN: usize = 16;

/// Payload-relative geometry of a windowed borrow slot, fixed per type at
/// registration. The borrow path sizes its loan and arms the window from
/// this; the seal validates against it.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct BorrowSlotGeometry {
    /// `size_of` the C/C++ message struct constructed at payload offset 0.
    pub c_size: usize,
    /// The wire head's end: fixed section + offset table
    /// (`WireLayout::data_floor`). The seal's head write covers
    /// `[0, data_floor)`.
    pub data_floor: usize,
    /// The window base: `align_up(max(c_size, data_floor), BORROW_TAIL_ALIGN)`.
    /// Everything below is rmw-owned at seal time (the struct is dead after
    /// its values are extracted + `fini` ran) and is left DETERMINISTIC —
    /// head bytes written, the remnant `[data_floor, tail_off)` zeroed.
    pub tail_off: usize,
}

/// The armed window's extent in ABSOLUTE addresses: `base` is where the
/// window was armed (`payload + tail_off`) and `cursor` is the bump
/// watermark the caller recovered via
/// [`crate::heaphook::bisect_window_cursor`] BEFORE disarming. `None` at
/// the seal means "nothing can be adopted" (a windowless borrow — the
/// thread's window belonged to another outstanding loan — or a degraded
/// hook): every member copies, which is exactly the plain copy path's byte
/// movement inside one loan.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct WindowExtent {
    /// The armed base (absolute address of `payload + tail_off`).
    pub base: usize,
    /// The bump watermark (absolute): `[base, cursor)` is bump-allocated.
    pub cursor: usize,
}

/// What a committed seal did — the caller's latch/counter inputs plus the
/// wire `total_size` term.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct BorrowSeal {
    /// Payload-relative frame end: `total_size = WireHeader::SIZE + this`.
    pub total_payload: usize,
    /// Forgeable members published zero-copy (an empty FORGEABLE member
    /// counts here — nothing to move is trivially adopted; an empty
    /// non-forgeable member counts as a structural copy, never here).
    pub adopted: usize,
    /// Forgeable members with data that FAILED the adopt test and were
    /// copied — the escapee class the degrade latch reports.
    pub escaped: usize,
    /// Structural copies (strings, nested messages, non-forgeable
    /// sequences) — the members the wire format copies BY DESIGN; never a
    /// degrade signal.
    pub copied: usize,
    /// Dead bytes inside `total_payload` owned by no field and not part of
    /// the head: the zeroed remnant, bump-region slack below the cursor
    /// (ships verbatim), and zeroed alignment slivers between copies.
    pub gap_bytes: usize,
}

/// Why a seal refused — decided BEFORE any mutation, so the struct is
/// intact and the caller falls back to the exact-size copy loan.
#[derive(Debug)]
pub enum SealRefusal {
    /// The copied members do not fit between the copy cursor and the end of
    /// the loaned payload.
    Overflow {
        /// Payload-relative end the copies would have needed.
        needed: usize,
        /// The loaned payload length.
        available: usize,
    },
    /// The adopted layout would ship more dead gap bytes than the caller's
    /// budget — a tight copy frame is cheaper than the slack.
    ExcessiveGaps {
        /// The gap bytes this layout would ship.
        gap_bytes: usize,
        /// The caller's budget.
        max_gap_bytes: usize,
    },
    /// A container header could not be encoded (corrupt size, hostile
    /// count) — same class as a flatten [`BridgeError::Encode`].
    Encode(BridgeError),
}

impl std::fmt::Display for SealRefusal {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::Overflow { needed, available } => write!(
                f,
                "copied members need payload bytes up to {needed} but the loan holds {available}"
            ),
            Self::ExcessiveGaps {
                gap_bytes,
                max_gap_bytes,
            } => write!(
                f,
                "adopted layout would ship {gap_bytes} dead gap bytes (budget {max_gap_bytes})"
            ),
            Self::Encode(e) => write!(f, "{e}"),
        }
    }
}

/// The adoption decision, as pure arithmetic over the recovered window
/// extent — the exact test the hook's `window_range_test` answers while
/// armed (`ptr >= base && ptr + len <= cursor`), plus the wire-side guards
/// the forged take applies in the other direction: the bytes must lie
/// inside the loaned payload at/above the data floor, and the pointer must
/// be element-aligned (a misaligned entry would be refused by every
/// reader). Returns the payload-relative offset to publish, or `None` ⇒
/// copy.
pub(crate) fn adopted_offset(
    window: Option<WindowExtent>,
    payload_base: usize,
    payload_len: usize,
    data_floor: usize,
    ptr: usize,
    len: usize,
    elem_align: usize,
) -> Option<usize> {
    let w = window?;
    if len == 0 {
        return None; // empty members are handled before this test
    }
    let end = ptr.checked_add(len)?;
    if ptr < w.base || end > w.cursor {
        return None;
    }
    if elem_align > 1 && !ptr.is_multiple_of(elem_align) {
        return None;
    }
    let off = ptr.checked_sub(payload_base)?;
    if off < data_floor || off.checked_add(len)? > payload_len {
        return None;
    }
    Some(off)
}

/// Plan the copy placements into `out` (cleared first): each `(len, align)`
/// in order, starting at `copy_cursor`, aligned payload-relative (the
/// payload base is 8-aligned, so payload-relative alignment implies the
/// absolute element alignment readers verify). `Err((needed, available))`
/// when the last placement would end past `payload_len` — computed WITHOUT
/// writing anything to the slot. `out` is a reused scratch vector (the
/// steady-state zero-alloc contract — see [`SealScratch`]).
pub(crate) fn plan_copy_offsets(
    copy_cursor: usize,
    payload_len: usize,
    lens_aligns: &[(usize, usize)],
    out: &mut Vec<usize>,
) -> Result<(), (usize, usize)> {
    out.clear();
    let mut cursor = copy_cursor;
    for &(len, align) in lens_aligns {
        let off = align_up(cursor, align.max(1));
        let end = match off.checked_add(len) {
            Some(e) => e,
            None => return Err((usize::MAX, payload_len)),
        };
        out.push(off);
        cursor = end;
    }
    if cursor > payload_len {
        return Err((cursor, payload_len));
    }
    Ok(())
}

/// One planned variable member (op order == offset-table order). Shared by
/// both bridges' seals — the per-member READS differ (rosidl C headers vs
/// C++ accessors), the placement/commit arithmetic must not.
pub(crate) enum SealItem {
    /// Zero-length member: table entry `(data_floor, 0)`, nothing placed.
    /// `forgeable` records whether the member COULD have adopted (a
    /// forge-flagged sequence that happened to be empty) — the stats must
    /// not let an empty `bool[]`/bounded sequence inflate the adopted
    /// count an operator diagnoses degrades with.
    Empty {
        /// True iff the member is forge-flagged (adoption-eligible).
        forgeable: bool,
    },
    /// Forgeable member adopted in place at this payload-relative offset.
    Adopted { off: usize, len: usize },
    /// Copied from `(ptr, len)` (a container header's storage — heap, the
    /// slot tail, or the struct region; the destination is above the copy
    /// cursor, so the ranges never overlap). `escaped` marks a forgeable
    /// member that failed the adopt test (the latch signal).
    CopyRaw {
        ptr: usize,
        len: usize,
        align: usize,
        escaped: bool,
    },
    /// Copied from an owned scratch encode (complex members, `vector<bool>`
    /// fetches): `start..start + len` into the [`SealScratch`]'s reused
    /// `owned` byte arena — a range, not an owned `Vec`, so the encode
    /// buffer survives publishes with its capacity intact (the
    /// steady-state zero-alloc contract).
    CopyOwned {
        start: usize,
        len: usize,
        align: usize,
    },
}

impl SealItem {
    fn placed_len(&self) -> usize {
        match self {
            SealItem::Empty { .. } => 0,
            SealItem::Adopted { len, .. } => *len,
            SealItem::CopyRaw { len, .. } => *len,
            SealItem::CopyOwned { len, .. } => *len,
        }
    }
    fn copy_len_align(&self) -> Option<(usize, usize)> {
        match self {
            SealItem::CopyRaw { len, align, .. } => Some((*len, *align)),
            SealItem::CopyOwned { len, align, .. } => Some((*len, *align)),
            _ => None,
        }
    }
}

/// The refusal-checked seal plan scalars: the frame totals plus where the
/// copy region starts. The plan's STORAGE (items, placement offsets, the
/// owned-encode arena) lives in the caller's reused [`SealScratch`].
/// Producing a plan writes NOTHING — every refusal happens here.
#[derive(Clone, Copy)]
pub(crate) struct SealPlan {
    /// Where the copy region starts (`max(tail_off, window cursor)`).
    copy_cursor: usize,
    pub(crate) total_payload: usize,
    pub(crate) gap_bytes: usize,
}

/// Per-publisher reusable seal buffers — cleared (never dropped) per
/// publish, so a steady-state windowed publish performs no heap
/// allocation in the seal machinery (the plan items, placement offsets,
/// offset-table entries, the off-slot head build, and every owned encode
/// all reuse capacity retained from earlier publishes). Owned by
/// `PublisherInner` under the publisher mutex; both bridges' seals thread
/// through it.
///
/// The one remaining per-publish allocation source is a `Complex`
/// member whose NESTED type is variable (a `std_msgs/Header`-class
/// member): its canonical sub-frame encode goes through the shared
/// `CanonicalBodyBuilder` (the same routine — and the same cost — the
/// ingress `CdrCodec` path pays per frame), whose internals are not
/// scratch-backed.
pub struct SealScratch {
    /// One [`SealItem`] per variable member, in declaration order.
    pub(crate) items: Vec<SealItem>,
    /// `(len, align)` per Copy* item — `plan_copy_offsets` input.
    copy_shapes: Vec<(usize, usize)>,
    /// One placement offset per Copy* item, in item order.
    copy_offsets: Vec<usize>,
    /// The offset-table `(off, len)` per item, in item order (filled by
    /// [`Self::fill_entries`]; both bridges destructure it beside
    /// `head_buf` for the table write).
    pub(crate) entries: Vec<(usize, usize)>,
    /// The off-slot wire-head build (`data_floor` bytes).
    pub(crate) head_buf: Vec<u8>,
    /// Byte arena for owned encodes ([`SealItem::CopyOwned`] ranges).
    pub(crate) owned: Vec<u8>,
}

impl SealScratch {
    pub fn new() -> Self {
        Self {
            items: Vec::new(),
            copy_shapes: Vec::new(),
            copy_offsets: Vec::new(),
            entries: Vec::new(),
            head_buf: Vec::new(),
            owned: Vec::new(),
        }
    }

    /// Clear every buffer for a fresh seal, KEEPING capacity — the whole
    /// point of the type.
    pub(crate) fn reset(&mut self) {
        self.items.clear();
        self.copy_shapes.clear();
        self.copy_offsets.clear();
        self.entries.clear();
        self.head_buf.clear();
        self.owned.clear();
    }

    /// Commit every Copy* item into the slot, in plan order, zeroing the
    /// rmw-owned alignment slivers between them (at/above the cursor —
    /// never handed to any allocation) for deterministic frames.
    ///
    /// # Safety
    /// `payload` must be the slot payload the plan was made for; the
    /// source ranges must still be readable.
    pub(crate) unsafe fn commit_copies(&self, plan: &SealPlan, payload: *mut u8) {
        let mut prev_end = plan.copy_cursor;
        let mut copy_i = 0usize;
        for item in &self.items {
            let (src_ptr, len): (*const u8, usize) = match item {
                SealItem::CopyRaw { ptr, len, .. } => (*ptr as *const u8, *len),
                SealItem::CopyOwned { start, len, .. } => (self.owned.as_ptr().add(*start), *len),
                _ => continue,
            };
            let off = self.copy_offsets[copy_i];
            copy_i += 1;
            if off > prev_end {
                std::ptr::write_bytes(payload.add(prev_end), 0, off - prev_end);
            }
            // Non-overlapping: the destination is at/above the cursor, the
            // source is a heap buffer (the arena included), the struct
            // region, or bump storage strictly below the cursor.
            std::ptr::copy_nonoverlapping(src_ptr, payload.add(off), len);
            prev_end = off + len;
        }
    }

    /// Fill `self.entries` with the offset-table `(off, len)` per item, in
    /// item (declaration) order. Empty members point at the data floor
    /// with length 0.
    pub(crate) fn fill_entries(&mut self, data_floor: usize) {
        let Self {
            items,
            copy_offsets,
            entries,
            ..
        } = self;
        entries.clear();
        let mut copy_i = 0usize;
        for item in items.iter() {
            entries.push(match item {
                SealItem::Empty { .. } => (data_floor, 0),
                SealItem::Adopted { off, len } => (*off, *len),
                SealItem::CopyRaw { len, .. } | SealItem::CopyOwned { len, .. } => {
                    let o = copy_offsets[copy_i];
                    copy_i += 1;
                    (o, *len)
                }
            });
        }
    }

    /// `(adopted, escaped, copied)` — the caller's latch/counter inputs.
    pub(crate) fn stats(&self) -> (usize, usize, usize) {
        let (mut adopted, mut escaped, mut copied) = (0usize, 0usize, 0usize);
        for item in &self.items {
            match item {
                // An empty FORGEABLE member is trivially adopted (nothing
                // to move); an empty NON-forgeable one is a zero-byte
                // structural copy — it was never adoption-eligible, so
                // counting it adopted would overstate zero-copy exactly
                // where an operator reads the number to diagnose degrades.
                SealItem::Empty { forgeable: true } | SealItem::Adopted { .. } => adopted += 1,
                SealItem::Empty { forgeable: false } => copied += 1,
                SealItem::CopyRaw { escaped: true, .. } => escaped += 1,
                SealItem::CopyRaw { escaped: false, .. } | SealItem::CopyOwned { .. } => {
                    copied += 1
                }
            }
        }
        (adopted, escaped, copied)
    }

    /// The seal outcome for a committed plan.
    pub(crate) fn outcome(&self, plan: &SealPlan) -> BorrowSeal {
        let (adopted, escaped, copied) = self.stats();
        BorrowSeal {
            total_payload: plan.total_payload,
            adopted,
            escaped,
            copied,
            gap_bytes: plan.gap_bytes,
        }
    }
}

impl Default for SealScratch {
    fn default() -> Self {
        Self::new()
    }
}

/// Plan a seal over the scratch's staged `items`: place every Copy* item
/// at/above the bump cursor, compute the frame end and the dead-gap
/// total, and refuse (Overflow / ExcessiveGaps) before anything is
/// written.
pub(crate) fn plan_seal(
    scratch: &mut SealScratch,
    geo: &BorrowSlotGeometry,
    window: Option<WindowExtent>,
    payload_base: usize,
    payload_len: usize,
    max_gap_bytes: usize,
) -> Result<SealPlan, SealRefusal> {
    // Copies start at/above the bump cursor (never below — live objects),
    // and at/above the tail base when nothing was bumped.
    let copy_cursor = window
        .map(|w| w.cursor.saturating_sub(payload_base))
        .unwrap_or(geo.tail_off)
        .max(geo.tail_off);
    let SealScratch {
        items,
        copy_shapes,
        copy_offsets,
        ..
    } = scratch;
    copy_shapes.clear();
    copy_shapes.extend(items.iter().filter_map(SealItem::copy_len_align));
    plan_copy_offsets(copy_cursor, payload_len, copy_shapes, copy_offsets)
        .map_err(|(needed, available)| SealRefusal::Overflow { needed, available })?;

    let mut total_payload = geo.data_floor;
    let mut placed_bytes = 0usize;
    {
        let mut copy_i = 0usize;
        for item in items.iter() {
            placed_bytes += item.placed_len();
            match item {
                SealItem::Adopted { off, len } => total_payload = total_payload.max(off + len),
                SealItem::CopyRaw { len, .. } | SealItem::CopyOwned { len, .. } => {
                    total_payload = total_payload.max(copy_offsets[copy_i] + len);
                    copy_i += 1;
                }
                SealItem::Empty { .. } => {}
            }
        }
    }
    let gap_bytes = total_payload - geo.data_floor - placed_bytes;
    if gap_bytes > max_gap_bytes {
        return Err(SealRefusal::ExcessiveGaps {
            gap_bytes,
            max_gap_bytes,
        });
    }
    Ok(SealPlan {
        copy_cursor,
        total_payload,
        gap_bytes,
    })
}

impl BridgedMessage {
    /// May `rmw_borrow_loaned_message` serve this type
    /// through the borrow window? A FIXED type keeps the plain loan path
    /// ([`Self::can_loan`]); a windowed borrow needs at least one
    /// forgeable primitive sequence to adopt (`forge_count > 0`, which
    /// also guarantees the typesupport's `init_function`/`fini_function`
    /// — the slot construction and release this path runs).
    pub fn can_borrow_windowed(&self) -> bool {
        !self.can_loan && self.forge_count > 0
    }

    /// The slot geometry for a windowed borrow, or `None` when
    /// [`Self::can_borrow_windowed`] does not hold.
    pub fn borrow_geometry(&self) -> Option<BorrowSlotGeometry> {
        if !self.can_borrow_windowed() {
            return None;
        }
        let data_floor = self.layout.data_floor();
        Some(BorrowSlotGeometry {
            c_size: self.c_size,
            data_floor,
            tail_off: align_up(self.c_size.max(data_floor), BORROW_TAIL_ALIGN),
        })
    }

    /// Seal a filled windowed-borrow slot into a wire frame IN PLACE — see
    /// the section comment above for the full mechanism. Plan-then-commit:
    /// on `Err` NOTHING was written and the struct at `payload` is intact
    /// (the caller copy-flattens from it); on `Ok` the struct region has
    /// been consumed (values extracted, forgeable headers emptied, the
    /// typesupport's `fini` run, the wire head written over it) and the
    /// caller must only stamp the `WireHeader` and send.
    ///
    /// `window` is the recovered extent (`None` ⇒ nothing adopts);
    /// `max_gap_bytes` bounds the dead bytes a zero-copy layout may ship.
    ///
    /// # Safety
    /// `payload` must be the borrow slot's payload region of exactly
    /// `payload_len >= geo.tail_off` bytes, holding a valid, initialized
    /// message of this bridged type at offset 0 (the state
    /// [`Self::init_loaned_payload`] + the caller's fill leave it in);
    /// the caller must have DISARMED the window; every adopted range and
    /// every container the headers reference must be readable.
    pub unsafe fn seal_borrowed_frame(
        &self,
        payload: *mut u8,
        payload_len: usize,
        geo: &BorrowSlotGeometry,
        window: Option<WindowExtent>,
        max_gap_bytes: usize,
        scratch: &mut SealScratch,
    ) -> Result<BorrowSeal, SealRefusal> {
        let payload_base = payload as usize;
        let data_floor = geo.data_floor;
        let c_msg = payload as *const c_void;

        // ── PLAN (read-only) ────────────────────────────────────────────
        scratch.reset();
        let SealScratch { items, owned, .. } = &mut *scratch;
        for op in &self.ops {
            match op {
                FieldOp::FixedCopy { .. } => {}
                FieldOp::String { c_offset, .. } => {
                    let s = &*(c_msg.add(*c_offset) as *const RosString);
                    let bytes =
                        ros_string_bytes(s).map_err(|d| SealRefusal::Encode(self.encode_err(d)))?;
                    items.push(SealItem::CopyRaw {
                        ptr: bytes.as_ptr() as usize,
                        len: bytes.len(),
                        align: 1,
                        escaped: false,
                    });
                }
                FieldOp::PrimSeq {
                    c_offset,
                    elem_size,
                    forge,
                    ..
                } => {
                    let seq = &*(c_msg.add(*c_offset) as *const RosSequence);
                    // Pre-mutation refusal like every other seal refusal:
                    // a Buffer-backed instance is neither adopted nor copied.
                    if prim_seq_is_rosidl_buffer(seq) {
                        return Err(SealRefusal::Encode(self.encode_err(
                            "sequence instance is a rosidl Buffer, not readable as bytes",
                        )));
                    }
                    let byte_len = seq
                        .size
                        .checked_mul(*elem_size)
                        .filter(|&n| n <= MAX_FRAME_BYTES)
                        .ok_or_else(|| {
                            SealRefusal::Encode(self.encode_err(
                                "sequence length × element size overflows or exceeds cap",
                            ))
                        })?;
                    if byte_len == 0 {
                        items.push(SealItem::Empty { forgeable: *forge });
                    } else if seq.data.is_null() {
                        return Err(SealRefusal::Encode(
                            self.encode_err("sequence has null data with nonzero size"),
                        ));
                    } else if *forge {
                        match adopted_offset(
                            window,
                            payload_base,
                            payload_len,
                            data_floor,
                            seq.data as usize,
                            byte_len,
                            *elem_size,
                        ) {
                            Some(off) => items.push(SealItem::Adopted { off, len: byte_len }),
                            None => items.push(SealItem::CopyRaw {
                                ptr: seq.data as usize,
                                len: byte_len,
                                align: *elem_size,
                                escaped: true,
                            }),
                        }
                    } else {
                        items.push(SealItem::CopyRaw {
                            ptr: seq.data as usize,
                            len: byte_len,
                            align: *elem_size,
                            escaped: false,
                        });
                    }
                }
                FieldOp::Complex {
                    c_offset,
                    member_index,
                    ..
                } => {
                    let member = self.member(*member_index);
                    // Encode appends into the reused arena; the item is a
                    // range, so the buffer's capacity survives publishes.
                    // (`check_out_cap` inside is cumulative across
                    // members — strictly stricter, and it matches the
                    // whole-frame accounting `frame_size` applies on the
                    // copy fallback.)
                    let start = owned.len();
                    encode_complex(&self.nested_layouts, member, c_msg.add(*c_offset), owned)
                        .map_err(|d| SealRefusal::Encode(self.encode_err(d)))?;
                    items.push(SealItem::CopyOwned {
                        start,
                        len: owned.len() - start,
                        align: 1,
                    });
                }
            }
        }

        let plan = plan_seal(
            scratch,
            geo,
            window,
            payload_base,
            payload_len,
            max_gap_bytes,
        )?;

        // ── COMMIT (mutating; nothing below can fail) ───────────────────
        // 1. Copies, in plan order (alignment slivers zeroed — see
        //    `SealScratch::commit_copies`).
        scratch.commit_copies(&plan, payload);

        // 2. Build the wire head OFF-SLOT first: the head region overlaps
        //    the struct the fixed copies still read from.
        scratch.head_buf.resize(data_floor, 0);
        scratch.fill_entries(data_floor);
        let SealScratch {
            entries, head_buf, ..
        } = &mut *scratch;
        for op in &self.ops {
            if let FieldOp::FixedCopy {
                c_offset,
                wire_offset,
                size,
                pad_ranges,
            } = op
            {
                std::ptr::copy_nonoverlapping(
                    payload.add(*c_offset),
                    head_buf.as_mut_ptr().add(*wire_offset),
                    *size,
                );
                // repr(C) padding inside the span is uninitialized C-side
                // garbage — zero it (padding determinism).
                for &(off, len) in pad_ranges {
                    head_buf[wire_offset + off..wire_offset + off + len].fill(0);
                }
            }
        }
        {
            let mut item_i = 0usize;
            for op in &self.ops {
                let var_idx = match op {
                    FieldOp::FixedCopy { .. } => continue,
                    FieldOp::String { var_idx, .. }
                    | FieldOp::PrimSeq { var_idx, .. }
                    | FieldOp::Complex { var_idx, .. } => *var_idx,
                };
                let (off, len) = entries[item_i];
                item_i += 1;
                let entry_base = self.layout.fixed_size + var_idx * 8;
                head_buf[entry_base..entry_base + 4].copy_from_slice(&(off as u32).to_le_bytes());
                head_buf[entry_base + 4..entry_base + 8]
                    .copy_from_slice(&(len as u32).to_le_bytes());
            }
        }

        // 3. Empty every top-level container header whose storage lies
        //    INSIDE the slot, so the `fini` below can never hand slot
        //    (shared-memory) bytes to the allocator — the take side's
        //    un-forge rule, widened to every in-slot case: an ADOPTED
        //    member (its bytes now belong to the frame), an in-window but
        //    misaligned escapee, an in-slot bump-allocated string. Slot
        //    bytes are never allocator-owned, so emptying leaks nothing;
        //    every copy was committed in step 1, so nothing is lost; and
        //    heap-side storage keeps its headers — `fini` freeing it is
        //    exactly the point. (A nested member's INTERNAL containers
        //    cannot be reached from here; the hook's quarantine no-op
        //    covers them in production.)
        let slot_range = payload_base..payload_base + payload_len;
        for op in &self.ops {
            match op {
                FieldOp::PrimSeq { c_offset, .. } => {
                    let seq = payload.add(*c_offset) as *mut RosSequence;
                    if slot_range.contains(&((*seq).data as usize)) {
                        *seq = EMPTY_ROS_SEQUENCE;
                    }
                }
                FieldOp::String { c_offset, .. } => {
                    let s = payload.add(*c_offset) as *mut RosString;
                    if slot_range.contains(&((*s).data as usize)) {
                        *s = EMPTY_ROS_STRING;
                    }
                }
                _ => {}
            }
        }

        // 4. Release the struct through the typesupport's own `fini` while
        //    its headers are still readable: heap-side containers (escaped
        //    copies' sources, strings) are freed on the matching
        //    allocator; storage the window bump-allocated into the slot is
        //    routed to the hook's quarantine no-op, never glibc.
        if let Some(fini) = (*self.members).fini_function {
            fini(payload as *mut c_void);
        }

        // 5. Now the struct is dead: write the head over it and zero the
        //    remnant up to the tail base (deterministic frames — the
        //    remnant would otherwise ship rosidl header residue, i.e.
        //    process addresses, into SHM).
        std::ptr::copy_nonoverlapping(head_buf.as_ptr(), payload, data_floor);
        std::ptr::write_bytes(payload.add(data_floor), 0, geo.tail_off - data_floor);

        Ok(scratch.outcome(&plan))
    }

    /// Release a windowed-borrow struct WITHOUT sealing — the refusal /
    /// cancel path: the caller has finished reading the struct (or never
    /// needed to) and the loan is about to be dropped or replaced by a
    /// copy loan.
    ///
    /// Before running the typesupport's `fini`, every forge-flagged
    /// member whose storage lies INSIDE the slot (`[payload, payload +
    /// payload_len)`) is emptied: slot bytes are shared-memory pool
    /// storage, never allocator-owned, so emptying leaks nothing — and it
    /// keeps `fini` from handing an SHM address to `free` even without
    /// the hook (defense in depth; the hook's quarantine no-op still
    /// covers what this walk cannot see — a string or a nested member's
    /// container the fill bump-allocated into the slot). Heap-side
    /// storage (escapees, ordinary strings) keeps its headers: `fini`
    /// freeing it is exactly the point.
    ///
    /// # Safety
    /// `payload` must hold a valid, initialized message of this bridged
    /// type in a slot of `payload_len` bytes, and must not be used as a
    /// message afterwards.
    pub unsafe fn fini_borrowed_payload(&self, payload: *mut c_void, payload_len: usize) {
        let slot_range = (payload as usize)..(payload as usize + payload_len);
        for op in &self.ops {
            match op {
                FieldOp::PrimSeq { c_offset, .. } => {
                    let seq = payload.add(*c_offset) as *mut RosSequence;
                    if slot_range.contains(&((*seq).data as usize)) {
                        *seq = EMPTY_ROS_SEQUENCE;
                    }
                }
                FieldOp::String { c_offset, .. } => {
                    let s = payload.add(*c_offset) as *mut RosString;
                    if slot_range.contains(&((*s).data as usize)) {
                        *s = EMPTY_ROS_STRING;
                    }
                }
                _ => {}
            }
        }
        if let Some(fini) = (*self.members).fini_function {
            fini(payload);
        }
    }
}

#[cfg(test)]
mod borrow_seal_plan_tests {
    use super::*;

    // ── adopted_offset: the pure adoption decision (each guard is one
    //    mutation target — dropping any admits a pointer every reader
    //    refuses or, worse, one outside the frame) ───────────────────────

    const BASE: usize = 0x1000; // payload base (absolute)
    const LEN: usize = 0x1000; // payload length
    const FLOOR: usize = 40; // data floor
    const W: WindowExtent = WindowExtent {
        base: BASE + 0x100,
        cursor: BASE + 0x900,
    };

    fn adopt(ptr: usize, len: usize, align: usize) -> Option<usize> {
        adopted_offset(Some(W), BASE, LEN, FLOOR, ptr, len, align)
    }

    #[test]
    fn a_bump_allocation_inside_the_window_adopts_at_its_payload_offset() {
        assert_eq!(adopt(BASE + 0x100, 0x200, 4), Some(0x100));
        assert_eq!(adopt(BASE + 0x400, 8, 8), Some(0x400));
    }

    #[test]
    fn the_window_bounds_are_half_open_and_exact() {
        // Ends exactly at the cursor: adopted.
        assert_eq!(adopt(BASE + 0x8fc, 4, 4), Some(0x8fc));
        // One byte past the cursor: escaped.
        assert_eq!(adopt(BASE + 0x8fd, 4, 1), None);
        // Below the window base: escaped (heap or another slot).
        assert_eq!(adopt(BASE + 0xff, 4, 1), None);
    }

    #[test]
    fn no_window_means_nothing_adopts() {
        assert_eq!(
            adopted_offset(None, BASE, LEN, FLOOR, BASE + 0x100, 8, 1),
            None
        );
    }

    #[test]
    fn a_misaligned_element_pointer_is_never_adopted() {
        assert_eq!(adopt(BASE + 0x101, 8, 4), None, "not 4-aligned");
        assert_eq!(adopt(BASE + 0x101, 8, 1), Some(0x101), "align 1 is free");
    }

    #[test]
    fn zero_length_and_wrapping_ranges_are_refused() {
        assert_eq!(adopt(BASE + 0x100, 0, 1), None, "empties are pre-handled");
        assert_eq!(
            adopted_offset(Some(W), BASE, LEN, FLOOR, usize::MAX, 2, 1),
            None
        );
    }

    #[test]
    fn defensive_floor_and_payload_bounds_hold_even_for_window_hits() {
        // A window claiming to sit below the data floor (impossible from a
        // correctly-armed window; defense in depth) is refused.
        let low = WindowExtent {
            base: BASE + 8,
            cursor: BASE + 64,
        };
        assert_eq!(
            adopted_offset(Some(low), BASE, LEN, FLOOR, BASE + 8, 8, 1),
            None
        );
        // A window past the payload end is refused too.
        let high = WindowExtent {
            base: BASE + LEN,
            cursor: BASE + LEN + 0x100,
        };
        assert_eq!(
            adopted_offset(Some(high), BASE, LEN, FLOOR, BASE + LEN, 8, 1),
            None
        );
    }

    // ── plan_copy_offsets: placement arithmetic ────────────────────────

    /// The out-param form, folded back to `Result<Vec, _>` for oracle
    /// comparison. A dirty pre-seeded scratch also pins the clear-first
    /// contract (a stale offset surviving a re-plan would misplace every
    /// later copy).
    fn plan_offsets(
        cursor: usize,
        payload_len: usize,
        shapes: &[(usize, usize)],
    ) -> Result<Vec<usize>, (usize, usize)> {
        let mut out = vec![usize::MAX; 3];
        plan_copy_offsets(cursor, payload_len, shapes, &mut out)?;
        Ok(out)
    }

    #[test]
    fn placements_pack_in_order_with_alignment() {
        // cursor 100 → [100,110); align 8 → 112 → [112,120); align 1 → 120.
        assert_eq!(
            plan_offsets(100, 1000, &[(10, 1), (8, 8), (3, 1)]),
            Ok(vec![100, 112, 120])
        );
    }

    #[test]
    fn an_exact_fit_is_accepted_and_one_byte_over_is_refused() {
        assert_eq!(plan_offsets(100, 110, &[(10, 1)]), Ok(vec![100]));
        assert_eq!(plan_offsets(100, 109, &[(10, 1)]), Err((110, 109)));
    }

    #[test]
    fn empty_copy_set_plans_nothing_and_clears_stale_state() {
        assert_eq!(plan_offsets(500, 1000, &[]), Ok(vec![]));
    }
}

#[cfg(all(test, cerulion_has_is_rosidl_buffer))]
mod lyrical_buffer_tests {
    use super::*;

    fn sequence_member(
        type_id: u8,
        is_rosidl_buffer: bool,
    ) -> ffi::rosidl_typesupport_introspection_c__MessageMember {
        ffi::rosidl_typesupport_introspection_c__MessageMember {
            type_id_: type_id,
            is_array_: true,
            array_size_: 0,
            is_upper_bound_: false,
            default_value_: std::ptr::null(),
            is_rosidl_buffer_: is_rosidl_buffer,
            ..Default::default()
        }
    }

    /// The typesupport's Buffer flag is the ONLY difference between the two
    /// members: the flagged one is never forged, the plain one is.
    #[test]
    fn a_rosidl_buffer_member_is_never_forged_on_the_c_path() {
        let flagged = sequence_member(ros_type::UINT8, true);
        let plain = sequence_member(ros_type::UINT8, false);
        unsafe {
            assert!(!is_forgeable_sequence(&flagged, &FieldType::Bytes));
            assert!(is_forgeable_sequence(&plain, &FieldType::Bytes));
        }
    }

    /// A Buffer-backed primitive INSTANCE is refused by the fill and by the
    /// encode read, and its pointer is never freed (a bogus non-null pointer
    /// would crash the test if it were).
    #[test]
    fn a_rosidl_buffer_instance_is_refused_never_freed_or_read() {
        let mut seq = RosPrimitiveSequence {
            data: 0x10 as *mut u8,
            size: 3,
            capacity: 3,
            is_rosidl_buffer: true,
            owns_rosidl_buffer: false,
        };
        let header = &mut seq as *mut RosPrimitiveSequence as *mut RosSequence;
        unsafe {
            assert!(prim_seq_is_rosidl_buffer(header));
            assert!(!assign_prim_sequence(header, &[1, 2, 3], 1));
        }
        assert_eq!(
            seq.data as usize, 0x10,
            "the Buffer pointer is left untouched"
        );
        let member = sequence_member(ros_type::UINT8, true);
        let view = unsafe { sequence_view(&member, header as *const c_void) };
        assert!(
            view.is_err(),
            "a Buffer-backed instance cannot be read as bytes"
        );
        unsafe {
            (*(header as *mut RosPrimitiveSequence)).is_rosidl_buffer = false;
            assert!(!prim_seq_is_rosidl_buffer(header));
        }
    }

    /// A MESSAGE sequence is 24 bytes and carries no flag: whatever byte
    /// follows its header (here a deliberately non-zero one) is never read
    /// as a Buffer flag.
    #[test]
    fn a_message_sequence_never_reads_a_flag_past_its_header() {
        #[repr(C)]
        struct Fixture {
            poses: RosSequence,
            next_field: u64,
        }
        let fixture = Fixture {
            poses: RosSequence {
                data: std::ptr::null_mut(),
                size: 0,
                capacity: 0,
            },
            next_field: u64::MAX,
        };
        let member = sequence_member(ros_type::MESSAGE, false);
        let header = &fixture.poses as *const RosSequence;
        unsafe {
            assert!(!seq_is_rosidl_buffer(&member, header));
        }
        assert_eq!(fixture.next_field, u64::MAX);
    }
}
