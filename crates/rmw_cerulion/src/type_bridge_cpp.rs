// SPDX-License-Identifier: AGPL-3.0-only
//! rosidl introspection_**cpp** → Cerulion wire format bridge:
//! the typesupport rclcpp — and therefore MoveIt 2
//! — hands rmw.
//!
//! Produces frames BYTE-IDENTICAL to [`crate::type_bridge`]'s C bridge
//! (same FQN schema hashes, same [`WireLayout`], same canonical complex
//! encoding) so C++ nodes, Python nodes, and native Cerulion nodes all
//! interoperate on the same topics.
//!
//! # ABI posture
//!
//! C++ generated messages contain `std::string` / `std::vector` — NOT
//! plain C structs. Access rules, strictest-first:
//!
//! - **Containers (vector/array)**: exclusively through the member's
//!   introspection function pointers (`size/get/get_const/fetch/assign/
//!   resize`) — the typesupport library's own compiled C++ does the
//!   container walking; no layout assumptions in Rust. `std::vector<T>`
//!   element-0 pointers are contiguous by the C++ standard (T ≠ bool);
//!   `std::vector<bool>` is bit-packed and goes element-by-element via
//!   `fetch`/`assign`.
//! - **`std::string`**: via the COMPILED shim (`shim/cppstring_shim.cpp`)
//!   — the platform's real string ABI by construction.
//! - **Fixed (primitive/nested-fixed) spans**: direct memcpy at the
//!   introspection offsets, with repr(C) padding zeroed via the same
//!   `pad_ranges` machinery as the C bridge (Principle #7).
//!
//! The fixed sections of C++ and C generated structs coincide (both are
//! standard-layout primitives at repr(C) offsets), so `can_loan` (true
//! zero-copy) applies to recursively-fixed messages exactly as on the C
//! side.

use std::collections::BTreeMap;
use std::os::raw::c_void;

use cerulion_core::codegen::layout::{LayoutResolver, VariableFieldLayout, WireLayout};
use cerulion_core::codegen::{
    CanonicalBodyBuilder, CanonicalBodyReader, FieldDef, FieldType, MessageSchema,
};
use cerulion_core::wire::WireHeader;

use crate::ffi;
use crate::ffi::introspection_cpp::{
    assign_u8_vector, cppstring_bytes, debug_assert_vector_u8_layout, is_unbounded_u8_vector,
    rmw_cerulion_cppstring_assign, vector_triplet_layout_verified, CppMessageMember,
    CppMessageMembers, VecTriplet,
};
use crate::type_bridge::{
    add_var_size, adopted_offset, align_up, check_out_cap, check_seq_bound, cursor_align_var,
    cursor_append_var, cursor_write_var_entry, forge_placement, frame_head_len, is_primitive_type,
    mask_bit, mask_has, padding_ranges, plan_seal, primitive_size, read_u32_prefix, read_var_entry,
    ros_type, shadow_layout, warn_bad_entry, BorrowSeal, BorrowSlotGeometry, BridgeError,
    ForgeOutcome, ForgePlacement, FrameCursor, NestedLayouts, SealItem, SealRefusal, SealScratch,
    WindowExtent, BORROW_TAIL_ALIGN, MAX_FRAME_BYTES,
};

/// One flatten/unflatten operation for a top-level C++ field.
#[derive(Debug)]
enum CppFieldOp {
    /// Fixed-section span: memcpy + padding zeroing (identical to the C
    /// bridge — fixed spans are plain standard-layout bytes in C++ too).
    FixedCopy {
        c_offset: usize,
        wire_offset: usize,
        size: usize,
        pad_ranges: Vec<(usize, usize)>,
    },
    /// `std::string` field → variable entry (raw UTF-8), via the shim.
    String { c_offset: usize, var_idx: usize },
    /// Primitive sequence (`std::vector<T>` / bounded) → variable entry
    /// (raw LE element bytes, element-aligned), via function pointers.
    PrimSeq {
        c_offset: usize,
        var_idx: usize,
        elem_size: usize,
        /// `std::vector<bool>` is bit-packed: element-by-element
        /// fetch/assign instead of contiguous memcpy.
        is_bool: bool,
        member_index: usize,
        /// On the FORGED loaned take this member's
        /// `std::vector` triplet is aimed at the held SHM sample instead
        /// of being filled (see [`is_forgeable_sequence_cpp`]). The copying
        /// take (`unflatten`) ignores the flag.
        forge: bool,
    },
    /// Complex variable field — canonical v1 encoding, recursing through
    /// nested introspection.
    Complex {
        c_offset: usize,
        var_idx: usize,
        member_index: usize,
    },
}

/// A C++ message type registered with the bridge. Mirror of
/// [`crate::type_bridge::BridgedMessage`] over cpp introspection.
pub struct CppBridgedMessage {
    pub qualified_name: String,
    pub layout: WireLayout,
    pub c_size: usize,
    pub can_loan: bool,
    members: *const CppMessageMembers,
    ops: Vec<CppFieldOp>,
    /// Wire layouts for every transitively nested type — see the
    /// C twin's [`crate::type_bridge::NestedLayouts`].
    nested_layouts: NestedLayouts,
    /// Whole-struct repr(C) padding ranges for the loaned
    /// publish path — see the C twin's field doc
    /// (`crate::type_bridge::BridgedMessage`). Empty unless `can_loan`.
    loan_pad_ranges: Vec<(usize, usize)>,
    /// Forgeable-sequence count — see the C twin's
    /// `forge_count` and [`Self::can_loan_take`].
    forge_count: usize,
}

// SAFETY: `members` points at rosidl's static typesupport data
// (immutable, process-lifetime); the bridge only reads through it.
unsafe impl Send for CppBridgedMessage {}
unsafe impl Sync for CppBridgedMessage {}

impl CppBridgedMessage {
    /// Build the bridge from a C++ introspection MessageMembers.
    ///
    /// # Safety
    /// `members` must point at valid, process-lifetime introspection
    /// data (the rosidl typesupport contract).
    pub unsafe fn new(members: *const CppMessageMembers) -> Result<Self, BridgeError> {
        if members.is_null() {
            return Err(BridgeError::NoIntrospection);
        }

        // Debug-only startup guard: verify std::vector<uint8_t>
        // has the default 3-pointer layout the uint8[] assign() fast path
        // reinterprets. Run-once; compiled out in release.
        debug_assert_vector_u8_layout();
        // The forged take WRITES vector triplets, so its
        // layout guard runs in every build (run-once; a failure disables
        // forging for every C++ type, loudly, and the copying take serves).
        let triplet_layout_ok = vector_triplet_layout_verified();

        // 1. Schema set (this type + transitive nested types).
        let mut schemas: BTreeMap<String, MessageSchema> = BTreeMap::new();
        let mut type_ptrs: BTreeMap<usize, String> = BTreeMap::new();
        collect_schemas_cpp(members, &mut schemas, &mut type_ptrs)?;
        let root_qualified = qualified_name_of_cpp(members)?;

        // 2. Same layout pipeline as native codegen + the C bridge.
        let schema_vec: Vec<MessageSchema> = schemas.into_values().collect();
        let (mut resolver, warnings) = LayoutResolver::new(schema_vec);
        if !warnings.is_empty() {
            return Err(BridgeError::Resolution(warnings));
        }
        let layout = resolver
            .layout_of(&root_qualified)
            .ok_or(BridgeError::NoIntrospection)?;

        // 2b. Retain a layout per nested introspection type so the
        //     nested/element body encoder writes the CANONICAL shape.
        let mut nested_layouts = NestedLayouts::default();
        for (ptr, qname) in &type_ptrs {
            let l = resolver
                .layout_of(qname)
                .ok_or(BridgeError::NoIntrospection)?;
            verify_nested_lockstep_cpp(*ptr as *const CppMessageMembers, qname, &l)?;
            nested_layouts.insert(*ptr, l);
        }

        // 3. Per-field plan with verified offsets/sizes.
        let m = &*members;
        let c_size = m.size_of_;
        let member_slice = std::slice::from_raw_parts(m.members_, m.member_count_ as usize);
        // (Layout-drift plausibility is enforced in collect_schemas_cpp,
        // which already visited the root AND every nested type in step 1.)

        let mut ops = Vec::with_capacity(member_slice.len());
        let mut fixed_iter = layout.fixed_fields.iter().peekable();
        let mut var_idx = 0usize;
        let mut loanable = layout.is_fixed() && layout.fixed_size == c_size;
        let mut forge_count = 0usize;

        for (i, member) in member_slice.iter().enumerate() {
            let fname = ffi::cstr(member.name_).unwrap_or("<field>").to_string();
            let c_offset = member.offset_ as usize;

            if is_variable_member_cpp(member) {
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
                let forge = is_forgeable_sequence_cpp(member, triplet_layout_ok);
                let op = plan_variable_op_cpp(c_offset, var_idx, i, v, forge);
                if matches!(op, CppFieldOp::PrimSeq { forge: true, .. }) {
                    forge_count += 1;
                }
                ops.push(op);
                var_idx += 1;
                loanable = false;
            } else {
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
                let c_field_size = fixed_member_size_cpp(member);
                if fl.size != c_field_size {
                    return Err(BridgeError::LayoutMismatch {
                        message: root_qualified.clone(),
                        detail: format!(
                            "fixed-field size mismatch for '{fname}': layout {} vs C++ {}",
                            fl.size, c_field_size
                        ),
                    });
                }
                if fl.offset != c_offset {
                    loanable = false;
                }
                let mut runs = Vec::new();
                member_data_runs_cpp(member, 0, &mut runs);
                let pad_ranges = padding_ranges(&mut runs, fl.size);
                ops.push(CppFieldOp::FixedCopy {
                    c_offset,
                    wire_offset: fl.offset,
                    size: fl.size,
                    pad_ranges,
                });
            }
        }

        // Whole-struct padding map for the loaned publish path
        // (same computation as `zero_struct_padding_cpp`, hoisted to
        // registration so publish does no per-frame walk or allocation).
        let loan_pad_ranges = if loanable {
            let mut runs = Vec::new();
            for nm in member_slice {
                member_data_runs_cpp(nm, nm.offset_ as usize, &mut runs);
            }
            padding_ranges(&mut runs, c_size)
        } else {
            Vec::new()
        };

        // A shadow is placement-constructed and destroyed
        // through the typesupport's own init/fini (the C++ ALL constructor
        // and destructor) — a C++ typesupport missing either cannot host one
        // (a zeroed `std::string` is not even a valid object on libstdc++),
        // so the type keeps the copying take. Real rosidl typesupports
        // always carry both; this is a hand-built-fixture / drift guard.
        if forge_count > 0 && (m.init_function.is_none() || m.fini_function.is_none()) {
            tracing::debug!(
                schema = %root_qualified,
                "typesupport lacks init_function/fini_function; forged loaned take disabled \
                 for this type (copying take instead)"
            );
            forge_count = 0;
            for op in &mut ops {
                if let CppFieldOp::PrimSeq { forge, .. } = op {
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

    pub fn schema_hash(&self) -> u64 {
        self.layout.schema_hash
    }

    /// The TAKE-side loan gate — see the C twin
    /// (`crate::type_bridge::BridgedMessage::can_loan_take`). Fixed types
    /// hand out the SHM pointer directly; a type with at least one
    /// forgeable `std::vector` member is served through an rmw-owned
    /// shadow whose forged triplets aim at the held sample.
    pub fn can_loan_take(&self) -> bool {
        self.can_loan || self.forge_count > 0
    }

    /// Number of top-level members the forged take aims at SHM.
    pub fn forged_sequence_count(&self) -> usize {
        self.forge_count
    }

    /// Construct one rmw-owned SHADOW C++ message —
    /// `c_size` bytes at `SHADOW_ALIGN`, then the typesupport's
    /// `init_function` with `ALL`, which placement-news the object (every
    /// `std::string`/`std::vector` member becomes a real, empty container
    /// on this process's C++ runtime). `None` only on allocation failure.
    ///
    /// # Safety
    /// [`Self::can_loan_take`] must hold with a nonzero
    /// [`Self::forged_sequence_count`] (which guarantees an `init_function`
    /// and a `fini_function`).
    pub unsafe fn new_shadow(&self) -> Option<*mut c_void> {
        let layout = shadow_layout(self.c_size);
        // hot-path-alloc-ok: shadows are built at most once per borrow-budget
        // slot per subscription and then RECYCLED; a steady-state take never
        // reaches this.
        let ptr = std::alloc::alloc(layout);
        if ptr.is_null() {
            return None;
        }
        let Some(init) = (*self.members).init_function else {
            // Unreachable behind the registration gate; never hand out a
            // never-constructed C++ object.
            std::alloc::dealloc(ptr, layout);
            return None;
        };
        init(ptr as *mut c_void, ffi::introspection_cpp::CPP_MSG_INIT_ALL);
        Some(ptr as *mut c_void)
    }

    /// Destroy a shadow from [`Self::new_shadow`]: un-forge (so `~vector`
    /// sees the null triplet and deallocates NOTHING in shared memory), run
    /// the typesupport's `fini_function` (the destructor — it frees the
    /// copied strings/nested containers on the C++ allocator that made
    /// them), deallocate the object's bytes.
    ///
    /// `forged` is the mask of the last take served through this shadow
    /// (`ForgeOutcome::forged`; `0` for a never-forged or already un-forged
    /// shadow) — members outside it hold copies `~vector` owns and frees.
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

    /// The forged take into a C++ shadow — the twin of
    /// `BridgedMessage::unflatten_forged` (whose doc carries the placement
    /// rule): fixed spans, strings and complex members are decoded exactly
    /// as [`Self::unflatten`] decodes them; every forgeable `std::vector`
    /// member at or above the data floor gets its triplet WRITTEN as
    /// `{begin, begin + len, begin + len}` over the entry's bytes inside
    /// `payload`, one below the floor is COPIED (`write_prim_seq_cpp`) and
    /// counted, an empty one becomes the empty triplet. All-or-nothing for the
    /// aliases; `Err` leaves the shadow un-forged and carries the copies made
    /// before the failure (a nonzero `below_floor` ⇒ retire the shadow).
    ///
    /// # Safety
    /// `shadow` from [`Self::new_shadow`]; `payload` must be a HELD SHM
    /// sample's post-header bytes that outlive every use of the forged
    /// members (the caller un-forges with the returned mask before releasing
    /// the sample).
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
                CppFieldOp::FixedCopy {
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
                CppFieldOp::String { c_offset, var_idx } => {
                    match read_var_entry(payload, table_base, *var_idx) {
                        Some(bytes) => {
                            rmw_cerulion_cppstring_assign(
                                shadow.add(*c_offset),
                                bytes.as_ptr() as *const std::os::raw::c_char,
                                bytes.len(),
                            );
                            true
                        }
                        None => false,
                    }
                }
                CppFieldOp::PrimSeq {
                    c_offset,
                    var_idx,
                    elem_size,
                    is_bool,
                    member_index,
                    forge: false,
                } => match read_var_entry(payload, table_base, *var_idx) {
                    Some(bytes) if bytes.len() % elem_size == 0 => write_prim_seq_cpp(
                        self.member(*member_index),
                        (shadow as *mut u8).add(*c_offset) as *mut c_void,
                        bytes,
                        bytes.len() / elem_size,
                        *is_bool,
                    ),
                    _ => false,
                },
                CppFieldOp::PrimSeq {
                    c_offset,
                    var_idx,
                    elem_size,
                    is_bool,
                    member_index,
                    forge: true,
                } => {
                    let bit = forge_idx;
                    forge_idx += 1;
                    let field = (shadow as *mut u8).add(*c_offset);
                    match read_var_entry(payload, table_base, *var_idx) {
                        // An empty entry has nothing to alias: the empty triplet,
                        // recorded as forged so the un-forge stays uniform.
                        Some([]) => {
                            std::ptr::write_unaligned(field as *mut VecTriplet, VecTriplet::EMPTY);
                            outcome.forged |= mask_bit(bit);
                            true
                        }
                        Some(bytes) if bytes.len().is_multiple_of(*elem_size) => {
                            match forge_placement(payload, bytes, *elem_size, data_floor, bit) {
                                ForgePlacement::Forge => {
                                    std::ptr::write_unaligned(
                                        field as *mut VecTriplet,
                                        VecTriplet::forged(bytes.as_ptr() as usize, bytes.len()),
                                    );
                                    outcome.forged |= mask_bit(bit);
                                    true
                                }
                                ForgePlacement::Copy { below_floor } => {
                                    outcome.below_floor += usize::from(below_floor);
                                    write_prim_seq_cpp(
                                        self.member(*member_index),
                                        field as *mut c_void,
                                        bytes,
                                        bytes.len() / elem_size,
                                        *is_bool,
                                    )
                                }
                                ForgePlacement::Malformed => false,
                            }
                        }
                        _ => false,
                    }
                }
                CppFieldOp::Complex {
                    c_offset,
                    var_idx,
                    member_index,
                } => match read_var_entry(payload, table_base, *var_idx) {
                    Some(bytes) => decode_complex_cpp(
                        &self.nested_layouts,
                        self.member(*member_index),
                        bytes,
                        (shadow as *mut u8).add(*c_offset) as *mut c_void,
                    ),
                    None => false,
                },
            };
            if !ok {
                // All-or-nothing for the ALIASES; copies made before the
                // failure survive and are reported through the `Err` so the
                // caller retires the shadow (see the C twin).
                self.unforge(shadow, outcome.forged);
                outcome.forged = 0;
                warn_bad_entry(&self.qualified_name, cpp_op_var_idx(op));
                return Err(outcome);
            }
        }
        Ok(outcome)
    }

    /// Re-point every `std::vector` member in `forged` (a
    /// `ForgeOutcome::forged` mask) at nothing — the all-null triplet a
    /// default-constructed vector holds, which its destructor treats as
    /// "nothing to deallocate". Members OUTSIDE the mask are untouched: they
    /// hold copies `~vector` owns. Idempotent; a zero mask is a no-op. MUST
    /// run before the aliased sample is released and before
    /// [`Self::destroy_shadow`].
    ///
    /// # Safety
    /// `shadow` must come from [`Self::new_shadow`] on this bridge.
    pub unsafe fn unforge(&self, shadow: *mut c_void, forged: u64) {
        let mut forge_idx = 0usize;
        for op in &self.ops {
            if let CppFieldOp::PrimSeq {
                c_offset,
                forge: true,
                ..
            } = op
            {
                if mask_has(forged, forge_idx) {
                    std::ptr::write_unaligned(
                        (shadow as *mut u8).add(*c_offset) as *mut VecTriplet,
                        VecTriplet::EMPTY,
                    );
                }
                forge_idx += 1;
            }
        }
    }

    /// True when every forgeable member of `shadow` selected by `mask` holds
    /// the all-null triplet (`u64::MAX` selects them all) — the observable
    /// the take-side tests pin the un-forge-before-release contract on.
    ///
    /// # Safety
    /// `shadow` must come from [`Self::new_shadow`] on this bridge.
    pub unsafe fn forged_members_are_empty(&self, shadow: *const c_void, mask: u64) -> bool {
        let mut forge_idx = 0usize;
        self.ops.iter().all(|op| match op {
            CppFieldOp::PrimSeq {
                c_offset,
                forge: true,
                ..
            } => {
                let selected = mask_has(mask, forge_idx);
                forge_idx += 1;
                if !selected {
                    return true;
                }
                std::ptr::read_unaligned((shadow as *const u8).add(*c_offset) as *const VecTriplet)
                    == VecTriplet::EMPTY
            }
            _ => true,
        })
    }

    /// Adopt-take: the C++ twin of
    /// [`crate::type_bridge::BridgedMessage::copy_forged_members`] (whose
    /// doc carries the contract): copy the masked members from the wire
    /// through `write_prim_seq_cpp` — the same path the copying decode
    /// takes, valid over the EMPTY (all-null, default-vector) triplets the
    /// preceding un-forge left.
    ///
    /// # Safety
    /// `msg` must be a valid, initialized C++ message of this bridged type
    /// whose masked members are currently EMPTY (un-forged); `payload` the
    /// same post-header bytes the preceding `unflatten_forged` walked.
    pub unsafe fn copy_forged_members(
        &self,
        payload: &[u8],
        msg: *mut c_void,
        forged: u64,
    ) -> bool {
        let table_base = self.layout.fixed_size;
        let mut forge_idx = 0usize;
        for op in &self.ops {
            if let CppFieldOp::PrimSeq {
                c_offset,
                var_idx,
                elem_size,
                is_bool,
                member_index,
                forge: true,
            } = op
            {
                let bit = forge_idx;
                forge_idx += 1;
                if !mask_has(forged, bit) {
                    continue;
                }
                let field = (msg as *mut u8).add(*c_offset) as *mut c_void;
                let ok = match read_var_entry(payload, table_base, *var_idx) {
                    Some(bytes) if bytes.len().is_multiple_of(*elem_size) => write_prim_seq_cpp(
                        self.member(*member_index),
                        field,
                        bytes,
                        bytes.len() / elem_size,
                        *is_bool,
                    ),
                    _ => false,
                };
                if !ok {
                    return warn_bad_entry(&self.qualified_name, *var_idx);
                }
            }
        }
        true
    }

    /// Adopt-take: the C++ twin of
    /// [`crate::type_bridge::BridgedMessage::forged_entry_ranges`] — the
    /// exact `{address, len}` range each FORGED triplet aims at, read back
    /// with `read_unaligned` (triplets are written unaligned). Empty
    /// triplets are skipped (`~vector` on the all-null state deallocates
    /// nothing).
    ///
    /// # Safety
    /// `msg` must be a valid C++ message of this bridged type whose members
    /// in `forged` were just forged by [`Self::unflatten_forged`].
    pub unsafe fn forged_entry_ranges(
        &self,
        msg: *const c_void,
        forged: u64,
        out: &mut Vec<(usize, usize)>,
    ) {
        let mut forge_idx = 0usize;
        for op in &self.ops {
            if let CppFieldOp::PrimSeq {
                c_offset,
                forge: true,
                ..
            } = op
            {
                let bit = forge_idx;
                forge_idx += 1;
                if !mask_has(forged, bit) {
                    continue;
                }
                let t = std::ptr::read_unaligned(
                    (msg as *const u8).add(*c_offset) as *const VecTriplet
                );
                if t.begin != 0 && t.end > t.begin {
                    out.push((t.begin, t.end - t.begin));
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
                CppFieldOp::FixedCopy { .. } => continue,
                CppFieldOp::String { var_idx, .. } | CppFieldOp::Complex { var_idx, .. } => {
                    (*var_idx, None)
                }
                CppFieldOp::PrimSeq {
                    var_idx, elem_size, ..
                } => (*var_idx, Some(*elem_size)),
            };
            let entry = read_var_entry(payload, table_base, var_idx);
            let verdict = crate::take_gate::var_entry_decodable(entry, elem_size);
            if !verdict.is_decodable() {
                return Err((var_idx, verdict));
            }
            // The DECLARED UPPER BOUND, mirrored from `write_prim_seq_cpp` —
            // the only decode arm on either bridge that enforces one. A
            // member whose entry resolves and whose length is a whole number
            // of elements can still exceed the type's own bound, and that
            // decode refuses it AFTER earlier members have been written,
            // which is the partial write this gate exists to prevent.
            //
            // Mirrored EXACTLY, never widened: the gate must refuse a SUBSET
            // of what the decode refuses, because refusing a frame the decode
            // would have SERVED drops a deliverable frame, which is worse
            // than the clobber.
            //
            // Two conditions of that decode arm are deliberately NOT mirrored
            // because neither is reachable from here. Its fixed-array arm
            // (`array_size_ > 0 && !is_upper_bound_`, count must equal N) is
            // unreachable through a variable entry: a fixed `T[N]` is laid
            // out INLINE in the fixed section and carries no offset-table
            // entry at all — a bridge built over one reports
            // `offset_table_bytes() == 0` — so the walk never sees it and a
            // mirror of that arm would be code no test could reach. And only
            // the non-forge arm is checked at all, because a forgeable
            // sequence is unbounded by construction.
            if let CppFieldOp::PrimSeq {
                elem_size,
                member_index,
                forge: false,
                ..
            } = op
            {
                let Some(bytes) = entry else { continue };
                if *elem_size == 0 {
                    continue;
                }
                let member = self.member(*member_index);
                if member.is_upper_bound_ && bytes.len() / elem_size > member.array_size_ {
                    return Err((var_idx, crate::take_gate::EntryVerdict::BoundViolated));
                }
            }
        }
        Ok(())
    }

    /// Adopt-take: the C++ twin of
    /// [`crate::type_bridge::BridgedMessage::release_forgeable_members`]
    /// (whose doc carries the reuse-leak rationale): release + EMPTY every
    /// forge-flagged vector's existing buffer in a CALLER-owned message
    /// before an adopting `unflatten_forged` overwrites its triplet.
    ///
    /// The release goes through the shim's `rmw_cerulion_vector_pod_release`
    /// — `::operator delete(begin)`, the pair `std::allocator<T>` allocates
    /// with — never libc `free` (memory
    /// safety: a caller that hands the take a message whose vector it filled
    /// itself has a genuine `operator new` buffer here, and freeing it with
    /// `free` breaks the allocation/deallocation pair). The two provenances
    /// the pre-pass can meet both route correctly through that one call: a
    /// genuine vector buffer takes the C++ pair it came from, and a
    /// previously-FORGED `begin` — an SHM address registered with the
    /// preloaded hook — still reaches the hook's interposed `free` (libstdc++
    /// and libc++ both implement `operator delete(void*)` as a call to
    /// `free`) and is released, exactly as the app's own `~vector` releases
    /// it. Reachable ONLY under the adopt grant; on every other path
    /// (shadows; non-armed takes) the triplets here are always EMPTY and
    /// nothing is released. The hook exposes no registry query, so
    /// provenance is not looked up — it does not need to be: the C++ pair
    /// is correct for both.
    ///
    /// Memory safety: the release is gated on
    /// the vector's OWNERSHIP state — `end_of_storage > begin`, i.e. a
    /// non-zero CAPACITY — never on `begin != 0`. A non-null `begin` does
    /// not imply an allocation to free, and this bridge MINTS the
    /// counter-example itself: `VecTriplet::forged` sets
    /// `end_of_storage == end == begin` for an EMPTY forged entry, so a
    /// zero-length sequence leaves a non-null SHM address behind a
    /// capacity of 0. Nothing was registered with the hook for it (a
    /// zero-length range produces no registration), so `operator delete`
    /// on it reaches the hook's interposed `free` as an UNKNOWN pointer
    /// and is passed to the real `free` — an invalid deallocation of a
    /// shared-memory address. An empty `/scan` published twice reaches it.
    /// Capacity is the ownership question the C++ ABI already answers, so
    /// no shim query is needed: `begin == end_of_storage` owns nothing,
    /// whatever `begin` says.
    ///
    /// # Safety
    /// `msg` must be a valid, initialized C++ message of this bridged type.
    pub unsafe fn release_forgeable_members(&self, msg: *mut c_void) {
        for op in &self.ops {
            if let CppFieldOp::PrimSeq {
                c_offset,
                forge: true,
                ..
            } = op
            {
                let field = (msg as *mut u8).add(*c_offset);
                let t = std::ptr::read_unaligned(field as *const VecTriplet);
                if t.owns_storage() {
                    ffi::introspection_cpp::rmw_cerulion_vector_pod_release(t.begin as *mut c_void);
                }
                std::ptr::write_unaligned(field as *mut VecTriplet, VecTriplet::EMPTY);
            }
        }
    }

    /// Repr(C) padding byte-ranges of the loanable struct
    /// (relative to the payload start). Empty unless `can_loan`.
    pub fn loan_pad_ranges(&self) -> &[(usize, usize)] {
        &self.loan_pad_ranges
    }

    /// Bring a freshly-loaned SHM payload slot to rosidl
    /// defaults — the C++ twin of
    /// [`crate::type_bridge::BridgedMessage::init_loaned_payload`]
    /// (whose doc carries the full rclcpp `LoanedMessage`
    /// no-placement-new story and the allocation-safety argument).
    ///
    /// C++ semantics: the generated `init_function` placement-news the
    /// message with `MessageInitialization::ALL`, whose constructor
    /// explicitly writes EVERY member — zeros where no default is
    /// declared, the declared default otherwise (rosidl_generator_cpp
    /// contract) — so no zero pre-pass is needed and the rclcpp loan
    /// lane pays no O(payload) memset (unlike the C twin, whose rosidl
    /// `__init` skips default-less members). repr(C) padding is NOT
    /// written by the constructor; the loaned publish path zeroes
    /// [`Self::loan_pad_ranges`] before send.
    ///
    /// A typesupport with NO `init_function` falls back to the zeroed
    /// baseline plus a breadcrumb (never silent divergence).
    ///
    /// # Safety
    /// `payload` must point at a writable region of at least
    /// `self.c_size` bytes (the loaned slot's payload region).
    pub unsafe fn init_loaned_payload(&self, payload: *mut c_void) {
        match (*self.members).init_function {
            Some(init) => init(payload, ffi::introspection_cpp::CPP_MSG_INIT_ALL),
            None => {
                std::ptr::write_bytes(payload as *mut u8, 0, self.c_size);
                tracing::debug!(
                    schema = %self.qualified_name,
                    "typesupport has no init_function; loaned message keeps the zeroed baseline (declared field defaults will NOT be applied)"
                );
            }
        }
    }

    fn member(&self, idx: usize) -> &CppMessageMember {
        unsafe {
            let m = &*self.members;
            &*(m.members_.add(idx))
        }
    }

    /// Size pre-pass: exact wire-frame byte count for this C++ message
    /// (mirror of [`crate::type_bridge::BridgedMessage::frame_size`] —
    /// same hostile-count caps; container lengths via the typesupport's
    /// own function pointers; never dereferences element data).
    ///
    /// # Safety
    /// `c_msg` must point at a valid C++ message of this bridged type.
    pub unsafe fn frame_size(&self, c_msg: *const c_void) -> Result<usize, BridgeError> {
        let mut size = self.head_len();
        for op in &self.ops {
            match op {
                CppFieldOp::FixedCopy { .. } => {}
                CppFieldOp::String { c_offset, .. } => {
                    let bytes = cppstring_bytes(c_msg.add(*c_offset), MAX_FRAME_BYTES)
                        .map_err(|d| self.encode_err(d))?;
                    size = add_var_size(size, bytes.len(), 1).map_err(|d| self.encode_err(d))?;
                }
                CppFieldOp::PrimSeq {
                    c_offset,
                    elem_size,
                    is_bool,
                    member_index,
                    ..
                } => {
                    let member = self.member(*member_index);
                    let field = c_msg.add(*c_offset);
                    let count = seq_size(member, field).map_err(|d| self.encode_err(d))?;
                    check_seq_bound(count, *elem_size).map_err(|d| self.encode_err(d))?;
                    let (byte_len, align) = if *is_bool {
                        (count, 1)
                    } else {
                        (count * elem_size, *elem_size)
                    };
                    size = add_var_size(size, byte_len, align).map_err(|d| self.encode_err(d))?;
                }
                CppFieldOp::Complex {
                    c_offset,
                    member_index,
                    ..
                } => {
                    // Sized by encoding into a scratch buffer — ONE
                    // codec, no size-only twin (see the C bridge).
                    let member = self.member(*member_index);
                    let mut buf = Vec::new();
                    encode_complex_cpp(
                        &self.nested_layouts,
                        member,
                        c_msg.add(*c_offset),
                        &mut buf,
                    )
                    .map_err(|d| self.encode_err(d))?;
                    size = add_var_size(size, buf.len(), 1).map_err(|d| self.encode_err(d))?;
                }
            }
        }
        Ok(size)
    }

    fn head_len(&self) -> usize {
        frame_head_len(&self.layout)
    }

    /// Flatten DIRECTLY into an exact-size initialized buffer. See
    /// [`crate::type_bridge::BridgedMessage::flatten_into`].
    ///
    /// # Safety
    /// `c_msg` must point at a valid C++ message of this bridged type.
    pub unsafe fn flatten_into(
        &self,
        c_msg: *const c_void,
        sequence: u32,
        timestamp_ns: u64,
        out: &mut [u8],
    ) -> Result<usize, BridgeError> {
        // SAFETY: the cursor never de-initializes — it only writes
        // concrete bytes (see the C bridge's identical cast).
        let uninit = std::slice::from_raw_parts_mut(
            out.as_mut_ptr() as *mut std::mem::MaybeUninit<u8>,
            out.len(),
        );
        self.flatten_into_uninit(c_msg, sequence, timestamp_ns, uninit)
    }

    /// Flatten DIRECTLY into a possibly-uninitialized exact-size buffer
    /// (the SHM-loan publish path). Same contract as
    /// [`crate::type_bridge::BridgedMessage::flatten_into_uninit`]:
    /// `Ok(n)` proves `n == out.len()` and every byte initialized; on
    /// `Err` nothing was written out of bounds and the loan must be
    /// dropped, never sent.
    ///
    /// # Safety
    /// `c_msg` must point at a valid C++ message of this bridged type.
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
                CppFieldOp::FixedCopy {
                    c_offset,
                    wire_offset,
                    size,
                    pad_ranges,
                } => {
                    let src =
                        std::slice::from_raw_parts((c_msg as *const u8).add(*c_offset), *size);
                    cur.write_initialized_at(payload_base + wire_offset, src)
                        .map_err(|d| self.encode_err(d))?;
                    for &(off, len) in pad_ranges {
                        cur.zero_initialized_at(payload_base + wire_offset + off, len)
                            .map_err(|d| self.encode_err(d))?;
                    }
                }
                CppFieldOp::String { c_offset, var_idx } => {
                    let bytes = cppstring_bytes(c_msg.add(*c_offset), MAX_FRAME_BYTES)
                        .map_err(|d| self.encode_err(d))?;
                    cursor_append_var(&mut cur, table_base, *var_idx, bytes, 1)
                        .map_err(|d| self.encode_err(d))?;
                }
                CppFieldOp::PrimSeq {
                    c_offset,
                    var_idx,
                    elem_size,
                    is_bool,
                    member_index,
                    forge: _,
                } => {
                    let member = self.member(*member_index);
                    let field = c_msg.add(*c_offset);
                    let count = seq_size(member, field).map_err(|d| self.encode_err(d))?;
                    check_seq_bound(count, *elem_size).map_err(|d| self.encode_err(d))?;
                    if *is_bool {
                        // vector<bool>: bit-packed, fetch element-wise —
                        // straight into the cursor's zeroed span (no
                        // intermediate buffer).
                        let fetch = member.fetch_function.ok_or_else(|| {
                            self.encode_err("bool sequence missing fetch_function")
                        })?;
                        let offset =
                            cursor_align_var(&mut cur, 1).map_err(|d| self.encode_err(d))?;
                        cursor_write_var_entry(&mut cur, table_base, *var_idx, offset, count)
                            .map_err(|d| self.encode_err(d))?;
                        let dst = cur.append_zeroed(count).map_err(|d| self.encode_err(d))?;
                        for (i, b) in dst.iter_mut().enumerate() {
                            let mut v: bool = false;
                            fetch(field, i, &mut v as *mut bool as *mut c_void);
                            *b = v as u8;
                        }
                    } else if count == 0 {
                        cursor_append_var(&mut cur, table_base, *var_idx, &[], *elem_size)
                            .map_err(|d| self.encode_err(d))?;
                    } else {
                        // Contiguous by the C++ standard for vector<T≠bool>
                        // and std::array; element 0 via the typesupport's
                        // own accessor.
                        let get = member.get_const_function.ok_or_else(|| {
                            self.encode_err("sequence missing get_const_function")
                        })?;
                        let base = get(field, 0) as *const u8;
                        if base.is_null() {
                            return Err(self.encode_err("sequence element pointer is null"));
                        }
                        let bytes = std::slice::from_raw_parts(base, count * elem_size);
                        cursor_append_var(&mut cur, table_base, *var_idx, bytes, *elem_size)
                            .map_err(|d| self.encode_err(d))?;
                    }
                }
                CppFieldOp::Complex {
                    c_offset,
                    var_idx,
                    member_index,
                } => {
                    let member = self.member(*member_index);
                    let mut buf = Vec::new();
                    encode_complex_cpp(
                        &self.nested_layouts,
                        member,
                        c_msg.add(*c_offset),
                        &mut buf,
                    )
                    .map_err(|d| self.encode_err(d))?;
                    cursor_append_var(&mut cur, table_base, *var_idx, &buf, 1)
                        .map_err(|d| self.encode_err(d))?;
                }
            }
        }

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

    /// Flatten a C++ message into a complete heap wire frame — thin
    /// wrapper over [`Self::frame_size`] + [`Self::flatten_into_uninit`]
    /// (ONE encode code path; same shape as the C bridge).
    ///
    /// # Safety
    /// `c_msg` must point at a valid C++ message of this bridged type.
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

    /// Flatten WITHOUT the WireHeader (service payload form).
    ///
    /// # Safety
    /// See [`Self::flatten`].
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

    /// Unflatten a received wire payload into an INITIALIZED C++
    /// message. Returns false (with a warning) on malformed frames or
    /// allocation failure — never delivers a silently-truncated message.
    ///
    /// # Safety
    /// `c_msg` must point at a valid, initialized C++ message of this
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
                CppFieldOp::FixedCopy {
                    c_offset,
                    wire_offset,
                    size,
                    pad_ranges: _,
                } => {
                    std::ptr::copy_nonoverlapping(
                        payload.as_ptr().add(*wire_offset),
                        (c_msg as *mut u8).add(*c_offset),
                        *size,
                    );
                }
                CppFieldOp::String { c_offset, var_idx } => {
                    let Some(bytes) = read_var_entry(payload, table_base, *var_idx) else {
                        return warn_bad_entry(&self.qualified_name, *var_idx);
                    };
                    rmw_cerulion_cppstring_assign(
                        c_msg.add(*c_offset),
                        bytes.as_ptr() as *const std::os::raw::c_char,
                        bytes.len(),
                    );
                }
                CppFieldOp::PrimSeq {
                    c_offset,
                    var_idx,
                    elem_size,
                    is_bool,
                    member_index,
                    forge: _,
                } => {
                    let Some(bytes) = read_var_entry(payload, table_base, *var_idx) else {
                        return warn_bad_entry(&self.qualified_name, *var_idx);
                    };
                    if bytes.len() % elem_size != 0 {
                        return warn_bad_entry(&self.qualified_name, *var_idx);
                    }
                    let member = self.member(*member_index);
                    let field = (c_msg as *mut u8).add(*c_offset) as *mut c_void;
                    let count = bytes.len() / elem_size;
                    if !write_prim_seq_cpp(member, field, bytes, count, *is_bool) {
                        return warn_bad_entry(&self.qualified_name, *var_idx);
                    }
                }
                CppFieldOp::Complex {
                    c_offset,
                    var_idx,
                    member_index,
                } => {
                    let Some(bytes) = read_var_entry(payload, table_base, *var_idx) else {
                        return warn_bad_entry(&self.qualified_name, *var_idx);
                    };
                    let member = self.member(*member_index);
                    let field = (c_msg as *mut u8).add(*c_offset) as *mut c_void;
                    if !decode_complex_cpp(&self.nested_layouts, member, bytes, field) {
                        return warn_bad_entry(&self.qualified_name, *var_idx);
                    }
                }
            }
        }
        true
    }

    fn encode_err(&self, detail: &str) -> BridgeError {
        BridgeError::Encode {
            message: self.qualified_name.clone(),
            detail: detail.to_string(),
        }
    }
}

// =====================================================================
// Schema collection (cpp namespaces use "::" separators)
// =====================================================================

unsafe fn qualified_name_of_cpp(members: *const CppMessageMembers) -> Result<String, BridgeError> {
    let m = &*members;
    let ns = ffi::cstr(m.message_namespace_).ok_or(BridgeError::NoIntrospection)?;
    let name = ffi::cstr(m.message_name_).ok_or(BridgeError::NoIntrospection)?;
    Ok(format!("{}/{name}", cpp_package(ns)))
}

/// "geometry_msgs::msg" → "geometry_msgs" (C uses "__", C++ uses "::").
///
/// Nothing validates the namespace
/// spelling, so a C++ typesupport carrying the C-style "geometry_msgs__msg"
/// would flow through a "::"-only split VERBATIM — the qualified name
/// would become "geometry_msgs__msg/Type", which (a) hashes differently from the
/// canonical "geometry_msgs/Type" (a `pkg__msg`- and a `pkg::msg`-spelled
/// publisher of ONE type would disagree on `schema_hash`, so neither could interop
/// with the other or with native readers), (b) misses BOTH the slice-ceiling
/// tier table AND any `CERULION_RMW_SLICE_CEILING` override (falling to the
/// 128 MiB blanket), and (c) renders the ROS graph type name as
/// "geometry_msgs__msg/msg/Type". Stripping the C-style interface suffix from
/// the first "::"-segment HERE — the one seam every namespace→package
/// derivation in this bridge routes through (`qualified_name_of_cpp`,
/// `collect_schemas_cpp`, nested `FieldType::Nested.package`) — heals every
/// consumer of the qualified name at once, deliberately NOT ad-hoc at the
/// ceiling call site.
///
/// The empty-guard keeps a degenerate namespace that IS the bare suffix
/// (e.g. literally "__msg") verbatim rather than minting an empty package.
/// ROS 2 package names cannot contain "__" (it is rosidl's separator), so a
/// stripped suffix can never have been part of a real package name.
fn cpp_package(ns: &str) -> &str {
    let first = ns.split("::").next().unwrap_or(ns);
    for suffix in ["__msg", "__srv", "__action"] {
        if let Some(stripped) = first.strip_suffix(suffix) {
            if !stripped.is_empty() {
                return stripped;
            }
        }
    }
    first
}

unsafe fn nested_members_of_cpp(member: &CppMessageMember) -> *const CppMessageMembers {
    if member.members_.is_null() {
        return std::ptr::null();
    }
    (*member.members_).data as *const CppMessageMembers
}

/// Verify a NESTED C++ type's introspection walk agrees with its
/// `WireLayout` — the twin of
/// [`crate::type_bridge::verify_nested_lockstep`]. See that function for why
/// this is a hard registration-time error.
unsafe fn verify_nested_lockstep_cpp(
    members: *const CppMessageMembers,
    qname: &str,
    layout: &WireLayout,
) -> Result<(), BridgeError> {
    let m = &*members;
    let member_slice = std::slice::from_raw_parts(m.members_, m.member_count_ as usize);
    let mut fixed_idx = 0usize;
    let mut var_idx = 0usize;
    for member in member_slice {
        let fname = ffi::cstr(member.name_).unwrap_or("<field>").to_string();
        if is_variable_member_cpp(member) {
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
            let c_field_size = fixed_member_size_cpp(member);
            if fl.size != c_field_size {
                return Err(BridgeError::LayoutMismatch {
                    message: qname.to_string(),
                    detail: format!(
                        "fixed-field size mismatch for '{fname}': layout {} vs C++ {}",
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

unsafe fn collect_schemas_cpp(
    members: *const CppMessageMembers,
    out: &mut BTreeMap<String, MessageSchema>,
    ptrs: &mut BTreeMap<usize, String>,
) -> Result<(), BridgeError> {
    let m = &*members;
    let ns = ffi::cstr(m.message_namespace_).ok_or(BridgeError::NoIntrospection)?;
    let name = ffi::cstr(m.message_name_).ok_or(BridgeError::NoIntrospection)?;
    let package = cpp_package(ns).to_string();
    let qualified = format!("{package}/{name}");
    // Dedupe recursion by introspection POINTER (see the C twin's
    // `collect_schemas`) so every distinct typesupport instance gets a
    // layout entry.
    if ptrs.insert(members as usize, qualified.clone()).is_some() {
        return Ok(());
    }
    if out.contains_key(&qualified) {
        return collect_nested_schemas_cpp(
            std::slice::from_raw_parts(m.members_, m.member_count_ as usize),
            out,
            ptrs,
        );
    }

    let mut schema = MessageSchema::new_in_package(name, &package);
    let member_slice = std::slice::from_raw_parts(m.members_, m.member_count_ as usize);
    for member in member_slice {
        // Layout-drift plausibility guard, applied to EVERY visited
        // type (root + transitively nested): the
        // CppMessageMember mirror is pinned to the Jazzy/rolling
        // header layout; drift must fail registration loudly.
        if member.offset_ as usize >= m.size_of_ || !(1..=18).contains(&member.type_id_) {
            return Err(BridgeError::LayoutMismatch {
                message: qualified.clone(),
                detail: format!(
                    "implausible introspection member (offset {} / size_of {} / type_id {}) — introspection_cpp ABI layout drift?",
                    member.offset_, m.size_of_, member.type_id_
                ),
            });
        }
        let fname = ffi::cstr(member.name_)
            .ok_or(BridgeError::NoIntrospection)?
            .to_string();
        let base = base_field_type_cpp(member, &qualified, &fname)?;
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

    collect_nested_schemas_cpp(member_slice, out, ptrs)
}

/// Recurse into every nested message type of `member_slice`.
unsafe fn collect_nested_schemas_cpp(
    member_slice: &[CppMessageMember],
    out: &mut BTreeMap<String, MessageSchema>,
    ptrs: &mut BTreeMap<usize, String>,
) -> Result<(), BridgeError> {
    for member in member_slice {
        if member.type_id_ == ros_type::MESSAGE {
            let nested = nested_members_of_cpp(member);
            if nested.is_null() {
                return Err(BridgeError::NoIntrospection);
            }
            collect_schemas_cpp(nested, out, ptrs)?;
        }
    }
    Ok(())
}

unsafe fn base_field_type_cpp(
    member: &CppMessageMember,
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
            if member.members_.is_null() {
                return Err(BridgeError::NoIntrospection);
            }
            let nested = nested_members_of_cpp(member);
            if nested.is_null() {
                return Err(BridgeError::NoIntrospection);
            }
            let nm = &*nested;
            let ns = ffi::cstr(nm.message_namespace_).ok_or(BridgeError::NoIntrospection)?;
            let nname = ffi::cstr(nm.message_name_).ok_or(BridgeError::NoIntrospection)?;
            FieldType::Nested {
                schema_name: nname.to_string(),
                package: Some(cpp_package(ns).to_string()),
                fixed: None,
            }
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

// =====================================================================
// Classification / sizing (mirrors the C bridge, recursive)
// =====================================================================

unsafe fn is_variable_member_cpp(member: &CppMessageMember) -> bool {
    if member.type_id_ == ros_type::STRING || member.type_id_ == ros_type::WSTRING {
        return true;
    }
    if member.is_array_ && (member.array_size_ == 0 || member.is_upper_bound_) {
        return true;
    }
    if member.type_id_ == ros_type::MESSAGE {
        return !nested_is_fixed_cpp(nested_members_of_cpp(member));
    }
    false
}

unsafe fn nested_is_fixed_cpp(members: *const CppMessageMembers) -> bool {
    if members.is_null() {
        return false;
    }
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
            && !nested_is_fixed_cpp(nested_members_of_cpp(member))
        {
            return false;
        }
    }
    true
}

/// C++ in-struct byte size of a FIXED member (primitive, std::array of
/// primitives, fixed nested, std::array of fixed nested). All such
/// members are standard-layout contiguous bytes — identical to C.
unsafe fn fixed_member_size_cpp(member: &CppMessageMember) -> usize {
    let base = if member.type_id_ == ros_type::MESSAGE {
        match nested_members_of_cpp(member) {
            p if p.is_null() => 0,
            p => (*p).size_of_,
        }
    } else {
        primitive_size(member.type_id_)
    };
    if member.is_array_ {
        base * member.array_size_
    } else {
        base
    }
}

unsafe fn member_data_runs_cpp(
    member: &CppMessageMember,
    base: usize,
    runs: &mut Vec<(usize, usize)>,
) {
    if member.type_id_ != ros_type::MESSAGE {
        runs.push((base, fixed_member_size_cpp(member)));
        return;
    }
    let nested = nested_members_of_cpp(member);
    if nested.is_null() {
        return;
    }
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
            member_data_runs_cpp(nm, base + i * stride + nm.offset_ as usize, runs);
        }
    }
}

fn plan_variable_op_cpp(
    c_offset: usize,
    var_idx: usize,
    member_index: usize,
    v: &VariableFieldLayout,
    forge: bool,
) -> CppFieldOp {
    let prim_seq = |elem_size: usize, is_bool: bool| CppFieldOp::PrimSeq {
        c_offset,
        var_idx,
        elem_size,
        is_bool,
        member_index,
        // A bit-packed `vector<bool>` has no byte range to aim at; the
        // predicate already refuses BOOLEAN, this keeps the two facts
        // adjacent.
        forge: forge && !is_bool,
    };
    match &v.field_type {
        FieldType::String => CppFieldOp::String { c_offset, var_idx },
        FieldType::Bytes => prim_seq(1, false),
        FieldType::DynamicArray { element_type } => match element_type.as_ref() {
            FieldType::Bool => prim_seq(1, true),
            FieldType::I8 | FieldType::U8 => prim_seq(1, false),
            FieldType::I16 | FieldType::U16 => prim_seq(2, false),
            FieldType::I32 | FieldType::U32 | FieldType::F32 => prim_seq(4, false),
            FieldType::I64 | FieldType::U64 | FieldType::F64 => prim_seq(8, false),
            _ => CppFieldOp::Complex {
                c_offset,
                var_idx,
                member_index,
            },
        },
        _ => CppFieldOp::Complex {
            c_offset,
            var_idx,
            member_index,
        },
    }
}

/// The offset-table index a top-level variable op reads (`usize::MAX` for
/// a fixed span, which cannot fail and never reaches the diagnostic).
fn cpp_op_var_idx(op: &CppFieldOp) -> usize {
    match op {
        CppFieldOp::FixedCopy { .. } => usize::MAX,
        CppFieldOp::String { var_idx, .. }
        | CppFieldOp::PrimSeq { var_idx, .. }
        | CppFieldOp::Complex { var_idx, .. } => *var_idx,
    }
}

/// May the forged loaned take aim this C++ member's
/// `std::vector` triplet at the held SHM sample instead of filling it? The
/// C twin's rule (`crate::type_bridge::is_forgeable_sequence` — unbounded,
/// non-`bool`, default-less, primitive; each exclusion is explained there)
/// plus the C++-only precondition: the process's `std::vector` really is
/// the three-pointer triplet ([`vector_triplet_layout_verified`]). A
/// BOUNDED sequence is additionally a different C++ type here (rosidl's
/// `BoundedVector`), which is reason enough on its own.
fn is_forgeable_sequence_cpp(member: &CppMessageMember, triplet_layout_ok: bool) -> bool {
    triplet_layout_ok
        && member.is_array_
        && member.array_size_ == 0
        && !member.is_upper_bound_
        && member.default_value_.is_null()
        && is_primitive_type(member.type_id_)
        && member.type_id_ != ros_type::BOOLEAN
}

// =====================================================================
// Sequence access through introspection function pointers
// =====================================================================

unsafe fn seq_size(member: &CppMessageMember, field: *const c_void) -> Result<usize, &'static str> {
    if member.array_size_ > 0 && !member.is_upper_bound_ {
        return Ok(member.array_size_);
    }
    let size_fn = member
        .size_function
        .ok_or("sequence missing size_function")?;
    Ok(size_fn(field))
}

/// Write a primitive sequence into the C++ container. An unbounded
/// `std::vector<uint8_t>` (the uint8[] payload class) takes the assign()
/// fast path -- one alloc+copy, NO value-init memset; every
/// other case resizes through the typesupport then bulk-copies (or
/// assigns per element for vector<bool>). Returns false on missing
/// accessors or fixed-array length mismatch.
unsafe fn write_prim_seq_cpp(
    member: &CppMessageMember,
    field: *mut c_void,
    bytes: &[u8],
    count: usize,
    is_bool: bool,
) -> bool {
    if member.array_size_ > 0 && !member.is_upper_bound_ {
        if count != member.array_size_ {
            return false;
        }
    } else {
        // BOUNDED sequences: resize beyond the bound makes rosidl's
        // BoundedVector THROW std::length_error through the extern "C"
        // fn pointer into Rust — foreign unwind is UB.
        // Reject the frame loudly instead, like every other malformed
        // input. (No array_size_ > 0 term: a degenerate bound of 0
        // must also reject count > 0.)
        if member.is_upper_bound_ && count > member.array_size_ {
            return false;
        }
        // FAST PATH: an unbounded std::vector<uint8_t> -- the
        // uint8[] payload class, e.g. sensor_msgs/Image.data -- is filled
        // with a single assign() (one alloc + copy), skipping resize()'s
        // value-init memset (a redundant RX `rep stos` in a perf
        // profile). The reinterpret-eligibility invariant is
        // named in `is_unbounded_u8_vector` (co-located with the
        // `assign_u8_vector` wrapper it guards): only `type_id_ == UINT8`
        // AND `!is_upper_bound_` guard against reinterpreting a
        // std::vector<int8_t>/<std::byte>/octet or a rosidl BoundedVector
        // as std::vector<uint8_t> (UB). `!is_bool` (bit-packed) and the
        // dynamic-array shape are established here; int8, octet, bounded
        // uint8, bool, and every multi-byte primitive keep the
        // resize()+copy path below. Fixed uint8 arrays land in the `if`
        // branch above; the bounded/hostile rejection ran just above, so
        // reject-before-mutation still holds. This is the single choke
        // point -- the nested-message decode path (~:1231) routes here too.
        if !is_bool && is_unbounded_u8_vector(member) {
            assign_u8_vector(field, bytes);
            return true;
        }
        let Some(resize) = member.resize_function else {
            return false;
        };
        resize(field, count);
    }
    if count == 0 {
        return true;
    }
    if is_bool {
        let Some(assign) = member.assign_function else {
            return false;
        };
        for (i, &b) in bytes.iter().enumerate() {
            let v: bool = b != 0;
            assign(field, i, &v as *const bool as *const c_void);
        }
        true
    } else {
        let Some(get) = member.get_function else {
            return false;
        };
        let base = get(field, 0) as *mut u8;
        if base.is_null() {
            return false;
        }
        std::ptr::copy_nonoverlapping(bytes.as_ptr(), base, bytes.len());
        true
    }
}

// =====================================================================
// Complex-field codec (canonical v1 — byte-identical to the C bridge)
// =====================================================================

unsafe fn encode_complex_cpp(
    layouts: &NestedLayouts,
    member: &CppMessageMember,
    field_ptr: *const c_void,
    out: &mut Vec<u8>,
) -> Result<(), &'static str> {
    if member.is_array_ {
        let count = seq_size(member, field_ptr)?;
        check_seq_bound(count, 1)?;
        if member.type_id_ == ros_type::STRING {
            out.extend_from_slice(&(count as u32).to_le_bytes());
            for i in 0..count {
                let elem = element_ptr(member, field_ptr, i)?;
                let bytes = cppstring_bytes(elem, MAX_FRAME_BYTES)?;
                check_out_cap(out, 4 + bytes.len())?;
                out.extend_from_slice(&(bytes.len() as u32).to_le_bytes());
                out.extend_from_slice(bytes);
            }
        } else {
            let nested = nested_members_of_cpp(member);
            if nested.is_null() {
                return Err("nested introspection missing");
            }
            if nested_is_fixed_cpp(nested) {
                for i in 0..count {
                    let elem = element_ptr(member, field_ptr, i)?;
                    encode_message_payload_cpp(layouts, nested, elem, out)?;
                }
            } else {
                out.extend_from_slice(&(count as u32).to_le_bytes());
                for i in 0..count {
                    let elem = element_ptr(member, field_ptr, i)?;
                    let mut buf = Vec::new();
                    encode_message_payload_cpp(layouts, nested, elem, &mut buf)?;
                    check_out_cap(out, 4 + buf.len())?;
                    out.extend_from_slice(&(buf.len() as u32).to_le_bytes());
                    out.extend_from_slice(&buf);
                }
            }
        }
        Ok(())
    } else {
        let nested = nested_members_of_cpp(member);
        if nested.is_null() {
            return Err("nested introspection missing");
        }
        encode_message_payload_cpp(layouts, nested, field_ptr, out)
    }
}

unsafe fn element_ptr(
    member: &CppMessageMember,
    field: *const c_void,
    i: usize,
) -> Result<*const c_void, &'static str> {
    let get = member
        .get_const_function
        .ok_or("sequence missing get_const_function")?;
    let p = get(field, i);
    if p.is_null() {
        return Err("sequence element pointer is null");
    }
    Ok(p)
}

unsafe fn element_ptr_mut(
    member: &CppMessageMember,
    field: *mut c_void,
    i: usize,
) -> Option<*mut c_void> {
    let get = member.get_function?;
    let p = get(field, i);
    if p.is_null() {
        None
    } else {
        Some(p)
    }
}

/// Encode a C++ message VALUE as its canonical Cerulion body — the exact
/// twin of [`crate::type_bridge::encode_message_payload`].
unsafe fn encode_message_payload_cpp(
    layouts: &NestedLayouts,
    members: *const CppMessageMembers,
    msg: *const c_void,
    out: &mut Vec<u8>,
) -> Result<(), &'static str> {
    if nested_is_fixed_cpp(members) {
        let m = &*members;
        let start = out.len();
        check_out_cap(out, m.size_of_)?;
        out.resize(start + m.size_of_, 0);
        std::ptr::copy_nonoverlapping(msg as *const u8, out.as_mut_ptr().add(start), m.size_of_);
        zero_struct_padding_cpp(members, &mut out[start..start + m.size_of_]);
        return Ok(());
    }

    let m = &*members;
    let member_slice = std::slice::from_raw_parts(m.members_, m.member_count_ as usize);

    let layout = layouts
        .get_cpp(members)
        .ok_or("no wire layout registered for a nested message type")?;
    let mut builder = CanonicalBodyBuilder::new(layout);

    // Fixed members: C offset -> WIRE offset (see the C twin).
    {
        let fixed = builder.fixed_mut();
        let mut fixed_idx = 0usize;
        for member in member_slice {
            if is_variable_member_cpp(member) {
                continue;
            }
            let fl = layout
                .fixed_fields
                .get(fixed_idx)
                .ok_or("fixed-field index past the layout (registration lockstep violated)")?;
            let size = fixed_member_size_cpp(member);
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
            if member.type_id_ == ros_type::MESSAGE {
                let mut runs = Vec::new();
                member_data_runs_cpp(member, 0, &mut runs);
                for (off, len) in padding_ranges(&mut runs, size) {
                    fixed[fl.offset + off..fl.offset + off + len].fill(0);
                }
            }
            fixed_idx += 1;
        }
    }

    for member in member_slice {
        if !is_variable_member_cpp(member) {
            continue;
        }
        let field_ptr = msg.add(member.offset_ as usize);
        let mut buf = Vec::new();
        if member.type_id_ == ros_type::STRING && !member.is_array_ {
            buf.extend_from_slice(cppstring_bytes(field_ptr, MAX_FRAME_BYTES)?);
        } else if member.type_id_ == ros_type::MESSAGE && !member.is_array_ {
            encode_message_payload_cpp(
                layouts,
                nested_members_of_cpp(member),
                field_ptr,
                &mut buf,
            )?;
        } else if member.is_array_ && is_primitive_type(member.type_id_) {
            let count = seq_size(member, field_ptr)?;
            let elem = primitive_size(member.type_id_);
            check_seq_bound(count, elem)?;
            if member.type_id_ == ros_type::BOOLEAN {
                let fetch = member.fetch_function.ok_or("bool seq missing fetch")?;
                buf.resize(count, 0);
                for (i, b) in buf.iter_mut().enumerate() {
                    let mut v: bool = false;
                    fetch(field_ptr, i, &mut v as *mut bool as *mut c_void);
                    *b = v as u8;
                }
            } else if count > 0 {
                let base = element_ptr(member, field_ptr, 0)? as *const u8;
                buf.extend_from_slice(std::slice::from_raw_parts(base, count * elem));
            }
        } else {
            encode_complex_cpp(layouts, member, field_ptr, &mut buf)?;
        }
        builder.push_variable(buf).map_err(|e| e.as_str())?;
    }

    let body = builder.finish().map_err(|e| e.as_str())?;
    check_out_cap(out, body.len())?;
    out.extend_from_slice(&body);
    Ok(())
}

unsafe fn zero_struct_padding_cpp(members: *const CppMessageMembers, buf: &mut [u8]) {
    let m = &*members;
    let member_slice = std::slice::from_raw_parts(m.members_, m.member_count_ as usize);
    let mut runs = Vec::new();
    for nm in member_slice {
        member_data_runs_cpp(nm, nm.offset_ as usize, &mut runs);
    }
    for (off, len) in padding_ranges(&mut runs, buf.len()) {
        buf[off..off + len].fill(0);
    }
}

// =====================================================================
// Complex-field decoding (inverse; resize through the typesupport)
// =====================================================================

unsafe fn decode_complex_cpp(
    layouts: &NestedLayouts,
    member: &CppMessageMember,
    bytes: &[u8],
    field_ptr: *mut c_void,
) -> bool {
    if member.is_array_ {
        if member.type_id_ == ros_type::STRING {
            let Some((count, mut rest)) = read_u32_prefix(bytes) else {
                return false;
            };
            if count > rest.len() / 4 {
                return false;
            }
            if !prepare_seq_cpp(member, field_ptr, count) {
                return false;
            }
            for i in 0..count {
                let Some((len, after)) = read_u32_prefix(rest) else {
                    return false;
                };
                if after.len() < len {
                    return false;
                }
                let Some(elem) = element_ptr_mut(member, field_ptr, i) else {
                    return false;
                };
                rmw_cerulion_cppstring_assign(
                    elem,
                    after.as_ptr() as *const std::os::raw::c_char,
                    len,
                );
                rest = &after[len..];
            }
            true
        } else {
            let nested = nested_members_of_cpp(member);
            if nested.is_null() {
                return false;
            }
            if nested_is_fixed_cpp(nested) {
                let stride = (*nested).size_of_;
                if stride == 0 || !bytes.len().is_multiple_of(stride) {
                    return false;
                }
                let count = bytes.len() / stride;
                if !prepare_seq_cpp(member, field_ptr, count) {
                    return false;
                }
                for i in 0..count {
                    let Some(elem) = element_ptr_mut(member, field_ptr, i) else {
                        return false;
                    };
                    std::ptr::copy_nonoverlapping(
                        bytes.as_ptr().add(i * stride),
                        elem as *mut u8,
                        stride,
                    );
                }
                true
            } else {
                let Some((count, mut rest)) = read_u32_prefix(bytes) else {
                    return false;
                };
                if count > rest.len() / 4 {
                    return false;
                }
                if !prepare_seq_cpp(member, field_ptr, count) {
                    return false;
                }
                for i in 0..count {
                    let Some((len, after)) = read_u32_prefix(rest) else {
                        return false;
                    };
                    if after.len() < len {
                        return false;
                    }
                    let Some(elem) = element_ptr_mut(member, field_ptr, i) else {
                        return false;
                    };
                    if !decode_message_payload_cpp(layouts, nested, &after[..len], elem) {
                        return false;
                    }
                    rest = &after[len..];
                }
                true
            }
        }
    } else {
        let nested = nested_members_of_cpp(member);
        if nested.is_null() {
            return false;
        }
        decode_message_payload_cpp(layouts, nested, bytes, field_ptr)
    }
}

/// Resize a C++ sequence (or validate a fixed array's length).
unsafe fn prepare_seq_cpp(member: &CppMessageMember, field: *mut c_void, count: usize) -> bool {
    if member.array_size_ > 0 && !member.is_upper_bound_ {
        return count == member.array_size_;
    }
    // Bounded: never resize past the bound (BoundedVector::resize
    // throws — foreign unwind into Rust is UB; no
    // array_size_ > 0 term — a bound of 0 must reject count > 0 too).
    if member.is_upper_bound_ && count > member.array_size_ {
        return false;
    }
    let Some(resize) = member.resize_function else {
        return false;
    };
    resize(field, count);
    true
}

/// Decode a canonical Cerulion body into a C++ message value — the twin of
/// [`crate::type_bridge::decode_message_payload`].
unsafe fn decode_message_payload_cpp(
    layouts: &NestedLayouts,
    members: *const CppMessageMembers,
    bytes: &[u8],
    msg: *mut c_void,
) -> bool {
    if nested_is_fixed_cpp(members) {
        let m = &*members;
        if bytes.len() != m.size_of_ {
            return false;
        }
        std::ptr::copy_nonoverlapping(bytes.as_ptr(), msg as *mut u8, m.size_of_);
        return true;
    }

    let m = &*members;
    let member_slice = std::slice::from_raw_parts(m.members_, m.member_count_ as usize);
    let Some(layout) = layouts.get_cpp(members) else {
        return false;
    };
    let Ok(reader) = CanonicalBodyReader::new(layout, bytes) else {
        return false;
    };

    // Fixed members: WIRE offset -> C offset (see the C twin).
    let fixed = reader.fixed();
    let mut fixed_idx = 0usize;
    for member in member_slice {
        if is_variable_member_cpp(member) {
            continue;
        }
        let Some(fl) = layout.fixed_fields.get(fixed_idx) else {
            return false;
        };
        let size = fixed_member_size_cpp(member);
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

    let mut var_idx = 0usize;
    for member in member_slice {
        if !is_variable_member_cpp(member) {
            continue;
        }
        let Some(chunk) = reader.variable(var_idx) else {
            return false;
        };
        var_idx += 1;
        let field_ptr = (msg as *mut u8).add(member.offset_ as usize) as *mut c_void;
        let ok = if member.type_id_ == ros_type::STRING && !member.is_array_ {
            rmw_cerulion_cppstring_assign(
                field_ptr,
                chunk.as_ptr() as *const std::os::raw::c_char,
                chunk.len(),
            );
            true
        } else if member.type_id_ == ros_type::MESSAGE && !member.is_array_ {
            decode_message_payload_cpp(layouts, nested_members_of_cpp(member), chunk, field_ptr)
        } else if member.is_array_ && is_primitive_type(member.type_id_) {
            let elem = primitive_size(member.type_id_);
            if chunk.len() % elem != 0 {
                false
            } else {
                write_prim_seq_cpp(
                    member,
                    field_ptr,
                    chunk,
                    chunk.len() / elem,
                    member.type_id_ == ros_type::BOOLEAN,
                )
            }
        } else {
            decode_complex_cpp(layouts, member, chunk, field_ptr)
        };
        if !ok {
            return false;
        }
    }
    true
}

// =====================================================================
// The borrow-window SEAL — C++ twin (the mechanism doc
// lives on the C side, `crate::type_bridge`'s seal section; the shared
// plan/commit arithmetic is `plan_seal`/`SealPlan` there, so the two
// bridges can only differ in how members are READ)
// =====================================================================

impl CppBridgedMessage {
    /// May `rmw_borrow_loaned_message` serve this type
    /// through the borrow window? Mirror of
    /// [`crate::type_bridge::BridgedMessage::can_borrow_windowed`]
    /// (`forge_count > 0` additionally implies the verified vector-triplet
    /// layout on this side).
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

    /// Seal a filled windowed-borrow slot into a wire frame IN PLACE — the
    /// C++ twin of
    /// [`crate::type_bridge::BridgedMessage::seal_borrowed_frame`], whose
    /// doc carries the contract (plan-then-commit; `Err` leaves the struct
    /// intact; `Ok` consumes it). Container reads go through the
    /// typesupport's own accessors exactly as the flatten path reads them
    /// (`std::vector<T≠bool>` is contiguous by the standard; element 0 via
    /// `get_const_function`; `vector<bool>` is bit-packed and fetched
    /// element-wise into an owned scratch — it is never forgeable).
    ///
    /// # Safety
    /// See the C twin: `payload` is the borrow slot's payload region of
    /// exactly `payload_len >= geo.tail_off` bytes holding a valid,
    /// initialized C++ message of this type at offset 0; the window is
    /// DISARMED; every referenced range is readable.
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
                CppFieldOp::FixedCopy { .. } => {}
                CppFieldOp::String { c_offset, .. } => {
                    let bytes = cppstring_bytes(c_msg.add(*c_offset), MAX_FRAME_BYTES)
                        .map_err(|d| SealRefusal::Encode(self.encode_err(d)))?;
                    items.push(SealItem::CopyRaw {
                        ptr: bytes.as_ptr() as usize,
                        len: bytes.len(),
                        align: 1,
                        escaped: false,
                    });
                }
                CppFieldOp::PrimSeq {
                    c_offset,
                    elem_size,
                    is_bool,
                    member_index,
                    forge,
                    ..
                } => {
                    let member = self.member(*member_index);
                    let field = c_msg.add(*c_offset);
                    let count = seq_size(member, field)
                        .map_err(|d| SealRefusal::Encode(self.encode_err(d)))?;
                    check_seq_bound(count, if *is_bool { 1 } else { *elem_size })
                        .map_err(|d| SealRefusal::Encode(self.encode_err(d)))?;
                    if *is_bool {
                        // Bit-packed: fetch into the reused owned arena (the
                        // same element walk the flatten path runs). Never
                        // forgeable, never adopted.
                        let fetch = member.fetch_function.ok_or_else(|| {
                            SealRefusal::Encode(
                                self.encode_err("bool sequence missing fetch_function"),
                            )
                        })?;
                        if count == 0 {
                            // `vector<bool>` is never forgeable.
                            items.push(SealItem::Empty { forgeable: false });
                        } else {
                            let start = owned.len();
                            owned.resize(start + count, 0);
                            for (i, b) in owned[start..].iter_mut().enumerate() {
                                let mut v: bool = false;
                                fetch(field, i, &mut v as *mut bool as *mut c_void);
                                *b = v as u8;
                            }
                            items.push(SealItem::CopyOwned {
                                start,
                                len: count,
                                align: 1,
                            });
                        }
                    } else if count == 0 {
                        items.push(SealItem::Empty { forgeable: *forge });
                    } else {
                        let base = element_ptr(member, field, 0)
                            .map_err(|d| SealRefusal::Encode(self.encode_err(d)))?
                            as usize;
                        let byte_len = count * elem_size;
                        if *forge {
                            match adopted_offset(
                                window,
                                payload_base,
                                payload_len,
                                data_floor,
                                base,
                                byte_len,
                                *elem_size,
                            ) {
                                Some(off) => items.push(SealItem::Adopted { off, len: byte_len }),
                                None => items.push(SealItem::CopyRaw {
                                    ptr: base,
                                    len: byte_len,
                                    align: *elem_size,
                                    escaped: true,
                                }),
                            }
                        } else {
                            items.push(SealItem::CopyRaw {
                                ptr: base,
                                len: byte_len,
                                align: *elem_size,
                                escaped: false,
                            });
                        }
                    }
                }
                CppFieldOp::Complex {
                    c_offset,
                    member_index,
                    ..
                } => {
                    let member = self.member(*member_index);
                    // Appends into the reused arena (see the C twin's note:
                    // the internal cap check is cumulative — strictly
                    // stricter, matching whole-frame accounting).
                    let start = owned.len();
                    encode_complex_cpp(&self.nested_layouts, member, c_msg.add(*c_offset), owned)
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
        scratch.commit_copies(&plan, payload);

        // Wire head OFF-SLOT first (it overlaps the struct the fixed
        // copies still read from).
        scratch.head_buf.resize(data_floor, 0);
        scratch.fill_entries(data_floor);
        let SealScratch {
            entries, head_buf, ..
        } = &mut *scratch;
        for op in &self.ops {
            if let CppFieldOp::FixedCopy {
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
                for &(off, len) in pad_ranges {
                    head_buf[wire_offset + off..wire_offset + off + len].fill(0);
                }
            }
        }
        {
            let mut item_i = 0usize;
            for op in &self.ops {
                let var_idx = match op {
                    CppFieldOp::FixedCopy { .. } => continue,
                    CppFieldOp::String { var_idx, .. }
                    | CppFieldOp::PrimSeq { var_idx, .. }
                    | CppFieldOp::Complex { var_idx, .. } => *var_idx,
                };
                let (off, len) = entries[item_i];
                item_i += 1;
                let entry_base = self.layout.fixed_size + var_idx * 8;
                head_buf[entry_base..entry_base + 4].copy_from_slice(&(off as u32).to_le_bytes());
                head_buf[entry_base + 4..entry_base + 8]
                    .copy_from_slice(&(len as u32).to_le_bytes());
            }
        }

        // Empty every forge-flagged vector triplet whose storage lies
        // INSIDE the slot, so the destructor below can never operator-
        // delete slot (shared-memory) bytes — adopted members AND in-slot
        // escapees (a misaligned in-window fill). Slot bytes are never
        // allocator-owned, so emptying leaks nothing, and every copy was
        // already committed. Only forge-flagged members are touched: their
        // verified three-pointer layout is what makes the raw triplet
        // write sound (a BoundedVector or `vector<bool>` has no such
        // guarantee — and no in-slot storage this walk could prove, since
        // only allocations reach the slot through the window and their
        // headers are opaque here; the hook's quarantine covers those, and
        // in-slot `std::string` storage, in production).
        for op in &self.ops {
            if let CppFieldOp::PrimSeq {
                c_offset,
                forge: true,
                ..
            } = op
            {
                let field = payload.add(*c_offset);
                // Word 0 of the verified triplet is `begin`.
                let begin = std::ptr::read_unaligned(field as *const usize);
                if begin >= payload_base && begin < payload_base + payload_len {
                    std::ptr::write_unaligned(field as *mut VecTriplet, VecTriplet::EMPTY);
                }
            }
        }

        // Destroy the struct through the typesupport's `fini` (the C++
        // destructor) while its members are still readable: heap-side
        // containers are freed by the C++ runtime; storage the window
        // bump-allocated into the slot routes to the hook's quarantine
        // no-op, never glibc.
        if let Some(fini) = (*self.members).fini_function {
            fini(payload as *mut c_void);
        }

        // The struct is dead: write the head over it, zero the remnant.
        std::ptr::copy_nonoverlapping(head_buf.as_ptr(), payload, data_floor);
        std::ptr::write_bytes(payload.add(data_floor), 0, geo.tail_off - data_floor);

        Ok(scratch.outcome(&plan))
    }

    /// Release a windowed-borrow struct WITHOUT sealing — see the C twin
    /// ([`crate::type_bridge::BridgedMessage::fini_borrowed_payload`]):
    /// forge-flagged vector triplets whose storage lies inside the slot
    /// are emptied first (slot bytes are never allocator-owned; emptying
    /// leaks nothing and keeps `~vector` off SHM addresses even without
    /// the hook), then the typesupport's `fini` (the destructor) runs.
    ///
    /// # Safety
    /// `payload` must hold a valid, initialized C++ message of this type
    /// in a slot of `payload_len` bytes, and must not be used as a
    /// message afterwards.
    pub unsafe fn fini_borrowed_payload(&self, payload: *mut c_void, payload_len: usize) {
        let base = payload as usize;
        for op in &self.ops {
            if let CppFieldOp::PrimSeq {
                c_offset,
                forge: true,
                ..
            } = op
            {
                let field = (payload as *mut u8).add(*c_offset);
                // The verified three-pointer triplet: word 0 is `begin`
                // (`vector_triplet_layout_verified` gates forge-flagged
                // members on exactly this layout).
                let begin = std::ptr::read_unaligned(field as *const usize);
                if begin >= base && begin < base + payload_len {
                    std::ptr::write_unaligned(field as *mut VecTriplet, VecTriplet::EMPTY);
                }
            }
        }
        if let Some(fini) = (*self.members).fini_function {
            fini(payload);
        }
    }
}

#[cfg(test)]
mod cpp_package_tests {
    use super::cpp_package;

    /// Both accepted namespace spellings normalize to ONE
    /// canonical package, so the qualified name (and everything keyed on it —
    /// schema hash, slice-ceiling lookup, graph type name) agrees across
    /// them. Each shape against a hand oracle.
    #[test]
    fn both_namespace_spellings_normalize_to_the_canonical_package() {
        // The C++ convention (control).
        assert_eq!(cpp_package("geometry_msgs::msg"), "geometry_msgs");
        assert_eq!(cpp_package("tf2_msgs::msg"), "tf2_msgs");
        // The C-style spelling must map to the package too.
        assert_eq!(cpp_package("geometry_msgs__msg"), "geometry_msgs");
        assert_eq!(cpp_package("tf2_msgs__msg"), "tf2_msgs");
        // The other interface kinds, both spellings.
        assert_eq!(cpp_package("pkg::srv"), "pkg");
        assert_eq!(cpp_package("pkg__srv"), "pkg");
        assert_eq!(cpp_package("pkg__action"), "pkg");
        // A bare package (no interface segment) stays itself.
        assert_eq!(cpp_package("pkg"), "pkg");
        // Mixed: first "::"-segment taken, THEN the suffix stripped.
        assert_eq!(cpp_package("tf2_msgs__msg::extra"), "tf2_msgs");
    }

    /// Edge/adversarial: a single trailing underscore is NOT the separator
    /// (packages may end in `_msg`), and a namespace that IS the bare suffix
    /// stays verbatim rather than minting an empty package.
    #[test]
    fn near_miss_shapes_are_left_verbatim() {
        assert_eq!(cpp_package("my_msg"), "my_msg");
        assert_eq!(cpp_package("foo_msg::msg"), "foo_msg");
        assert_eq!(cpp_package("__msg"), "__msg");
        assert_eq!(cpp_package(""), "");
    }
}
