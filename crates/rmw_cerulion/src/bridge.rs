// SPDX-License-Identifier: AGPL-3.0-only
//! Language-agnostic bridge dispatch.
//!
//! rclpy hands rmw the C typesupport; rclcpp hands the C++ one. Both
//! resolve to introspection data and produce BYTE-IDENTICAL wire frames
//! — [`AnyBridge`] is the enum that lets the rest of the rmw layer not
//! care which language the host node speaks.
//!
//! Enum dispatch over `Box<dyn>` per the project convention (see the
//! `AnyPublisher` rationale in cerulion_core): no vtable, no extra
//! allocation, no `dyn` pointer crossing the cdylib boundary.

use std::os::raw::c_void;

use cerulion_core::codegen::layout::WireLayout;

use crate::type_bridge::{
    BorrowSeal, BorrowSlotGeometry, BridgeError, BridgedMessage, ForgeOutcome, SealRefusal,
    SealScratch, WindowExtent,
};
use crate::type_bridge_cpp::CppBridgedMessage;

pub enum AnyBridge {
    /// `rosidl_typesupport_introspection_c` (rclpy, rclc).
    C(BridgedMessage),
    /// `rosidl_typesupport_introspection_cpp` (rclcpp — MoveIt).
    Cpp(CppBridgedMessage),
}

impl AnyBridge {
    pub fn qualified_name(&self) -> &str {
        match self {
            Self::C(b) => &b.qualified_name,
            Self::Cpp(b) => &b.qualified_name,
        }
    }

    pub fn layout(&self) -> &WireLayout {
        match self {
            Self::C(b) => &b.layout,
            Self::Cpp(b) => &b.layout,
        }
    }

    pub fn schema_hash(&self) -> u64 {
        match self {
            Self::C(b) => b.schema_hash(),
            Self::Cpp(b) => b.schema_hash(),
        }
    }

    pub fn can_loan(&self) -> bool {
        match self {
            Self::C(b) => b.can_loan,
            Self::Cpp(b) => b.can_loan,
        }
    }

    /// The TAKE-side loan gate — distinct from
    /// [`Self::can_loan`] (the publish-side, fixed-only rule). True for a
    /// fixed type (the SHM pointer is handed out directly) AND for a type
    /// with at least one forgeable primitive sequence (an rmw-owned shadow
    /// carries the copied remainder while the sequence aims at SHM). See
    /// `BridgedMessage::can_loan_take` / `CppBridgedMessage::can_loan_take`.
    pub fn can_loan_take(&self) -> bool {
        match self {
            Self::C(b) => b.can_loan_take(),
            Self::Cpp(b) => b.can_loan_take(),
        }
    }

    /// May `rmw_borrow_loaned_message` serve this type
    /// through the borrow window (the heap hook)? False for a
    /// fixed type (which keeps the plain loan path — [`Self::can_loan`])
    /// and for every type with no forgeable primitive sequence to adopt.
    /// The CALLER additionally gates on the hook handshake
    /// (`crate::heaphook::active_hook`) — this is the TYPE half only.
    pub fn can_borrow_windowed(&self) -> bool {
        match self {
            Self::C(b) => b.can_borrow_windowed(),
            Self::Cpp(b) => b.can_borrow_windowed(),
        }
    }

    /// The windowed-borrow slot geometry, or `None` when
    /// [`Self::can_borrow_windowed`] does not hold.
    pub fn borrow_geometry(&self) -> Option<BorrowSlotGeometry> {
        match self {
            Self::C(b) => b.borrow_geometry(),
            Self::Cpp(b) => b.borrow_geometry(),
        }
    }

    /// Seal a filled windowed-borrow slot into a wire frame IN PLACE —
    /// plan-then-commit; `Err` leaves the struct intact for the exact-size
    /// copy-loan fallback, `Ok` consumes it (values extracted, adopted
    /// headers emptied, `fini` run, the head written). See
    /// [`crate::type_bridge::BridgedMessage::seal_borrowed_frame`].
    ///
    /// # Safety
    /// See the per-bridge docs: `payload`/`payload_len` describe the borrow
    /// slot's payload region holding a valid message of this type at
    /// offset 0; the window is DISARMED; referenced ranges are readable.
    pub unsafe fn seal_borrowed_frame(
        &self,
        payload: *mut u8,
        payload_len: usize,
        geo: &BorrowSlotGeometry,
        window: Option<WindowExtent>,
        max_gap_bytes: usize,
        scratch: &mut SealScratch,
    ) -> Result<BorrowSeal, SealRefusal> {
        match self {
            Self::C(b) => {
                b.seal_borrowed_frame(payload, payload_len, geo, window, max_gap_bytes, scratch)
            }
            Self::Cpp(b) => {
                b.seal_borrowed_frame(payload, payload_len, geo, window, max_gap_bytes, scratch)
            }
        }
    }

    /// Release a windowed-borrow struct WITHOUT sealing (the refusal /
    /// cancel path): in-slot forgeable storage is emptied (never handed
    /// to an allocator), then the typesupport's `fini` runs.
    ///
    /// # Safety
    /// `payload` must hold a valid, initialized message of this type in a
    /// slot of `payload_len` bytes, and must not be used as a message
    /// afterwards.
    pub unsafe fn fini_borrowed_payload(&self, payload: *mut c_void, payload_len: usize) {
        match self {
            Self::C(b) => b.fini_borrowed_payload(payload, payload_len),
            Self::Cpp(b) => b.fini_borrowed_payload(payload, payload_len),
        }
    }

    /// How many top-level members the forged take aims at SHM (zero for a
    /// fixed type, whose take needs no shadow).
    pub fn forged_sequence_count(&self) -> usize {
        match self {
            Self::C(b) => b.forged_sequence_count(),
            Self::Cpp(b) => b.forged_sequence_count(),
        }
    }

    /// Construct one rmw-owned shadow message (see the per-bridge docs).
    ///
    /// # Safety
    /// [`Self::can_loan_take`] must hold with a nonzero
    /// [`Self::forged_sequence_count`].
    pub unsafe fn new_shadow(&self) -> Option<*mut c_void> {
        match self {
            Self::C(b) => b.new_shadow(),
            Self::Cpp(b) => b.new_shadow(),
        }
    }

    /// Un-forge (the members in `forged`), `fini` and free a shadow from
    /// [`Self::new_shadow`].
    ///
    /// # Safety
    /// `shadow` must come from [`Self::new_shadow`] on this bridge and must
    /// not be used afterwards.
    pub unsafe fn destroy_shadow(&self, shadow: *mut c_void, forged: u64) {
        match self {
            Self::C(b) => b.destroy_shadow(shadow, forged),
            Self::Cpp(b) => b.destroy_shadow(shadow, forged),
        }
    }

    /// The forged take into a shadow — copies the small remainder, aims
    /// every forgeable sequence at or above the data floor at `payload`'s
    /// own bytes, copies one below it (counted). All-or-nothing for the
    /// aliases; `Err` leaves the shadow un-forged and carries the copies made
    /// before the failure (a nonzero `below_floor` ⇒ retire the shadow). See
    /// [`crate::type_bridge::BridgedMessage::unflatten_forged`].
    ///
    /// # Safety
    /// `shadow` from [`Self::new_shadow`]; `payload` must be a HELD SHM
    /// sample's post-header bytes that outlive every use of the forged
    /// members (un-forge with the returned mask before releasing the sample).
    pub unsafe fn unflatten_forged(
        &self,
        payload: &[u8],
        shadow: *mut c_void,
    ) -> Result<ForgeOutcome, ForgeOutcome> {
        match self {
            Self::C(b) => b.unflatten_forged(payload, shadow),
            Self::Cpp(b) => b.unflatten_forged(payload, shadow),
        }
    }

    /// Re-point the members of `shadow` selected by `forged` at nothing.
    /// Idempotent; a zero mask is a no-op.
    ///
    /// # Safety
    /// `shadow` must come from [`Self::new_shadow`] on this bridge.
    pub unsafe fn unforge(&self, shadow: *mut c_void, forged: u64) {
        match self {
            Self::C(b) => b.unforge(shadow, forged),
            Self::Cpp(b) => b.unforge(shadow, forged),
        }
    }

    /// True when no forgeable member of `shadow` selected by `mask`
    /// references anything (`u64::MAX` selects them all).
    ///
    /// # Safety
    /// `shadow` must come from [`Self::new_shadow`] on this bridge.
    pub unsafe fn forged_members_are_empty(&self, shadow: *const c_void, mask: u64) -> bool {
        match self {
            Self::C(b) => b.forged_members_are_empty(shadow, mask),
            Self::Cpp(b) => b.forged_members_are_empty(shadow, mask),
        }
    }

    /// Adopt-take: copy the members selected by `forged` from the wire
    /// into `msg` — the registration-failure rollback's copy arm (run AFTER
    /// [`Self::unforge`]; see
    /// [`crate::type_bridge::BridgedMessage::copy_forged_members`]).
    ///
    /// # Safety
    /// `msg` valid + initialized message of this type with the masked
    /// members EMPTY; `payload` the bytes the preceding `unflatten_forged`
    /// walked.
    pub unsafe fn copy_forged_members(
        &self,
        payload: &[u8],
        msg: *mut c_void,
        forged: u64,
    ) -> bool {
        match self {
            Self::C(b) => b.copy_forged_members(payload, msg, forged),
            Self::Cpp(b) => b.copy_forged_members(payload, msg, forged),
        }
    }

    /// Adopt-take: the exact `{address, len}` byte range each FORGED
    /// member of `msg` aims at (empty forged members skipped) — what the
    /// adoption branch registers with the heap hook, per-field.
    ///
    /// # Safety
    /// `msg` valid message of this type whose members in `forged` were just
    /// forged by [`Self::unflatten_forged`].
    pub unsafe fn forged_entry_ranges(
        &self,
        msg: *const c_void,
        forged: u64,
        out: &mut Vec<(usize, usize)>,
    ) {
        match self {
            Self::C(b) => b.forged_entry_ranges(msg, forged, out),
            Self::Cpp(b) => b.forged_entry_ranges(msg, forged, out),
        }
    }

    /// Can this frame's variable entries be resolved
    /// WITHOUT writing anything? The adopted take asks before its first
    /// write, so a malformed frame is refused with the caller's message
    /// BYTE-UNTOUCHED instead of half-overwritten. `Err` names the offending
    /// member and the verdict, so the refusal line can say WHICH and WHY (see
    /// [`crate::type_bridge::BridgedMessage::frame_entries_readable`] for
    /// what it does and does not cover).
    pub fn frame_entries_readable(
        &self,
        payload: &[u8],
    ) -> Result<(), (usize, crate::take_gate::EntryVerdict)> {
        match self {
            Self::C(b) => b.frame_entries_readable(payload),
            Self::Cpp(b) => b.frame_entries_readable(payload),
        }
    }

    /// Adopt-take: free + EMPTY every forge-flagged member's existing
    /// storage in a CALLER-owned message before an adopting take overwrites
    /// its headers (reuse safety — see
    /// [`crate::type_bridge::BridgedMessage::release_forgeable_members`]).
    /// Only ever called under the adopt grant.
    ///
    /// # Safety
    /// `msg` must be a valid, initialized message of this bridged type.
    pub unsafe fn release_forgeable_members(&self, msg: *mut c_void) {
        match self {
            Self::C(b) => b.release_forgeable_members(msg),
            Self::Cpp(b) => b.release_forgeable_members(msg),
        }
    }

    /// Repr(C) padding byte-ranges of the loanable struct
    /// (offset, len — relative to the payload start). Empty unless
    /// [`Self::can_loan`]. The loaned publish path zeroes these before
    /// send (deterministic frames, Principle #7 — and no process memory
    /// leaked into SHM through padding).
    pub fn loan_pad_ranges(&self) -> &[(usize, usize)] {
        match self {
            Self::C(b) => b.loan_pad_ranges(),
            Self::Cpp(b) => b.loan_pad_ranges(),
        }
    }

    /// Initialize a freshly-loaned SHM payload slot to rosidl
    /// defaults — the loaned message's real construction (rclcpp's
    /// `LoanedMessage` never placement-news on the loaned branch). See
    /// the per-bridge docs for the C-vs-C++ split (the C `__init` needs
    /// a zero pre-pass; the C++ ALL constructor writes every member).
    ///
    /// # Safety
    /// `payload` must point at a writable region of at least the bridged
    /// type's C size (the loaned slot's payload region), and the bridge
    /// must be loanable ([`Self::can_loan`] — recursively fixed, so the
    /// typesupport init allocates nothing).
    pub unsafe fn init_loaned_payload(&self, payload: *mut c_void) {
        match self {
            Self::C(b) => b.init_loaned_payload(payload),
            Self::Cpp(b) => b.init_loaned_payload(payload),
        }
    }

    /// # Safety
    /// `c_msg` must point at a valid message of this bridged type, in
    /// the bridge's language representation.
    pub unsafe fn flatten(
        &self,
        c_msg: *const c_void,
        sequence: u32,
        timestamp_ns: u64,
    ) -> Result<Vec<u8>, BridgeError> {
        match self {
            Self::C(b) => b.flatten(c_msg, sequence, timestamp_ns),
            Self::Cpp(b) => b.flatten(c_msg, sequence, timestamp_ns),
        }
    }

    /// Exact wire-frame size pre-pass (hostile-count validated; never
    /// dereferences sequence data). See
    /// [`crate::type_bridge::BridgedMessage::frame_size`].
    ///
    /// # Safety
    /// See [`Self::flatten`].
    pub unsafe fn frame_size(&self, c_msg: *const c_void) -> Result<usize, BridgeError> {
        match self {
            Self::C(b) => b.frame_size(c_msg),
            Self::Cpp(b) => b.frame_size(c_msg),
        }
    }

    /// Flatten DIRECTLY into a possibly-uninitialized exact-size buffer
    /// (the SHM-loan publish path). `Ok(n)` proves `n == out.len()` and
    /// every byte of `out` initialized; on `Err` nothing was written
    /// out of bounds and a loan backing `out` must be dropped, never
    /// sent. See
    /// [`crate::type_bridge::BridgedMessage::flatten_into_uninit`].
    ///
    /// # Safety
    /// See [`Self::flatten`].
    pub unsafe fn flatten_into_uninit(
        &self,
        c_msg: *const c_void,
        sequence: u32,
        timestamp_ns: u64,
        out: &mut [std::mem::MaybeUninit<u8>],
    ) -> Result<usize, BridgeError> {
        match self {
            Self::C(b) => b.flatten_into_uninit(c_msg, sequence, timestamp_ns, out),
            Self::Cpp(b) => b.flatten_into_uninit(c_msg, sequence, timestamp_ns, out),
        }
    }

    /// # Safety
    /// See [`Self::flatten`].
    pub unsafe fn flatten_payload_only(
        &self,
        c_msg: *const c_void,
    ) -> Result<Vec<u8>, BridgeError> {
        match self {
            Self::C(b) => b.flatten_payload_only(c_msg),
            Self::Cpp(b) => b.flatten_payload_only(c_msg),
        }
    }

    /// # Safety
    /// `c_msg` must point at a valid, INITIALIZED message of this
    /// bridged type (the rmw_take contract).
    pub unsafe fn unflatten(&self, payload: &[u8], c_msg: *mut c_void) -> bool {
        match self {
            Self::C(b) => b.unflatten(payload, c_msg),
            Self::Cpp(b) => b.unflatten(payload, c_msg),
        }
    }
}
