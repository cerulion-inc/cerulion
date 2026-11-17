// SPDX-License-Identifier: AGPL-3.0-only
//! Hand-mirrored `rosidl_typesupport_introspection_cpp` ABI types
//! (the C++ typesupport bridge that lets rclcpp — and
//! therefore MoveIt 2 — run over rmw_cerulion).
//!
//! Why hand-mirrored instead of bindgen: the introspection_cpp header is
//! C++ (namespaced, includes `<string>`), which bindgen handles poorly,
//! but the two structs we need are plain data + C function pointers —
//! a stable, documented layout (message_introspection.hpp). The layout
//! below takes its shape from the capability cfgs build.rs derives from
//! the very bindings this build compiles against, so the mirror and the
//! C-side `era_pins` can never disagree about the era. Five era groups,
//! three `MessageMember` shapes, each keyed on the capability that
//! introduced it:
//!
//! | build | `MessageMember` | `MessageMembers` | `ServiceMembers` |
//! |---|---|---|---|
//! | Foxy, Galactic (`not(cerulion_has_fetch_function)`) | 96, `resize_function` at 88 | 56 | 32 |
//! | Humble (`not(cerulion_has_is_key)`) | 112 | 56 | 32 |
//! | Iron (`cerulion_has_event_members`) | 112 | 56 | 40 |
//! | Jazzy, Kilted (`cerulion_has_is_key`) | 112, `is_key_` at 32 | 64 | 40 |
//! | Lyrical, Rolling (`cerulion_has_is_rosidl_buffer`) | 120, `is_rosidl_buffer_` at 112 | 64 | 40 |
//!
//! Foxy and Galactic predate `fetch_function` and `assign_function`, so
//! their member carries neither, and their generator leaves every other
//! accessor of a `bool` sequence null too. That member is still a
//! `std::vector<bool>`, which the bridge reads and fills through the shim
//! (`is_accessorless_bool_sequence`). Every other member kind is reachable
//! through the accessors that do exist.
//!
//! Container access is through the member's function pointers, except in
//! the compiled shim (`shim/cppstring_shim.cpp`: `std::string`, the `uint8[]`
//! fill, the accessorless `std::vector<bool>`, `rosidl::Buffer`, the adopted
//! buffer release) and where the forged take and the borrow seal write a
//! `VecTriplet`, only when `vector_triplet_layout_verified` holds.

use std::os::raw::{c_char, c_void};

/// Identifier string the rosidl typesupport dispatcher resolves for the
/// C++ introspection handle.
pub const INTROSPECTION_CPP_IDENTIFIER: &[u8] = b"rosidl_typesupport_introspection_cpp\0";

/// Mirror of `rosidl_runtime_cpp::MessageInitialization::ALL` (= 0):
/// full initialization — the generated constructor writes EVERY member,
/// declared defaults applied. Passed to
/// [`CppMessageMembers::init_function`] by the loaned-borrow
/// path.
pub const CPP_MSG_INIT_ALL: u32 = 0;

/// `fetch_function`'s signature: copy element `index` OUT into a
/// pre-allocated value. Present from Humble on ([`CppMessageMember`]).
pub type CppFetchFn = unsafe extern "C" fn(*const c_void, usize, *mut c_void);

/// `assign_function`'s signature: copy a value INTO element `index`.
/// Present from Humble on ([`CppMessageMember`]).
pub type CppAssignFn = unsafe extern "C" fn(*mut c_void, usize, *const c_void);

/// Mirror of `rosidl_typesupport_introspection_cpp::MessageMember`
/// (Jazzy/Kilted layout, with the Lyrical/Rolling tail field and the
/// pre-Humble absence of `fetch`/`assign` under their own cfgs).
#[repr(C)]
#[derive(Debug, Clone, Copy)]
pub struct CppMessageMember {
    pub name_: *const c_char,
    pub type_id_: u8,
    pub string_upper_bound_: usize,
    /// For ROS_TYPE_MESSAGE: the nested type's typesupport handle
    /// (same `rosidl_message_type_support_t` shape as the C side).
    pub members_: *const super::rosidl_message_type_support_t,
    /// Jazzy onward (keyed topics); absent on Humble and Iron, where
    /// `is_array_` sits at this offset instead.
    #[cfg(cerulion_has_is_key)]
    pub is_key_: bool,
    pub is_array_: bool,
    pub array_size_: usize,
    pub is_upper_bound_: bool,
    pub offset_: u32,
    pub default_value_: *const c_void,
    /// Array/sequence element count.
    pub size_function: Option<unsafe extern "C" fn(*const c_void) -> usize>,
    /// Const pointer to element `index`.
    pub get_const_function: Option<unsafe extern "C" fn(*const c_void, usize) -> *const c_void>,
    /// Mutable pointer to element `index`.
    pub get_function: Option<unsafe extern "C" fn(*mut c_void, usize) -> *mut c_void>,
    /// Copy element `index` OUT into a pre-allocated value (needed for
    /// `std::vector<bool>`, whose elements are not addressable). Humble
    /// onward; absent on Foxy and Galactic, where `resize_function` sits
    /// at this offset instead. Read through [`member_fetch`], which
    /// yields `None` where the field does not exist.
    #[cfg(cerulion_has_fetch_function)]
    pub fetch_function: Option<CppFetchFn>,
    /// Copy a value INTO element `index`. Humble onward, like
    /// `fetch_function`; read through [`member_assign`].
    #[cfg(cerulion_has_fetch_function)]
    pub assign_function: Option<CppAssignFn>,
    /// Resize the sequence (allocates through the C++ container).
    pub resize_function: Option<unsafe extern "C" fn(*mut c_void, usize)>,
    /// Lyrical/Rolling only: whether the member is a `rosidl_runtime_cpp::Buffer`
    /// (ros2/rosidl#942, appended last so every earlier offset is unchanged).
    /// Read by `is_unbounded_u8_vector` and by the C++ forge classifier:
    /// a Buffer member is a 16-byte pimpl object whose bytes live behind a
    /// heap-allocated impl, never a `std::vector`, so it takes the
    /// introspection accessor path (resize, get, copy) and is never forged
    /// or handed to the vector shim. It also keeps the stride matching the
    /// distro's member array.
    #[cfg(cerulion_has_is_rosidl_buffer)]
    pub is_rosidl_buffer_: bool,
}

/// Mirror of `rosidl_typesupport_introspection_cpp::MessageMembers`
/// (Jazzy/rolling layout).
#[repr(C)]
#[derive(Debug, Clone, Copy)]
pub struct CppMessageMembers {
    pub message_namespace_: *const c_char,
    pub message_name_: *const c_char,
    pub member_count_: u32,
    pub size_of_: usize,
    /// Jazzy onward; absent on Humble and Iron (56-byte `MessageMembers`).
    #[cfg(cerulion_has_is_key)]
    pub has_any_key_member_: bool,
    pub members_: *const CppMessageMember,
    /// rosidl init (takes a MessageInitialization enum). The
    /// loaned-borrow path calls it with [`CPP_MSG_INIT_ALL`] — rclcpp's
    /// `LoanedMessage` never placement-news on the loaned branch, so
    /// this call IS the loaned message's construction (rosidl defaults,
    /// not zeros). `rmw_take` never calls it: its contract hands us an
    /// initialized message.
    pub init_function: Option<unsafe extern "C" fn(*mut c_void, u32)>,
    pub fini_function: Option<unsafe extern "C" fn(*mut c_void)>,
}

/// Mirror of `rosidl_typesupport_introspection_cpp::ServiceMembers`.
#[repr(C)]
#[derive(Debug, Clone, Copy)]
pub struct CppServiceMembers {
    pub service_namespace_: *const c_char,
    pub service_name_: *const c_char,
    pub request_members_: *const CppMessageMembers,
    pub response_members_: *const CppMessageMembers,
    /// Service-event introspection (Iron+: Iron appended it while `is_key_`
    /// only arrived with Jazzy, so it hangs on its own capability). The
    /// bridge never reads it —
    /// it only touches `request_members_`/`response_members_` —
    /// but it is part of the real struct, so the mirror carries it for
    /// layout parity (a by-value copy of a truncated mirror would read
    /// past its end). Matches the C side's `event_members_`.
    #[cfg(cerulion_has_event_members)]
    pub event_members_: *const CppMessageMembers,
}

/// Compile-time layout pin for the hand-mirrored C++ introspection
/// structs (64-bit). The C structs are bindgen-stamped against the REAL
/// headers with `size_of`/`offset_of` assertions (`vendored_bindings.rs`)
/// — but the C++ ones CANNOT be: bindgen can't parse the namespaced C++
/// `message_introspection.hpp`, which is exactly why they are
/// hand-mirrored AND why `build.rs`'s deployment bindgen path only ever
/// includes the C headers (`wrapper.h`). So unlike the C structs, these
/// C++ structs get NO automated cross-check against any live header —
/// this const block plus a hand-verification against the
/// Jazzy/rolling layout ARE the only guard. It pins the *Rust mirror's
/// own* layout: a future edit that reorders a field, changes a width, or
/// inserts/removes one breaks the build. A distro that changes the C++
/// `MessageMember` layout *itself* is NOT caught here — that must be
/// re-verified by hand (and is why the runtime layout-drift plausibility
/// check in `collect_schemas_cpp` exists as a second line of defense).
///
/// The pin (and these mirrors) bake in 8-byte pointers/usize, so the
/// offsets are 64-bit-only. Make the unsupported-width intent explicit at
/// the rmw layer instead of relying on the C-side (ungated) bindgen
/// asserts to be the thing that fails first on a 32-bit build.
#[cfg(not(target_pointer_width = "64"))]
compile_error!(
    "rmw_cerulion supports 64-bit targets only — the rosidl introspection \
     ABI mirrors assume 8-byte pointers/usize (Jazzy on x86-64/aarch64)"
);

#[cfg(target_pointer_width = "64")]
const _: () = {
    use std::mem::{align_of, offset_of, size_of};

    #[cfg(not(cerulion_has_fetch_function))]
    assert!(size_of::<CppMessageMember>() == 96);
    #[cfg(all(cerulion_has_fetch_function, not(cerulion_has_is_rosidl_buffer)))]
    assert!(size_of::<CppMessageMember>() == 112);
    #[cfg(cerulion_has_is_rosidl_buffer)]
    assert!(size_of::<CppMessageMember>() == 120);
    #[cfg(cerulion_has_is_rosidl_buffer)]
    assert!(offset_of!(CppMessageMember, is_rosidl_buffer_) == 112);
    // The C++ mirror and the bindgen-generated C member must be the same
    // size in EVERY era: the two introspection languages grow in lockstep,
    // so a mirror that lags its era is caught here at compile time, not by
    // a misread member array at runtime. Both sides of the
    // `fetch_function` boundary carry the check, so the 96-byte shape is
    // pinned against its own C twin the way the 112- and 120-byte shapes
    // are against theirs.
    #[cfg(cerulion_has_fetch_function)]
    assert!(
        size_of::<CppMessageMember>()
            == size_of::<super::rosidl_typesupport_introspection_c__MessageMember>()
    );
    #[cfg(not(cerulion_has_fetch_function))]
    assert!(
        size_of::<CppMessageMember>()
            == size_of::<super::rosidl_typesupport_introspection_c__MessageMember>()
    );
    assert!(align_of::<CppMessageMember>() == 8);
    assert!(offset_of!(CppMessageMember, name_) == 0);
    assert!(offset_of!(CppMessageMember, type_id_) == 8);
    assert!(offset_of!(CppMessageMember, string_upper_bound_) == 16);
    assert!(offset_of!(CppMessageMember, members_) == 24);
    #[cfg(cerulion_has_is_key)]
    assert!(offset_of!(CppMessageMember, is_key_) == 32);
    #[cfg(cerulion_has_is_key)]
    assert!(offset_of!(CppMessageMember, is_array_) == 33);
    #[cfg(not(cerulion_has_is_key))]
    assert!(offset_of!(CppMessageMember, is_array_) == 32);
    assert!(offset_of!(CppMessageMember, array_size_) == 40);
    assert!(offset_of!(CppMessageMember, is_upper_bound_) == 48);
    assert!(offset_of!(CppMessageMember, offset_) == 52);
    assert!(offset_of!(CppMessageMember, default_value_) == 56);
    assert!(offset_of!(CppMessageMember, size_function) == 64);
    assert!(offset_of!(CppMessageMember, get_const_function) == 72);
    assert!(offset_of!(CppMessageMember, get_function) == 80);
    #[cfg(cerulion_has_fetch_function)]
    assert!(offset_of!(CppMessageMember, fetch_function) == 88);
    #[cfg(cerulion_has_fetch_function)]
    assert!(offset_of!(CppMessageMember, assign_function) == 96);
    #[cfg(cerulion_has_fetch_function)]
    assert!(offset_of!(CppMessageMember, resize_function) == 104);
    // Foxy and Galactic: no `fetch`/`assign`, so `resize_function` is the
    // last field and closes the struct at 96.
    #[cfg(not(cerulion_has_fetch_function))]
    assert!(offset_of!(CppMessageMember, resize_function) == 88);

    #[cfg(cerulion_has_is_key)]
    assert!(size_of::<CppMessageMembers>() == 64);
    #[cfg(not(cerulion_has_is_key))]
    assert!(size_of::<CppMessageMembers>() == 56);
    assert!(align_of::<CppMessageMembers>() == 8);
    assert!(offset_of!(CppMessageMembers, message_namespace_) == 0);
    assert!(offset_of!(CppMessageMembers, message_name_) == 8);
    assert!(offset_of!(CppMessageMembers, member_count_) == 16);
    assert!(offset_of!(CppMessageMembers, size_of_) == 24);
    #[cfg(cerulion_has_is_key)]
    assert!(offset_of!(CppMessageMembers, has_any_key_member_) == 32);
    #[cfg(cerulion_has_is_key)]
    assert!(offset_of!(CppMessageMembers, members_) == 40);
    #[cfg(cerulion_has_is_key)]
    assert!(offset_of!(CppMessageMembers, init_function) == 48);
    #[cfg(cerulion_has_is_key)]
    assert!(offset_of!(CppMessageMembers, fini_function) == 56);
    #[cfg(not(cerulion_has_is_key))]
    assert!(offset_of!(CppMessageMembers, members_) == 32);
    #[cfg(not(cerulion_has_is_key))]
    assert!(offset_of!(CppMessageMembers, init_function) == 40);
    #[cfg(not(cerulion_has_is_key))]
    assert!(offset_of!(CppMessageMembers, fini_function) == 48);

    #[cfg(cerulion_has_event_members)]
    assert!(size_of::<CppServiceMembers>() == 40);
    #[cfg(not(cerulion_has_event_members))]
    assert!(size_of::<CppServiceMembers>() == 32);
    assert!(align_of::<CppServiceMembers>() == 8);
    assert!(offset_of!(CppServiceMembers, service_namespace_) == 0);
    assert!(offset_of!(CppServiceMembers, service_name_) == 8);
    assert!(offset_of!(CppServiceMembers, request_members_) == 16);
    assert!(offset_of!(CppServiceMembers, response_members_) == 24);
    #[cfg(cerulion_has_event_members)]
    assert!(offset_of!(CppServiceMembers, event_members_) == 32);
};

/// The member's `fetch` accessor, or `None` where the C++ `MessageMember`
/// of this build carries no such field (Foxy and Galactic). The callers
/// that need it are the `std::vector<bool>` element walks, which on `None`
/// read an accessorless member through the shim and refuse any other;
/// every other member kind reads through accessors that exist on every era.
#[inline]
pub fn member_fetch(member: &CppMessageMember) -> Option<CppFetchFn> {
    #[cfg(cerulion_has_fetch_function)]
    let f = member.fetch_function;
    #[cfg(not(cerulion_has_fetch_function))]
    let f = {
        let _ = member;
        None
    };
    f
}

/// The member's `assign` accessor, or `None` where the C++
/// `MessageMember` of this build carries no such field (Foxy and
/// Galactic). See [`member_fetch`].
#[inline]
pub fn member_assign(member: &CppMessageMember) -> Option<CppAssignFn> {
    #[cfg(cerulion_has_fetch_function)]
    let f = member.assign_function;
    #[cfg(not(cerulion_has_fetch_function))]
    let f = {
        let _ = member;
        None
    };
    f
}

// ====================================================================
// std::string shim (compiled C++ — shim/cppstring_shim.cpp).
// ====================================================================

extern "C" {
    /// Read a `std::string`'s (data, len) without copying.
    pub fn rmw_cerulion_cppstring_view(s: *const c_void, data: *mut *const c_char, len: *mut usize);
    /// Replace a `std::string`'s contents (allocates through the
    /// string; abort-on-OOM like rosidl typesupports).
    pub fn rmw_cerulion_cppstring_assign(s: *mut c_void, data: *const c_char, len: usize);
    /// `sizeof(std::string)` on this platform.
    pub fn rmw_cerulion_cppstring_sizeof() -> usize;
    /// Placement-construct a `std::string` at `at` (test fixtures).
    pub fn rmw_cerulion_cppstring_construct(at: *mut c_void, data: *const c_char, len: usize);
    /// Destruct a placement-constructed `std::string` (test fixtures).
    pub fn rmw_cerulion_cppstring_destruct(s: *mut c_void);

    /// Replace a `std::vector<uint8_t>`'s contents in ONE alloc+copy
    /// (the value-init-free fast path for `uint8[]` payloads).
    /// The caller gates it to unbounded uint8 sequences -- see the shim
    /// comment for the UB rationale on int8/byte/BoundedVector.
    ///
    /// `pub(crate)` (NOT `pub`): this raw extern `static_cast`s any
    /// `void*` to `std::vector<uint8_t>*` and overwrites its contents, so
    /// calling it on anything else is heap corruption. The ONLY caller is
    /// the safe wrapper [`assign_u8_vector`] below (which the production
    /// bridge routes through under the [`is_unbounded_u8_vector`] gate);
    /// tests declare their own `extern "C"` block rather than reach this.
    pub(crate) fn rmw_cerulion_vector_u8_assign(v: *mut c_void, data: *const u8, len: usize);
    /// `sizeof(std::vector<uint8_t>)` on this platform (test fixtures).
    pub fn rmw_cerulion_vector_u8_sizeof() -> usize;
    /// Placement-construct an empty `std::vector<uint8_t>` (test fixtures).
    pub fn rmw_cerulion_vector_u8_construct(at: *mut c_void);
    /// Destruct a placement-constructed `std::vector<uint8_t>` (fixtures).
    pub fn rmw_cerulion_vector_u8_destruct(v: *mut c_void);
    /// `size()` of a `std::vector<uint8_t>` (test fixtures).
    pub fn rmw_cerulion_vector_u8_size(v: *const c_void) -> usize;
    /// `capacity()` of a `std::vector<uint8_t>` (test fixtures).
    pub fn rmw_cerulion_vector_u8_capacity(v: *const c_void) -> usize;
    /// `data()` of a `std::vector<uint8_t>` (test fixtures).
    pub fn rmw_cerulion_vector_u8_data(v: *const c_void) -> *const u8;

    /// The `std::vector<bool>` operations behind an accessorless `bool`
    /// sequence (Foxy and Galactic, `is_accessorless_bool_sequence`). Each
    /// `static_cast`s its pointer to `std::vector<bool>*`, so a pointer to
    /// anything else is undefined behaviour, and `get`/`set` index without a
    /// bounds check. The bridge reaches them only through `vector_bool_len`,
    /// `vector_bool_read` and `vector_bool_assign` below.
    ///
    /// `size()` and element `i`: read only, `pub` like the `uint8_t` readers
    /// above, so fixtures read a vector back through the same calls.
    pub fn rmw_cerulion_vector_bool_size(v: *const c_void) -> usize;
    pub fn rmw_cerulion_vector_bool_get(v: *const c_void, i: usize) -> bool;
    /// `resize(n)` (0 on success, 1 when it threw) and setting element `i`:
    /// `pub(crate)` (NOT `pub`), because they write through the cast; tests
    /// declare their own `extern "C"` block rather than reach these.
    pub(crate) fn rmw_cerulion_vector_bool_resize(v: *mut c_void, n: usize) -> std::os::raw::c_int;
    pub(crate) fn rmw_cerulion_vector_bool_set(v: *mut c_void, i: usize, value: bool);
    /// `sizeof(std::vector<bool>)` on this platform (test fixtures).
    pub fn rmw_cerulion_vector_bool_sizeof() -> usize;
    /// Placement-construct a `std::vector<bool>` of `n` elements, element `k`
    /// true when `bits[k]` is nonzero (test fixtures).
    pub fn rmw_cerulion_vector_bool_construct(at: *mut c_void, bits: *const u8, n: usize);
    /// Destruct a placement-constructed `std::vector<bool>` (test fixtures).
    pub fn rmw_cerulion_vector_bool_destruct(v: *mut c_void);

    /// The introspection accessors, each called inside a `noexcept` C++
    /// wrapper that catches every exception and returns 0 on success,
    /// nonzero when the accessor threw. A nonzero status is a refused frame
    /// on the Rust side, never a foreign unwind. No accessor pointer is
    /// called directly from Rust except a non-Buffer member's
    /// `size_function`. A Lyrical `rosidl::Buffer` member, whose accessors
    /// throw on a non-CPU backend, has its size, bytes and fill read and
    /// written through [`crate::rosidl_buffer`] instead.
    pub(crate) fn rmw_cerulion_member_get_const(
        f: unsafe extern "C" fn(*const c_void, usize) -> *const c_void,
        m: *const c_void,
        index: usize,
        out: *mut *const c_void,
    ) -> std::os::raw::c_int;
    pub(crate) fn rmw_cerulion_member_get(
        f: unsafe extern "C" fn(*mut c_void, usize) -> *mut c_void,
        m: *mut c_void,
        index: usize,
        out: *mut *mut c_void,
    ) -> std::os::raw::c_int;
    pub(crate) fn rmw_cerulion_member_resize(
        f: unsafe extern "C" fn(*mut c_void, usize),
        m: *mut c_void,
        size: usize,
    ) -> std::os::raw::c_int;
    pub(crate) fn rmw_cerulion_member_fetch(
        f: unsafe extern "C" fn(*const c_void, usize, *mut c_void),
        m: *const c_void,
        index: usize,
        out: *mut c_void,
    ) -> std::os::raw::c_int;
    pub(crate) fn rmw_cerulion_member_assign(
        f: unsafe extern "C" fn(*mut c_void, usize, *const c_void),
        m: *mut c_void,
        index: usize,
        value: *const c_void,
    ) -> std::os::raw::c_int;
    /// Test fixture: an accessor that always throws (proves the catch).
    #[cfg(test)]
    pub(crate) fn rmw_cerulion_throwing_get_const(m: *const c_void, index: usize) -> *const c_void;
    /// `sizeof(rosidl_typesupport_introspection_cpp::MessageMember)` from the
    /// distro's own C++ header, or 0 when that header was not on the shim's
    /// include path (a vendored build). Read by a test only.
    #[cfg(test)]
    pub(crate) fn rmw_cerulion_cpp_message_member_sizeof() -> usize;
    /// Adopt-take: release a primitive `std::vector`'s
    /// BUFFER through `::operator delete` — the pair `std::allocator`
    /// allocates with — never libc `free`. The one production caller is the
    /// C++ bridge's `release_forgeable_members` reuse pre-pass; see the
    /// shim's doc for why a previously-forged SHM `begin` still reaches the
    /// preloaded hook through it. `pub(crate)`: a raw deallocation of
    /// whatever pointer it is handed.
    ///
    /// Precondition, recorded because it cannot be checked here (an
    /// allocator mismatch otherwise): the buffer belongs to a vector
    /// instantiated with the DEFAULT `std::allocator`, which is the pair
    /// `::operator delete` matches. Nothing in the C++ introspection
    /// contract can narrow that: [`super::CppMessageMember`] exposes
    /// `size_`/`get_`/`get_const_`/`fetch_`/`assign_`/`resize_function` and
    /// no deallocator, and carries no allocator handle — so neither of the
    /// two obvious remedies is constructible from it. There is no
    /// provenance bit to gate on, and no allocator to release through; a
    /// `resize_function(field, 0)` clears the elements allocator-correctly
    /// but leaves the capacity, so overwriting the triplet afterwards would
    /// leak the block rather than mismatch it.
    ///
    /// The precondition is not this line's alone: `CppMessageMembers`
    /// carries `size_of_` and every member `offset_` computed for that same
    /// default instantiation, so a message built with a different allocator
    /// is outside the contract for every field the bridge reads, not just
    /// for the one it releases. The layout gate that DOES exist —
    /// [`vector_triplet_layout_verified`], measured
    /// against this shim's own `std::vector<uint8_t>` — refuses the whole
    /// forge path when the three-pointer shape does not hold.
    pub(crate) fn rmw_cerulion_vector_pod_release(begin: *mut c_void);
}

/// Is `member` an UNBOUNDED `uint8` sequence — the ONE introspection
/// shape whose C++ container is a plain `std::vector<uint8_t>` we may
/// reinterpret for the [`assign_u8_vector`] fast path?
///
/// The reinterpret-eligibility invariant, named + co-located with the
/// wrapper it guards. `type_id_ == UINT8` excludes `int8`
/// (`std::vector<int8_t>`), `octet` (`std::vector<std::byte>` /
/// `unsigned char` — layout-compatible but NOT type-`uint8`, kept off
/// the path so the gate boundary stays exactly UINT8), `bool`
/// (bit-packed), and every multi-byte primitive. `!is_upper_bound_`
/// keeps a rosidl `BoundedVector<uint8_t, N>` on the resize()+copy path, so
/// this gate is exactly the unbounded `uint8[]` class; the bridge refuses a
/// count over the bound before either path writes. The caller additionally checks `!is_bool` and
/// that this is a dynamic (non-fixed) array; a fixed `uint8[N]` array is
/// classified elsewhere and never reaches this predicate's fast path. On
/// Lyrical and Rolling a member flagged `is_rosidl_buffer_` is a rosidl
/// Buffer, never a vector, and is excluded first.
pub(crate) fn is_unbounded_u8_vector(member: &CppMessageMember) -> bool {
    // Lyrical and Rolling: an unbounded `uint8[]` member is a
    // `rosidl::Buffer<uint8_t>` (16 bytes, storage behind a heap pimpl), not
    // a `std::vector`; the shim's `static_cast` would read its two pointers
    // and the 8 bytes past the object as a vector triplet. Never a vector.
    #[cfg(cerulion_has_is_rosidl_buffer)]
    if member.is_rosidl_buffer_ {
        return false;
    }
    member.type_id_ == crate::type_bridge::ros_type::UINT8 && !member.is_upper_bound_
}

/// Fill a `std::vector<uint8_t>` from `bytes` in ONE alloc+copy, skipping
/// `resize`'s value-init memset. The SOLE safe entry point to
/// the raw [`rmw_cerulion_vector_u8_assign`] extern.
///
/// The default-`std::allocator` 3-pointer `std::vector` layout this
/// reinterpret assumes is verified once at bridge build by
/// [`debug_assert_vector_u8_layout`] (debug only).
///
/// # Safety
/// `field` MUST point at a live `std::vector<uint8_t>` built with the
/// platform's DEFAULT `std::allocator` — i.e. the introspection member
/// is an unbounded `uint8` sequence (`type_id_ == UINT8`, `is_array_`,
/// `!is_upper_bound_`; equivalently [`is_unbounded_u8_vector`] is true
/// AND the array is dynamic). Reinterpreting any other container
/// (`std::vector<int8_t>`/`<std::byte>`, a rosidl `BoundedVector`, a
/// custom-allocator vector, or a non-vector) as `std::vector<uint8_t>`
/// is heap corruption / UB. `bytes` must be a valid slice; `assign`
/// copies exactly `bytes.len()` bytes (`0` is fine — a no-op assign to
/// empty).
pub(crate) unsafe fn assign_u8_vector(field: *mut c_void, bytes: &[u8]) {
    rmw_cerulion_vector_u8_assign(field, bytes.as_ptr(), bytes.len());
}

/// Is `member` a `bool` sequence (unbounded, or bounded) whose typesupport
/// gives it NO accessor at all: `size`, `get`, `get_const` and `resize` null,
/// and no `fetch` or `assign`?
///
/// That is exactly what `rosidl_typesupport_introspection_cpp` emits for every
/// `bool[]` and `bool[<=N]` member before Humble (Foxy and Galactic share one
/// template, `msg__type_support.cpp.em`, which skips the accessors of a
/// `std::vector<bool>` because it has no element addresses). Every C++ message
/// rosidl generates declares such a member as `std::vector<bool>` with the
/// default allocator, or as `rosidl_runtime_cpp::BoundedVector<bool, N>`,
/// whose only base is that vector at offset 0 and which adds no member of its
/// own. The bridge therefore reads and fills it through the shim's
/// `std::vector<bool>` operations; the wire form is the one the `fetch` and
/// `assign` accessors produce from Humble on (one byte per element, 1 or 0),
/// so either end can be any distro.
///
/// Any other combination is not this shape: one with `fetch` and `assign`
/// (Humble on) goes through those accessors, and one lacking them, such as a
/// `size` function alone, is no generator's output and stays refused.
/// A fixed `bool[N]` is not a sequence and is copied as plain bytes.
pub(crate) fn is_accessorless_bool_sequence(member: &CppMessageMember) -> bool {
    member.type_id_ == crate::type_bridge::ros_type::BOOLEAN
        && member.is_array_
        && (member.array_size_ == 0 || member.is_upper_bound_)
        && member.size_function.is_none()
        && member.get_const_function.is_none()
        && member.get_function.is_none()
        && member.resize_function.is_none()
        && member_fetch(member).is_none()
        && member_assign(member).is_none()
}

/// Element count of an accessorless `bool` sequence.
///
/// # Safety
/// `field` points at the member's field inside a live C++ message, and
/// [`is_accessorless_bool_sequence`] holds for the member, so the field is a
/// `std::vector<bool>` (or the `BoundedVector` that is one at offset 0).
pub(crate) unsafe fn vector_bool_len(field: *const c_void) -> usize {
    // SAFETY: the caller's contract makes `field` a live std::vector<bool>,
    // which is the shim's only requirement for size().
    unsafe { rmw_cerulion_vector_bool_size(field) }
}

/// Copy the elements of an accessorless `bool` sequence into `dst` in the
/// wire form, one byte per element, 1 for true and 0 for false. Err, with
/// `dst` unwritten, when the sequence does not hold exactly `dst.len()`
/// elements (it changed after the caller sized the frame).
///
/// # Safety
/// As [`vector_bool_len`].
pub(crate) unsafe fn vector_bool_read(
    field: *const c_void,
    dst: &mut [u8],
) -> Result<(), &'static str> {
    // SAFETY: the caller's contract makes `field` a live std::vector<bool>.
    let len = unsafe { rmw_cerulion_vector_bool_size(field) };
    if dst.len() != len {
        return Err("the bool sequence's length changed between the size and copy passes");
    }
    for (i, b) in dst.iter_mut().enumerate() {
        // SAFETY: `field` is a live std::vector<bool> and `i < dst.len() ==
        // size()`, so the unchecked index in the shim is in range.
        *b = u8::from(unsafe { rmw_cerulion_vector_bool_get(field, i) });
    }
    Ok(())
}

/// Fill an accessorless `bool` sequence with exactly `bytes.len()` elements,
/// element `i` true when `bytes[i]` is nonzero (the decode rule of the
/// `assign` path). False when the resize threw; the frame is then refused.
///
/// # Safety
/// As [`vector_bool_len`], with `field` writable. A BOUNDED member's
/// caller must already have refused a `bytes.len()` above its bound: the
/// resize goes through the `std::vector<bool>` base, which does not know the
/// bound.
pub(crate) unsafe fn vector_bool_assign(field: *mut c_void, bytes: &[u8]) -> bool {
    // SAFETY: the caller's contract makes `field` a live, writable
    // std::vector<bool>; the shim catches a throwing resize and reports 1.
    if unsafe { rmw_cerulion_vector_bool_resize(field, bytes.len()) } != 0 {
        return false;
    }
    for (i, &b) in bytes.iter().enumerate() {
        // SAFETY: the resize above made size() == bytes.len() > i.
        unsafe { rmw_cerulion_vector_bool_set(field, i, b != 0) };
    }
    true
}

/// Debug-only, run-once check that `std::vector<uint8_t>` has the default
/// 3-pointer (begin/end/cap) libstdc++/libc++ layout the fast-path
/// reinterpret in [`assign_u8_vector`] assumes. An exotic STL or a
/// custom allocator would change `sizeof`, making that reinterpret UB;
/// called at C++ bridge build so it trips LOUDLY at startup instead.
/// Compiled out in release (`debug_assertions` off) — the FFI `sizeof`
/// call never runs.
#[inline]
pub(crate) fn debug_assert_vector_u8_layout() {
    #[cfg(debug_assertions)]
    {
        use std::sync::Once;
        static CHECKED: Once = Once::new();
        CHECKED.call_once(|| {
            let sz = unsafe { rmw_cerulion_vector_u8_sizeof() };
            let expected = 3 * std::mem::size_of::<usize>();
            debug_assert_eq!(
                sz, expected,
                "std::vector<uint8_t> is {sz} bytes, not the expected 3 pointers \
                 ({expected}); an exotic STL or custom allocator would make the \
                 uint8[] assign() fast-path reinterpret UB"
            );
        });
    }
}

/// The `std::vector<T>` representation the forged loaned take
/// writes and reads: the default-`std::allocator` THREE-POINTER
/// triplet `{begin, end, end_of_storage}` — libstdc++'s `_Vector_impl_data`
/// and libc++'s `__begin_/__end_/__end_cap_` alike. `repr(C)`, three
/// machine words, written with `ptr::write_unaligned` at the member's
/// introspection offset.
///
/// A FORGED triplet aims `begin` at a held SHM sample's bytes with
/// `end == end_of_storage == begin + len` (capacity == size, so every
/// growth path reallocates rather than writing past the frame); the
/// UN-FORGED state is the all-null triplet, which is exactly what a
/// default-constructed vector holds and what `~vector` treats as
/// "nothing to deallocate" — so a shadow message whose forged members were
/// un-forged before its destructor runs frees NOTHING through the C++
/// allocator for a forged buffer.
#[repr(C)]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct VecTriplet {
    pub begin: usize,
    pub end: usize,
    pub end_of_storage: usize,
}

impl VecTriplet {
    /// The un-forged / empty representation.
    pub const EMPTY: Self = Self {
        begin: 0,
        end: 0,
        end_of_storage: 0,
    };

    /// Does this vector OWN a heap allocation? This is the question a
    /// release must ask, and the answer is
    /// CAPACITY, not `begin != 0`. `std::vector`'s three pointers make
    /// capacity `end_of_storage - begin`, so `begin == end_of_storage`
    /// owns nothing however non-null `begin` is — which is exactly what
    /// [`Self::forged`] mints for a zero-length entry: a live SHM address
    /// with no allocation behind it and no hook registration to release
    /// it. Deleting that pointer is an invalid deallocation.
    pub fn owns_storage(&self) -> bool {
        self.end_of_storage > self.begin
    }

    /// A triplet aimed at `len` bytes starting at `begin`, capacity == size.
    pub fn forged(begin: usize, len: usize) -> Self {
        let end = begin + len;
        Self {
            begin,
            end,
            end_of_storage: end,
        }
    }
}

/// Run-once, RELEASE-MODE verification that this process's `std::vector`
/// really has the [`VecTriplet`] representation the forged take relies on
/// — the `abi_layout` / `cppstring_shim` precedent applied to the one C++
/// layout assumption this crate makes beyond function pointers.
///
/// Measured through the compiled shim on a REAL `std::vector<uint8_t>`:
/// `sizeof` is three words; a default-constructed vector reads as the
/// all-null triplet (the un-forge representation); after `assign` of five
/// bytes the three words are `data()`, `data() + size()` and
/// `data() + capacity()`. Any disagreement (an exotic STL, a debug-iterator
/// build that widens the object, a custom allocator baked into the
/// typesupport) is reported ONCE at `warn!` and every C++ type is then
/// refused for the forged take — the copying take, loudly, never a
/// forged pointer written into a container of unknown shape.
///
/// Unlike [`debug_assert_vector_u8_layout`] (the `assign` fast
/// path's debug-only `sizeof` check) this runs in every build: the forged
/// take WRITES the container's private words, so a wrong layout here is
/// heap corruption in the host process, not a wasted memcpy.
pub(crate) fn vector_triplet_layout_verified() -> bool {
    use std::sync::OnceLock;
    static VERIFIED: OnceLock<bool> = OnceLock::new();
    *VERIFIED.get_or_init(probe_vector_triplet_layout)
}

/// The probe behind [`vector_triplet_layout_verified`]; returns the verdict
/// and logs the first failure it finds.
fn probe_vector_triplet_layout() -> bool {
    let words = std::mem::size_of::<VecTriplet>();
    // SAFETY: fixture helpers over a stack slot that is at least
    // `sizeof(std::vector<uint8_t>)` bytes and suitably aligned; every
    // constructed vector is destructed before the slot goes away.
    unsafe {
        let sz = rmw_cerulion_vector_u8_sizeof();
        if sz != words {
            tracing::warn!(
                sizeof_vector = sz,
                expected = words,
                "std::vector is not the three-pointer layout; forged loaned takes are \
                 disabled for every C++ type (copying take instead)"
            );
            return false;
        }
        #[repr(C, align(16))]
        struct Slot([usize; 3]);
        let mut slot = Slot([usize::MAX; 3]);
        let v = slot.0.as_mut_ptr() as *mut c_void;
        rmw_cerulion_vector_u8_construct(v);
        let empty = std::ptr::read_unaligned(v as *const VecTriplet);
        if empty != VecTriplet::EMPTY {
            rmw_cerulion_vector_u8_destruct(v);
            tracing::warn!(
                "a default-constructed std::vector is not the all-null triplet; forged \
                 loaned takes are disabled for every C++ type (copying take instead)"
            );
            return false;
        }
        let bytes = [1u8, 2, 3, 4, 5];
        rmw_cerulion_vector_u8_assign(v, bytes.as_ptr(), bytes.len());
        let forged = std::ptr::read_unaligned(v as *const VecTriplet);
        let data = rmw_cerulion_vector_u8_data(v) as usize;
        let size = rmw_cerulion_vector_u8_size(v);
        let capacity = rmw_cerulion_vector_u8_capacity(v);
        rmw_cerulion_vector_u8_destruct(v);
        let expected = VecTriplet {
            begin: data,
            end: data + size,
            end_of_storage: data + capacity,
        };
        if forged != expected || size != bytes.len() {
            tracing::warn!(
                begin = forged.begin,
                end = forged.end,
                end_of_storage = forged.end_of_storage,
                data,
                size,
                capacity,
                "std::vector's words are not {{begin, end, end_of_storage}}; forged loaned \
                 takes are disabled for every C++ type (copying take instead)"
            );
            return false;
        }
    }
    true
}

/// Borrow a `std::string`'s bytes. Caps at `max` BEFORE trusting the
/// length (corrupt-header posture, same as the C bridge).
///
/// # Safety
/// `s` must point at a live `std::string`; the borrow is valid only
/// while the string is neither mutated nor destroyed.
pub unsafe fn cppstring_bytes<'a>(s: *const c_void, max: usize) -> Result<&'a [u8], &'static str> {
    let mut data: *const c_char = std::ptr::null();
    let mut len: usize = 0;
    rmw_cerulion_cppstring_view(s, &mut data, &mut len);
    if len == 0 {
        return Ok(&[]);
    }
    if len > max {
        return Err("std::string size exceeds MAX_FRAME_BYTES (corrupt)");
    }
    if data.is_null() {
        return Err("std::string with null data and nonzero size (corrupt)");
    }
    Ok(std::slice::from_raw_parts(data.cast::<u8>(), len))
}

#[cfg(test)]
mod accessor_wrapper_tests {
    use super::*;

    /// An accessor that throws is caught by the wrapper and reported as a
    /// nonzero status with the out pointer untouched; the process neither
    /// aborts nor unwinds into Rust.
    #[test]
    fn a_throwing_accessor_is_a_nonzero_status_not_an_unwind() {
        let mut out: *const c_void = std::ptr::null();
        let status = unsafe {
            rmw_cerulion_member_get_const(
                rmw_cerulion_throwing_get_const,
                std::ptr::null(),
                0,
                &mut out,
            )
        };
        assert_ne!(status, 0, "the throw must surface as a status");
        assert!(out.is_null());
    }

    /// On a real-header build the hand-written mirror is exactly the C++
    /// header's `MessageMember`.
    ///
    /// The probe returns 0 only when
    /// `rosidl_typesupport_introspection_cpp/message_introspection.hpp` was
    /// not on the shim's include path, which is the VENDORED build: there is
    /// no distro header to compare with, and the test says "not applicable"
    /// rather than claiming agreement. A GENERATED build is the opposite
    /// case and must never take that exit: the shim's three `static_assert`
    /// arms all sit behind the same `__has_include`, so a 0 here would mean
    /// the whole C++ era pin, this test included, silently did nothing on
    /// the one build shape that can check it.
    #[test]
    fn the_cpp_member_mirror_matches_the_distro_header_where_one_exists() {
        let from_header = unsafe { rmw_cerulion_cpp_message_member_sizeof() };
        #[cfg(cerulion_rmw_generated_bindings)]
        assert_ne!(
            from_header, 0,
            "a build against real distro headers must find \
             rosidl_typesupport_introspection_cpp/message_introspection.hpp on the shim's \
             include path; 0 means the shim's era static_asserts compiled to nothing"
        );
        if from_header == 0 {
            return;
        }
        assert_eq!(std::mem::size_of::<CppMessageMember>(), from_header);
    }

    /// A non-throwing accessor passes its value through with status 0.
    #[test]
    fn a_plain_accessor_passes_through_with_status_zero() {
        unsafe extern "C" fn first(m: *const c_void, _i: usize) -> *const c_void {
            m
        }
        let marker = 7u8;
        let mut out: *const c_void = std::ptr::null();
        let status = unsafe {
            rmw_cerulion_member_get_const(first, &marker as *const u8 as *const c_void, 0, &mut out)
        };
        assert_eq!(status, 0);
        assert_eq!(out as usize, &marker as *const u8 as usize);
    }
}

#[cfg(test)]
mod accessorless_bool_tests {
    use super::*;
    use crate::type_bridge::ros_type;

    unsafe extern "C" fn any_size(_: *const c_void) -> usize {
        0
    }
    unsafe extern "C" fn any_get_const(m: *const c_void, _: usize) -> *const c_void {
        m
    }
    unsafe extern "C" fn any_get(m: *mut c_void, _: usize) -> *mut c_void {
        m
    }
    unsafe extern "C" fn any_resize(_: *mut c_void, _: usize) {}
    #[cfg(cerulion_has_fetch_function)]
    unsafe extern "C" fn any_fetch(_: *const c_void, _: usize, _: *mut c_void) {}
    #[cfg(cerulion_has_fetch_function)]
    unsafe extern "C" fn any_assign(_: *mut c_void, _: usize, _: *const c_void) {}

    /// The Foxy and Galactic `bool[]` member: every accessor null.
    fn foxy_bool_seq() -> CppMessageMember {
        CppMessageMember {
            name_: c"flags".as_ptr(),
            type_id_: ros_type::BOOLEAN,
            string_upper_bound_: 0,
            members_: std::ptr::null(),
            #[cfg(cerulion_has_is_key)]
            is_key_: false,
            is_array_: true,
            array_size_: 0,
            is_upper_bound_: false,
            offset_: 0,
            default_value_: std::ptr::null(),
            size_function: None,
            get_const_function: None,
            get_function: None,
            #[cfg(cerulion_has_fetch_function)]
            fetch_function: None,
            #[cfg(cerulion_has_fetch_function)]
            assign_function: None,
            resize_function: None,
            #[cfg(cerulion_has_is_rosidl_buffer)]
            is_rosidl_buffer_: false,
        }
    }

    /// The predicate admits exactly the template's shape: an unbounded or
    /// bounded `bool` sequence with every accessor null. Each single accessor
    /// set, a fixed `bool[N]`, a scalar `bool` and a non-bool sequence are
    /// all outside it, so each keeps the path it had (the accessors, the
    /// fixed-section copy, or the refusal).
    #[test]
    fn only_a_bool_sequence_with_every_accessor_null_is_accessorless() {
        assert!(is_accessorless_bool_sequence(&foxy_bool_seq()));
        let mut bounded = foxy_bool_seq();
        bounded.array_size_ = 4;
        bounded.is_upper_bound_ = true;
        assert!(is_accessorless_bool_sequence(&bounded));

        let mut with: Vec<(&str, CppMessageMember)> = Vec::new();
        let mut m = foxy_bool_seq();
        m.size_function = Some(any_size);
        with.push(("size", m));
        let mut m = foxy_bool_seq();
        m.get_const_function = Some(any_get_const);
        with.push(("get_const", m));
        let mut m = foxy_bool_seq();
        m.get_function = Some(any_get);
        with.push(("get", m));
        let mut m = foxy_bool_seq();
        m.resize_function = Some(any_resize);
        with.push(("resize", m));
        #[cfg(cerulion_has_fetch_function)]
        {
            let mut m = foxy_bool_seq();
            m.fetch_function = Some(any_fetch);
            with.push(("fetch", m));
            let mut m = foxy_bool_seq();
            m.assign_function = Some(any_assign);
            with.push(("assign", m));
        }
        for (accessor, m) in &with {
            assert!(
                !is_accessorless_bool_sequence(m),
                "a bool sequence with a {accessor} accessor is not the accessorless shape"
            );
        }

        let mut fixed = foxy_bool_seq();
        fixed.array_size_ = 3;
        assert!(
            !is_accessorless_bool_sequence(&fixed),
            "bool[3] is not a sequence"
        );
        let mut scalar = foxy_bool_seq();
        scalar.is_array_ = false;
        assert!(!is_accessorless_bool_sequence(&scalar), "a scalar bool");
        let mut bytes = foxy_bool_seq();
        bytes.type_id_ = ros_type::UINT8;
        assert!(!is_accessorless_bool_sequence(&bytes), "a uint8 sequence");
    }

    /// The read refuses a destination of any length other than the
    /// vector's, longer (which would index past its end) or shorter, and
    /// writes nothing into it; a destination of the vector's own length
    /// reads every element as 1 or 0.
    #[test]
    fn the_read_refuses_a_destination_whose_length_is_not_the_vectors() {
        #[repr(C, align(16))]
        struct Slot([u8; 64]);
        let mut slot = Slot([0; 64]);
        let at = slot.0.as_mut_ptr() as *mut c_void;
        assert!(unsafe { rmw_cerulion_vector_bool_sizeof() } <= 64);
        let elems = [1u8, 0, 1];
        unsafe { rmw_cerulion_vector_bool_construct(at, elems.as_ptr(), elems.len()) };

        let mut long = [0xA5u8; 4];
        assert!(unsafe { vector_bool_read(at, &mut long) }.is_err());
        assert_eq!(long, [0xA5; 4], "a refused read writes nothing");
        let mut short = [0xA5u8; 2];
        assert!(unsafe { vector_bool_read(at, &mut short) }.is_err());
        assert_eq!(short, [0xA5; 2], "a refused read writes nothing");
        let mut exact = [0xA5u8; 3];
        assert!(unsafe { vector_bool_read(at, &mut exact) }.is_ok());
        assert_eq!(exact, [1, 0, 1]);
        assert_eq!(unsafe { vector_bool_len(at) }, 3);
        unsafe { rmw_cerulion_vector_bool_destruct(at) };
    }
}
