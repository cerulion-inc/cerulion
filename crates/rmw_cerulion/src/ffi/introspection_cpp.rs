// SPDX-License-Identifier: AGPL-3.0-only
//! Hand-mirrored `rosidl_typesupport_introspection_cpp` ABI types
//! (the C++ typesupport bridge that lets rclcpp — and
//! therefore MoveIt 2 — run over rmw_cerulion).
//!
//! Why hand-mirrored instead of bindgen: the introspection_cpp header is
//! C++ (namespaced, includes `<string>`), which bindgen handles poorly,
//! but the two structs we need are plain data + C function pointers —
//! a stable, documented layout (message_introspection.hpp). The layout
//! below matches **Jazzy and Kilted** (`is_key_` on MessageMember,
//! `has_any_key_member_` on MessageMembers — both added for Iron+
//! keyed-topic support) and, under `cfg(cerulion_has_is_rosidl_buffer)`,
//! the **Lyrical/Rolling** shape, which appends one `bool is_rosidl_buffer_`
//! to MessageMember (112 to 120 bytes) and changes nothing else. The cfg is
//! derived by build.rs from the very bindings this build compiles against,
//! so the mirror and the C-side `era_pins` can never disagree about the
//! era. Older distros (Humble, Foxy) lack `is_key_` and are NOT supported
//! by this bridge yet.
//!
//! Container access is exclusively through the member's function
//! pointers (`size/get/get_const/fetch/assign/resize`) — never through
//! assumed `std::vector` internals. The ONLY C++ ABI we touch directly
//! is `std::string`, via the compiled shim (`shim/cppstring_shim.cpp`),
//! which uses the platform's real ABI by construction.

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

/// Mirror of `rosidl_typesupport_introspection_cpp::MessageMember`
/// (Jazzy/Kilted layout, plus the Lyrical/Rolling tail field under its cfg).
#[repr(C)]
#[derive(Debug, Clone, Copy)]
pub struct CppMessageMember {
    pub name_: *const c_char,
    pub type_id_: u8,
    pub string_upper_bound_: usize,
    /// For ROS_TYPE_MESSAGE: the nested type's typesupport handle
    /// (same `rosidl_message_type_support_t` shape as the C side).
    pub members_: *const super::rosidl_message_type_support_t,
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
    /// `std::vector<bool>`, whose elements are not addressable).
    pub fetch_function: Option<unsafe extern "C" fn(*const c_void, usize, *mut c_void)>,
    /// Copy a value INTO element `index`.
    pub assign_function: Option<unsafe extern "C" fn(*mut c_void, usize, *const c_void)>,
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
    /// Action/service-event introspection (Jazzy+). The bridge never reads it —
    /// it only touches `request_members_`/`response_members_` —
    /// but it is part of the real struct, so the mirror carries it for
    /// layout parity (a by-value copy of a truncated mirror would read
    /// past its end). Matches the C side's `event_members_`.
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

    #[cfg(not(cerulion_has_is_rosidl_buffer))]
    assert!(size_of::<CppMessageMember>() == 112);
    #[cfg(cerulion_has_is_rosidl_buffer)]
    assert!(size_of::<CppMessageMember>() == 120);
    #[cfg(cerulion_has_is_rosidl_buffer)]
    assert!(offset_of!(CppMessageMember, is_rosidl_buffer_) == 112);
    // The C++ mirror and the bindgen-generated C member must be the same
    // size in every era the bridge supports (Jazzy onward, where `is_key_`
    // exists): the two introspection languages grow in lockstep, so a
    // mirror that lags its era is caught here at compile time, not by a
    // misread member array at runtime. Pre-Jazzy builds refuse the C++ arm
    // at registration instead, so the check is not asserted there.
    #[cfg(cerulion_has_is_key)]
    assert!(
        size_of::<CppMessageMember>()
            == size_of::<super::rosidl_typesupport_introspection_c__MessageMember>()
    );
    assert!(align_of::<CppMessageMember>() == 8);
    assert!(offset_of!(CppMessageMember, name_) == 0);
    assert!(offset_of!(CppMessageMember, type_id_) == 8);
    assert!(offset_of!(CppMessageMember, string_upper_bound_) == 16);
    assert!(offset_of!(CppMessageMember, members_) == 24);
    assert!(offset_of!(CppMessageMember, is_key_) == 32);
    assert!(offset_of!(CppMessageMember, is_array_) == 33);
    assert!(offset_of!(CppMessageMember, array_size_) == 40);
    assert!(offset_of!(CppMessageMember, is_upper_bound_) == 48);
    assert!(offset_of!(CppMessageMember, offset_) == 52);
    assert!(offset_of!(CppMessageMember, default_value_) == 56);
    assert!(offset_of!(CppMessageMember, size_function) == 64);
    assert!(offset_of!(CppMessageMember, get_const_function) == 72);
    assert!(offset_of!(CppMessageMember, get_function) == 80);
    assert!(offset_of!(CppMessageMember, fetch_function) == 88);
    assert!(offset_of!(CppMessageMember, assign_function) == 96);
    assert!(offset_of!(CppMessageMember, resize_function) == 104);

    assert!(size_of::<CppMessageMembers>() == 64);
    assert!(align_of::<CppMessageMembers>() == 8);
    assert!(offset_of!(CppMessageMembers, message_namespace_) == 0);
    assert!(offset_of!(CppMessageMembers, message_name_) == 8);
    assert!(offset_of!(CppMessageMembers, member_count_) == 16);
    assert!(offset_of!(CppMessageMembers, size_of_) == 24);
    assert!(offset_of!(CppMessageMembers, has_any_key_member_) == 32);
    assert!(offset_of!(CppMessageMembers, members_) == 40);
    assert!(offset_of!(CppMessageMembers, init_function) == 48);
    assert!(offset_of!(CppMessageMembers, fini_function) == 56);

    assert!(size_of::<CppServiceMembers>() == 40);
    assert!(align_of::<CppServiceMembers>() == 8);
    assert!(offset_of!(CppServiceMembers, service_namespace_) == 0);
    assert!(offset_of!(CppServiceMembers, service_name_) == 8);
    assert!(offset_of!(CppServiceMembers, request_members_) == 16);
    assert!(offset_of!(CppServiceMembers, response_members_) == 24);
    assert!(offset_of!(CppServiceMembers, event_members_) == 32);
};

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
/// excludes a rosidl `BoundedVector<uint8_t, N>`, whose layout differs
/// from `std::vector`. The caller additionally checks `!is_bool` and
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
    Ok(std::slice::from_raw_parts(data as *const u8, len))
}
