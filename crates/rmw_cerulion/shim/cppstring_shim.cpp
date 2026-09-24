// std::string ABI shim for the introspection_cpp bridge.
//
// rclcpp hands rmw the C++ typesupport; its introspection data exposes
// containers through function pointers (size/get/resize/fetch/assign),
// which makes std::vector access ABI-safe from Rust — but plain
// `std::string` FIELDS have no accessor functions (C++ typesupports
// cast directly). Reading/writing a std::string from Rust would mean
// hard-coding the libstdc++/libc++ layout; instead these helpers are
// COMPILED C++, so the platform's real ABI is always used.
//
// Kept deliberately tiny: view (read) + assign (write). Every function
// is `noexcept`: a C++ exception escaping a noexcept function calls
// std::terminate — DEFINED behavior — instead of unwinding through the
// extern "C" boundary into Rust frames (which is UB). The realistic
// throw here is std::bad_alloc from assign on a huge wire-controlled
// length; terminate-on-OOM matches rosidl typesupport behavior.
#include <cstddef>
#include <cstdint>
#include <new>
#include <string>
#include <vector>

extern "C" {

/// Read a std::string's (data, len) without copying.
void rmw_cerulion_cppstring_view(const void *s, const char **data, size_t *len) noexcept {
  const auto *str = static_cast<const std::string *>(s);
  *data = str->data();
  *len = str->size();
}

/// Replace a std::string's contents (allocates through the string).
void rmw_cerulion_cppstring_assign(void *s, const char *data, size_t len) noexcept {
  static_cast<std::string *>(s)->assign(data, len);
}

/// Replace a std::vector<uint8_t>'s contents in ONE alloc+copy.
/// assign() sizes the vector and copies exactly `len` bytes with NO
/// value-init memset -- unlike resize()+copy, whose resize zero-fills the
/// whole span first (a redundant `rep stos` on the receive
/// path). The Rust caller GATES this to uint8[] payloads (introspection
/// type id UINT8, unbounded): a std::vector<int8_t>/<std::byte> or a
/// rosidl BoundedVector has a different layout, so reinterpreting it as
/// vector<uint8_t> would be UB -- those keep the resize()+copy path.
///
/// Reinterpret assumption: `v` is a std::vector<uint8_t> built with the
/// platform's DEFAULT std::allocator (the 3-pointer libstdc++/libc++
/// layout). The Rust bridge verifies sizeof == 3 pointers once at build
/// (debug); a custom-allocator/exotic-STL layout is out of contract.
///
/// noexcept fail-fast: a throw escaping here calls std::terminate
/// (DEFINED). Two realistic throws, BOTH terminate: std::bad_alloc (OOM
/// on a huge wire-controlled `len`) and std::length_error (`len` >
/// max_size()). This is DELIBERATELY ASYMMETRIC with the C bridge
/// (assign_prim_sequence), which drops-and-warns on OOM: here terminate
/// is the safer choice -- it matches rosidl typesupport behavior, a
/// robot must not run on a silently-truncated perception frame, and it
/// beats a resize()+copy path, whose BoundedVector length_error
/// would UB-unwind through the extern "C" boundary into Rust frames.
void rmw_cerulion_vector_u8_assign(void *v, const uint8_t *data, size_t len) noexcept {
  static_cast<std::vector<uint8_t> *>(v)->assign(data, data + len);
}

}  // extern "C"

// ---------------------------------------------------------------------
// Fixture helpers (used by tests to build REAL C++ strings inside
// hand-built message buffers; harmless to ship — a few bytes of code).
// ---------------------------------------------------------------------
extern "C" {

/// sizeof(std::string) on this platform (32 on libstdc++ and libc++).
size_t rmw_cerulion_cppstring_sizeof() noexcept { return sizeof(std::string); }

/// Placement-construct a std::string at `at` (caller provides aligned
/// storage of rmw_cerulion_cppstring_sizeof() bytes).
void rmw_cerulion_cppstring_construct(void *at, const char *data, size_t len) noexcept {
  new (at) std::string(data, len);
}

/// Destruct a placement-constructed std::string.
void rmw_cerulion_cppstring_destruct(void *s) noexcept {
  using std::string;
  static_cast<string *>(s)->~string();
}

/// sizeof(std::vector<uint8_t>) on this platform (24 on libstdc++ and
/// libc++). Fixture helper.
size_t rmw_cerulion_vector_u8_sizeof() noexcept {
  return sizeof(std::vector<uint8_t>);
}

/// Placement-construct an empty std::vector<uint8_t> at `at` (caller
/// provides aligned storage of rmw_cerulion_vector_u8_sizeof() bytes).
void rmw_cerulion_vector_u8_construct(void *at) noexcept {
  new (at) std::vector<uint8_t>();
}

/// Destruct a placement-constructed std::vector<uint8_t>.
void rmw_cerulion_vector_u8_destruct(void *v) noexcept {
  static_cast<std::vector<uint8_t> *>(v)->~vector();
}

/// size() of a std::vector<uint8_t> (fixture helper).
size_t rmw_cerulion_vector_u8_size(const void *v) noexcept {
  return static_cast<const std::vector<uint8_t> *>(v)->size();
}

/// capacity() of a std::vector<uint8_t> (fixture helper).
size_t rmw_cerulion_vector_u8_capacity(const void *v) noexcept {
  return static_cast<const std::vector<uint8_t> *>(v)->capacity();
}

/// data() of a std::vector<uint8_t> (fixture helper; borrow valid until
/// the vector is mutated or destroyed).
const uint8_t *rmw_cerulion_vector_u8_data(const void *v) noexcept {
  return static_cast<const std::vector<uint8_t> *>(v)->data();
}

/// Adopt-take memory safety: release the BUFFER of a
/// `std::vector<T>` of primitives through the pair it was allocated with.
/// `std::allocator<T>::deallocate` is `::operator delete` on memory from
/// `::operator new` — NOT libc `free` — so the adopting take's reuse
/// pre-pass (which must drop a caller's existing vector storage before it
/// forges the triplet over it) calls this instead of `free(begin)`. The
/// element type does not matter here: `T` is a primitive, so there is no
/// destructor to run and no over-alignment to honour; the buffer is all
/// there is. Both cases the pre-pass meets route correctly: a genuine
/// vector buffer takes the C++ pair, and a previously-FORGED `begin` (an
/// SHM address registered with the preloaded hook) still reaches the
/// hook's interposed `free` — libstdc++ and libc++ both implement
/// `operator delete(void*)` as a call to `free` — and is released, exactly
/// as the app's own `~vector` would release it. Never called with a null
/// `begin` (the caller checks). Fixture-visible, production-reachable.
void rmw_cerulion_vector_pod_release(void *begin) noexcept {
  ::operator delete(begin);
}


// The three introspection accessors that can THROW: on Lyrical and Rolling a
// rosidl::Buffer member's get/get_const/resize call throw_if_not_cpu_backend()
// first, and a C++ exception crossing the accessor pointer into Rust would be
// a foreign unwind (undefined behaviour). Each wrapper calls the accessor
// inside try/catch and returns 0 on success, 1 when it threw; the Rust side
// treats 1 as a refused frame. size_function is documented as non-throwing
// on every backend and is called directly.
int rmw_cerulion_member_get_const(const void *(*f)(const void *, size_t), const void *m, size_t index,
                                  const void **out) noexcept {
  try {
    *out = f(m, index);
    return 0;
  } catch (...) {
    return 1;
  }
}
int rmw_cerulion_member_get(void *(*f)(void *, size_t), void *m, size_t index, void **out) noexcept {
  try {
    *out = f(m, index);
    return 0;
  } catch (...) {
    return 1;
  }
}
int rmw_cerulion_member_resize(void (*f)(void *, size_t), void *m, size_t size) noexcept {
  try {
    f(m, size);
    return 0;
  } catch (...) {
    return 1;
  }
}
// fetch and assign (the vector<bool> element copies) are wrapped as well, so
// that no introspection accessor pointer is ever called directly from Rust.
int rmw_cerulion_member_fetch(void (*f)(const void *, size_t, void *), const void *m, size_t index,
                              void *out) noexcept {
  try {
    f(m, index, out);
    return 0;
  } catch (...) {
    return 1;
  }
}
int rmw_cerulion_member_assign(void (*f)(void *, size_t, const void *), void *m, size_t index,
                               const void *value) noexcept {
  try {
    f(m, index, value);
    return 0;
  } catch (...) {
    return 1;
  }
}
// Test fixture for the Rust unit test of the wrappers: an accessor that
// always throws, so the catch is proven rather than assumed.
const void *rmw_cerulion_throwing_get_const(const void *, size_t) { throw 1; }

}  // extern "C"

// The hand-mirrored C++ MessageMember (src/ffi/introspection_cpp.rs) is pinned
// against the C++ header itself wherever that header is on the include path
// (the distro jobs): 120 bytes with is_rosidl_buffer_ at 112 on Lyrical and
// Rolling, 112 bytes before. The Rust pins check the mirror against its C
// twin; this one checks it against the C++ truth.
#if __has_include(<rosidl_typesupport_introspection_cpp/message_introspection.hpp>)
#include <rosidl_typesupport_introspection_cpp/message_introspection.hpp>
#if defined(RMW_CERULION_HAS_IS_ROSIDL_BUFFER)
static_assert(sizeof(rosidl_typesupport_introspection_cpp::MessageMember) == 120,
              "C++ MessageMember is 120 bytes on Lyrical and Rolling");
static_assert(offsetof(rosidl_typesupport_introspection_cpp::MessageMember, is_rosidl_buffer_) == 112,
              "is_rosidl_buffer_ sits at 112");
#endif
// Era-independent twin of the asserts above, read by a Rust test: the C++
// header's own sizeof, or 0 where the header is not on the include path
// (a vendored build), so the Rust mirror is compared with the C++ truth on
// every real-header build whatever the era.
extern "C" size_t rmw_cerulion_cpp_message_member_sizeof() noexcept {
  return sizeof(rosidl_typesupport_introspection_cpp::MessageMember);
}
#else
extern "C" size_t rmw_cerulion_cpp_message_member_sizeof() noexcept { return 0; }
#endif
