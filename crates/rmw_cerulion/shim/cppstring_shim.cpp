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

}  // extern "C"
