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


// Every introspection accessor the bridge calls, except size_function, goes
// through one of these wrappers: a C++ exception crossing the accessor pointer into Rust would be
// a foreign unwind (undefined behaviour). Each wrapper calls the accessor
// inside try/catch and returns 0 on success, 1 when it threw; the Rust side
// treats 1 as a refused frame. A rosidl::Buffer member (Lyrical and Rolling),
// whose generated get/get_const/resize throw on a non-CPU backend, is never
// read or written through them: the rmw_cerulion_buffer_u8_* functions below
// handle it. size_function of a non-Buffer member is called directly; a
// Buffer member's size is read through rmw_cerulion_buffer_u8_size, which
// catches a throwing backend.
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

// ---------------------------------------------------------------------
// rosidl::Buffer<uint8_t> (Lyrical and Rolling). On these eras an unbounded
// `uint8[]` member of a C++ message is a `rosidl::Buffer<uint8_t>`, and a
// buffer-backed `uint8[]` sequence of a C message points at one. A Buffer
// whose backend is not "cpu" (a CUDA buffer from Isaac ROS 5.0, for
// example) throws from every element accessor, so the bridge reads and
// writes Buffers only through the functions below. A non-CPU Buffer is
// copied to host memory with `to_vector()` (the downgrade); a take leaves a
// Buffer it fills on the CPU backend holding the frame. build.rs puts the distro's own
// `rosidl_buffer` headers on the include path, or on a vendored-bindings
// build the copy in `shim/vendored/` taken from the same rosidl commit as
// the vendored bindings.
// ---------------------------------------------------------------------
#if defined(RMW_CERULION_HAS_IS_ROSIDL_BUFFER)
#include <cstring>
#include <memory>
#include <rosidl_buffer/buffer.hpp>

namespace {
using U8Buffer = rosidl::Buffer<uint8_t>;
using U8CpuImpl = rosidl::CpuBufferImpl<uint8_t>;

// The CPU storage of a Buffer on the "cpu" backend, or null for any other
// backend. The storage comes from the Buffer's own conversion operator, which
// reads the CPU implementation pointer the Buffer cached when that
// implementation was set (in whichever library built the Buffer), so no
// type identity is compared across shared objects here. A Buffer that names
// "cpu" but has no CPU implementation throws from the operator, and is then
// treated like any other backend.
std::vector<uint8_t> *cpu_storage_of(U8Buffer *b, const std::string &backend) {
  if (backend != "cpu") {
    return nullptr;
  }
  try {
    return &static_cast<std::vector<uint8_t> &>(*b);
  } catch (const std::runtime_error &) {
    return nullptr;
  }
}

// Copy `name` into `out` (capacity `cap`, always NUL terminated when cap > 0).
void copy_backend_name(const std::string &name, char *out, size_t cap) noexcept {
  if (out == nullptr || cap == 0) {
    return;
  }
  size_t n = name.size() < cap - 1 ? name.size() : cap - 1;
  std::memcpy(out, name.data(), n);
  out[n] = '\0';
}
}  // namespace

extern "C" {

/// sizeof(rosidl::Buffer<uint8_t>) on this platform (16: the impl pointer
/// and the cached CPU impl pointer).
size_t rmw_cerulion_buffer_u8_sizeof() noexcept { return sizeof(U8Buffer); }

/// Element count of a Buffer on any backend (`BufferImplBase::size` is
/// required of every backend). 0 on success, 2 when the backend threw.
int rmw_cerulion_buffer_u8_size(const void *buf, size_t *out) noexcept {
  try {
    *out = static_cast<const U8Buffer *>(buf)->size();
    return 0;
  } catch (...) {
    return 2;
  }
}

/// Read a Buffer's bytes.
///   0: CPU backend, or an empty Buffer on any backend. `*data`/`*len`
///      view the Buffer's own storage (or nothing), valid until the Buffer
///      is mutated or destroyed; `*owned` is null.
///   1: another backend, downgraded. `to_vector()` copied the bytes to host
///      memory into a heap `std::vector<uint8_t>` returned in `*owned`
///      (release it with rmw_cerulion_buffer_u8_copy_free); `*data`/`*len`
///      view that copy.
///   2: the backend threw (the name, a size, or the host copy); nothing is
///      owned and nothing is viewed.
/// The backend's name is written to `backend` (truncated to `backend_cap`
/// with a NUL) whenever it could be read.
int rmw_cerulion_buffer_u8_read(const void *buf, const uint8_t **data, size_t *len, void **owned,
                                char *backend, size_t backend_cap) noexcept {
  *data = nullptr;
  *len = 0;
  *owned = nullptr;
  if (backend != nullptr && backend_cap > 0) {
    backend[0] = '\0';
  }
  auto *b = const_cast<U8Buffer *>(static_cast<const U8Buffer *>(buf));
  try {
    const std::string name = b->get_backend_type();
    copy_backend_name(name, backend, backend_cap);
    if (const std::vector<uint8_t> *storage = cpu_storage_of(b, name)) {
      *data = storage->data();
      *len = storage->size();
      return 0;
    }
    if (b->size() == 0) {
      // Nothing to copy: an empty Buffer reads as empty on any backend.
      return 0;
    }
    std::unique_ptr<std::vector<uint8_t>> copy(new std::vector<uint8_t>(b->to_vector()));
    *data = copy->data();
    *len = copy->size();
    *owned = copy.release();
    return 1;
  } catch (...) {
    *data = nullptr;
    *len = 0;
    return 2;
  }
}

/// Release the host copy rmw_cerulion_buffer_u8_read returned in `*owned`.
/// Null is a no-op.
void rmw_cerulion_buffer_u8_copy_free(void *owned) noexcept {
  delete static_cast<std::vector<uint8_t> *>(owned);
}

/// Make `buf` a CPU Buffer holding exactly [data, data + len).
///   0: it was already a CPU Buffer and its storage was assigned in place.
///   1: it held another backend; it now holds a new CPU implementation and
///      the previous implementation was destroyed by its own destructor.
///   2: an allocation threw; the Buffer is unchanged.
int rmw_cerulion_buffer_u8_assign(void *buf, const uint8_t *data, size_t len) noexcept {
  auto *b = static_cast<U8Buffer *>(buf);
  try {
    if (std::vector<uint8_t> *storage = cpu_storage_of(b, b->get_backend_type())) {
      storage->assign(data, data + len);
      return 0;
    }
    U8Buffer fresh(std::vector<uint8_t>(data, data + len));
    *b = std::move(fresh);
    return 1;
  } catch (...) {
    return 2;
  }
}

}  // extern "C"

// Test fixtures: a Buffer on a backend that is not "cpu", built the way an
// accelerator backend builds one (its own BufferImplBase subclass), and the
// four introspection accessors exactly as rosidl_typesupport_introspection_cpp
// generates them for a Buffer member on Lyrical (they throw on a non-CPU
// backend, as the generated ones do). Harmless to ship, like the string
// fixtures above.
namespace {
class FixtureDeviceImpl : public rosidl::BufferImplBase<uint8_t> {
 public:
  FixtureDeviceImpl(std::vector<uint8_t> bytes, std::string name, bool throw_on_copy,
                    size_t reported_size = SIZE_MAX)
      : bytes_(std::move(bytes)),
        name_(std::move(name)),
        throw_on_copy_(throw_on_copy),
        reported_size_(reported_size) {}
  std::string get_backend_type() const override { return name_; }
  size_t size() const override { return reported_size_ == SIZE_MAX ? bytes_.size() : reported_size_; }
  std::unique_ptr<rosidl::BufferImplBase<uint8_t>> to_cpu() const override {
    if (throw_on_copy_) {
      throw std::runtime_error("fixture device copy failed");
    }
    auto cpu = std::make_unique<U8CpuImpl>();
    cpu->get_storage() = bytes_;
    return cpu;
  }
  std::unique_ptr<rosidl::BufferImplBase<uint8_t>> clone() const override {
    return std::make_unique<FixtureDeviceImpl>(bytes_, name_, throw_on_copy_, reported_size_);
  }

 private:
  std::vector<uint8_t> bytes_;
  std::string name_;
  bool throw_on_copy_;
  size_t reported_size_;
};
}  // namespace

extern "C" {

/// Placement-construct a Buffer at `at` (rmw_cerulion_buffer_u8_sizeof()
/// bytes, pointer aligned). A null `backend` builds a CPU Buffer holding the
/// bytes; any other name builds a fixture device Buffer reporting that name,
/// whose host copy throws when `throw_on_copy` is nonzero.
void rmw_cerulion_fixture_buffer_u8_construct(void *at, const uint8_t *data, size_t len,
                                              const char *backend, int throw_on_copy) noexcept {
  std::vector<uint8_t> bytes(data, data + len);
  if (backend == nullptr) {
    new (at) U8Buffer(std::move(bytes));
  } else {
    new (at) U8Buffer(std::unique_ptr<rosidl::BufferImplBase<uint8_t>>(
        new FixtureDeviceImpl(std::move(bytes), backend, throw_on_copy != 0)));
  }
}

/// Destruct a placement-constructed Buffer.
void rmw_cerulion_fixture_buffer_u8_destruct(void *b) noexcept { static_cast<U8Buffer *>(b)->~U8Buffer(); }

/// Heap twins of the two above, for a C sequence's `data` pointer
/// (`rosidl_buffer_uint8_destroy` is `delete` on such a pointer).
void *rmw_cerulion_fixture_buffer_u8_new(const uint8_t *data, size_t len, const char *backend,
                                         int throw_on_copy) noexcept {
  void *at = ::operator new(sizeof(U8Buffer));
  rmw_cerulion_fixture_buffer_u8_construct(at, data, len, backend, throw_on_copy);
  return at;
}
void rmw_cerulion_fixture_buffer_u8_delete(void *b) noexcept { delete static_cast<U8Buffer *>(b); }

/// A heap fixture device Buffer whose size() reports `reported_size` while
/// its host copy holds the `len` bytes: a backend that disagrees with itself.
void *rmw_cerulion_fixture_buffer_u8_new_misreporting(const uint8_t *data, size_t len,
                                                      size_t reported_size) noexcept {
  return new U8Buffer(std::unique_ptr<rosidl::BufferImplBase<uint8_t>>(new FixtureDeviceImpl(
      std::vector<uint8_t>(data, data + len), "fixture_device", false, reported_size)));
}

/// The Buffer's backend name, copied like rmw_cerulion_buffer_u8_read does.
void rmw_cerulion_fixture_buffer_u8_backend(const void *b, char *out, size_t cap) noexcept {
  copy_backend_name(static_cast<const U8Buffer *>(b)->get_backend_type(), out, cap);
}

// The generated accessors (msg__type_support.cpp.em, the buffer_field_names
// branch). Not noexcept: they throw exactly where the generated ones do.
size_t rmw_cerulion_fixture_buffer_u8_size_function(const void *m) {
  return static_cast<const U8Buffer *>(m)->size();
}
const void *rmw_cerulion_fixture_buffer_u8_get_const_function(const void *m, size_t index) {
  const auto &member = *static_cast<const U8Buffer *>(m);
  member.throw_if_not_cpu_backend();
  return &member[index];
}
void *rmw_cerulion_fixture_buffer_u8_get_function(void *m, size_t index) {
  auto &member = *static_cast<U8Buffer *>(m);
  member.throw_if_not_cpu_backend();
  return &member[index];
}
void rmw_cerulion_fixture_buffer_u8_resize_function(void *m, size_t size) {
  auto *member = static_cast<U8Buffer *>(m);
  member->throw_if_not_cpu_backend();
  member->resize(size);
}

}  // extern "C"
#endif  // RMW_CERULION_HAS_IS_ROSIDL_BUFFER

// The hand-mirrored C++ MessageMember (src/ffi/introspection_cpp.rs) is pinned
// against the C++ header itself on every real-header build: build.rs looks for
// this header under every include root it knows and hands this translation unit
// the directory that resolves it, so the include below works on the flat
// pre-Galactic layout, on the per-package one every later distro installs, and
// on a colcon isolated install, where each C++ package has a prefix of its own.
// Only a build with no ROS headers at all takes the #else, and a generated build
// that cannot find this header fails in build.rs rather than compiling the
// checks away. All three member shapes are
// pinned, keyed on the same defines
// build.rs derives from the bindings: 120 bytes with is_rosidl_buffer_ at 112
// on Lyrical and Rolling, 112 bytes from Humble to Kilted, and 96 bytes with
// resize_function last on Foxy and Galactic, which have no fetch or assign.
// The Rust pins check the mirror against its C twin; this one checks it
// against the C++ truth.
#if __has_include(<rosidl_typesupport_introspection_cpp/message_introspection.hpp>)
#include <rosidl_typesupport_introspection_cpp/message_introspection.hpp>
#if defined(RMW_CERULION_HAS_IS_ROSIDL_BUFFER)
static_assert(sizeof(rosidl_typesupport_introspection_cpp::MessageMember) == 120,
              "C++ MessageMember is 120 bytes on Lyrical and Rolling");
static_assert(offsetof(rosidl_typesupport_introspection_cpp::MessageMember, is_rosidl_buffer_) == 112,
              "is_rosidl_buffer_ sits at 112");
#elif defined(RMW_CERULION_HAS_FETCH_FUNCTION)
static_assert(sizeof(rosidl_typesupport_introspection_cpp::MessageMember) == 112,
              "C++ MessageMember is 112 bytes from Humble through Kilted");
static_assert(offsetof(rosidl_typesupport_introspection_cpp::MessageMember, fetch_function) == 88,
              "fetch_function sits at 88");
#else
static_assert(sizeof(rosidl_typesupport_introspection_cpp::MessageMember) == 96,
              "C++ MessageMember is 96 bytes on Foxy and Galactic");
static_assert(offsetof(rosidl_typesupport_introspection_cpp::MessageMember, resize_function) == 88,
              "resize_function is the last field at 88 before Humble");
#endif
// Era-independent twin of the asserts above, read by a Rust test: the C++
// header's own sizeof, or 0 where no ROS headers were found at all
// (a vendored build), so the Rust mirror is compared with the C++ truth on
// every real-header build whatever the era.
extern "C" size_t rmw_cerulion_cpp_message_member_sizeof() noexcept {
  return sizeof(rosidl_typesupport_introspection_cpp::MessageMember);
}
#else
extern "C" size_t rmw_cerulion_cpp_message_member_sizeof() noexcept { return 0; }
#endif
