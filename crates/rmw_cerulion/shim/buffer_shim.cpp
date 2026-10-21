// Buffer backend host for rmw_cerulion (ROS 2 Lyrical and Rolling).
//
// rmw_cerulion loads the installed rosidl::Buffer backend plugins (NVIDIA's
// `cuda` backend among them) through rosidl_buffer_backend_registry, and builds
// the serialization context the way rmw_zenoh_cpp and rmw_fastrtps_cpp build it
// (their buffer_backend_loader.cpp): one backend instance per declared plugin,
// descriptor ops bound to the instance, and FastCDR descriptor serializers taken
// from the backend's descriptor type support. Messages that carry Buffer fields
// are then serialized with the message's own generated
// rosidl_typesupport_fastrtps_cpp `*_with_endpoint` callbacks, so the bytes are
// the CDR those two rmws put on their wire.
//
// The backends live for the rest of the process: a Buffer the application still
// holds keeps a deleter inside the plugin library, so the libraries are never
// unloaded.
//
// Every extern "C" function is noexcept and catches everything: an exception
// never unwinds into Rust. The reason for a failure is kept per thread and read
// with rmw_cerulion_bb_last_error.
//
// Compiled only when build.rs found the registry, the backend interface, the
// fastrtps typesupport buffer serialization and fastcdr headers (cfg
// cerulion_has_buffer_backends).
#include <atomic>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <memory>
#include <mutex>
#include <shared_mutex>
#include <set>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include <fastcdr/Cdr.h>
#include <fastcdr/FastBuffer.h>
#include <fastcdr/exceptions/NotEnoughMemoryException.h>
#include <rmw/topic_endpoint_info.h>
#include <rosidl_buffer/buffer.hpp>
#include <rosidl_buffer_backend/buffer_backend.hpp>
#include <rosidl_buffer_backend_registry/buffer_backend_registry.hpp>
#include <rosidl_runtime_c/message_type_support_struct.h>
#include <rosidl_typesupport_fastrtps_cpp/buffer_serialization.hpp>
#include <rosidl_typesupport_fastrtps_cpp/message_type_support.h>

namespace {

using SerializationContext = rosidl_typesupport_fastrtps_cpp::BufferSerializationContext;
using Callbacks = message_type_support_callbacks_t;
using U8Buffer = rosidl::Buffer<uint8_t>;

constexpr const char *kFastrtpsCppIdentifier = "rosidl_typesupport_fastrtps_cpp";

// Counter slots read by rmw_cerulion_bb_counter (src/buffer_backends.rs names them).
enum Counter : int {
  kDescriptorsCreated = 0,
  kDescriptorsRefused = 1,
  kImports = 2,
  kEmptyImports = 3,
  kCounterCount = 4,
};

std::atomic<uint64_t> g_counters[kCounterCount];

thread_local std::string t_last_error;
// Set by the wrapped from_descriptor op when a backend hands back an empty
// buffer, read by rmw_cerulion_bb_deserialize after the message is decoded.
thread_local bool t_empty_import = false;
// Descriptor creations this thread asked a backend for (created or refused):
// a serialization that asks none held no Buffer on a backend other than cpu.
thread_local uint64_t t_descriptor_calls = 0;
// This thread's share of the created / refused counters, so a serialization
// retried with a larger buffer takes back the counts its first attempt made.
thread_local uint64_t t_created = 0;
thread_local uint64_t t_refused = 0;

void count_descriptor(bool created) {
  g_counters[created ? kDescriptorsCreated : kDescriptorsRefused].fetch_add(
      1, std::memory_order_relaxed);
  ++(created ? t_created : t_refused);
  ++t_descriptor_calls;
}

struct Context {
  std::unique_ptr<rosidl_buffer_backend_registry::BufferBackendRegistry> registry;
  std::unordered_map<std::string, std::shared_ptr<rosidl::BufferBackend>> backends;
  SerializationContext ser;
  std::string names_csv;
};

// Never destroyed (see the file comment). Readers (serialization, endpoint
// hooks) hold g_ctx_mutex shared for the whole call; loading the plugins and
// installing the test fixture backend hold it exclusive.
Context *g_ctx = nullptr;
std::shared_mutex g_ctx_mutex;
int g_init_result = -2;  // -2: not tried yet.

const rosidl_message_type_support_t *resolve(const rosidl_message_type_support_t *ts,
                                             const char *identifier) {
  const rosidl_message_type_support_t *current = ts;
  for (int i = 0; i < 4 && current != nullptr; ++i) {
    if (current->typesupport_identifier != nullptr &&
        std::strcmp(current->typesupport_identifier, identifier) == 0) {
      return current;
    }
    if (current->func == nullptr) {
      return nullptr;
    }
    current = current->func(current, identifier);
  }
  return nullptr;
}

void rebuild_names(Context &ctx) {
  ctx.names_csv.clear();
  for (const auto &entry : ctx.backends) {
    if (entry.first == "cpu") {
      continue;
    }
    if (!ctx.names_csv.empty()) {
      ctx.names_csv += ",";
    }
    ctx.names_csv += entry.first;
  }
}

// Bind one backend instance into the context: its ops (counted) and its FastCDR
// descriptor serializers. Returns false with t_last_error set when the backend's
// descriptor type cannot be serialized.
bool bind_backend(Context &ctx, const std::shared_ptr<rosidl::BufferBackend> &backend) {
  const std::string type = backend->get_backend_type();
  const rosidl_message_type_support_t *descriptor_ts = backend->get_descriptor_type_support();
  const rosidl_message_type_support_t *fastrtps_ts =
      descriptor_ts != nullptr ? resolve(descriptor_ts, kFastrtpsCppIdentifier) : nullptr;
  if (fastrtps_ts == nullptr || fastrtps_ts->data == nullptr) {
    t_last_error = "backend '" + type +
                   "' has no rosidl_typesupport_fastrtps_cpp descriptor type support";
    return false;
  }
  const auto *callbacks = static_cast<const Callbacks *>(fastrtps_ts->data);
  std::weak_ptr<rosidl::BufferBackend> weak = backend;

  rosidl::BufferDescriptorOps ops;
  ops.create_descriptor_with_endpoint = [weak](const void *impl,
                                               const rmw_topic_endpoint_info_t &ep) {
    auto strong = weak.lock();
    if (!strong) {
      throw std::runtime_error("buffer backend destroyed");
    }
    std::shared_ptr<void> descriptor = strong->create_descriptor_with_endpoint(impl, ep);
    count_descriptor(descriptor != nullptr);
    return descriptor;
  };
  ops.from_descriptor_with_endpoint = [weak](const void *descriptor,
                                             const rmw_topic_endpoint_info_t &ep) {
    auto strong = weak.lock();
    if (!strong) {
      throw std::runtime_error("buffer backend destroyed");
    }
    auto impl = strong->from_descriptor_with_endpoint(descriptor, ep);
    g_counters[kImports].fetch_add(1, std::memory_order_relaxed);
    const auto *base = static_cast<const rosidl::BufferImplBase<uint8_t> *>(impl.get());
    if (base == nullptr || base->size() == 0) {
      g_counters[kEmptyImports].fetch_add(1, std::memory_order_relaxed);
      t_empty_import = true;
    }
    return impl;
  };
  ctx.ser.descriptor_ops[type] = ops;

  rosidl_typesupport_fastrtps_cpp::BufferDescriptorSerializers serializers;
  serializers.serialize = [callbacks](eprosima::fastcdr::Cdr &cdr,
                                      const std::shared_ptr<void> &descriptor,
                                      const rmw_topic_endpoint_info_t &ep,
                                      const SerializationContext &c) {
    if (!descriptor) {
      throw std::runtime_error("descriptor pointer is null");
    }
    if (callbacks->cdr_serialize_with_endpoint != nullptr) {
      callbacks->cdr_serialize_with_endpoint(descriptor.get(), cdr, ep, c);
    } else {
      callbacks->cdr_serialize(descriptor.get(), cdr);
    }
  };
  serializers.deserialize = [callbacks, weak](eprosima::fastcdr::Cdr &cdr,
                                              const rmw_topic_endpoint_info_t &ep,
                                              const SerializationContext &c) {
    auto strong = weak.lock();
    if (!strong) {
      throw std::runtime_error("buffer backend destroyed");
    }
    std::shared_ptr<void> descriptor = strong->create_empty_descriptor();
    if (!descriptor) {
      throw std::runtime_error("backend returned a null descriptor instance");
    }
    if (callbacks->cdr_deserialize_with_endpoint != nullptr) {
      callbacks->cdr_deserialize_with_endpoint(cdr, descriptor.get(), ep, c);
    } else {
      callbacks->cdr_deserialize(cdr, descriptor.get());
    }
    return descriptor;
  };
  ctx.ser.descriptor_serializers[type] = std::move(serializers);
  ctx.backends[type] = backend;
  return true;
}

rmw_topic_endpoint_info_t endpoint(const uint8_t *gid, rmw_endpoint_type_t type,
                                   const char *topic_type) {
  rmw_topic_endpoint_info_t ep;
  std::memset(&ep, 0, sizeof(ep));
  ep.endpoint_type = type;
  ep.topic_type = topic_type;
  if (gid != nullptr) {
    std::memcpy(ep.endpoint_gid, gid, RMW_GID_STORAGE_SIZE);
  }
  return ep;
}

std::vector<std::string> split_csv(const char *csv) {
  std::vector<std::string> out;
  if (csv == nullptr) {
    return out;
  }
  std::string cur;
  for (const char *p = csv;; ++p) {
    if (*p == ',' || *p == '\0') {
      if (!cur.empty()) {
        out.push_back(cur);
      }
      cur.clear();
      if (*p == '\0') {
        break;
      }
    } else {
      cur.push_back(*p);
    }
  }
  return out;
}

}  // namespace

extern "C" {

/// Load the installed backend plugins, once per process. Returns the number of
/// backends other than `cpu`, or -1 when the registry failed (the reason is in
/// rmw_cerulion_bb_last_error). A backend whose descriptor cannot be serialized
/// is skipped; its reason is the last error even on success.
int rmw_cerulion_bb_init() noexcept {
  std::unique_lock<std::shared_mutex> lock(g_ctx_mutex);
  if (g_init_result != -2) {
    return g_init_result;
  }
  try {
    auto *ctx = new Context();
    ctx->registry = std::make_unique<rosidl_buffer_backend_registry::BufferBackendRegistry>();
    for (const std::string &name : ctx->registry->get_backend_names()) {
      std::shared_ptr<rosidl::BufferBackend> backend = ctx->registry->create_backend_instance(name);
      if (!backend) {
        t_last_error = "backend plugin '" + name + "' could not be instantiated";
        continue;
      }
      (void)bind_backend(*ctx, backend);
    }
    rebuild_names(*ctx);
    g_ctx = ctx;
    int n = 0;
    for (const auto &entry : ctx->backends) {
      n += entry.first == "cpu" ? 0 : 1;
    }
    g_init_result = n;
  } catch (const std::exception &e) {
    t_last_error = std::string("buffer backend registry failed: ") + e.what();
    g_init_result = -1;
  } catch (...) {
    t_last_error = "buffer backend registry failed";
    g_init_result = -1;
  }
  return g_init_result;
}

/// Copy the comma-separated types of the loaded backends other than `cpu` into
/// [out, out + cap) with a NUL ("" when none, or before rmw_cerulion_bb_init).
/// Returns the length without the NUL; a result >= `cap` means the copy was
/// cut short.
size_t rmw_cerulion_bb_backend_names(char *out, size_t cap) noexcept {
  try {
    std::shared_lock<std::shared_mutex> lock(g_ctx_mutex);
    const std::string names = g_ctx != nullptr ? g_ctx->names_csv : std::string();
    if (cap > 0) {
      const size_t n = names.size() < cap - 1 ? names.size() : cap - 1;
      std::memcpy(out, names.data(), n);
      out[n] = '\0';
    }
    return names.size();
  } catch (...) {
    if (cap > 0) {
      out[0] = '\0';
    }
    return 0;
  }
}

/// 1 when the loaded backend `type` advertises empty metadata, 0 when it
/// advertises some, -1 when no such backend is loaded or it threw.
int rmw_cerulion_bb_metadata_is_empty(const char *type) noexcept {
  try {
    std::shared_lock<std::shared_mutex> lock(g_ctx_mutex);
    if (g_ctx == nullptr) {
      return -1;
    }
    auto it = g_ctx->backends.find(type);
    if (it == g_ctx->backends.end() || !it->second) {
      return -1;
    }
    return it->second->get_backend_metadata().empty() ? 1 : 0;
  } catch (...) {
    return -1;
  }
}

/// The reason for this thread's last failure.
const char *rmw_cerulion_bb_last_error() noexcept { return t_last_error.c_str(); }

/// Descriptor creations this thread has asked a backend for, created or refused.
uint64_t rmw_cerulion_bb_thread_descriptor_calls() noexcept { return t_descriptor_calls; }

/// The process counter in slot `which` (see Counter), 0 for an unknown slot.
uint64_t rmw_cerulion_bb_counter(int which) noexcept {
  if (which < 0 || which >= kCounterCount) {
    return 0;
  }
  return g_counters[which].load(std::memory_order_relaxed);
}

/// 1 when these fastrtps_cpp callbacks describe a message with Buffer fields
/// and carry both endpoint-aware callbacks, else 0.
int rmw_cerulion_bb_has_buffer_fields(const void *callbacks) noexcept {
  const auto *cb = static_cast<const Callbacks *>(callbacks);
  return cb != nullptr && cb->has_buffer_fields && cb->cdr_serialize_with_endpoint != nullptr &&
                 cb->cdr_deserialize_with_endpoint != nullptr && cb->cdr_serialize != nullptr
             ? 1
             : 0;
}

/// Tell every loaded backend a local endpoint exists. 0 on success, 2 when a
/// backend threw.
int rmw_cerulion_bb_on_creating(const uint8_t *gid, int is_publisher,
                                const char *topic_type) noexcept {
  try {
    std::unordered_map<std::string, std::shared_ptr<rosidl::BufferBackend>> backends;
    {
      std::shared_lock<std::shared_mutex> lock(g_ctx_mutex);
      if (g_ctx == nullptr) {
        return 0;
      }
      backends = g_ctx->backends;
    }
    const rmw_topic_endpoint_info_t ep = endpoint(
        gid, is_publisher ? RMW_ENDPOINT_PUBLISHER : RMW_ENDPOINT_SUBSCRIPTION, topic_type);
    for (const auto &entry : backends) {
      if (entry.second) {
        entry.second->on_creating_endpoint(ep);
      }
    }
    return 0;
  } catch (const std::exception &e) {
    t_last_error = std::string("on_creating_endpoint threw: ") + e.what();
    return 2;
  } catch (...) {
    t_last_error = "on_creating_endpoint threw";
    return 2;
  }
}

/// A publisher discovered a subscription that accepts the backends in
/// `accepted_csv` (each advertising empty metadata). Every loaded backend is
/// told, and the result is 1 when at least one backend other than `cpu` that the
/// subscription accepts reports it compatible, 0 when none does, 2 when a
/// backend threw.
int rmw_cerulion_bb_on_discovering(const uint8_t *gid, const char *topic_type,
                                   const char *accepted_csv) noexcept {
  try {
    std::unordered_map<std::string, std::shared_ptr<rosidl::BufferBackend>> backends;
    {
      std::shared_lock<std::shared_mutex> lock(g_ctx_mutex);
      if (g_ctx == nullptr) {
        return 0;
      }
      backends = g_ctx->backends;
    }
    std::unordered_map<std::string, std::string> supported;
    for (const std::string &name : split_csv(accepted_csv)) {
      supported[name] = "";
    }
    supported["cpu"] = "";
    const rmw_topic_endpoint_info_t ep = endpoint(gid, RMW_ENDPOINT_SUBSCRIPTION, topic_type);
    const std::vector<rmw_topic_endpoint_info_t> existing;
    int accelerated = 0;
    for (const auto &entry : backends) {
      if (!entry.second) {
        continue;
      }
      const auto verdict = entry.second->on_discovering_endpoint(ep, existing, supported);
      if (verdict.first && entry.first != "cpu" && supported.count(entry.first) != 0) {
        accelerated = 1;
      }
    }
    return accelerated;
  } catch (const std::exception &e) {
    t_last_error = std::string("on_discovering_endpoint threw: ") + e.what();
    return 2;
  } catch (...) {
    t_last_error = "on_discovering_endpoint threw";
    return 2;
  }
}

/// Serialize `msg` (a C++ message of the type `callbacks` describe) as CDR
/// (encapsulation CDR_LE, then the XCDRv1 body) into [out, out + cap).
/// With a `gid`, through cdr_serialize_with_endpoint for that subscription, so a
/// backend may write a descriptor in place of a Buffer's bytes; without one,
/// through the plain cdr_serialize, which writes every Buffer as bytes.
/// Bytes the encoder skips for alignment are left as the caller had them.
///   0: done, *len is the CDR length.
///   1: `cap` is too small; *len is the type support's size estimate.
///   2: serialization threw; the reason is the last error.
int rmw_cerulion_bb_serialize(const void *callbacks, const void *msg, const uint8_t *gid,
                              const char *topic_type, uint8_t *out, size_t cap,
                              size_t *len) noexcept {
  const auto *cb = static_cast<const Callbacks *>(callbacks);
  *len = 0;
  const uint64_t created = t_created;
  const uint64_t refused = t_refused;
  const uint64_t calls = t_descriptor_calls;
  try {
    std::shared_lock<std::shared_mutex> lock(g_ctx_mutex);
    eprosima::fastcdr::FastBuffer buffer(reinterpret_cast<char *>(out), cap);
    eprosima::fastcdr::Cdr cdr(buffer, eprosima::fastcdr::Cdr::DEFAULT_ENDIAN,
                               eprosima::fastcdr::CdrVersion::XCDRv1);
    cdr.serialize_encapsulation();
    bool ok = false;
    if (gid != nullptr) {
      const SerializationContext *ser = g_ctx != nullptr ? &g_ctx->ser : nullptr;
      if (ser == nullptr) {
        t_last_error = "buffer backends are not loaded";
        return 2;
      }
      const rmw_topic_endpoint_info_t ep = endpoint(gid, RMW_ENDPOINT_SUBSCRIPTION, topic_type);
      ok = cb->cdr_serialize_with_endpoint(msg, cdr, ep, *ser);
    } else {
      ok = cb->cdr_serialize(msg, cdr);
    }
    if (!ok) {
      t_last_error = "the type support's serializer reported failure";
      return 2;
    }
    *len = cdr.get_serialized_data_length();
    return 0;
  } catch (const eprosima::fastcdr::exception::NotEnoughMemoryException &) {
    // The caller retries with a larger buffer and the backend is asked again:
    // this attempt's descriptor counts are taken back.
    g_counters[kDescriptorsCreated].fetch_sub(t_created - created, std::memory_order_relaxed);
    g_counters[kDescriptorsRefused].fetch_sub(t_refused - refused, std::memory_order_relaxed);
    t_created = created;
    t_refused = refused;
    t_descriptor_calls = calls;
    try {
      *len = static_cast<size_t>(cb->get_serialized_size(msg)) + 4;
    } catch (...) {
      *len = cap * 2;
    }
    if (*len <= cap) {
      *len = cap * 2;
    }
    return 1;
  } catch (const std::exception &e) {
    t_last_error = std::string("serialization threw: ") + e.what();
    return 2;
  } catch (...) {
    t_last_error = "serialization threw";
    return 2;
  }
}

/// Decode CDR written by rmw_cerulion_bb_serialize (or by rmw_fastrtps_cpp or
/// rmw_zenoh_cpp) into `msg`, an initialized C++ message of the type `callbacks`
/// describe, through cdr_deserialize_with_endpoint with the publisher's `gid`. A
/// descriptor is turned back into a Buffer by its backend.
///   0: done.
///   2: decoding threw or failed; the reason is the last error.
///   3: decoded, but a backend returned an empty Buffer for a descriptor (a
///      stale or unimportable one): the message must not be delivered.
int rmw_cerulion_bb_deserialize(const void *callbacks, const uint8_t *cdr_bytes, size_t len,
                                void *msg, const uint8_t *gid, const char *topic_type) noexcept {
  const auto *cb = static_cast<const Callbacks *>(callbacks);
  t_empty_import = false;
  try {
    std::shared_lock<std::shared_mutex> lock(g_ctx_mutex);
    const SerializationContext *ser = g_ctx != nullptr ? &g_ctx->ser : nullptr;
    if (ser == nullptr) {
      t_last_error = "buffer backends are not loaded";
      return 2;
    }
    eprosima::fastcdr::FastBuffer buffer(
        reinterpret_cast<char *>(const_cast<uint8_t *>(cdr_bytes)), len);
    eprosima::fastcdr::Cdr cdr(buffer, eprosima::fastcdr::Cdr::DEFAULT_ENDIAN,
                               eprosima::fastcdr::CdrVersion::XCDRv1);
    cdr.read_encapsulation();
    const rmw_topic_endpoint_info_t ep = endpoint(gid, RMW_ENDPOINT_PUBLISHER, topic_type);
    if (!cb->cdr_deserialize_with_endpoint(cdr, msg, ep, *ser)) {
      t_last_error = "the type support's deserializer reported failure (is the backend loaded?)";
      return 2;
    }
    return t_empty_import ? 3 : 0;
  } catch (const std::exception &e) {
    t_last_error = std::string("deserialization threw: ") + e.what();
    return 2;
  } catch (...) {
    t_last_error = "deserialization threw";
    return 2;
  }
}

}  // extern "C"

// ---------------------------------------------------------------------------
// Test fixture backend. A backend implemented the way an accelerator backend is
// (its own BufferImplBase subclass and a descriptor message), so the rmw's
// routing and the CDR layout can be checked without a GPU. Its descriptor has
// the field layout of NVIDIA's CudaBufferDescriptor.msg and is written and read
// with the calls the generated rosidl_typesupport_fastrtps_cpp code makes for
// that layout. "Device memory" is a process-wide table keyed by ipc_uid; a
// descriptor whose uid is not in the table imports as an empty buffer, as the
// cuda backend imports a stale one. Installed only by a test; harmless to ship.
namespace {

struct FixtureDescriptor {
  uint64_t size = 0;
  std::string element_type_name;
  int32_t device_id = 0;
  bool use_ipc = false;
  int32_t vmm_pid = 0;
  uint32_t vmm_block_id = 0;
  uint64_t vmm_block_size = 0;
  std::string vmm_socket_path;
  uint64_t ipc_uid = 0;
  uint8_t ipc_event_handle[64] = {};
};

std::mutex g_fixture_mutex;
std::unordered_map<uint64_t, std::vector<uint8_t>> g_fixture_memory;
std::set<std::vector<uint8_t>> g_fixture_denied;

class FixtureDeviceImpl : public rosidl::BufferImplBase<uint8_t> {
 public:
  FixtureDeviceImpl(std::string type, FixtureDescriptor d, std::vector<uint8_t> bytes)
      : type_(std::move(type)), d_(std::move(d)), bytes_(std::move(bytes)) {}
  std::string get_backend_type() const override { return type_; }
  size_t size() const override { return bytes_.size(); }
  std::unique_ptr<rosidl::BufferImplBase<uint8_t>> to_cpu() const override {
    auto cpu = std::make_unique<rosidl::CpuBufferImpl<uint8_t>>();
    cpu->get_storage() = bytes_;
    return cpu;
  }
  std::unique_ptr<rosidl::BufferImplBase<uint8_t>> clone() const override {
    return std::make_unique<FixtureDeviceImpl>(type_, d_, bytes_);
  }
  const FixtureDescriptor &descriptor() const { return d_; }
  const std::vector<uint8_t> &bytes() const { return bytes_; }

 private:
  std::string type_;
  FixtureDescriptor d_;
  std::vector<uint8_t> bytes_;
};

// The calls rosidl_typesupport_fastrtps_cpp generates for CudaBufferDescriptor.msg.
void fixture_write(eprosima::fastcdr::Cdr &cdr, const FixtureDescriptor &d) {
  cdr << d.size;
  cdr << d.element_type_name;
  cdr << d.device_id;
  cdr << d.use_ipc;
  cdr << d.vmm_pid;
  cdr << d.vmm_block_id;
  cdr << d.vmm_block_size;
  cdr << d.vmm_socket_path;
  cdr << d.ipc_uid;
  cdr.serialize_array(d.ipc_event_handle, 64);
}

void fixture_read(eprosima::fastcdr::Cdr &cdr, FixtureDescriptor &d) {
  cdr >> d.size;
  cdr >> d.element_type_name;
  cdr >> d.device_id;
  cdr >> d.use_ipc;
  cdr >> d.vmm_pid;
  cdr >> d.vmm_block_id;
  cdr >> d.vmm_block_size;
  cdr >> d.vmm_socket_path;
  cdr >> d.ipc_uid;
  cdr.deserialize_array(d.ipc_event_handle, 64);
}

std::vector<uint8_t> gid_key(const rmw_topic_endpoint_info_t &ep) {
  return std::vector<uint8_t>(ep.endpoint_gid, ep.endpoint_gid + RMW_GID_STORAGE_SIZE);
}

bool fixture_denied(const rmw_topic_endpoint_info_t &ep) {
  std::lock_guard<std::mutex> lock(g_fixture_mutex);
  return g_fixture_denied.count(gid_key(ep)) != 0;
}

class FixtureBackend : public rosidl::BufferBackend {
 public:
  explicit FixtureBackend(std::string type) : type_(std::move(type)) {}
  std::string get_backend_type() const override { return type_; }
  // The descriptor is serialized by this file's own serializers (see
  // rmw_cerulion_bb_install_fixture_backend), not through a type support.
  const rosidl_message_type_support_t *get_descriptor_type_support() const override {
    return nullptr;
  }
  std::shared_ptr<void> create_empty_descriptor() const override {
    return std::make_shared<FixtureDescriptor>();
  }
  std::shared_ptr<void> create_descriptor_with_endpoint(
      const void *impl, const rmw_topic_endpoint_info_t &ep) const override {
    if (fixture_denied(ep)) {
      return nullptr;
    }
    const auto *base = static_cast<const rosidl::BufferImplBase<uint8_t> *>(impl);
    const auto *device = dynamic_cast<const FixtureDeviceImpl *>(base);
    if (device == nullptr) {
      return nullptr;
    }
    {
      std::lock_guard<std::mutex> lock(g_fixture_mutex);
      g_fixture_memory[device->descriptor().ipc_uid] = device->bytes();
    }
    return std::make_shared<FixtureDescriptor>(device->descriptor());
  }
  std::unique_ptr<void, void (*)(void *)> from_descriptor_with_endpoint(
      const void *descriptor, const rmw_topic_endpoint_info_t &) const override {
    const auto *d = static_cast<const FixtureDescriptor *>(descriptor);
    std::vector<uint8_t> bytes;
    {
      std::lock_guard<std::mutex> lock(g_fixture_mutex);
      auto it = g_fixture_memory.find(d->ipc_uid);
      if (it != g_fixture_memory.end()) {
        bytes = it->second;
      }
    }
    auto *impl = new FixtureDeviceImpl(type_, *d, std::move(bytes));
    return {impl, [](void *p) { delete static_cast<rosidl::BufferImplBase<uint8_t> *>(p); }};
  }
  std::pair<bool, std::vector<std::set<uint32_t>>> on_discovering_endpoint(
      const rmw_topic_endpoint_info_t &ep, const std::vector<rmw_topic_endpoint_info_t> &,
      const std::unordered_map<std::string, std::string> &supported) override {
    return {supported.count(type_) != 0 && !fixture_denied(ep), {}};
  }

 private:
  std::string type_;
};

}  // namespace

extern "C" {

/// Install the fixture backend under `type` beside the loaded plugins (after
/// rmw_cerulion_bb_init). 0 on success, 2 when it threw.
int rmw_cerulion_bb_install_fixture_backend(const char *type) noexcept {
  try {
    (void)rmw_cerulion_bb_init();
    std::unique_lock<std::shared_mutex> lock(g_ctx_mutex);
    if (g_ctx == nullptr) {
      t_last_error = "buffer backends are not loaded";
      return 2;
    }
    auto backend = std::make_shared<FixtureBackend>(type);
    std::weak_ptr<rosidl::BufferBackend> weak = backend;
    const std::string name = type;
    rosidl::BufferDescriptorOps ops;
    ops.create_descriptor_with_endpoint = [weak](const void *impl,
                                                 const rmw_topic_endpoint_info_t &ep) {
      auto descriptor = weak.lock()->create_descriptor_with_endpoint(impl, ep);
      count_descriptor(descriptor != nullptr);
      return descriptor;
    };
    ops.from_descriptor_with_endpoint = [weak](const void *descriptor,
                                               const rmw_topic_endpoint_info_t &ep) {
      auto impl = weak.lock()->from_descriptor_with_endpoint(descriptor, ep);
      g_counters[kImports].fetch_add(1, std::memory_order_relaxed);
      const auto *base = static_cast<const rosidl::BufferImplBase<uint8_t> *>(impl.get());
      if (base == nullptr || base->size() == 0) {
        g_counters[kEmptyImports].fetch_add(1, std::memory_order_relaxed);
        t_empty_import = true;
      }
      return impl;
    };
    g_ctx->ser.descriptor_ops[name] = ops;
    rosidl_typesupport_fastrtps_cpp::BufferDescriptorSerializers serializers;
    serializers.serialize = [](eprosima::fastcdr::Cdr &cdr, const std::shared_ptr<void> &d,
                               const rmw_topic_endpoint_info_t &, const SerializationContext &) {
      fixture_write(cdr, *static_cast<const FixtureDescriptor *>(d.get()));
    };
    serializers.deserialize = [](eprosima::fastcdr::Cdr &cdr, const rmw_topic_endpoint_info_t &,
                                 const SerializationContext &) {
      auto d = std::make_shared<FixtureDescriptor>();
      fixture_read(cdr, *d);
      return std::shared_ptr<void>(d);
    };
    g_ctx->ser.descriptor_serializers[name] = std::move(serializers);
    g_ctx->backends[name] = backend;
    rebuild_names(*g_ctx);
    return 0;
  } catch (const std::exception &e) {
    t_last_error = std::string("fixture backend install threw: ") + e.what();
    return 2;
  } catch (...) {
    t_last_error = "fixture backend install threw";
    return 2;
  }
}

/// Make the fixture backend refuse the endpoint with this gid (its discovery
/// verdict is false and it creates no descriptor for it), or accept it again.
void rmw_cerulion_bb_fixture_deny(const uint8_t *gid, int deny) noexcept {
  try {
    std::lock_guard<std::mutex> lock(g_fixture_mutex);
    std::vector<uint8_t> key(gid, gid + RMW_GID_STORAGE_SIZE);
    if (deny) {
      g_fixture_denied.insert(key);
    } else {
      g_fixture_denied.erase(key);
    }
  } catch (...) {
  }
}

/// Forget the fixture "device memory" behind `ipc_uid`, so a descriptor that
/// names it imports as an empty buffer (a stale descriptor).
void rmw_cerulion_bb_fixture_forget(uint64_t ipc_uid) noexcept {
  try {
    std::lock_guard<std::mutex> lock(g_fixture_mutex);
    g_fixture_memory.erase(ipc_uid);
  } catch (...) {
  }
}

/// The descriptor fields a fixture device Buffer carries (the CudaBufferDescriptor layout).
struct RmwCerulionFixtureDescriptor {
  uint64_t size;
  const char *element_type_name;
  int32_t device_id;
  uint8_t use_ipc;
  int32_t vmm_pid;
  uint32_t vmm_block_id;
  uint64_t vmm_block_size;
  const char *vmm_socket_path;
  uint64_t ipc_uid;
  uint8_t ipc_event_handle[64];
};

/// Make the Buffer at `buf` a fixture device Buffer on backend `type` holding
/// [data, data + len) and carrying the descriptor `d`. 0 on success, 2 on a throw.
int rmw_cerulion_bb_fixture_buffer_assign(void *buf, const char *type, const uint8_t *data,
                                          size_t len,
                                          const RmwCerulionFixtureDescriptor *d) noexcept {
  try {
    FixtureDescriptor fd;
    fd.size = d->size;
    fd.element_type_name = d->element_type_name;
    fd.device_id = d->device_id;
    fd.use_ipc = d->use_ipc != 0;
    fd.vmm_pid = d->vmm_pid;
    fd.vmm_block_id = d->vmm_block_id;
    fd.vmm_block_size = d->vmm_block_size;
    fd.vmm_socket_path = d->vmm_socket_path;
    fd.ipc_uid = d->ipc_uid;
    std::memcpy(fd.ipc_event_handle, d->ipc_event_handle, 64);
    *static_cast<U8Buffer *>(buf) = U8Buffer(std::unique_ptr<rosidl::BufferImplBase<uint8_t>>(
        new FixtureDeviceImpl(type, fd, std::vector<uint8_t>(data, data + len))));
    return 0;
  } catch (const std::exception &e) {
    t_last_error = std::string("fixture buffer assign threw: ") + e.what();
    return 2;
  } catch (...) {
    t_last_error = "fixture buffer assign threw";
    return 2;
  }
}

/// Read a Buffer the way a GPU consumer of the fixture backend would: its backend
/// type into `type_out`, its size, and, for a fixture device Buffer, its bytes
/// (a view valid while the Buffer lives) and descriptor. Returns 1 for a fixture
/// device Buffer, 0 for any other, 2 on a throw.
int rmw_cerulion_bb_fixture_buffer_inspect(const void *buf, char *type_out, size_t type_cap,
                                           size_t *size, const uint8_t **bytes,
                                           uint64_t *ipc_uid, int32_t *vmm_pid) noexcept {
  try {
    const auto *b = static_cast<const U8Buffer *>(buf);
    const std::string type = b->get_backend_type();
    if (type_cap > 0) {
      const size_t n = type.size() < type_cap - 1 ? type.size() : type_cap - 1;
      std::memcpy(type_out, type.data(), n);
      type_out[n] = '\0';
    }
    *size = b->size();
    *bytes = nullptr;
    const auto *device = dynamic_cast<const FixtureDeviceImpl *>(b->get_impl());
    if (device == nullptr) {
      return 0;
    }
    *bytes = device->bytes().data();
    *ipc_uid = device->descriptor().ipc_uid;
    *vmm_pid = device->descriptor().vmm_pid;
    return 1;
  } catch (...) {
    return 2;
  }
}

}  // extern "C"
