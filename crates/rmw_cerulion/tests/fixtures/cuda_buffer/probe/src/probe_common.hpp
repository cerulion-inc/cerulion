// Shared helpers for the S2 capture probes: the rmw-equivalent buffer backend
// context, hex output, a fixed endpoint info, and the monotonic clock.
//
// The context is built the way rmw_fastrtps_cpp and rmw_zenoh_cpp build it on
// Lyrical (buffer_backend_loader.cpp in both): one BufferBackendRegistry, one
// instance per declared backend, descriptor ops bound to the instance, and
// FastCDR descriptor serializers taken from the backend's descriptor type
// support.
#ifndef CUDA_BUFFER_S2_PROBE__PROBE_COMMON_HPP_
#define CUDA_BUFFER_S2_PROBE__PROBE_COMMON_HPP_

#include <time.h>

#include <cstdint>
#include <cstdio>
#include <cstring>
#include <memory>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <vector>

#include "rcutils/error_handling.h"
#include "rmw/topic_endpoint_info.h"
#include "rosidl_buffer_backend/buffer_backend.hpp"
#include "rosidl_buffer_backend_registry/buffer_backend_registry.hpp"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_fastrtps_cpp/buffer_serialization.hpp"
#include "rosidl_typesupport_fastrtps_cpp/identifier.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"

namespace s2probe
{

struct BackendContext
{
  rosidl_typesupport_fastrtps_cpp::BufferSerializationContext ser;
  std::unique_ptr<rosidl_buffer_backend_registry::BufferBackendRegistry> registry;
  std::unordered_map<std::string, std::shared_ptr<rosidl::BufferBackend>> backends;
  std::vector<std::string> plugin_names;
};

inline const message_type_support_callbacks_t * fastrtps_callbacks(
  const rosidl_message_type_support_t * ts)
{
  const rosidl_message_type_support_t * f = get_message_typesupport_handle(
    ts, rosidl_typesupport_fastrtps_cpp::typesupport_identifier);
  if (!f) {
    rcutils_reset_error();
    throw std::runtime_error("no rosidl_typesupport_fastrtps_cpp handle");
  }
  return static_cast<const message_type_support_callbacks_t *>(f->data);
}

inline void init_context(BackendContext & ctx)
{
  ctx.registry = std::make_unique<rosidl_buffer_backend_registry::BufferBackendRegistry>();
  ctx.plugin_names = ctx.registry->get_backend_names();
  for (const auto & name : ctx.plugin_names) {
    auto backend = ctx.registry->create_backend_instance(name);
    if (!backend) {continue;}
    const std::string type = backend->get_backend_type();
    ctx.backends[type] = backend;
    rosidl::BufferDescriptorOps ops;
    ops.create_descriptor_with_endpoint =
      [backend](const void * impl, const rmw_topic_endpoint_info_t & ep) {
        return backend->create_descriptor_with_endpoint(impl, ep);
      };
    ops.from_descriptor_with_endpoint =
      [backend](const void * d, const rmw_topic_endpoint_info_t & ep) {
        return backend->from_descriptor_with_endpoint(d, ep);
      };
    ctx.ser.descriptor_ops[type] = ops;
    const auto * cb = fastrtps_callbacks(backend->get_descriptor_type_support());
    rosidl_typesupport_fastrtps_cpp::BufferDescriptorSerializers s;
    s.serialize = [cb](eprosima::fastcdr::Cdr & cdr, const std::shared_ptr<void> & d,
        const rmw_topic_endpoint_info_t & ep,
        const rosidl_typesupport_fastrtps_cpp::BufferSerializationContext & c) {
        if (cb->cdr_serialize_with_endpoint) {
          cb->cdr_serialize_with_endpoint(d.get(), cdr, ep, c);
        } else {
          cb->cdr_serialize(d.get(), cdr);
        }
      };
    s.deserialize = [cb, backend](eprosima::fastcdr::Cdr & cdr,
        const rmw_topic_endpoint_info_t & ep,
        const rosidl_typesupport_fastrtps_cpp::BufferSerializationContext & c) {
        auto d = backend->create_empty_descriptor();
        if (cb->cdr_deserialize_with_endpoint) {
          cb->cdr_deserialize_with_endpoint(cdr, d.get(), ep, c);
        } else {
          cb->cdr_deserialize(cdr, d.get());
        }
        return d;
      };
    ctx.ser.descriptor_serializers[type] = std::move(s);
  }
}

// An endpoint the backend has never discovered: its gid is absent from the
// backend's decision cache and from the host endpoint registry, so
// create_descriptor_with_endpoint takes the plain (non intra-process) path.
inline rmw_topic_endpoint_info_t fixed_endpoint(uint8_t fill)
{
  rmw_topic_endpoint_info_t ep = rmw_get_zero_initialized_topic_endpoint_info();
  ep.endpoint_type = RMW_ENDPOINT_SUBSCRIPTION;
  ep.topic_type = "sensor_msgs/msg/Image";
  for (size_t i = 0; i < RMW_GID_STORAGE_SIZE; ++i) {
    ep.endpoint_gid[i] = static_cast<uint8_t>(fill + i);
  }
  return ep;
}

inline std::string hex(const uint8_t * p, size_t n)
{
  static const char * d = "0123456789abcdef";
  std::string s;
  s.reserve(n * 2);
  for (size_t i = 0; i < n; ++i) {
    s.push_back(d[p[i] >> 4]);
    s.push_back(d[p[i] & 0xf]);
  }
  return s;
}

inline std::string hex(const std::vector<uint8_t> & v) {return hex(v.data(), v.size());}

inline uint64_t mono_ns()
{
  timespec ts;
  clock_gettime(CLOCK_MONOTONIC, &ts);
  return static_cast<uint64_t>(ts.tv_sec) * 1000000000ull + static_cast<uint64_t>(ts.tv_nsec);
}

inline void write_file(const std::string & path, const std::vector<uint8_t> & bytes)
{
  FILE * f = std::fopen(path.c_str(), "wb");
  if (!f) {throw std::runtime_error("cannot open " + path);}
  std::fwrite(bytes.data(), 1, bytes.size(), f);
  std::fclose(f);
}

}  // namespace s2probe

#endif  // CUDA_BUFFER_S2_PROBE__PROBE_COMMON_HPP_
