// Captures the CDR bytes of NVIDIA's CudaBufferDescriptor and of a
// sensor_msgs/Image whose data field is CUDA-backed, as the Lyrical rmws
// serialize them (rosidl_typesupport_fastrtps_cpp, encapsulation first).
//
// Usage: descriptor_cdr_probe <out_dir>
// Writes <out_dir>/*.cdr and prints one JSON object on stdout.
#include <cuda_runtime.h>
#include <unistd.h>

#include <cstdio>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

#include "cuda_buffer/cuda_buffer_api.hpp"
#include "cuda_buffer_backend_msgs/msg/cuda_buffer_descriptor.hpp"
#include "fastcdr/Cdr.h"
#include "fastcdr/FastBuffer.h"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "sensor_msgs/msg/image.hpp"

#include "probe_common.hpp"

using eprosima::fastcdr::Cdr;
using eprosima::fastcdr::CdrVersion;
using eprosima::fastcdr::FastBuffer;
using Desc = cuda_buffer_backend_msgs::msg::CudaBufferDescriptor;
using Image = sensor_msgs::msg::Image;

namespace
{

template<typename Fn>
std::vector<uint8_t> encode(CdrVersion ver, Fn && body)
{
  std::vector<char> storage(1 << 20);
  FastBuffer fb(storage.data(), storage.size());
  Cdr cdr(fb, Cdr::DEFAULT_ENDIAN, ver);
  cdr.serialize_encapsulation();
  body(cdr);
  const size_t n = cdr.get_serialized_data_length();
  return std::vector<uint8_t>(storage.begin(), storage.begin() + n);
}

Image fixed_image_header()
{
  Image m;
  m.header.stamp.sec = 1;
  m.header.stamp.nanosec = 2;
  m.header.frame_id = "s2";
  m.height = 4;
  m.width = 4;
  m.encoding = "rgb8";
  m.is_bigendian = 0;
  m.step = 12;
  return m;
}

long find_marker(const std::vector<uint8_t> & b)
{
  const uint8_t m[8] = {0xff, 0xff, 0xff, 0xff, 0x32, 0x53, 0x4f, 0x52};
  for (size_t i = 0; i + 8 <= b.size(); ++i) {
    if (std::memcmp(&b[i], m, 8) == 0) {return static_cast<long>(i);}
  }
  return -1;
}

std::string q(const std::string & s) {return "\"" + s + "\"";}

}  // namespace

int main(int argc, char ** argv)
{
  if (argc != 2) {
    std::fprintf(stderr, "usage: %s <out_dir>\n", argv[0]);
    return 2;
  }
  const std::string out = argv[1];
  s2probe::BackendContext ctx;
  s2probe::init_context(ctx);
  if (ctx.backends.find("cuda") == ctx.backends.end()) {
    std::fprintf(stderr, "cuda backend not registered\n");
    return 1;
  }

  const auto * desc_cb = s2probe::fastrtps_callbacks(
    rosidl_typesupport_cpp::get_message_type_support_handle<Desc>());
  const auto * img_cb = s2probe::fastrtps_callbacks(
    rosidl_typesupport_cpp::get_message_type_support_handle<Image>());

  // 1. A descriptor with fixed field values: deterministic bytes.
  Desc fixed;
  fixed.size = 48;
  fixed.element_type_name = "h";
  fixed.device_id = 0;
  fixed.use_ipc = true;
  fixed.vmm_pid = 4242;
  fixed.vmm_block_id = 7;
  fixed.vmm_block_size = 2097152;
  fixed.vmm_socket_path = "cuda_vmm_4242_7";
  fixed.ipc_uid = 0x0123456789abcdefull;
  for (size_t i = 0; i < fixed.ipc_event_handle.size(); ++i) {
    fixed.ipc_event_handle[i] = static_cast<uint8_t>(i);
  }
  auto fixed_x1 = encode(CdrVersion::XCDRv1, [&](Cdr & c) {desc_cb->cdr_serialize(&fixed, c);});
  auto fixed_dds = encode(CdrVersion::DDS_CDR, [&](Cdr & c) {desc_cb->cdr_serialize(&fixed, c);});
  s2probe::write_file(out + "/descriptor_fixed.xcdr1.cdr", fixed_x1);
  s2probe::write_file(out + "/descriptor_fixed.ddscdr.cdr", fixed_dds);

  // 2. Image with a CPU buffer: the legacy uint8[] wire form.
  Image cpu = fixed_image_header();
  cpu.data = std::vector<uint8_t>(48, 0xab);
  auto cpu_x1 = encode(CdrVersion::XCDRv1, [&](Cdr & c) {img_cb->cdr_serialize(&cpu, c);});
  s2probe::write_file(out + "/image_cpu_legacy.xcdr1.cdr", cpu_x1);

  // 3. Image with a CUDA buffer, written on the GPU.
  cudaStream_t stream;
  cudaStreamCreate(&stream);
  Image gpu = fixed_image_header();
  gpu.data = cuda_buffer_backend::allocate_buffer(48);
  {
    auto wh = cuda_buffer_backend::from_output_buffer(gpu.data, stream);
    cudaMemsetAsync(wh.get_ptr(), 0xab, 48, stream);
  }
  cudaStreamSynchronize(stream);

  // 3a. Endpoint-unaware serialization: forced CPU conversion.
  auto plain_x1 = encode(CdrVersion::XCDRv1, [&](Cdr & c) {img_cb->cdr_serialize(&gpu, c);});
  s2probe::write_file(out + "/image_cuda_plain.xcdr1.cdr", plain_x1);

  // 3b. Endpoint-aware serialization: the marker path.
  const auto ep = s2probe::fixed_endpoint(0x50);
  std::vector<uint8_t> desc_x1, desc_dds;
  desc_x1 = encode(CdrVersion::XCDRv1, [&](Cdr & c) {
        img_cb->cdr_serialize_with_endpoint(&gpu, c, ep, ctx.ser);
      });
  desc_dds = encode(CdrVersion::DDS_CDR, [&](Cdr & c) {
        img_cb->cdr_serialize_with_endpoint(&gpu, c, ep, ctx.ser);
      });
  s2probe::write_file(out + "/image_cuda_descriptor.xcdr1.cdr", desc_x1);
  s2probe::write_file(out + "/image_cuda_descriptor.ddscdr.cdr", desc_dds);

  // The live descriptor, as the backend built it, for the volatile fields.
  auto live = ctx.backends["cuda"]->create_descriptor_with_endpoint(gpu.data.get_impl(), ep);
  const auto * d = static_cast<const Desc *>(live.get());

  // 4. Round trip of 3b: deserialize with endpoint, read the bytes back.
  Image back;
  bool rt_ok = false;
  std::string rt_backend;
  size_t rt_size = 0;
  std::string rt_first;
  {
    std::vector<char> storage(desc_x1.begin(), desc_x1.end());
    FastBuffer fb(storage.data(), storage.size());
    Cdr cdr(fb, Cdr::DEFAULT_ENDIAN, CdrVersion::XCDRv1);
    cdr.read_encapsulation();
    rt_ok = img_cb->cdr_deserialize_with_endpoint(cdr, &back, ep, ctx.ser);
    rt_backend = back.data.get_backend_type();
    rt_size = back.data.size();
    if (rt_backend == "cuda" && rt_size > 0) {
      std::vector<uint8_t> h(rt_size);
      auto rh = cuda_buffer_backend::from_input_buffer(back.data, stream);
      cudaMemcpyAsync(h.data(), rh.get_ptr(), rt_size, cudaMemcpyDeviceToHost, stream);
      cudaStreamSynchronize(stream);
      rt_first = s2probe::hex(h.data(), std::min<size_t>(rt_size, 8));
    }
  }

  std::ostringstream js;
  js << "{\n";
  js << "  \"pid\": " << getpid() << ",\n";
  js << "  \"plugin_names\": [";
  for (size_t i = 0; i < ctx.plugin_names.size(); ++i) {
    js << (i ? ", " : "") << q(ctx.plugin_names[i]);
  }
  js << "],\n";
  js << "  \"descriptor_fixed_xcdr1\": " << q(s2probe::hex(fixed_x1)) << ",\n";
  js << "  \"descriptor_fixed_ddscdr\": " << q(s2probe::hex(fixed_dds)) << ",\n";
  js << "  \"image_cpu_legacy_xcdr1\": " << q(s2probe::hex(cpu_x1)) << ",\n";
  js << "  \"image_cuda_plain_xcdr1\": " << q(s2probe::hex(plain_x1)) << ",\n";
  js << "  \"image_cuda_descriptor_xcdr1\": " << q(s2probe::hex(desc_x1)) << ",\n";
  js << "  \"image_cuda_descriptor_ddscdr\": " << q(s2probe::hex(desc_dds)) << ",\n";
  js << "  \"marker_offset_xcdr1\": " << find_marker(desc_x1) << ",\n";
  js << "  \"has_buffer_fields\": " << (img_cb->has_buffer_fields ? "true" : "false") << ",\n";
  js << "  \"live_descriptor\": {\"size\": " << d->size
     << ", \"element_type_name\": " << q(d->element_type_name)
     << ", \"device_id\": " << d->device_id
     << ", \"use_ipc\": " << (d->use_ipc ? "true" : "false")
     << ", \"vmm_pid\": " << d->vmm_pid
     << ", \"vmm_block_id\": " << d->vmm_block_id
     << ", \"vmm_block_size\": " << d->vmm_block_size
     << ", \"vmm_socket_path\": " << q(d->vmm_socket_path)
     << ", \"ipc_uid\": " << d->ipc_uid
     << ", \"ipc_event_handle\": " << q(s2probe::hex(d->ipc_event_handle.data(), 64)) << "},\n";
  js << "  \"round_trip\": {\"ok\": " << (rt_ok ? "true" : "false")
     << ", \"backend\": " << q(rt_backend) << ", \"size\": " << rt_size
     << ", \"first_bytes\": " << q(rt_first) << "}\n";
  js << "}\n";
  std::cout << js.str();
  cudaStreamDestroy(stream);
  return 0;
}
