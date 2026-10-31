// Probes NVIDIA cuda_buffer_backend's inter-process protocol below the rmw:
// the per-block abstract Unix socket that hands out the VMM fd (SCM_RIGHTS),
// the per-block IPCMetadata shared-memory segment, the pool's 100 ms reuse
// grace window, and the import of a stale descriptor.
//
// Roles (one JSON object per output line):
//   ipc_probe serve
//       Reads commands on stdin:
//         alloc <slot> <bytes> <fill>   allocate a CUDA buffer, fill it on the
//                                       GPU, build its descriptor
//         allocnd <slot> <bytes> <fill> the same without the descriptor
//         describe <slot>               build the descriptor again
//         release <slot>                drop the buffer (back to the pool)
//         quit
//   ipc_probe fetch <socket_name> <pid> <block_id> <block_size>
//       Speaks the socket protocol by hand, imports the fd with the driver
//       API, and dumps the block's IPCMetadata segment.
//   ipc_probe importd
//       Keeps a warm CUDA context and reads commands on stdin:
//         import <id> <size> <socket> <pid> <block_id> <block_size> <uid>
//                      import through the backend plugin
//                      (from_descriptor_with_endpoint), as an rmw take does,
//                      and hold the resulting buffer
//         drop <id>    release it
//         quit
#include <cuda.h>
#include <cuda_runtime.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <sys/socket.h>
#include <sys/stat.h>
#include <sys/un.h>
#include <unistd.h>

#include <algorithm>
#include <chrono>
#include <cstddef>
#include <cstdio>
#include <cstring>
#include <iostream>
#include <map>
#include <sstream>
#include <string>
#include <thread>
#include <typeinfo>
#include <vector>

#include "cuda_buffer/cuda_buffer_api.hpp"
#include "cuda_buffer/cuda_memory_pool.hpp"
#include "cuda_buffer_backend_msgs/msg/cuda_buffer_descriptor.hpp"

#include "probe_common.hpp"

using Desc = cuda_buffer_backend_msgs::msg::CudaBufferDescriptor;

namespace
{

std::string q(const std::string & s) {return "\"" + s + "\"";}

std::string desc_json(const Desc & d)
{
  std::ostringstream o;
  o << "{\"size\": " << d.size << ", \"element_type_name\": " << q(d.element_type_name)
    << ", \"device_id\": " << d.device_id << ", \"use_ipc\": " << (d.use_ipc ? "true" : "false")
    << ", \"vmm_pid\": " << d.vmm_pid << ", \"vmm_block_id\": " << d.vmm_block_id
    << ", \"vmm_block_size\": " << d.vmm_block_size
    << ", \"vmm_socket_path\": " << q(d.vmm_socket_path) << ", \"ipc_uid\": " << d.ipc_uid
    << ", \"ipc_event_handle_nonzero\": "
    << (std::any_of(d.ipc_event_handle.begin(), d.ipc_event_handle.end(),
        [](uint8_t b) {return b != 0;}) ? "true" : "false") << "}";
  return o.str();
}

struct MetaRead
{
  bool ok{false};
  off_t file_size{0};
  unsigned mode{0};
  std::vector<uint8_t> raw;
  int32_t refcount{0};
  uint64_t uid{0};
  uint64_t ts_us{0};
};

MetaRead read_meta(int pid, unsigned block)
{
  MetaRead m;
  const std::string path = "/dev/shm/cuda_vmm_" + std::to_string(pid) + "_" +
    std::to_string(block);
  int fd = open(path.c_str(), O_RDONLY);
  if (fd < 0) {return m;}
  struct stat st;
  fstat(fd, &st);
  m.file_size = st.st_size;
  m.mode = st.st_mode & 07777;
  m.raw.resize(static_cast<size_t>(st.st_size));
  ssize_t n = pread(fd, m.raw.data(), m.raw.size(), 0);
  close(fd);
  if (n != st.st_size || n < 24) {return m;}
  std::memcpy(&m.refcount, &m.raw[0], 4);
  std::memcpy(&m.uid, &m.raw[8], 8);
  std::memcpy(&m.ts_us, &m.raw[16], 8);
  m.ok = true;
  return m;
}

std::string meta_json(const MetaRead & m)
{
  std::ostringstream o;
  o << "{\"ok\": " << (m.ok ? "true" : "false") << ", \"file_size\": " << m.file_size
    << ", \"mode\": \"" << std::oct << m.mode << std::dec << "\", \"raw\": "
    << q(s2probe::hex(m.raw)) << ", \"refcount\": " << m.refcount << ", \"uid\": " << m.uid
    << ", \"publish_timestamp_us\": " << m.ts_us << "}";
  return o.str();
}

uint64_t steady_us()
{
  return static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::microseconds>(
      std::chrono::steady_clock::now().time_since_epoch()).count());
}

int serve()
{
  s2probe::BackendContext ctx;
  s2probe::init_context(ctx);
  auto backend = ctx.backends.at("cuda");
  const auto ep = s2probe::fixed_endpoint(0x60);
  cudaStream_t stream;
  cudaStreamCreate(&stream);
  std::map<int, rosidl::Buffer<uint8_t>> slots;
  std::cout << "{\"event\": \"ready\", \"pid\": " << getpid() << "}" << std::endl;
  std::string line;
  while (std::getline(std::cin, line)) {
    std::istringstream in(line);
    std::string cmd;
    in >> cmd;
    if (cmd == "alloc" || cmd == "allocnd" || cmd == "describe") {
      int slot = 0;
      in >> slot;
      if (cmd == "alloc" || cmd == "allocnd") {
        size_t bytes = 0;
        int fill = 0;
        in >> bytes >> fill;
        slots[slot] = cuda_buffer_backend::allocate_buffer(bytes);
        auto wh = cuda_buffer_backend::from_output_buffer(slots[slot], stream);
        cudaMemsetAsync(wh.get_ptr(), fill, bytes, stream);
      }
      cudaStreamSynchronize(stream);
      if (cmd == "allocnd") {
        // Allocated and written, no descriptor built yet: the window between
        // the pool handing a block back out and the next publish.
        auto * impl = dynamic_cast<cuda_buffer_backend::CudaBufferImpl<uint8_t> *>(
          slots[slot].get_impl());
        auto pool = cuda_buffer_backend::get_or_create_global_pool();
        auto * blk = pool->find_block_for_va(reinterpret_cast<CUdeviceptr>(
            impl->get_cuda_buffer().get_device_ptr()));
        std::cout << "{\"event\": \"allocnd\", \"slot\": " << slot << ", \"steady_us\": "
                  << steady_us() << ", \"block_id\": " << (blk ? static_cast<long>(blk->block_id) : -1L)
                  << ", \"meta\": "
                  << (blk ? meta_json(read_meta(getpid(), blk->block_id)) : "null") << "}"
                  << std::endl;
        continue;
      }
      auto d = backend->create_descriptor_with_endpoint(slots[slot].get_impl(), ep);
      const auto * dd = static_cast<const Desc *>(d.get());
      std::cout << "{\"event\": " << q(cmd) << ", \"slot\": " << slot << ", \"steady_us\": "
                << steady_us() << ", \"descriptor\": " << (dd ? desc_json(*dd) : "null")
                << ", \"meta\": "
                << (dd ? meta_json(read_meta(getpid(), dd->vmm_block_id)) : "null") << "}"
                << std::endl;
    } else if (cmd == "release") {
      int slot = 0;
      in >> slot;
      slots.erase(slot);
      std::cout << "{\"event\": \"release\", \"slot\": " << slot << ", \"steady_us\": "
                << steady_us() << "}" << std::endl;
    } else if (cmd == "quit") {
      break;
    }
  }
  slots.clear();
  cudaStreamDestroy(stream);
  return 0;
}

struct sockaddr_un abstract_addr(const std::string & name, socklen_t & len)
{
  struct sockaddr_un a{};
  a.sun_family = AF_UNIX;
  std::memcpy(a.sun_path + 1, name.data(), name.size());
  len = static_cast<socklen_t>(offsetof(sockaddr_un, sun_path) + 1 + name.size());
  return a;
}

int fetch(const std::string & name, int pid, unsigned block, size_t block_size)
{
  std::ostringstream o;
  o << "{\"role\": \"fetch\", \"socket\": " << q("@" + name);
  o << ", \"sizeof_IPCMetadata\": " << sizeof(cuda_buffer_backend::IPCMetadata)
    << ", \"offsetof_refcount\": " << offsetof(cuda_buffer_backend::IPCMetadata, refcount)
    << ", \"offsetof_uid\": " << offsetof(cuda_buffer_backend::IPCMetadata, uid)
    << ", \"offsetof_publish_timestamp_us\": "
    << offsetof(cuda_buffer_backend::IPCMetadata, publish_timestamp_us);
  int received_fd = -1;
  for (int attempt = 0; attempt < 2; ++attempt) {
    int s = socket(AF_UNIX, SOCK_STREAM, 0);
    socklen_t len = 0;
    auto addr = abstract_addr(name, len);
    if (connect(s, reinterpret_cast<sockaddr *>(&addr), len) != 0) {
      o << ", \"connect_errno_" << attempt << "\": " << errno;
      close(s);
      continue;
    }
    char data[16] = {0};
    alignas(cmsghdr) char ctrl[CMSG_SPACE(4 * sizeof(int))];
    std::memset(ctrl, 0, sizeof(ctrl));
    iovec iov{data, sizeof(data)};
    msghdr msg{};
    msg.msg_iov = &iov;
    msg.msg_iovlen = 1;
    msg.msg_control = ctrl;
    msg.msg_controllen = sizeof(ctrl);
    ssize_t n = recvmsg(s, &msg, MSG_CMSG_CLOEXEC);
    // A second read shows the server closes after the one message.
    char extra[4];
    ssize_t n2 = recv(s, extra, sizeof(extra), 0);
    close(s);
    cmsghdr * c = CMSG_FIRSTHDR(&msg);
    int nfds = 0;
    int fd = -1;
    if (c && c->cmsg_level == SOL_SOCKET && c->cmsg_type == SCM_RIGHTS) {
      nfds = static_cast<int>((c->cmsg_len - CMSG_LEN(0)) / sizeof(int));
      std::memcpy(&fd, CMSG_DATA(c), sizeof(int));
    }
    o << ", \"recv_" << attempt << "\": {\"bytes\": " << n << ", \"data\": "
      << q(s2probe::hex(reinterpret_cast<uint8_t *>(data), n > 0 ? n : 0))
      << ", \"msg_flags\": " << msg.msg_flags << ", \"msg_controllen\": " << msg.msg_controllen
      << ", \"cmsg_len\": " << (c ? c->cmsg_len : 0) << ", \"cmsg_level\": "
      << (c ? c->cmsg_level : -1) << ", \"cmsg_type\": " << (c ? c->cmsg_type : -1)
      << ", \"fds\": " << nfds << ", \"read_after\": " << n2 << "}";
    if (fd >= 0) {
      char link[256] = {0};
      std::string p = "/proc/self/fd/" + std::to_string(fd);
      ssize_t ln = readlink(p.c_str(), link, sizeof(link) - 1);
      struct stat st;
      fstat(fd, &st);
      o << ", \"fd_" << attempt << "\": {\"readlink\": " << q(ln > 0 ? link : "")
        << ", \"st_mode_fmt\": " << ((st.st_mode & S_IFMT) >> 12) << "}";
      if (received_fd < 0) {received_fd = fd;} else {close(fd);}
    }
  }
  if (received_fd >= 0) {
    cuInit(0);
    cudaFree(nullptr);
    CUmemGenericAllocationHandle h = 0;
    CUresult r = cuMemImportFromShareableHandle(
      &h, reinterpret_cast<void *>(static_cast<intptr_t>(received_fd)),
      CU_MEM_HANDLE_TYPE_POSIX_FILE_DESCRIPTOR);
    o << ", \"import_result\": " << r;
    if (r == CUDA_SUCCESS) {
      CUmemAllocationProp prop{};
      cuMemGetAllocationPropertiesFromHandle(&prop, h);
      CUdeviceptr va = 0;
      cuMemAddressReserve(&va, block_size, 0, 0, 0);
      CUresult mr = cuMemMap(va, block_size, 0, h, 0);
      CUmemAccessDesc acc{};
      acc.location.type = CU_MEM_LOCATION_TYPE_DEVICE;
      acc.location.id = 0;
      acc.flags = CU_MEM_ACCESS_FLAGS_PROT_READWRITE;
      cuMemSetAccess(va, block_size, &acc, 1);
      uint8_t head[8] = {0};
      CUresult cr = cuMemcpyDtoH(head, va, sizeof(head));
      o << ", \"prop\": {\"type\": " << prop.type << ", \"location_type\": "
        << prop.location.type << ", \"location_id\": " << prop.location.id
        << ", \"requested_handle_types\": " << prop.requestedHandleTypes << "}"
        << ", \"map_result\": " << mr << ", \"copy_result\": " << cr
        << ", \"block_head\": " << q(s2probe::hex(head, sizeof(head)));
      cuMemUnmap(va, block_size);
      cuMemAddressFree(va, block_size);
      cuMemRelease(h);
    }
    close(received_fd);
  }
  o << ", \"meta\": " << meta_json(read_meta(pid, block)) << "}";
  std::cout << o.str() << std::endl;
  return 0;
}

int importd()
{
  s2probe::BackendContext ctx;
  s2probe::init_context(ctx);
  auto backend = ctx.backends.at("cuda");
  const auto ep = s2probe::fixed_endpoint(0x70);
  cudaStream_t stream;
  cudaStreamCreate(&stream);
  cudaFree(nullptr);
  struct Held
  {
    rosidl::Buffer<uint8_t> buf;
    int pid;
    unsigned block;
  };
  std::map<int, Held> held;
  std::cout << "{\"event\": \"ready\", \"pid\": " << getpid() << "}" << std::endl;
  std::string line;
  while (std::getline(std::cin, line)) {
    std::istringstream in(line);
    std::string cmd;
    in >> cmd;
    if (cmd == "import") {
      int id = 0;
      Desc d;
      in >> id >> d.size >> d.vmm_socket_path >> d.vmm_pid >> d.vmm_block_id >>
      d.vmm_block_size >> d.ipc_uid;
      d.element_type_name = typeid(uint8_t).name();
      d.device_id = 0;
      d.use_ipc = true;
      std::memset(d.ipc_event_handle.data(), 0, d.ipc_event_handle.size());
      std::ostringstream o;
      const uint64_t t0 = steady_us();
      o << "{\"event\": \"import\", \"id\": " << id << ", \"steady_us\": " << t0
        << ", \"expected_uid\": " << d.ipc_uid
        << ", \"meta_before\": " << meta_json(read_meta(d.vmm_pid, d.vmm_block_id));
      auto erased = backend->from_descriptor_with_endpoint(&d, ep);
      std::unique_ptr<rosidl::BufferImplBase<uint8_t>> typed(
        static_cast<rosidl::BufferImplBase<uint8_t> *>(erased.release()));
      rosidl::Buffer<uint8_t> buf(std::move(typed));
      o << ", \"backend\": " << q(buf.get_backend_type()) << ", \"size\": " << buf.size();
      if (buf.size() > 0) {
        std::vector<uint8_t> h(std::min<size_t>(buf.size(), 8));
        auto rh = cuda_buffer_backend::from_input_buffer(buf, stream);
        cudaMemcpyAsync(h.data(), rh.get_ptr(), h.size(), cudaMemcpyDeviceToHost, stream);
        cudaStreamSynchronize(stream);
        o << ", \"first_bytes\": " << q(s2probe::hex(h));
      }
      o << ", \"meta_holding\": " << meta_json(read_meta(d.vmm_pid, d.vmm_block_id)) << "}";
      held[id] = Held{std::move(buf), d.vmm_pid, d.vmm_block_id};
      std::cout << o.str() << std::endl;
    } else if (cmd == "drop") {
      int id = 0;
      in >> id;
      auto it = held.find(id);
      if (it == held.end()) {continue;}
      const int pid = it->second.pid;
      const unsigned block = it->second.block;
      held.erase(it);
      std::this_thread::sleep_for(std::chrono::milliseconds(50));
      std::cout << "{\"event\": \"drop\", \"id\": " << id << ", \"steady_us\": "
                << steady_us() << ", \"meta_after\": " << meta_json(read_meta(pid, block))
                << "}" << std::endl;
    } else if (cmd == "quit") {
      break;
    }
  }
  held.clear();
  cudaStreamDestroy(stream);
  return 0;
}

}  // namespace

int main(int argc, char ** argv)
{
  if (argc >= 2 && std::string(argv[1]) == "serve") {return serve();}
  if (argc == 6 && std::string(argv[1]) == "fetch") {
    return fetch(argv[2], std::stoi(argv[3]), static_cast<unsigned>(std::stoul(argv[4])),
             std::stoull(argv[5]));
  }
  if (argc == 2 && std::string(argv[1]) == "importd") {return importd();}
  std::fprintf(stderr, "usage: see header comment\n");
  return 2;
}
