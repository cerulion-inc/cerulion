# cuda_buffer interop fixtures

Golden captures of NVIDIA's `cuda_buffer_backend` for `rosidl::Buffer`, as two
ROS 2 Lyrical rmws use it on a Jetson. Later rmw_cerulion conformance tests
read these files. None of them is built or run by cargo.

| | |
|---|---|
| Upstream | `ros2/rosidl_buffer_backends` at `4f3b257db3a5cc4c8e62d526f11f344c1e5801dc` |
| Distro | Lyrical binaries on Ubuntu 26.04 arm64; versions in `golden/env/ros-lyrical-packages.tsv` |
| rmws | `rmw_fastrtps_cpp` 9.4.10, `rmw_zenoh_cpp` 0.10.6 |
| Hardware | Jetson Orin NX 16 GB, L4T R36.4.3, CUDA 12.6 (`golden/env/host.txt`) |
| Captured | 2026-10-21 |

## Layout

```
container/      Dockerfile (Lyrical underlay), run.sh, build-overlay.sh
probe/          ament package cuda_buffer_s2_probe: the capture binaries
scripts/        stage scripts, the CDR decoder, the collector
capture.sh      runs every stage and refreshes golden/
golden/         the fixtures
```

`./capture.sh` needs Linux on an NVIDIA Jetson with CUDA 12.6, docker and the
nvidia runtime. It rebuilds the image, builds the overlay (upstream
packages plus `probe/`), runs each stage under flock on one lock file (`$S2_LOCK` sets it), and copies the
outputs into `golden/` with pids, socket inodes and the container hostname
normalized. `./capture.sh <stage>` reruns one stage.

The container gets the host CUDA toolkit as a read-only bind mount. The L4T
nvidia runtime injects `libcuda` only when `NVIDIA_VISIBLE_DEVICES` is set,
and loads it from `/usr/lib/aarch64-linux-gnu/nvidia`. The image sets both.
Every process runs in one container with `--network none`.

## What each fixture pins

### Descriptor CDR (`golden/descriptor/`)

`descriptor_cdr_probe` builds the serialization context the way both rmws
build it on Lyrical (`buffer_backend_loader.cpp`), then serializes with the
generated `rosidl_typesupport_fastrtps_cpp` callbacks. Every file starts with
the 4-byte CDR_LE encapsulation `00 01 00 00`. CDR alignment counts from the
end of the encapsulation.

- `descriptor_fixed.*.cdr`: a `CudaBufferDescriptor` with fixed field values,
  140 bytes. The bytes are deterministic.
- `image_cpu_legacy.xcdr1.cdr`: a 4x4 rgb8 `sensor_msgs/Image` with a CPU
  buffer, 96 bytes. The `data` field is a plain `uint8[]`: uint32 length, then
  the bytes.
- `image_cuda_plain.xcdr1.cdr`: the same image with a CUDA buffer, through
  the endpoint-unaware `cdr_serialize`. The backend converts the buffer to a
  host vector, so the bytes equal the CPU form.
- `image_cuda_descriptor.*.cdr`: the same image through
  `cdr_serialize_with_endpoint`, 204 bytes. This is the marker path:

| offset | bytes | field |
|---:|---:|---|
| 44 | 4 | `0xFFFFFFFF` |
| 48 | 4 | `0x524F5332` ("ROS2" as a little-endian uint32: `32 53 4f 52`) |
| 52 | 9 | backend name, CDR string `"cuda"` |
| 68 | 8 | `size` (uint64) |
| 76 | 6 | `element_type_name` (`"h"`, the `typeid(uint8_t).name()` mangling) |
| 84 | 4 | `device_id` |
| 88 | 1 | `use_ipc` |
| 92 | 4 | `vmm_pid` |
| 96 | 4 | `vmm_block_id` |
| 100 | 8 | `vmm_block_size` (the 2 MiB VMM granularity on Orin) |
| 108 | 20 | `vmm_socket_path` (`"cuda_vmm_<pid>_<block>"`, an abstract socket name) |
| 132 | 8 | `ipc_uid` |
| 140 | 64 | `ipc_event_handle` |

The `xcdr1` (rmw_fastrtps_cpp) and `ddscdr` (rmw_zenoh_cpp) encodings are
byte-identical for both message types. A reader tells the two `data` forms
apart by the first two uint32 words. A legacy length can never be
0xFFFFFFFF followed by "ROS2".

`./scripts/decode_cdr.py` decodes every file from the `.msg` layouts alone and
checks the values against `probe.json`. `decoded.json` holds the field table
with offsets. On Tegra the first uint32 of `ipc_event_handle` is the pid of
the process that created the event.

The same layout was captured from real sends (`golden/fallback/*/same_uid/wire.json`;
the marker path appears 25 times in 25 samples on both rmws). Padding bytes
differ by rmw: rmw_zenoh_cpp writes zeros, while rmw_fastrtps_cpp leaves
earlier contents of its reused serialization buffer in them. Compare fields,
not raw bytes.

### Per-block fd exchange (`golden/ipc/`)

`ipc_lifecycle.py` drives three `ipc_probe` processes under strace:
`serve`, which allocates and describes buffers; `importd`, which imports
through the backend plugin as an rmw take does; and `fetch`, which speaks
the socket by hand. `checks.json` lists 23 expected-versus-observed checks,
and all of them pass. The `*.unix_shm.strace.txt` files are the normalized
transcripts.

- On its first descriptor, each block gets a listening `SOCK_STREAM` socket
  named `@cuda_vmm_<pid>_<block_id>` in the abstract namespace. Abstract
  names are scoped to the network namespace, so importer and exporter must
  share one.
- The server's epoll thread accepts and sends one `sendmsg`: 1 data byte
  `'X'` (0x58), with one `SCM_RIGHTS` cmsg (`cmsg_len` 20, `msg_controllen`
  24) carrying one fd. It then closes the connection. Every connect gets a
  fresh copy of the fd.
- On Orin the fd is a dma-buf (`/proc/self/fd` reads `/dmabuf:`). It is the
  `cuMemExportToShareableHandle(POSIX_FILE_DESCRIPTOR)` export of the block.
  `cuMemImportFromShareableHandle`, then reserve, map and set access, gives
  the publisher's bytes.
- The importer caches by (pid, block_id). The socket round trip happens once
  per block for the life of the importing process.

### IPCMetadata (`golden/ipc/`, `fetch` event)

- Location: `/dev/shm/cuda_vmm_<pid>_<block_id>`. The segment is 24 bytes,
  created with `O_CREAT|O_RDWR` and mode 0666, which comes out as 0644
  under the default umask.

| offset | type | field |
|---:|---|---|
| 0 | `std::atomic<int32_t>` | `refcount` (importers holding the block) |
| 4 | | padding |
| 8 | `std::atomic<uint64_t>` | `uid`, equal to the descriptor's `ipc_uid` |
| 16 | `std::atomic<uint64_t>` | `publish_timestamp_us`, steady clock, set when the uid is assigned |

- An import increments `refcount` before it checks the uid, and the plugin
  undoes the increment when the check fails. The count read 1 while the
  probe held an import and 0 after it dropped it.

### Grace window and stale descriptors (`golden/ipc/checks.json`, sections B to D)

The pool reuses a released block only when `refcount == 0` and at least
100 ms have passed since `publish_timestamp_us`.

- **Inside the window.** A block released and immediately reallocated is not
  handed back (`realloc inside grace gets a new block`, about 13 ms after
  describe). A late import of the released block's descriptor still reads
  the original bytes.
- **Held import.** A held import pins the block past the window
  (`held block not reused after grace`).
- **After the window, before re-describe.** The block goes to the next
  allocation. Until that owner builds its descriptor, the segment still holds
  the old uid, so an import of the old descriptor succeeds and reads the
  **new owner's bytes** (`old descriptor, reused block, before re-describe`).
  The uid guard covers a stale descriptor only after the next publish of
  that block.
- **After re-describe.** The old descriptor is stale.
  `from_descriptor_with_endpoint` logs `Stale block ...; dropping` and
  `IPC import failed, falling back to CPU`, then returns an **empty CUDA
  buffer** (size 0, backend `cuda`) with no error to the caller. Nothing
  falls back to CPU. In the plugin source, a descriptor whose `device_id` is
  not the local device takes the same path. The capture hardware has one GPU, so that
  path was not exercised.

### CPU fallback (`golden/fallback/`)

The probe pair `s2_lat` publishes 25 CUDA-backed 4 KiB images to one
subscriber that accepts `cuda`. Each row has the publisher's socket sends
under strace. Fast DDS runs with SUBNET discovery, UDPv4 only and a loopback
static peer. Its LOCALHOST mode adds its own SHM transport, and data never
reaches a socket.

| case | rmw_fastrtps_cpp | rmw_zenoh_cpp |
|---|---|---|
| same uid, same device | 25 cuda; 25 marker-path sends | 25 cuda; 25 marker-path sends |
| `CUDA_BUFFER_UID_OVERRIDE=99999` on the subscriber | 25 cpu; legacy sends only | 25 cpu; legacy sends only |
| `CUDA_BUFFER_DEVICE_ID_OVERRIDE=999` on the subscriber | 25 cpu; legacy sends only | 25 cpu; legacy sends only |
| real uid 1001 subscriber, publisher started first with umask 000 | 25 cpu | 25 cpu |
| real uid 1001 subscriber started first | publisher (uid 0) aborts | publisher aborts |
| real uid 1001 subscriber, publisher first, umask 022 | subscriber aborts | subscriber aborts |

The fallback is decided per subscriber at the publisher, from the host-wide
endpoint registry `/dev/shm/ros2_hem_d0_h<hosthash>` (domain fixed at 0).
For a mismatched uid or device, `create_descriptor_with_endpoint` returns
null and the field goes out in legacy form.

A real second uid works only when the registry segment is writable by both
users. The first process creates it with mode 0666 minus its umask, and a
process that cannot open it read-write dies with an uncaught
`std::runtime_error("Failed to open shared memory: Permission denied")`.
With `fs.protected_regular=2` (the value in this container), uid 0 cannot
open it either when another uid created it.

## Latency baseline (`golden/latency/`)

Method: `s2_lat` at 50 Hz, 1000 samples after 100 warmup, reliable QoS depth 10, sensor_msgs/Image of the given payload size. One-way time, from CLOCK_MONOTONIC at publish to the payload being usable on the subscriber's CUDA stream (`from_input_buffer`, then a stream sync). Every sample in the cuda rows arrived with backend `cuda` and the right fill; none was lost.

Each table cell is p50 / p99 in µs.

**Stamp after the producer's stream sync (`ready/`)**: publish, transport, import, and the wait on the consumer side. This is the transport figure.

| mode | rmw | 4 KiB | 64 KiB | 1 MiB | 4 MiB | 16 MiB | 64 MiB |
|---|---|---:|---:|---:|---:|---:|---:|
| inter-process | fastrtps | 995 / 1754 | 969 / 1266 | 917 / 1443 | 985 / 1426 | 846 / 1492 | 892 / 1484 |
| inter-process | zenoh | 1083 / 1684 | 975 / 1723 | 1060 / 1763 | 849 / 1405 | 900 / 1581 | 1110 / 1724 |
| one process, rclcpp intra-process on | fastrtps | 124 / 372 | 133 / 1091 | 330 / 1093 | 329 / 1150 | 89 / 1300 | 325 / 1166 |
| one process, rclcpp intra-process on | zenoh | 97 / 330 | 91 / 1152 | 90 / 1081 | 87 / 1074 | 154 / 903 | 94 / 1172 |
| one process, through the rmw | fastrtps | 213 / 522 | 446 / 518 | 450 / 516 | 442 / 583 | 240 / 517 | 244 / 617 |
| one process, through the rmw | zenoh | 160 / 483 | 174 / 450 | 372 / 416 | 179 / 428 | 166 / 435 | 204 / 411 |

**Stamp after the fill is enqueued (`enqueued/`)**: the same path, plus the producer's GPU write (a `cudaMemsetAsync` of the whole payload). The CPU rows use a subscriber that accepts only `cpu`, so the publisher copies device to host and sends legacy CDR.

| mode | rmw | 4 KiB | 64 KiB | 1 MiB | 4 MiB | 16 MiB | 64 MiB |
|---|---|---:|---:|---:|---:|---:|---:|
| inter-process | fastrtps | 1080 / 1529 | 1089 / 1348 | 1116 / 1347 | 1219 / 1443 | 1649 / 1868 | 3376 / 3855 |
| inter-process | zenoh | 1082 / 1621 | 1092 / 1535 | 1122 / 1589 | 1220 / 1575 | 1647 / 1870 | 3372 / 3851 |
| one process, rclcpp intra-process on | fastrtps | 634 / 846 | 645 / 1126 | 666 / 889 | 773 / 1250 | 1202 / 1414 | 2921 / 3404 |
| one process, rclcpp intra-process on | zenoh | 627 / 847 | 637 / 864 | 662 / 898 | 769 / 994 | 1195 / 1225 | 3107 / 3135 |
| one process, through the rmw | fastrtps | 888 / 1040 | 930 / 1035 | 1044 / 1083 | 905 / 1164 | 1417 / 1644 | 3066 / 3325 |
| one process, through the rmw | zenoh | 957 / 982 | 946 / 979 | 774 / 987 | 860 / 1092 | 1284 / 1521 | 3017 / 3273 |
| inter-process, CPU fallback | fastrtps | 1079 / 1580 | 1193 / 1777 | 2512 / 24395 | 5846 / 26270 | 19115 / 59688 | 562844 / 614907 (13 of 1000) |
| inter-process, CPU fallback | zenoh | 1350 / 1977 | 1351 / 2110 | 3526 / 4248 | 9302 / 10121 | 34302 / 38680 | 154045 / 161912 (488 of 1000) |

How to read these:
- On the GPU path, inter-process latency is about 0.85 to 1.1 ms p50 on both rmws, and it does not depend on payload size once the producer's write is excluded.
- In the enqueued table, the growth with size, up to 3.4 ms at 64 MiB, is the fill kernel itself.
- The CPU fallback saturates at 64 MiB and 50 Hz on both rmws. The parenthesized counts are the samples delivered.
- The GPU ran under its dynamic governor (`nvhost_podgov`; 306 MHz at capture, 1173 MHz max). Power mode was 40 W, with the CPU governor `schedutil`. No clock or power setting was pinned for the capture.
- The one-process medians move between about 90 and 450 µs from cell to cell, so read them as a band, not a curve.

`samples.tsv` in each directory holds every post-warmup sample in µs. `summary.tsv` adds p90, p99.9 and max.

## Observations outside the protocol

- Each process that used CUDA IPC events left a
  `/dev/shm/cuda.shm.0.<pid hex>.1` (80 KiB) after it exited. One fallback
  stage of about 90 processes left 89 of them.
- `class_loader` warns at exit that the backend library is unloaded while
  backend objects exist. This happens on every process that loads the
  plugin.
