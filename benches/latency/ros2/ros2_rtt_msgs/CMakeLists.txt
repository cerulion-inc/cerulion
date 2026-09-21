cmake_minimum_required(VERSION 3.8)
project(ros2_rtt_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Fixed-size POD message types — one per payload size in the
# round-trip sweep. Each Pod<N> is exactly N bytes in the C++ struct
# layout (uint64 ts_ns + uint8[N-8] payload), which is the property
# that lets CycloneDDS+iceoryx and FastDDS+SHM publishers take the
# zero-copy `borrow_loaned_message()` path. sensor_msgs/Image carries
# a variable-length data field, which forces all RMWs to fall back to
# heap-allocate-and-copy regardless of transport. Same wire size,
# different cost model.
rosidl_generate_interfaces(${PROJECT_NAME}
  msg/Pod64.msg
  msg/Pod256.msg
  msg/Pod1024.msg
  msg/Pod4096.msg
  msg/Pod16384.msg
  msg/Pod65536.msg
  msg/Pod262144.msg
  msg/Pod1048576.msg
  msg/Pod4194304.msg
  msg/Pod16777216.msg
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
