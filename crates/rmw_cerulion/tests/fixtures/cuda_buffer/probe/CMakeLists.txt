cmake_minimum_required(VERSION 3.20)
project(cuda_buffer_s2_probe CXX)
set(CMAKE_CXX_STANDARD 17)

find_package(ament_cmake REQUIRED)
find_package(CUDAToolkit REQUIRED)
find_package(cuda_buffer REQUIRED)
find_package(cuda_buffer_backend_msgs REQUIRED)
find_package(fastcdr REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rmw REQUIRED)
find_package(rosidl_buffer REQUIRED)
find_package(rosidl_buffer_backend REQUIRED)
find_package(rosidl_buffer_backend_registry REQUIRED)
find_package(rosidl_typesupport_cpp REQUIRED)
find_package(rosidl_typesupport_fastrtps_cpp REQUIRED)
find_package(sensor_msgs REQUIRED)

set(probe_libs
  cuda_buffer::cuda_buffer
  ${cuda_buffer_backend_msgs_TARGETS}
  fastcdr
  rmw::rmw
  rosidl_buffer::rosidl_buffer
  rosidl_buffer_backend::rosidl_buffer_backend
  rosidl_buffer_backend_registry::rosidl_buffer_backend_registry
  rosidl_typesupport_cpp::rosidl_typesupport_cpp
  rosidl_typesupport_fastrtps_cpp::rosidl_typesupport_fastrtps_cpp
  ${sensor_msgs_TARGETS}
  CUDA::cudart
  CUDA::cuda_driver)

add_executable(descriptor_cdr_probe src/descriptor_cdr_probe.cpp)
target_link_libraries(descriptor_cdr_probe ${probe_libs})
add_executable(ipc_probe src/ipc_probe.cpp)
target_link_libraries(ipc_probe ${probe_libs} rt)
add_executable(s2_lat src/s2_lat.cpp)
target_link_libraries(s2_lat ${probe_libs} rclcpp::rclcpp)

install(TARGETS descriptor_cdr_probe ipc_probe s2_lat DESTINATION lib/${PROJECT_NAME})
ament_package()
