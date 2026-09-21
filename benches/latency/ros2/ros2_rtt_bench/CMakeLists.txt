cmake_minimum_required(VERSION 3.8)
project(ros2_rtt_bench CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rcl REQUIRED)
find_package(rmw REQUIRED)
find_package(std_msgs REQUIRED)
# sensor_msgs: the TYPE-CLASS axis's variable-message cells
# (CER_BENCH_MSG=image → sensor_msgs/msg/Image, the canonical unbounded
# ROS 2 sensor type — see src/msg_class_dispatch.hpp).
find_package(sensor_msgs REQUIRED)
find_package(ros2_rtt_msgs REQUIRED)

include_directories(src)

# Three measurement processes — one node per process matches a
# canonical ROS2 deployment. Each binary is compiled with all 10 Pod<N>
# template instantiations so a single `colcon build` covers the
# entire payload-size sweep; main() picks the right Msg type at
# startup from CER_BENCH_PAYLOAD_SIZE.
#
# The May-2026 intra_* lanes (ipc_bench_naive /
# ipc_bench_forward) were dropped; the intra-process axis returned 2026-08-14 as the
# COMPOSED lane (composed_rtt_node below — evidence:
# memo.md §2).

# ament_target_dependencies() was deprecated in Kilted and REMOVED in
# Lyrical ("Unknown CMake command" — hit live on ros:lyrical,
# 2026-08-12; PITFALLS #22). Modern namespaced targets are portable
# back to Humble. rcl + rmw symbols/headers arrive transitively through
# rclcpp::rclcpp's PUBLIC dependencies, so the rcl-lane binaries need
# no extra link line.
set(BENCH_DEPS
  rclcpp::rclcpp
  ros2_rtt_msgs::ros2_rtt_msgs__rosidl_typesupport_cpp
  std_msgs::std_msgs__rosidl_typesupport_cpp
  sensor_msgs::sensor_msgs__rosidl_typesupport_cpp)

add_executable(ping_node src/ping_node.cpp)
target_link_libraries(ping_node ${BENCH_DEPS})

add_executable(pong_node src/pong_node.cpp)
target_link_libraries(pong_node ${BENCH_DEPS})

add_executable(latency_node src/latency_node.cpp)
target_link_libraries(latency_node ${BENCH_DEPS})

# pong_node_rcl + latency_node_rcl: same wire shape as the headline
# binaries but replace rclcpp::Subscription's typed callback (which
# memcpy's the payload into a heap Msg before invoking the user
# callback) with rcl_take_loaned_message + rcl_return_loaned_
# message_from_subscription — the `performance_test` pattern,
# the only way to get true zero-copy receive on ROS2. This is the
# `loan` recv lane (RECV_PATH=loan).
add_executable(pong_node_rcl src/pong_node_rcl.cpp)
target_link_libraries(pong_node_rcl ${BENCH_DEPS})

add_executable(latency_node_rcl src/latency_node_rcl.cpp)
target_link_libraries(latency_node_rcl ${BENCH_DEPS})

# composed_rtt_node: the COMPOSED lane (usage-memo §2): all
# three roles as manually-composed nodes in ONE process on ONE
# SingleThreadedExecutor, with rclcpp intra-process comms toggled by
# CER_BENCH_IPC={on,off} (NodeOptions::use_intra_process_comms). Manual
# composition in main() — no rclcpp_components dependency; a component
# container would add launch infrastructure outside the measured window
# without changing the data path.
add_executable(composed_rtt_node src/composed_rtt_node.cpp)
target_link_libraries(composed_rtt_node ${BENCH_DEPS})

install(TARGETS
  ping_node
  pong_node
  latency_node
  pong_node_rcl
  latency_node_rcl
  composed_rtt_node
  DESTINATION lib/${PROJECT_NAME}
)

# The two hand ORACLES under test/ — COMPILED, never run, and never
# installed. Every expectation in them is a `static_assert` over the
# shipping ROS-free headers (src/stamp_codec.hpp, src/sample_gate.hpp),
# so the COMPILER is the runner and this build's exit code IS the
# verdict. They produce no runnable artifact, which is why they are
# OBJECT libraries and appear in no install() rule.
#
# UNCONDITIONAL, and deliberately not behind `if(BUILD_TESTING)`. The
# defect these targets close is that the oracle files sat in the package
# and colcon compiled NEITHER of them, so the container build ran no
# oracle at all while two comments named it as the decision's proof. A
# gate on a variable this package's own build line never sets
# (`colcon build --packages-select ... --cmake-args -DCMAKE_BUILD_TYPE=
# Release`, docker/Dockerfile) would reproduce exactly that — a target
# that looks wired and compiles nowhere — so the cost is paid instead:
# two header-only translation units of integer constant evaluation, no
# link step, no ROS dependency.
#
# NO `-Werror` here, on purpose. A failed `static_assert` is an ERROR
# with or without it, so the verdict does not need it; adding it would
# let an unrelated warning from whatever GCC the distro image ships fail
# a MEASUREMENT build. The warning-clean claim is enforced where it can
# fail harmlessly instead: check_percentile_parity.py compiles both files
# with `-Wall -Wextra -Wpedantic -Werror -fsyntax-only` on the desk.
add_library(stamp_codec_oracle OBJECT test/stamp_codec_oracle.cpp)
add_library(sample_gate_oracle OBJECT test/sample_gate_oracle.cpp)

ament_package()
