// pod_dispatch.hpp — runtime payload-size → Pod<N> message-type
// dispatch for ros2_rtt_bench.
//
// Each {ping,pong,latency}_node main() reads CER_BENCH_PAYLOAD_SIZE
// at startup and instantiates a templated node class with the matching
// fixed-size POD message type. POD ("plain") layout is the necessary
// condition for `Publisher::can_loan_messages() == true` on every RMW
// that implements loans (rmw_fastrtps: is_plain since Iron, PR #568;
// rmw_cyclonedds: additionally build/version-gated — see
// ping_node.cpp's per-RMW loaned= semantics note). A single
// non-templated node + variable-size sensor_msgs/Image would force
// every RMW into the heap-allocate-and-copy fallback.
//
// 16 MB messages: Pod16777216 contains a std::array<uint8_t, 16777208>
// — never put one on the stack. The templated code preallocates the
// loan-fallback message ONCE at node construction with
// std::make_unique<Msg>() (heap; reused every iteration — Mode-A/G3);
// the loan path puts it in RMW-owned shared memory.

#pragma once

#include <cstddef>
#include <cstdio>

#include <ros2_rtt_msgs/msg/pod64.hpp>
#include <ros2_rtt_msgs/msg/pod256.hpp>
#include <ros2_rtt_msgs/msg/pod1024.hpp>
#include <ros2_rtt_msgs/msg/pod4096.hpp>
#include <ros2_rtt_msgs/msg/pod16384.hpp>
#include <ros2_rtt_msgs/msg/pod65536.hpp>
#include <ros2_rtt_msgs/msg/pod262144.hpp>
#include <ros2_rtt_msgs/msg/pod1048576.hpp>
#include <ros2_rtt_msgs/msg/pod4194304.hpp>
#include <ros2_rtt_msgs/msg/pod16777216.hpp>

// Invokes RUNNER<Pod<size>>() and returns its int result. RUNNER must
// be a function template `template <typename Msg> int RUNNER();`.
//
// Defined as a macro because templating a function-of-a-function-template
// over the dispatch is more boilerplate than the macro itself.
#define ROS2_RTT_BENCH_DISPATCH_BY_SIZE(SIZE_VAR, RUNNER)                  \
  do {                                                                    \
    using namespace ros2_rtt_msgs::msg;                                   \
    switch (SIZE_VAR) {                                                   \
      case 64:        return RUNNER<Pod64>();                             \
      case 256:       return RUNNER<Pod256>();                            \
      case 1024:      return RUNNER<Pod1024>();                           \
      case 4096:      return RUNNER<Pod4096>();                           \
      case 16384:     return RUNNER<Pod16384>();                          \
      case 65536:     return RUNNER<Pod65536>();                          \
      case 262144:    return RUNNER<Pod262144>();                         \
      case 1048576:   return RUNNER<Pod1048576>();                        \
      case 4194304:   return RUNNER<Pod4194304>();                        \
      case 16777216:  return RUNNER<Pod16777216>();                       \
      default:                                                            \
        std::fprintf(stderr, "unsupported CER_BENCH_PAYLOAD_SIZE=%zu — "  \
                     "valid sizes: 64,256,1024,4096,16384,65536,262144,"  \
                     "1048576,4194304,16777216\n", (size_t)(SIZE_VAR));   \
        return 2;                                                         \
    }                                                                     \
  } while (0)
