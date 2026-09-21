// stamp_codec.hpp — the builtin_interfaces/Time stamp codec the image
// class rides, kept deliberately ROS-FREE.
//
// WHY ITS OWN HEADER. The image class stamps `header.stamp`
// (builtin_interfaces/Time: int32 sec + uint32 nanosec), which is the
// field a real Image publisher stamps — but int32 seconds is a CEILING
// the pod class's uint64 `ts_ns` does not have, and a bare
// `static_cast<int32_t>(t / 1e9)` has nothing checking it. On a
// host whose CLOCK_MONOTONIC has passed INT32_MAX seconds (2147483647 s
// ~= 68.05 years of uptime) that cast is out of range, the reader
// sign-extends whatever came back, and the cell reports a latency that
// is not a latency — silently, under a label that says it is one. The
// number is the product here, so an unmeasurable run must REFUSE, never
// round-trip a wrong figure.
//
// The refusal is a start-up check (see msg_class_dispatch.hpp's
// ROS2_RTT_BENCH_DISPATCH_BY_CLASS), and it is the real guard. This header
// holds the arithmetic it decides on, with NO rclcpp / sensor_msgs
// include, for one reason: a desk without a ROS distro can COMPILE the
// hand oracle against THIS code — the oracle is a wall of
// `static_assert`s driven with `-fsyntax-only`, so nothing is ever
// executed
// (test/stamp_codec_oracle.cpp, driven by
// check_percentile_parity.py::check_image_stamp_codec) instead of
// against a copy of it. A copy is what this tree keeps paying for
// elsewhere; the oracle includes the shipping header.

#pragma once

#include <cstdint>

namespace ros2_rtt_bench {

inline constexpr uint64_t kNanosPerSecond = 1000000000ULL;

// builtin_interfaces/msg/Time::sec is an int32 — this is its ceiling in
// seconds, spelled rather than inherited from <climits> so the oracle
// pins the number the wire actually carries.
inline constexpr int64_t kRosTimeSecMax = 2147483647;

// Headroom the start-up check demands ON TOP of the current instant: a
// bench cell is minutes, so one day means no run that PASSES the check
// can cross the ceiling while it is running. (Checking only "right now"
// would admit a run that starts one second below the ceiling and
// silently crosses it mid-sweep — the same wrong number, harder to
// attribute.)
inline constexpr uint64_t kStampHeadroomSeconds = 86400;

// A field-for-field MIRROR of builtin_interfaces/msg/Time, INCLUDING its
// ability to hold values encode_ros_stamp never mints (a negative `sec`, a
// `nanosec` at or above 1e9): read_stamp builds one straight off the wire.
// That is why decode_ros_stamp below has no rescue arm — the invariant
// belongs to the encoder, not to this struct. A plain aggregate, so it is
// a literal type the oracle's static_asserts can evaluate and so braced
// init from the ROS fields makes a future type change a compile error
// rather than a silent narrowing.
struct RosStamp {
  int32_t sec;
  uint32_t nanosec;
};

// ros_stamp_encodable — can `t_ns`, plus `headroom_s` further seconds,
// ride a builtin_interfaces/Time without narrowing? The `headroom_s <=
// limit` term is not decoration: it keeps `limit - headroom_s` from
// wrapping when a caller passes an absurd headroom, which would turn
// the guard into an accept-everything.
inline constexpr bool ros_stamp_encodable(uint64_t t_ns, uint64_t headroom_s)
{
  const uint64_t limit = static_cast<uint64_t>(kRosTimeSecMax);
  const uint64_t sec = t_ns / kNanosPerSecond;
  return headroom_s <= limit && sec <= limit - headroom_s;
}

// encode_ros_stamp — total, and SATURATING at kRosTimeSecMax rather
// than narrowing out of range.
//
// SATURATION IS DIRECTIONAL, and that is the property the design leans on:
// a saturated stamp is always <= the true send instant, so the latency it
// yields is the true one PLUS at least a whole second, growing a second
// per second. It can never under-report and can never wrap — where an
// unchecked out-of-range conversion can do both, and a wrapped stamp can
// read as a plausibly small latency. Anything "improving" this into a
// symmetric or wrapping fallback gives that back.
//
// Its precondition is `ros_stamp_encodable(t_ns, 0)`, which the
// start-up refusal establishes for the whole run, so the saturating arm
// is unreachable in a run that was allowed to start. It exists so that
// a caller which somehow reached it gets a DEFINED value (a frozen
// stamp, which reads as an absurd latency at the very first sample)
// instead of an out-of-range conversion — belt to the start-up
// refusal's braces, never a substitute for it.
inline constexpr RosStamp encode_ros_stamp(uint64_t t_ns)
{
  const uint64_t sec = t_ns / kNanosPerSecond;
  const uint64_t limit = static_cast<uint64_t>(kRosTimeSecMax);
  return RosStamp{
    static_cast<int32_t>(sec > limit ? limit : sec),
    static_cast<uint32_t>(t_ns % kNanosPerSecond)};
}

// decode_ros_stamp — the exact inverse of encode_ros_stamp over every
// instant that encoder does not SATURATE. (Over a saturated one it
// cannot be: encode threw the excess away, which is what makes the
// resulting reading absurd rather than plausible.)
//
// A NEGATIVE `sec` is not one of them, and this bench only ever reads
// frames its own ping node published, so no arm here tries to rescue
// one: the int64 arithmetic reproduces what the field says, and the
// resulting sample is absurd on its face either way. What saturation
// removes is the case where OUR OWN encoder mints such a stamp.
inline constexpr uint64_t decode_ros_stamp(RosStamp s)
{
  return static_cast<uint64_t>(
    static_cast<int64_t>(s.sec) * static_cast<int64_t>(kNanosPerSecond) +
    static_cast<int64_t>(s.nanosec));
}

// stamp_range_deliverable — THE decision the start-up refusal makes,
// here rather than in the dispatch header so it is `constexpr` and the
// hand oracle can pin its POLARITY.
//
// That is not tidiness. While this lived as an `if` inside
// image_stamp_range_deliverable(), inverting the condition, or changing
// the trailing `return false` to `return true`, both go uncaught by the
// whole suite: nothing compiles that header (it needs
// rclcpp), so the only arm reaching it read its TEXT, and text arms see
// tokens rather than decisions. The wrapper there now owns the message
// and the `!`, and nothing else.
inline constexpr bool stamp_range_deliverable(uint64_t now_ns)
{
  return ros_stamp_encodable(now_ns, kStampHeadroomSeconds);
}

}  // namespace ros2_rtt_bench
