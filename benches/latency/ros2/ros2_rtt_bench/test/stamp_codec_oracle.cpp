// stamp_codec_oracle.cpp — hand oracles for the image class's
// builtin_interfaces/Time stamp codec.
//
// COMPILED, NOT RUN, and deliberately so. Every function in
// stamp_codec.hpp is `constexpr` integer arithmetic, so each expectation
// below is a `static_assert` the COMPILER evaluates: the build's exit
// code IS the verdict, and a failure names the file, the line and the
// message. Two things that buys over a runtime main():
//
//   - It needs no ROS distro AND no ability to execute a freshly built
//     binary, so it runs anywhere a compiler runs (including a CI lane
//     or a desk that only builds).
//   - Constant evaluation refuses undefined behaviour outright. Note
//     what that does NOT cover here: an out-of-range unsigned-to-signed
// conversion — the earlier bug — is IMPLEMENTATION-DEFINED, not
//     UB, and a constant expression evaluates it happily. The hand
//     VALUES below are what catch it; constexpr strictness is a second
//     net, not the first one.
//
// It includes the SHIPPING header (../src/stamp_codec.hpp), never a
// transcription of it — a second copy of the arithmetic is the drift
// this bench tree keeps paying for elsewhere. That the shipping
// MsgAdapter<Image> actually CALLS this codec is a separate claim, and
// a separate arm asserts it: check_percentile_parity.py::
// check_image_stamp_codec reads msg_class_dispatch.hpp's own text.
//
// Driver: check_percentile_parity.py::check_image_stamp_codec, with
// `-std=c++17 -Wall -Wextra -Wpedantic -Werror -fsyntax-only`.

#include <cstdint>

#include "../src/stamp_codec.hpp"

namespace {

using ros2_rtt_bench::decode_ros_stamp;
using ros2_rtt_bench::encode_ros_stamp;
using ros2_rtt_bench::kNanosPerSecond;
using ros2_rtt_bench::kRosTimeSecMax;
using ros2_rtt_bench::kStampHeadroomSeconds;
using ros2_rtt_bench::ros_stamp_encodable;
using ros2_rtt_bench::stamp_range_deliverable;
using ros2_rtt_bench::RosStamp;

constexpr bool stamp_is(RosStamp s, int32_t sec, uint32_t nanosec)
{
  return s.sec == sec && s.nanosec == nanosec;
}

// stamp_is gates all eight encode arms below, so a one-line `return true`
// here silences every one of them at once — with the file still carrying
// its full static_assert COUNT, which is what the driver's floor checks.
// Measured: the earlier narrowing encoder plus that edit compiles
// clean. This is the anti-tautology assert that makes the comparator
// itself falsifiable.
static_assert(!stamp_is(encode_ros_stamp(0), 1, 0),
              "stamp_is must be able to answer FALSE — a comparator that "
              "always agrees silences every encode oracle in this file");
static_assert(!stamp_is(encode_ros_stamp(1000000000ULL), 1, 1U),
              "stamp_is must compare the NANOSECOND field too");

// The last instant that fits: sec == INT32_MAX with a full nanosecond
// remainder. `kFirstOver` is one NANOSECOND past it — the first instant
// whose whole seconds exceed the ceiling.
constexpr uint64_t kLastOk =
  static_cast<uint64_t>(kRosTimeSecMax) * kNanosPerSecond + 999999999ULL;
constexpr uint64_t kFirstOver =
  (static_cast<uint64_t>(kRosTimeSecMax) + 1ULL) * kNanosPerSecond;
// A host exactly `kStampHeadroomSeconds` below the ceiling.
constexpr uint64_t kAtEdge =
  (static_cast<uint64_t>(kRosTimeSecMax) - kStampHeadroomSeconds) *
  kNanosPerSecond;

// ---- the constants the wire actually carries ---------------------------
// Drift guards. If either number moves, every boundary oracle below is
// measuring a different edge than the one it names.
static_assert(kRosTimeSecMax == 2147483647,
              "kRosTimeSecMax must be INT32_MAX — it is "
              "builtin_interfaces/Time::sec's ceiling, not a tunable");
static_assert(kStampHeadroomSeconds == 86400ULL,
              "kStampHeadroomSeconds must be one day: a bench cell is "
              "minutes, so a day means no run that passes the start-up "
              "check can cross the ceiling while it is running");
static_assert(kNanosPerSecond == 1000000000ULL, "kNanosPerSecond");

// ---- encode: hand-written expectations ---------------------------------
static_assert(stamp_is(encode_ros_stamp(0), 0, 0), "encode(0) == {0, 0}");
static_assert(stamp_is(encode_ros_stamp(999999999ULL), 0, 999999999U),
              "encode(999999999 ns) stays in second 0");
static_assert(stamp_is(encode_ros_stamp(1000000000ULL), 1, 0),
              "encode(1 s exactly) == {1, 0}");
static_assert(stamp_is(encode_ros_stamp(1500000000ULL), 1, 500000000U),
              "encode(1.5 s) == {1, 500000000}");
// A plausible real host: 11 days 13 h 46 m 40 s of uptime.
static_assert(
  stamp_is(encode_ros_stamp(1000000000000000ULL + 123456789ULL),
           1000000, 123456789U),
  "encode(1000000 s + 123456789 ns) == {1000000, 123456789}");
static_assert(stamp_is(encode_ros_stamp(kLastOk), 2147483647, 999999999U),
              "encode(the last encodable instant) == {INT32_MAX, 999999999}");

// THE regression. The earlier encoder wrote
// `static_cast<int32_t>(t / 1e9)` here, an out-of-range conversion whose
// result the reader then sign-extended, so the cell reported a number
// that was not a latency. This encoder SATURATES instead: the value is
// defined (and frozen, hence obviously wrong at the very first sample)
// rather than implementation-defined. The start-up refusal in
// msg_class_dispatch.hpp is what keeps this arm unreachable in a real
// run; these two pin that the fallback is not a wrap.
static_assert(stamp_is(encode_ros_stamp(kFirstOver), 2147483647, 0U),
              "encode(one second past the ceiling) must SATURATE at "
              "INT32_MAX, never wrap to a negative second");
static_assert(
  stamp_is(encode_ros_stamp(kFirstOver * 3ULL + 500000000ULL),
           2147483647, 500000000U),
  "encode(far past the ceiling) saturates too, and keeps the real "
  "nanosecond remainder");

// ---- decode: hand-written expectations ---------------------------------
static_assert(decode_ros_stamp(RosStamp{0, 0}) == 0ULL, "decode({0, 0})");
static_assert(decode_ros_stamp(RosStamp{1, 500000000U}) == 1500000000ULL,
              "decode({1, 500000000}) == 1.5e9 ns");
static_assert(
  decode_ros_stamp(RosStamp{2147483647, 999999999U}) == 2147483647999999999ULL,
  "decode({INT32_MAX, 999999999}) == 2147483647999999999 ns");

// The largest instant there is, so the saturating arm is pinned at its own
// extreme and not only one second past the edge.
static_assert(stamp_is(encode_ros_stamp(UINT64_MAX), 2147483647, 709551615U),
              "encode(UINT64_MAX) saturates the seconds and keeps the real "
              "nanosecond remainder (18446744073709551615 % 1e9)");

// A NEGATIVE `sec` is not something encode_ros_stamp can mint, and
// stamp_codec.hpp says decode deliberately has no rescue arm for one: it
// reproduces what the field says. Pinned so that "fixing" decode into a
// clamp — which would quietly change what a corrupt frame reports — is a
// deliberate act rather than a tidy-up.
static_assert(decode_ros_stamp(RosStamp{-1, 0}) ==
                static_cast<uint64_t>(-static_cast<int64_t>(kNanosPerSecond)),
              "decode({-1, 0}) reproduces the field, it does not clamp");
static_assert(decode_ros_stamp(RosStamp{-1, 999999999U}) ==
                static_cast<uint64_t>(-1),
              "decode({-1, 999999999}) reproduces the field, it does not "
              "clamp");

// The other value RosStamp can hold that encode never mints: a `nanosec`
// at or above 1e9. The header says read_stamp builds one straight off the
// wire, so the arithmetic must be defined for it — it simply carries.
static_assert(decode_ros_stamp(RosStamp{1, 1000000000U}) == 2000000000ULL,
              "decode({1, 1e9}) carries into the seconds rather than "
              "saturating or wrapping");
// ...and the largest-magnitude negative second, not just -1.
static_assert(decode_ros_stamp(RosStamp{-2147483648, 0}) ==
                static_cast<uint64_t>(
                  static_cast<int64_t>(-2147483648LL) *
                  static_cast<int64_t>(kNanosPerSecond)),
              "decode({INT32_MIN, 0}) reproduces the field without "
              "overflowing the int64 intermediate");

// ---- round trip, anchored on the hand oracles above --------------------
static_assert(decode_ros_stamp(encode_ros_stamp(0)) == 0ULL, "round trip 0");
static_assert(decode_ros_stamp(encode_ros_stamp(1ULL)) == 1ULL,
              "round trip 1 ns");
static_assert(decode_ros_stamp(encode_ros_stamp(999999999ULL)) == 999999999ULL,
              "round trip 999999999 ns");
static_assert(
  decode_ros_stamp(encode_ros_stamp(1500000000ULL)) == 1500000000ULL,
  "round trip 1.5 s");
static_assert(
  decode_ros_stamp(encode_ros_stamp(1000000000000000ULL + 123456789ULL)) ==
    1000000000000000ULL + 123456789ULL,
  "round trip 1000000 s + 123456789 ns");
static_assert(decode_ros_stamp(encode_ros_stamp(kLastOk)) == kLastOk,
              "round trip the last encodable instant");

// ---- the question the start-up refusal asks ----------------------------
static_assert(ros_stamp_encodable(0, kStampHeadroomSeconds),
              "a fresh host is encodable with a day of headroom");
static_assert(ros_stamp_encodable(kLastOk, 0),
              "the last encodable instant needs zero headroom");
static_assert(!ros_stamp_encodable(kFirstOver, 0),
              "one second past the ceiling is refused even at zero headroom");

// The headroom edge, pinned on BOTH sides at one second: a host exactly
// `kStampHeadroomSeconds` below the ceiling still runs, one second
// nearer does not.
static_assert(ros_stamp_encodable(kAtEdge, kStampHeadroomSeconds),
              "exactly one day of headroom left: accepted");
static_assert(
  !ros_stamp_encodable(kAtEdge + kNanosPerSecond, kStampHeadroomSeconds),
  "one second less than a day of headroom: refused");
// The nanosecond remainder must not buy a second: the guard reads whole
// seconds, so the same second stamped 999999999 ns in is still accepted.
static_assert(
  ros_stamp_encodable(kAtEdge + 999999999ULL, kStampHeadroomSeconds),
  "999999999 ns into the edge second: still accepted");

// ---- the decision the start-up refusal makes ---------------------------
// Its POLARITY, pinned here rather than by a text arm: while this lived
// as an `if` in the (rclcpp-only, never-compiled) dispatch header, both
// inverting it and turning its `return false` into `return true` survived
// the entire suite.
static_assert(stamp_range_deliverable(0),
              "a freshly booted host is measurable");
static_assert(stamp_range_deliverable(kAtEdge),
              "exactly one day of headroom left: still measurable");
static_assert(!stamp_range_deliverable(kAtEdge + kNanosPerSecond),
              "one second less than a day of headroom: REFUSED");
static_assert(!stamp_range_deliverable(kLastOk),
              "the last encodable instant has no headroom left at all, so "
              "a run started there could cross the ceiling while running");

// An absurd headroom must REFUSE, not wrap `limit - headroom_s` into a
// huge number and accept everything.
static_assert(
  !ros_stamp_encodable(0, static_cast<uint64_t>(kRosTimeSecMax) + 1ULL),
  "a headroom above the ceiling refuses instead of wrapping the "
  "subtraction into an accept-everything");
static_assert(
  ros_stamp_encodable(0, static_cast<uint64_t>(kRosTimeSecMax)),
  "a headroom exactly at the ceiling still accepts t=0 — the wrap guard "
  "must not cost the boundary itself");

}  // namespace
