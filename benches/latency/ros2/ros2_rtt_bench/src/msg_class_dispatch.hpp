// msg_class_dispatch.hpp — the TYPE-CLASS axis (CER_BENCH_MSG ∈ {pod,
// image}) for ros2_rtt_bench: one MsgAdapter<Msg> trait that lets the
// {ping,pong,latency}_node templates carry BOTH message classes, plus
// the class-aware dispatch macro their main()s call.
//
// The two classes measure DIFFERENT cost models over the SAME wire
// bytes (the hypothesis this axis exists to test — see METHODOLOGY.md
// § "The type-class axis"):
//
//   pod    — fixed-size Pod<N> (ros2_rtt_msgs): "plain" layout, the
//            necessary condition for Publisher::can_loan_messages() on
//            every RMW that implements loans. The incumbent cells; the
//            cell name carries NO class token (pinned prefixes).
//   image  — sensor_msgs/msg/Image, THE canonical unbounded ROS 2
//            sensor type: a variable-length `data` vector makes the
//            type non-plain, so every RMW is structurally forced into
//            the full-serialize + delivery-memcpy fallback (no loan on
//            publish OR take, no FastDDS DataSharing — the run_bench.sh
//            image×{loan,zc} rejections spell the structural reasons).
//            ONE template instantiation covers every payload size —
//            the size is a runtime resize, which is itself the
//            type-class difference on display (Pod<N> needs the
//            10-type compile-time dispatch below it).
//
// Matched-quantity rule (METHODOLOGY § type-class axis): a pod cell at
// sweep point N carries N total message bytes (uint64 ts_ns +
// uint8[N-8]); an image cell at N carries N bytes in the unbounded
// `data` array, with the Image type's fixed header fields riding as a
// small constant overhead (~60 B CDR), negligible at the 256 KiB+
// sizes where the class hypothesis bites and disclosed at the small
// ones. Cross-class comparisons within one stack quote that rule; the
// two classes NEVER merge into one plotted series.
//
// Mode-A/G3 adaptations for the image class (fill exclusion):
//   - prepare() is called ONCE at node construction, outside the
//     measurement loop: it resizes `data` to N (paying the O(N)
//     zero-fill exactly where the pod path pays its rosidl
//     zero-fill — at prealloc, never in the timed window) and sets
//     the fixed header fields.
//   - Per iteration the ONLY work between stamp and publish is the
//     stamp write (header.stamp for image, ts_ns for pod). The
//     serialize + delivery copy that publish() then performs on the
//     N-byte vector IS the transport cost being measured — exactly
//     the receive-side-copy rule §3 already applies to the rclcpp
//     lane.
//   - The stamp rides header.stamp (builtin_interfaces/Time,
//     sec+nanosec), the field a real Image publisher stamps —
//     CLOCK_MONOTONIC ns round-trips exactly, THROUGH A CEILING the pod
//     class's uint64 ts_ns does not have: `sec` is an int32, so an
//     uptime PAST 2147483647 s (~68.05 years) narrows out of range and
//     the reader reconstructs a number that is not a latency (that exact
//     second still fits, which is why the guard demands headroom rather
//     than testing equality). The codec lives in stamp_codec.hpp and the run REFUSES
//     at start-up (below) when this host is within a day of that
//     ceiling — a number is the product here, so an unmeasurable run
// must fail, not report.

#pragma once

#include <cstddef>
#include <cstdint>
#include <cstdio>

#include <sensor_msgs/msg/image.hpp>

#include "common.hpp"
#include "pod_dispatch.hpp"
#include "stamp_codec.hpp"

namespace ros2_rtt_bench {

// bench_msg_class() lives in common.hpp, beside bench_qos_label(): the
// three pod-only binaries (composed, latency_rcl, pong_rcl) must validate
// CER_BENCH_MSG too, and they include common.hpp but deliberately NOT
// this header — pulling sensor_msgs/msg/image.hpp into a pod-only lane
// for one env accessor would be the wrong dependency.

// MsgAdapter<Msg> — the per-class seam the node templates write/read
// stamps through. The default (Pod<N>) specialization preserves the
// incumbent cells' behavior byte-for-byte: ts_ns is the stamp field
// (ping writes the send stamp there; pong forwards it verbatim and
// latency reads it back — the field carries one value per round trip,
// not one per node), prepare() is a no-op (rosidl's ctor already
// zero-filled at prealloc), payload_bytes == sizeof(Msg) (Pod<N> is
// exactly N bytes by construction — the property the .bin filename
// encodes).
template <typename Msg>
struct MsgAdapter {
  // This primary template IS the Pod<N> case: it declares kVariable =
  // false, no prepare(), and payload_bytes == sizeof(Msg). Nothing in
  // the template said so, so any future message with a `ts_ns` member
  // and a vector/string field would compile here, claim to be fixed-size,
  // get a meaningless sizeof label and KEEP LOANS ENABLED — the exact
  // mislabel the Image specialization exists to prevent. Assert the
  // claim the template makes about itself.
  static_assert(rosidl_generator_traits::has_fixed_size<Msg>::value &&
                  std::is_trivially_copyable<Msg>::value,
                "MsgAdapter's primary template claims kVariable=false, an "
                "empty prepare() and payload_bytes==sizeof(Msg) — true only "
                "for a plain, fixed-size type. A non-plain type needs its "
                "own specialization (see MsgAdapter<sensor_msgs::msg::Image>).");
  static constexpr bool kVariable = false;
  static void prepare(Msg &, size_t) {}
  static void write_stamp(Msg & m, uint64_t t) { m.ts_ns = t; }
  static uint64_t read_stamp(const Msg & m) { return m.ts_ns; }
  static size_t payload_bytes(size_t) { return sizeof(Msg); }
};

template <>
struct MsgAdapter<sensor_msgs::msg::Image> {
  static constexpr bool kVariable = true;

  // ONCE, at node construction (Mode-A prealloc — the image twin of the
  // pod path's rosidl zero-fill): size the unbounded array to the sweep
  // point and set the fixed fields the way a real Image publisher
  // would. Never called in the timed window.
  static void prepare(sensor_msgs::msg::Image & m, size_t n) {
    m.height = 1;
    m.width = static_cast<uint32_t>(n);
    m.step = static_cast<uint32_t>(n);
    m.is_bigendian = 0;
    m.encoding = "rt";
    m.data.resize(n);
  }

  // Both halves go through stamp_codec.hpp — the ROS-free codec the
  // hand oracle compiles against — so the arithmetic under test IS the
  // arithmetic that ships. The bare `static_cast<int32_t>(t / 1e9)`
  // these two lines used to hold is what reported.
  static void write_stamp(sensor_msgs::msg::Image & m, uint64_t t) {
    const RosStamp s = encode_ros_stamp(t);
    m.header.stamp.sec = s.sec;
    m.header.stamp.nanosec = s.nanosec;
  }

  static uint64_t read_stamp(const sensor_msgs::msg::Image & m) {
    return decode_ros_stamp(
      RosStamp{m.header.stamp.sec, m.header.stamp.nanosec});
  }

  // The .bin filename size label = the unbounded data array's length
  // (the matched quantity), not sizeof(Image) (a small vector-bearing
  // struct whose sizeof says nothing about the wire).
  static size_t payload_bytes(size_t env_n) { return env_n; }
};

// image_stamp_range_deliverable — the start-up refusal for the int32
// seconds ceiling described at the top of this header.
//
// Called from the image arm of the dispatch macro, so the three binaries
// that dispatch BY CLASS inherit it (the other three executables in this
// package refuse any non-`pod` class outright in their own main()s). The
// pong node does not merely forward a stamp — it calls
// write_stamp(out, read_stamp(in)), so it RE-ENCODES, and the guard is
// directly load-bearing for it too, not merely inherited.
//
// Prints and returns false rather than exiting, so the caller answers
// with the same exit 2 the sibling size refusal uses (`common.hpp`'s
// strict readers all agree on 2 = setup error).
inline bool image_stamp_range_deliverable() {
  // wall_ns() is CLOCK_MONOTONIC (CLOCK_UPTIME_RAW on macOS) — BOOT-
  // relative, which is what makes the ceiling ~68 years of UPTIME and
  // makes "reboot the host" a real remedy below. If wall_ns() ever became
  // realtime-based this guard would start refusing every run in January
  // 2038 while advising a reboot that cannot help, so the two must move
  // together.
  //
  // The DECISION is stamp_range_deliverable() in stamp_codec.hpp, which
  // the hand oracle compiles and pins on both sides. Nothing compiles
  // THIS header outside the container, so everything below the call is
  // message-writing only — deliberately, because a text arm can see that
  // a token appears and cannot see which way an `if` points.
  const uint64_t now = wall_ns();
  if (stamp_range_deliverable(now)) {
    return true;
  }
  std::fprintf(
    stderr,
    "CER_BENCH_MSG=image: this host's monotonic clock reads %llu s, which "
    "leaves less than %llu s under the %lld s ceiling that "
    "builtin_interfaces/Time::sec (an int32) can carry. The image class "
    "stamps header.stamp, so this run would narrow the seconds field and "
    "report a number that is not a latency — refusing instead. Reboot the "
    "host, or measure the pod class (CER_BENCH_MSG=pod), whose uint64 "
    "ts_ns has no such ceiling.\n",
    static_cast<unsigned long long>(now / kNanosPerSecond),
    static_cast<unsigned long long>(kStampHeadroomSeconds),
    static_cast<long long>(kRosTimeSecMax));
  return false;
}

// valid_sweep_size — the image arm has ONE template instantiation, so
// it cannot lean on the Pod switch to reject an off-sweep size; reject
// explicitly with the same message shape.
inline bool valid_sweep_size(size_t n) {
  switch (n) {
    case 64: case 256: case 1024: case 4096: case 16384:
    case 65536: case 262144: case 1048576: case 4194304: case 16777216:
      return true;
    default:
      return false;
  }
}

}  // namespace ros2_rtt_bench

// Class-aware dispatch: CER_BENCH_MSG=image → the single Image
// instantiation (size validated — it is a runtime resize there);
// pod (default) → the compile-time Pod<N> switch. Used by the rclcpp
// trio's main()s. The rcl loan-lane + composed-lane binaries keep the
// pod-only ROS2_RTT_BENCH_DISPATCH_BY_SIZE and REFUSE image via
// ros2_rtt_bench::bench_msg_class() guards in their main()s
// (run_bench.sh also rejects those combinations up front — unbounded
// types cannot loan, and the lanes are pod-only by enumeration).
#define ROS2_RTT_BENCH_DISPATCH_BY_CLASS(SIZE_VAR, RUNNER)                 \
  do {                                                                     \
    if (ros2_rtt_bench::bench_msg_class() == "image") {                    \
      if (!ros2_rtt_bench::image_stamp_range_deliverable()) {              \
        return 2;                                                          \
      }                                                                    \
      if (!ros2_rtt_bench::valid_sweep_size(SIZE_VAR)) {                   \
        std::fprintf(stderr, "unsupported CER_BENCH_PAYLOAD_SIZE=%zu — "   \
                     "valid sizes: 64,256,1024,4096,16384,65536,262144,"   \
                     "1048576,4194304,16777216\n", (size_t)(SIZE_VAR));    \
        return 2;                                                          \
      }                                                                    \
      return RUNNER<sensor_msgs::msg::Image>();                            \
    }                                                                      \
    ROS2_RTT_BENCH_DISPATCH_BY_SIZE(SIZE_VAR, RUNNER);                     \
  } while (0)
