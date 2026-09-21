// sample_gate.hpp — the decision every latency sink makes about one
// echo's (send, receive) stamp pair, kept deliberately ROS-FREE.
//
// WHY IT EXISTS. All three latency sinks — latency_node.cpp (rclcpp),
// latency_node_rcl.cpp (the loaned-take variant) and
// composed_rtt_node.cpp (single-process) — held the same two-condition
// guard inline:
//
//     if (send_ns == 0 || now_ns <= send_ns) { return; }
//
// and dropped the echo with no counter and no line. Both conditions are
// real and neither is a detail:
//
//   send_ns == 0   nothing stamped the message. The sink cannot tell a
//                  genuinely unstamped echo from one whose stamp was
//                  lost in the type conversion, and either way the
//                  sample does not exist.
//   now_ns <= send_ns
//                  the receive instant is not strictly after the send
//                  instant. `now_ns - send_ns` is UNSIGNED, so the
//                  equal case would record a zero round trip and the
//                  less-than case would WRAP to ~1.8e19 ns — which is
//                  why the guard is there, and why it must stay. A
//                  duplicate stamp at the clock's resolution and a
//                  non-monotone one land here together.
//
// A dropped echo is not neutral: it is a round trip the chain really
// completed and the cell will not count, so the run silently needs more
// echoes than it reports asking for, and a cell whose stamps are ALL
// unusable never finalizes at all — it rides the cell timeout with
// nothing said about why. The bench's product is a number, so a sample
// it declines to take is accounting, not noise.
//
// WHY A HEADER OF ITS OWN, and ROS-free. common.hpp pulls in rclcpp, so
// a desk without a ROS distro cannot compile a hand oracle against it.
// This holds the DECISION only — no clock read, no message type, no
// counter — as one `constexpr` function, so
// test/sample_gate_oracle.cpp can pin which way it points with a wall
// of `static_assert`s that the COMPILER evaluates and nothing executes.
// Same reasoning, and the same shape, as stamp_codec.hpp beside it.
//
// What this header does NOT own: the counting and the reporting. Those
// live at each call site (one counter per verdict, printed on that
// sink's `DELIVERY role=latency` line), and that they are really wired
// there is a separate claim asserted over the shipping text by
// check_percentile_parity.py::check_sample_gate_accounting — the real
// path needs a ROS 2 container to run.

#pragma once

#include <cstdint>

namespace ros2_rtt_bench {

// Why one echo's stamp pair did or did not yield a latency sample.
//
// A CLOSED set, and the sinks switch over it with NO `default:` label —
// so `-Wall`'s `-Wswitch` names every call site that has not grown a
// counter for a verdict added here. (A warning, not an error: the
// colcon build compiles with `-Wall -Wextra -Wpedantic` and no
// `-Werror`. The oracle's own build adds `-Werror`, but the oracle does
// not switch, so what it pins is the arithmetic, not this.)
enum class StampVerdict : uint8_t {
  // now_ns - send_ns is a latency.
  kUsable = 0,
  // send_ns == 0 — the echo carried no stamp at all.
  kUnstamped,
  // now_ns <= send_ns — the pair yields no POSITIVE round trip: equal
  // instants (a duplicate stamp at the clock's resolution) or a send
  // instant in the receiver's future (non-monotone across the pair).
  // Named for the arithmetic rather than for one of its two causes,
  // because the sink cannot tell them apart and must not claim to.
  kNonPositiveRtt,
};

// classify_stamp_pair — total, ROS-free, and the ONLY place the two
// drop conditions are spelled.
//
// ORDER IS PART OF THE ANSWER, for exactly ONE pair. The two conditions
// overlap only at `(send_ns == 0, now_ns == 0)`: the stamps are UNSIGNED,
// so an unstamped echo with any POSITIVE receive instant does not satisfy
// `now_ns <= send_ns` at all (measured — `0 == 0` holds while `5 <= 0`
// does not). Asking `send_ns == 0` first is what makes that one pair come
// back kUnstamped, so "nothing wrote a stamp" — a wiring fault, fixed in
// the publisher — is never attributed to clock behaviour, which is a
// different investigation. Narrow, and still load-bearing: it is the pair
// a receiver sees when an unstamped echo arrives before its own clock has
// left zero, and the oracle pins it directly
// (`classify_stamp_pair(0, 0) == kUnstamped`), which is also the single
// assertion the ORDER linkage control perturbs.
inline constexpr StampVerdict classify_stamp_pair(uint64_t send_ns,
                                                  uint64_t now_ns)
{
  if (send_ns == 0) {
    return StampVerdict::kUnstamped;
  }
  if (now_ns <= send_ns) {
    return StampVerdict::kNonPositiveRtt;
  }
  return StampVerdict::kUsable;
}

}  // namespace ros2_rtt_bench
