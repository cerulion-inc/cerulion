// sample_gate_oracle.cpp — hand oracles for the latency sinks' stamp-pair
// gate.
//
// COMPILED, NOT RUN, for the same two reasons stamp_codec_oracle.cpp
// beside it is: `classify_stamp_pair` is `constexpr` integer comparison,
// so each expectation below is a `static_assert` the COMPILER evaluates
// (the build's exit code IS the verdict, and a failure names file, line
// and message), and that needs no ROS distro and no ability to execute a
// freshly built binary.
//
// It includes the SHIPPING header (../src/sample_gate.hpp), never a
// transcription of it. That the three sinks actually CALL it, and count
// and report each verdict, is a separate claim asserted over their own
// text by check_percentile_parity.py::check_sample_gate_accounting.
//
// Driver: check_percentile_parity.py::check_sample_gate_accounting, with
// `-std=c++17 -Wall -Wextra -Wpedantic -Werror -fsyntax-only`.

#include <cstdint>

#include "../src/sample_gate.hpp"

namespace {

using ros2_rtt_bench::classify_stamp_pair;
using ros2_rtt_bench::StampVerdict;

constexpr uint64_t kU64Max = UINT64_C(0xFFFFFFFFFFFFFFFF);
// A plausible CLOCK_MONOTONIC reading: ~11.6 days of uptime.
constexpr uint64_t kNow = UINT64_C(1'000'000'000'000'000);

// ---------------------------------------------------------------------
// USABLE — and only when the receive instant is STRICTLY later.
// ---------------------------------------------------------------------
static_assert(classify_stamp_pair(kNow - 1000, kNow) == StampVerdict::kUsable,
              "an ordinary round trip (1 us) is a usable sample");
static_assert(classify_stamp_pair(1, 2) == StampVerdict::kUsable,
              "the smallest possible positive round trip is usable: a "
              "one-nanosecond difference is a latency, not a drop");
static_assert(classify_stamp_pair(1, kU64Max) == StampVerdict::kUsable,
              "an absurdly long round trip is still arithmetically a "
              "latency — this gate declines UNMEASURABLE pairs, it is not "
              "an outlier filter, and silently dropping slow samples is "
              "how a bench flatters itself");

// ---------------------------------------------------------------------
// UNSTAMPED — send_ns == 0, whatever the receive instant is.
// ---------------------------------------------------------------------
static_assert(classify_stamp_pair(0, kNow) == StampVerdict::kUnstamped,
              "send_ns == 0 is an unstamped echo");
static_assert(classify_stamp_pair(0, 0) == StampVerdict::kUnstamped,
              "send_ns == 0 wins over now_ns <= send_ns when BOTH hold: "
              "an unstamped echo is a publisher wiring fault and must not "
              "be reported as clock behaviour");
static_assert(classify_stamp_pair(0, kU64Max) == StampVerdict::kUnstamped,
              "a zero stamp is unstamped however late the echo arrives");

// ---------------------------------------------------------------------
// NON-POSITIVE RTT — now_ns <= send_ns, the unsigned-wrap guard.
// ---------------------------------------------------------------------
static_assert(classify_stamp_pair(kNow, kNow) == StampVerdict::kNonPositiveRtt,
              "EQUAL instants: a duplicate stamp at the clock's "
              "resolution would record a zero round trip");
static_assert(classify_stamp_pair(kNow, kNow - 1) ==
                StampVerdict::kNonPositiveRtt,
              "one nanosecond of non-monotonicity: now_ns - send_ns is "
              "UNSIGNED and would wrap to ~1.8e19 ns, which is the whole "
              "reason this guard exists");
static_assert(classify_stamp_pair(kU64Max, 1) ==
                StampVerdict::kNonPositiveRtt,
              "the worst non-monotone pair is still just a drop, never a "
              "sample");
static_assert(classify_stamp_pair(1, 1) == StampVerdict::kNonPositiveRtt,
              "the smallest non-zero equal pair: the boundary is `<=`, so "
              "equality drops");
static_assert(classify_stamp_pair(2, 1) == StampVerdict::kNonPositiveRtt,
              "strictly earlier receive instant, at the smallest values "
              "that avoid the unstamped arm");

// ---------------------------------------------------------------------
// THE BOUNDARY, pinned on BOTH sides at one point — a `<` instead of a
// `<=` (or the reverse) changes exactly this pair and nothing else.
// ---------------------------------------------------------------------
static_assert(classify_stamp_pair(kNow, kNow + 1) == StampVerdict::kUsable,
              "send + 1ns: the first pair on the usable side");
static_assert(classify_stamp_pair(kNow, kNow) != StampVerdict::kUsable,
              "send exactly: the last pair on the dropped side — asserted "
              "as NOT usable as well as by verdict above, so a gate "
              "widened to `<` fails here whatever it renames the verdict");

// ---------------------------------------------------------------------
// TOTALITY. Every pair gets exactly one of the three verdicts, and the
// two drop verdicts are DISTINCT values — merging them back into one
// enum entry would make a single counter out of two conditions with
// different remedies, which is the shape this change exists to end.
// ---------------------------------------------------------------------
static_assert(StampVerdict::kUsable != StampVerdict::kUnstamped &&
                StampVerdict::kUsable != StampVerdict::kNonPositiveRtt &&
                StampVerdict::kUnstamped != StampVerdict::kNonPositiveRtt,
              "the three verdicts are distinct: one counter per condition");

constexpr bool exactly_one_verdict(uint64_t send_ns, uint64_t now_ns)
{
  const StampVerdict v = classify_stamp_pair(send_ns, now_ns);
  return v == StampVerdict::kUsable || v == StampVerdict::kUnstamped ||
         v == StampVerdict::kNonPositiveRtt;
}

static_assert(exactly_one_verdict(0, 0), "total at (0, 0)");
static_assert(exactly_one_verdict(0, kU64Max), "total at (0, max)");
static_assert(exactly_one_verdict(kU64Max, 0), "total at (max, 0)");
static_assert(exactly_one_verdict(kU64Max, kU64Max), "total at (max, max)");
static_assert(exactly_one_verdict(kNow, kNow + 1), "total at the boundary");

// The identity the DELIVERY line's readers lean on: every echo the sink
// processed is exactly one of kept, unstamped or non-positive, so
// received == samples + unstamped + non_positive_rtt holds by
// construction rather than by the call sites remembering to keep it.
constexpr bool partitions(uint64_t send_ns, uint64_t now_ns)
{
  const StampVerdict v = classify_stamp_pair(send_ns, now_ns);
  const int kept = (v == StampVerdict::kUsable) ? 1 : 0;
  const int uns = (v == StampVerdict::kUnstamped) ? 1 : 0;
  const int npr = (v == StampVerdict::kNonPositiveRtt) ? 1 : 0;
  return kept + uns + npr == 1;
}

static_assert(partitions(0, kNow), "one echo counts once (unstamped)");
static_assert(partitions(kNow, kNow), "one echo counts once (non-positive)");
static_assert(partitions(kNow - 1, kNow), "one echo counts once (kept)");

}  // namespace
