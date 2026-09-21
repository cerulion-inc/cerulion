// common.hpp — shared utilities for ros2_rtt_bench {ping,pong,latency}_node.
//
// - wall_ns()         CLOCK_MONOTONIC nanoseconds, matches
//                     cerulion_core::clock::wall_ns so latency numbers
//                     are directly comparable to the Cerulion benches.
// - CpuDmaLock        RAII handle that holds /dev/cpu_dma_latency open
//                     with a 0i32 written. Linux-only; no-op on macOS.
// - env_str/size/bool env-var readers. env_size/env_bool are STRICT
//                     full-string parsers (matching native/src/lib.rs
//                     env_u64's panic discipline): unset → default;
//                     set-but-garbage → print var name + value and
//                     hard-exit(2). Never a silent catch-to-default —
//                     a typo'd knob measuring under the right label is
//                     worse than a failed cell.
// - bench_qos()       QoS axis: CER_BENCH_QOS ∈ {be1, rel10,
//                     stock} mapped to an rclcpp::QoS at node
//                     construction. Unknown values are a hard error
//                     (exit 2) — never a silent fallback to a default
//                     profile.
// - bench_pacing_*    CER_BENCH_PACING ∈ {quiescent, fixed100,
//                     backtoback}: one source tree, three pacing modes
//                     (replaces the old duplicated
//                     cerulion_round_trip{,_quiescent} pair). quiescent
//                     and fixed100 are the SAME mechanism in-node (a
//                     wall-timer kick at CER_BENCH_TARGET_RATE_HZ) —
//                     the variants differ only in WHICH rate the driver
//                     exports (per-size sensor rate vs the uniform
//                     100 Hz ladder rung).
// - raw_dump_samples  fwrite raw LE u64 nanosecond samples to a .bin
//                     file under CER_BENCH_RAW_DUMP_DIR. compile_csv.py
//                     post-processes these into per-percentile CSVs.
// - is_plain_check    log rosidl + std type-traits at startup so the
//                     sweep wrapper can grep for "Msg::is_plain: 0"
//                     and abort the cell (exit-11 smoke check).
//
// All inline / header-only so each node binary picks up a private copy
// without an extra translation unit.

#pragma once

#include <cerrno>
#include <chrono>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fcntl.h>
#include <string>
#include <thread>
#include <type_traits>
#include <unistd.h>
#include <vector>

#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/qos.hpp>
#include <rosidl_runtime_cpp/traits.hpp>

#if defined(__APPLE__)
#include <time.h>
#endif

namespace ros2_rtt_bench {

inline uint64_t wall_ns() {
#if defined(__APPLE__)
  return clock_gettime_nsec_np(CLOCK_UPTIME_RAW);
#else
  struct timespec ts;
  clock_gettime(CLOCK_MONOTONIC, &ts);
  return static_cast<uint64_t>(ts.tv_sec) * 1'000'000'000ULL +
         static_cast<uint64_t>(ts.tv_nsec);
#endif
}

class CpuDmaLock {
public:
  CpuDmaLock() : fd_(-1) {
#ifdef __linux__
    fd_ = ::open("/dev/cpu_dma_latency", O_WRONLY);
    if (fd_ < 0) {
      // In docker containers without --device /dev/cpu_dma_latency mounted
      // the device file simply doesn't exist. Soft-warn and continue —
      // measurements still run, p99 may degrade due to C-state exits.
      std::fprintf(
        stderr,
        "cpu_dma_lock: open(/dev/cpu_dma_latency) failed: %s — "
        "continuing without DMA lock (p99 may degrade)\n",
        std::strerror(errno));
      return;
    }
    int32_t latency_us = 0;
    auto n = ::write(fd_, &latency_us, sizeof(latency_us));
    if (n != static_cast<ssize_t>(sizeof(latency_us))) {
      std::fprintf(
        stderr, "cpu_dma_lock: write(/dev/cpu_dma_latency) failed: %s\n",
        std::strerror(errno));
      ::close(fd_);
      fd_ = -1;
    }
#endif
  }

  ~CpuDmaLock() {
#ifdef __linux__
    if (fd_ >= 0) {
      ::close(fd_);
    }
#endif
  }

  CpuDmaLock(const CpuDmaLock &) = delete;
  CpuDmaLock & operator=(const CpuDmaLock &) = delete;

private:
  int fd_;
};

inline std::string env_str(const char * name, const char * default_value) {
  const char * v = std::getenv(name);
  return v ? std::string(v) : std::string(default_value);
}

// env_size — STRICT full-string unsigned parse, matching
// native/src/lib.rs::env_u64's panic discipline: unset → default;
// set-but-garbage (empty, trailing junk, sign, overflow) → print the
// var name + value and hard-exit(2). The old try/catch(...)-to-default
// silently mapped a typo'd CER_BENCH_TARGET_SAMPLES to the default —
// a mislabeled measurement, worse than a failed cell.
inline size_t env_size(const char * name, size_t default_value) {
  const char * v = std::getenv(name);
  if (!v) {
    return default_value;
  }
  // Reject anything strtoull would silently tolerate: leading
  // whitespace, a leading '-' (wraps!), empty string. '+' is accepted
  // to match Rust's u64::from_str.
  const bool leading_ok = (v[0] >= '0' && v[0] <= '9') || v[0] == '+';
  errno = 0;
  char * end = nullptr;
  unsigned long long parsed = leading_ok ? std::strtoull(v, &end, 10) : 0;
  if (!leading_ok || end == v || *end != '\0' || errno == ERANGE) {
    std::fprintf(
      stderr, "%s must be a non-negative integer, got '%s'\n", name, v);
    std::exit(2);
  }
  return static_cast<size_t>(parsed);
}

// check_sample_budget — reject a sample budget that cannot be allocated.
//
// env_size accepts any non-negative integer up to SIZE_MAX, so a
// fat-fingered CER_BENCH_TARGET_SAMPLES/CER_BENCH_WARMUP pair wraps both
// `samples_.reserve(target + warmup)` and the `samples_.size() >= target +
// warmup` completion threshold: the run then either attempts an absurd
// allocation or "succeeds" instantly having dumped nothing. Both are worse
// than a refusal, so refuse — exit 2, the setup-error code every other
// strict reader in this header uses.
//
// The cap is a practical allocation limit, not a wrap check: 100e6 samples
// is 800 MB of u64, four orders of magnitude above the tail-resolved 55000
// the quiescent schedule pins at its widest. Callers pass the validated
// total straight to reserve(); once it has passed here, every later
// `target + warmup` in the node is provably wrap-free.
inline size_t check_sample_budget(
  const char * who, size_t target, size_t warmup)
{
  constexpr size_t kMaxTotalSamples = 100000000ULL;
  if (warmup > kMaxTotalSamples || target > kMaxTotalSamples - warmup) {
    std::fprintf(
      stderr,
      "%s: CER_BENCH_TARGET_SAMPLES=%zu + CER_BENCH_WARMUP=%zu exceeds the "
      "%zu-sample ceiling (or overflows size_t) — refusing to run a "
      "measurement whose capacity and completion threshold would wrap.\n",
      who, target, warmup, kMaxTotalSamples);
    std::exit(2);
  }
  return target + warmup;
}

// env_bool — STRICT token parse (same discipline as env_size): unset →
// default; {1,true,yes} → true; {0,false,no} → false; anything else
// (empty included) → print the var name + value and hard-exit(2). The
// old reader mapped any unrecognized string ("ture", "on") to false —
// a mistyped CER_BENCH_DISABLE_LOAN silently mislabels the `loaned`
// column.
inline bool env_bool(const char * name, bool default_value) {
  const char * v = std::getenv(name);
  if (!v) {
    return default_value;
  }
  std::string s(v);
  if (s == "1" || s == "true" || s == "yes") {
    return true;
  }
  if (s == "0" || s == "false" || s == "no") {
    return false;
  }
  std::fprintf(
    stderr, "%s must be one of 1/true/yes/0/false/no, got '%s'\n",
    name, s.c_str());
  std::exit(2);
}

// bench_qos_label — the validated CER_BENCH_QOS value ("be1" default).
// Exits loudly on anything else: a cell measured under an unlabeled /
// mistyped QoS profile would be a mislabeled result, which is worse
// than a failed cell.
inline const std::string & bench_qos_label() {
  static const std::string label = [] {
    std::string q = env_str("CER_BENCH_QOS", "be1");
    if (q != "be1" && q != "rel10" && q != "stock") {
      std::fprintf(
        stderr,
        "CER_BENCH_QOS must be 'be1', 'rel10' or 'stock', got '%s' — "
        "refusing to measure under an unlabeled QoS profile\n",
        q.c_str());
      std::exit(2);
    }
    return q;
  }();
  return label;
}

// bench_msg_class — validated CER_BENCH_MSG value ("pod" default).
// Same loud-or-die discipline as bench_qos_label(): a cell measured
// under a mistyped class label would be a mislabeled result.
inline const std::string & bench_msg_class() {
  static const std::string cls = [] {
    std::string m = env_str("CER_BENCH_MSG", "pod");
    if (m != "pod" && m != "image") {
      std::fprintf(
        stderr,
        "CER_BENCH_MSG must be 'pod' or 'image', got '%s' — refusing to "
        "measure under an unlabeled message class\n",
        m.c_str());
      std::exit(2);
    }
    return m;
  }();
  return cls;
}

// bench_qos — the QoS profile for EVERY publisher/subscription in the
// chain (ping, echo, kick, and the loaned-capability probe — uniform,
// so the `loaned` column reflects the cell's actual QoS).
//
//   be1 (default) — BEST_EFFORT / VOLATILE / KEEP_LAST(1). The May-2026
//     campaign pin: minimal-buffering QoS (depth-1 history, no
//     reliability ACK/repair machinery in the window). Do not
//     read be1 as "the combination
//     that activates each RMW's true zero-copy path"; that overclaims:
//     rmw_fastrtps loans gate on is_plain ONLY (reliability never
//     gates them in any era — PR ros2/rmw_fastrtps#568), DataSharing
//     is forced OFF by rmw_fastrtps defaults regardless of QoS (the
//     `zc` lane is where it engages), reliability is not in Fast DDS's
//     documented data-sharing constraint list, and on CycloneDDS
//     0.10.5 the docs say RELIABLE is REQUIRED for iceoryx exchange
//     while the code gates check only that reliability is PRESENT —
//     so what each QoS lane actually engages is MEASURED per lane
//     (verify_shm.sh's QoS-threaded probe + the per-row loaned=
//     column), never asserted from QoS alone.
//   rel10 — RELIABLE / VOLATILE / KEEP_LAST(10). The rmw
//     head-to-head pin (what MoveIt-class ROS 2 stacks actually run
//     under). Durability stays VOLATILE in both so reliability+depth
//     is the only axis that moves.
//   stock — rmw_qos_profile_default, requested the way an unconfigured
//     rclcpp user requests it: `rclcpp::QoS(rclcpp::KeepLast(10))`
//     with NO policy overrides — every non-history policy initializes
//     from rmw_qos_profile_default (RELIABLE / VOLATILE /
//     KEEP_LAST(10); ros2/rmw qos_profiles.h lines 51–62, cited in
//     memo.md §3). Numerically rel10-equivalent BY
//     CONSTRUCTION of the default profile; kept as its own label so the
//     stock / composed lanes' cell names say "the default was
//     requested", not "a pin happened to match it". Used by the
//     zero-config `stock` lane and the `composed` lane (memo §2/§3).
inline rclcpp::QoS bench_qos() {
  const std::string & q = bench_qos_label();
  if (q == "rel10") {
    rclcpp::QoS qos(rclcpp::KeepLast(10));
    qos.reliable().durability_volatile();
    return qos;
  }
  if (q == "stock") {
    // The default profile as rclcpp hands it out — no modifier calls,
    // so this cannot drift from rmw_qos_profile_default (only the
    // history depth is spelled, and 10 IS the profile's depth).
    return rclcpp::QoS(rclcpp::KeepLast(10));
  }
  rclcpp::QoS qos(rclcpp::KeepLast(1));
  qos.best_effort().durability_volatile();
  return qos;
}

// bench_node_options — NodeOptions for EVERY bench node: strips the
// default per-node chatter machinery (/rosout publisher, parameter
// services, /parameter_events publisher). None of it is part of the
// measured path for ANY rmw, and under rmw_cerulion removing it is
// LOAD-BEARING: every rclcpp node publishes /rosout and
// /parameter_events by default, rmw_cerulion provisions 2 publisher
// slots per topic, so the THIRD node of the cell aborts at
// construction with ExceedsMaxSupportedPublishers on 'rosout'
// (measured on box-x86 2026-08-12; that is a product limitation —
// this workaround is bench hygiene, not the fix).
inline rclcpp::NodeOptions bench_node_options() {
  rclcpp::NodeOptions opts;
  opts.enable_rosout(false);
  opts.start_parameter_services(false);
  opts.start_parameter_event_publisher(false);
  return opts;
}

// bench_pacing_backtoback — CER_BENCH_PACING ∈ {quiescent (default),
// fixed100, backtoback}. quiescent AND fixed100 = wall-timer kick at
// CER_BENCH_TARGET_RATE_HZ (queues drain between iterations) — the two
// paced variants are indistinguishable inside the node, because the
// rate itself always travels via CER_BENCH_TARGET_RATE_HZ (fixed100's
// uniform target / fallback-ladder rung is exported per invocation by
// bench.py::docker_args_for_cell); backtoback = kick published from
// the echo callback (saturation, the old cerulion_round_trip loop
// body). Unknown values are a hard error — a mislabeled pacing regime
// changes the measurement's meaning. (This
// gate is the FIFTH pacing consumer — bench.py, run_bench.sh,
// native/src/lib.rs and run_workspace.sh are the other four. A value
// they accept and this gate rejects makes this exit(2) kill every ROS 2
// node under it, which the in-container watchdog then reports as rc=13
// "cannot sustain" at every ladder rung. A new pacing value must be
// added HERE as well as the four driver-side lockstep sites.)
inline bool bench_pacing_backtoback() {
  static const bool backtoback = [] {
    std::string p = env_str("CER_BENCH_PACING", "quiescent");
    if (p != "quiescent" && p != "fixed100" && p != "backtoback") {
      std::fprintf(
        stderr,
        "CER_BENCH_PACING must be 'quiescent', 'fixed100' or "
        "'backtoback', got '%s'\n",
        p.c_str());
      std::exit(2);
    }
    return p == "backtoback";
  }();
  return backtoback;
}

// bench_readiness_is_probe: which readiness rule the latency node uses
// before it starts the measured window. Read ONCE, from the env the
// driver sets, so both latency binaries answer the same way.
//
// Every RMW here keeps the three-condition graph rule: a matched /kick
// subscriber, a matched /echo publisher, and count_subscribers("ping")
// above zero. The third is a ROS graph query, and rmw_cerulion answers
// graph queries from a registry that only holds the calling process's
// own endpoints (rmw_cerulion runtime.rs: cross process endpoint
// discovery is a follow up). The bench runs ping, pong and latency as
// three separate processes, so that query returns zero forever and the
// chain never bootstraps, even though the two matched counts both read
// one and the data path is fine.
//
// Under rmw_cerulion only, the third condition is therefore replaced by
// a readiness test made of data rather than of graph metadata: keep the
// two matched conditions, then publish bounded probe kicks and treat the
// chain as ready when the first echo comes back. An echo is proof the
// whole ping to pong to echo path carries frames, which is strictly what
// the graph query was standing in for. The probe kicks and the echoes
// they produce are warm up. They are never measured, and the ready line
// names the rule the run used so a reader never has to guess which of
// the two a row was gated by.
inline bool bench_readiness_is_probe() {
  static const bool probe = [] {
    const char * rmw = std::getenv("RMW_IMPLEMENTATION");
    return rmw != nullptr && std::strcmp(rmw, "rmw_cerulion") == 0;
  }();
  return probe;
}

// The label written into the ready line. One spelling, two call sites.
inline const char * bench_readiness_label() {
  return bench_readiness_is_probe() ? "matched+probe" : "matched+graph";
}

// raw_dump_samples — write fwrite-friendly raw nanosecond samples to a
// .bin file under CER_BENCH_RAW_DUMP_DIR, named
// "<CER_BENCH_RAW_NAME>_<size>.bin". compile_csv.py reads these post-hoc
// and emits the per-percentile summary CSVs. Returns the path written
// (empty if the env var is unset or fopen failed). Logs the path so
// the sweep wrapper can grep for "RAW_DUMP_DONE".
inline std::string raw_dump_samples(
    rclcpp::Logger logger,
    const std::vector<uint64_t> & samples,
    size_t payload_bytes)
{
  std::string raw_dir = env_str("CER_BENCH_RAW_DUMP_DIR", "");
  if (raw_dir.empty()) {
    RCLCPP_WARN(
      logger,
      "RAW_DUMP_SKIP CER_BENCH_RAW_DUMP_DIR not set; %zu samples discarded",
      samples.size());
    return {};
  }
  std::string raw_name = env_str("CER_BENCH_RAW_NAME", "samples");
  std::string path = raw_dir + "/" + raw_name + "_" +
    std::to_string(payload_bytes) + ".bin";
  std::FILE * fp = std::fopen(path.c_str(), "wb");
  if (!fp) {
    RCLCPP_ERROR(
      logger, "raw_dump: cannot open %s: %s",
      path.c_str(), std::strerror(errno));
    return {};
  }
  size_t w = std::fwrite(samples.data(), sizeof(uint64_t), samples.size(), fp);
  std::fclose(fp);
  if (w != samples.size()) {
    RCLCPP_ERROR(
      logger, "raw_dump: short write %zu of %zu samples to %s",
      w, samples.size(), path.c_str());
    return {};
  }
  RCLCPP_INFO(
    logger, "RAW_DUMP_DONE payload=%zu samples=%zu bytes=%zu path=%s",
    payload_bytes, samples.size(),
    samples.size() * sizeof(uint64_t), path.c_str());
  return path;
}

// check_rate_hz — reject a target rate whose wall period floors to zero.
//
// Every LIVE pacer in this tree derives its period as
// `1'000'000'000ULL / rate_hz`: latency_node's and composed_rtt_node's
// create_wall_timer, and latency_node_rcl's next_kick deadline. Integer
// division means a rate above 1 GHz gives a ZERO nanosecond period — the
// timer fires as fast as the executor allows (or, in the rcl node, the
// deadline is permanently already-due) and the cell is a saturation run,
// while the .rate sidecar, the CSV's achieved_rate_hz column and the plot
// annotation all carry the REQUESTED rate. A silently mislabeled figure is
// worse than a refusal, so refuse: exit 2, the setup-error code every other
// strict reader in this header uses.
//
// (The RateLimiter class below is NOT in that list: it has no call site in
// this tree — the live ROS 2 pacing is the timer/deadline path above. Its
// ctor is therefore left alone rather than given a guard nothing reaches.)
//
// void, where the sibling check_sample_budget RETURNS its validated value:
// that one is consumed in the same expression as the reserve() it protects,
// which is what makes the two impossible to diverge. This one cannot be —
// each node re-reads the rate inside a backtoback ternary in its member
// initializer, so the validation is a main()-level refusal and the callers
// gate it on the same pacing mode.
//
// 0 is NOT rejected here — it is the ungated backtoback value, and each
// main already refuses it separately when the pacing mode requires a rate.
// The native RateLimiter (native/src/lib.rs) and the workspace ping node
// refuse the same input at the same ceiling.
constexpr size_t MAX_RATE_HZ = 1000000000ULL;

inline void check_rate_hz(const char * who, size_t rate_hz)
{
  if (rate_hz > MAX_RATE_HZ) {
    std::fprintf(
      stderr,
      "%s: CER_BENCH_TARGET_RATE_HZ=%zu exceeds the %zu Hz ceiling — the "
      "wall period 1e9/rate floors to 0 ns, which paces nothing while the "
      "run keeps the requested rate as its label.\n",
      who, rate_hz, MAX_RATE_HZ);
    std::exit(2);
  }
}

// RateLimiter — pace iteration to a target Hz with hybrid sleep+busywait.
//
// Used only under CER_BENCH_PACING=quiescent (the default), where
// CER_BENCH_TARGET_RATE_HZ must be >0 (validated in main()). Under
// backtoback pacing the latency node kicks from its echo callback and
// this class is never consulted. The first wait_next() call locks in
// the t0_ epoch so bootstrap variability doesn't drift the schedule.
//
// Hybrid sleep+busywait: clock_nanosleep accuracy on Linux is ~50 µs which
// would dominate small-payload latency measurements. We sleep coarse
// (50 µs short of deadline) then busy-wait the rest for sub-µs accuracy.
// CPU cost: ~50 µs busy-wait per iteration ≈ 5% at 1 kHz, negligible at
// 10 Hz. Even with this, small-payload p50 inflates ~60 µs vs back-to-back
// because the bench process gets descheduled between iterations and pays
// a context-switch on each echo arrival — see METHODOLOGY.md "wake-up
// cost" section. That inflation is fundamental to the quiescent regime.
class RateLimiter {
public:
  explicit RateLimiter(size_t rate_hz) : rate_hz_(rate_hz), iter_(0) {}

  // Call after recording a sample, before publishing the next iteration.
  // First call captures t0_; subsequent calls sleep_until t0_ + period * iter.
  void wait_next() {
    if (iter_ == 0) {
      t0_ = std::chrono::steady_clock::now();
    }
    ++iter_;
    auto deadline = t0_ + std::chrono::nanoseconds(
      static_cast<int64_t>(1'000'000'000ULL * iter_ / rate_hz_));
    auto now = std::chrono::steady_clock::now();
    auto remaining = deadline - now;
    if (remaining > std::chrono::microseconds(100)) {
      std::this_thread::sleep_until(deadline - std::chrono::microseconds(50));
    }
    while (std::chrono::steady_clock::now() < deadline) { /* busy-wait */ }
  }

  size_t rate_hz() const { return rate_hz_; }

private:
  size_t rate_hz_;
  size_t iter_;
  std::chrono::steady_clock::time_point t0_;
};

// is_plain_check<Msg> — log type-traits at node startup.
// A non-plain Pod<N> means rmw can_loan returns false
// at the rmw layer regardless of QoS, and the loaned / CDR-memcpy
// assumption is wrong.
//
// Portable across Humble / Jazzy / Kilted: rosidl's `is_plain` trait is
// not public in Humble (only `has_fixed_size` and `has_bounded_size`
// are exposed in `rosidl_generator_traits`). For the Pod<N> structs in
// ros2_rtt_msgs the operative property is `is_trivially_copyable` AND
// `has_fixed_size` — that's what the rmw needs to enable loan/CDR-memcpy.
// "Msg::is_plain: 1" is logged iff both hold. The sweep wrapper's
// check_is_plain reads this line PER CLASS (CER_BENCH_MSG): a pod cell
// aborts on ": 0", an image cell on ": 1", and either aborts with exit
// 11 when the line is ABSENT on a run that otherwise succeeded — the
// class label is then unverified, not verified.
template <typename Msg>
inline void is_plain_check(rclcpp::Logger logger) {
  constexpr bool fixed = rosidl_generator_traits::has_fixed_size<Msg>::value;
  constexpr bool tcopy = std::is_trivially_copyable<Msg>::value;
  constexpr bool plain = fixed && tcopy;
  RCLCPP_INFO(
    logger,
    "Msg::is_plain: %d (has_fixed_size=%d, is_trivially_copyable=%d, sizeof=%zu)",
    static_cast<int>(plain),
    static_cast<int>(fixed),
    static_cast<int>(tcopy),
    sizeof(Msg));
}

}  // namespace ros2_rtt_bench
