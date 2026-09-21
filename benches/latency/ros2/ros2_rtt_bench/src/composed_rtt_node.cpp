// composed_rtt_node.cpp — round-trip benchmark, COMPOSED lane (one
// process; the ROS 2 "fastest-common" usage pattern).
//
// Evidence base: memo.md §2 — composition
// (component containers) is mainstream ROS 2 (Nav2 ships composed
// bringup by default since Humble, nav2 PR #2750; Autoware composes 49
// launch files), while rclcpp intra-process comms is OFF by default
// even inside a container (`use_intra_process_comms_ {false}`,
// rclcpp node_options.hpp) and Nav2 only gained the opt-in in the
// Kilted→Lyrical cycle (PR #5804, off by default). This binary
// therefore measures BOTH shades of the composed pattern under one
// cell axis: CER_BENCH_IPC ∈ {on, off} → NodeOptions
// use_intra_process_comms. Cell names:
// {distro}_composed_ipc{on,off}_rclcpp_chrt{N}.
//
// Manual composition, not a component container: the three nodes
// (ping / pong / latency — same roles and topic wiring as the
// 3-process lane) are constructed in main() and added to ONE
// SingleThreadedExecutor. Same address space, same executor — the
// property composition buys — without the launch/container
// infrastructure, which sits outside the measured window either way.
//
// QoS: `stock` (rmw_qos_profile_default = RELIABLE / VOLATILE /
// KEEP_LAST(10) — see common.hpp::bench_qos). The composed lane models
// common usage, and common usage runs the default profile (memo §3).
// run_bench.sh enforces CER_BENCH_QOS=stock for this lane so the
// qos-less cell name cannot hide a varying axis.
//
// Bench discipline (identical to the 3-process lane where the pattern
// permits, deviations NAMED):
//   - bench_node_options() chatter stripping (rosout / parameter
//     services / parameter_events off) + use_intra_process_comms.
//   - stamp-last: the wall_ns() write is the LAST write before
//     publish; only the 8-byte ts_ns write sits between stamp and
//     publish (G3 invariant, both nodes).
//   - Mode-A (payload fill excluded): NO payload bytes are ever
//     written. DEVIATION from the 3-process lane's preallocate-once
//     shape, named: intra-process pub/sub transfers OWNERSHIP
//     (publish(unique_ptr) is the documented 0-copy gesture —
//     design.ros2.org intra-process article, memo §2), so a message
//     cannot be preallocated once and reused; each iteration allocates
//     a fresh Msg with rosidl's MessageInitialization::SKIP, which
//     skips the payload-array zero-fill. The O(N) fill stays OUT of
//     the timed window (the Mode-A property); the O(1)-ish per-message
//     heap allocation stays IN it, deliberately — allocation per
//     publish IS a structural cost of the intra-process ownership
//     model that every composed+IPC user pays. Both IPC shades use the
//     SAME unique_ptr gesture so the ipc{on,off} delta isolates
//     use_intra_process_comms alone.
//   - raw .bin dump via raw_dump_samples (same G1 measured-count
//     contract; run_bench.sh's sample-count gate applies unchanged).
//   - delivery counters: the SAME three "DELIVERY role=" lines as the
//     3-process lane (latency prints its own from report_delivery — at
//     finalize on a healthy cell, else after the spin; ping/pong print
//     after the executor spin ends), greppable by run_bench.sh.
//   - pacing-agnostic: CER_BENCH_PACING quiescent (wall kick timer at
//     CER_BENCH_TARGET_RATE_HZ) and backtoback (kick from the echo
//     callback) both implemented, same env contract as latency_node.
//   - NO loan paths: borrow_loaned_message callers are benchmarks and
//     vendor SDKs, not composed applications (memo §1), and this lane
//     models the pattern as written in the wild — plain typed publish
//     + ConstSharedPtr callbacks (which do not take ownership, so the
//     IPC-on path is the documented 0-copy promotion). The ready lines
//     print loaned=0 because the binary HAS no loan call site — a
//     fact, not a probe result.
//
// Bootstrap: unlike the 3-process lane (which can only observe /kick
// subscriber count from the latency process), everything is in-process
// here, so readiness is whole-chain: kick, ping AND echo each matched
// >= 1 subscription (inter- or intra-process count) before the first
// kick is published. Under stock RELIABLE QoS a pre-match publish
// would still be lost (VOLATILE durability), so the poll is
// load-bearing for backtoback pacing, where the bootstrap kick is the
// only driver.

#include <chrono>
#include <cstddef>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <memory>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/exceptions.hpp>
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <std_msgs/msg/empty.hpp>

#include "common.hpp"
#include "pod_dispatch.hpp"
#include "sample_gate.hpp"

namespace ros2_rtt_bench {

// composed_ipc_on — CER_BENCH_IPC ∈ {on, off}, REQUIRED. No default:
// the IPC shade is IN the cell name (ipcon / ipcoff), so an unset env
// would be an unlabeled axis — the mislabeling class this suite hard
// exits on everywhere else (see common.hpp env parsers).
inline bool composed_ipc_on() {
  static const bool on = [] {
    const char * v = std::getenv("CER_BENCH_IPC");
    if (!v) {
      std::fprintf(
        stderr,
        "composed_rtt_node: CER_BENCH_IPC must be set to 'on' or 'off' — "
        "the composed cell name carries the IPC shade, so there is no "
        "default to fall back to\n");
      std::exit(2);
    }
    std::string s(v);
    if (s == "on") {
      return true;
    }
    if (s == "off") {
      return false;
    }
    std::fprintf(
      stderr, "CER_BENCH_IPC must be 'on' or 'off', got '%s'\n", s.c_str());
    std::exit(2);
  }();
  return on;
}

inline const char * composed_ipc_label() {
  return composed_ipc_on() ? "on" : "off";
}

// bench_node_options() + the composed lane's one knob. NodeOptions is
// copyable; the chained setter mutates the temporary before the copy.
inline rclcpp::NodeOptions composed_node_options() {
  return bench_node_options().use_intra_process_comms(composed_ipc_on());
}

// Fresh outbound message per publish (the intra-process ownership
// model), with the payload zero-fill SKIPPED — see the Mode-A
// deviation note in the file header. The payload bytes are
// indeterminate and never read by the bench (uint8 arrays; same
// "the buffer holds whatever it holds" rule as the 3-process lane's
// loan path).
template <typename Msg>
inline std::unique_ptr<Msg> fresh_msg() {
  return std::make_unique<Msg>(rosidl_runtime_cpp::MessageInitialization::SKIP);
}

template <typename Msg>
class ComposedPingNode : public rclcpp::Node {
public:
  ComposedPingNode()
  : rclcpp::Node("ros2_rtt_bench_ping", composed_node_options())
  {
    is_plain_check<Msg>(this->get_logger());
    rclcpp::QoS qos = bench_qos();
    publisher_ = this->create_publisher<Msg>("ping", qos);
    kick_sub_ = this->create_subscription<std_msgs::msg::Empty>(
      "kick", qos,
      [this](std_msgs::msg::Empty::ConstSharedPtr) { publish_ping(); });
    RCLCPP_INFO(
      this->get_logger(),
      "composed ping ready (msg_size=%zu ipc=%s loaned=0 qos=%s)",
      sizeof(Msg), composed_ipc_label(), bench_qos_label().c_str());
  }

  size_t published_count() const { return published_; }
  size_t ping_matched() const {
    return publisher_->get_subscription_count() +
           publisher_->get_intra_process_subscription_count();
  }

private:
  void publish_ping() {
    ++published_;
    // G3 invariant: allocation happens BEFORE the stamp; the stamp is
    // the last write before publish.
    auto msg = fresh_msg<Msg>();
    msg->ts_ns = wall_ns();
    publisher_->publish(std::move(msg));
  }

  size_t published_{0};
  typename rclcpp::Publisher<Msg>::SharedPtr publisher_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr kick_sub_;
};

template <typename Msg>
class ComposedPongNode : public rclcpp::Node {
public:
  ComposedPongNode()
  : rclcpp::Node("ros2_rtt_bench_pong", composed_node_options())
  {
    is_plain_check<Msg>(this->get_logger());
    rclcpp::QoS qos = bench_qos();
    publisher_ = this->create_publisher<Msg>("echo", qos);
    sub_ = this->create_subscription<Msg>(
      "ping", qos,
      [this](typename Msg::ConstSharedPtr msg) { echo(msg); });
    RCLCPP_INFO(
      this->get_logger(),
      "composed pong ready (msg_size=%zu ipc=%s loaned=0 qos=%s)",
      sizeof(Msg), composed_ipc_label(), bench_qos_label().c_str());
  }

  size_t echoed_count() const { return echoed_; }
  size_t echo_matched() const {
    return publisher_->get_subscription_count() +
           publisher_->get_intra_process_subscription_count();
  }

private:
  void echo(const typename Msg::ConstSharedPtr & in) {
    ++echoed_;
    // G3 invariant: the ts_ns copy is the only work between receive
    // and publish (payload bytes are not copied in → out — transport
    // cost, not user memcpy cost, same rule as the 3-process pong).
    auto out = fresh_msg<Msg>();
    out->ts_ns = in->ts_ns;
    publisher_->publish(std::move(out));
  }

  size_t echoed_{0};
  typename rclcpp::Publisher<Msg>::SharedPtr publisher_;
  typename rclcpp::Subscription<Msg>::SharedPtr sub_;
};

template <typename Msg>
class ComposedLatencyNode : public rclcpp::Node {
public:
  ComposedLatencyNode()
  : rclcpp::Node("ros2_rtt_bench_latency", composed_node_options()),
    payload_size_(sizeof(Msg)),
    target_(env_size("CER_BENCH_TARGET_SAMPLES",
                     bench_pacing_backtoback() ? 10000 : 9000)),
    warmup_(env_size("CER_BENCH_WARMUP", 1000)),
    chrt_(env_bool("CER_BENCH_CHRT", false)),
    backtoback_(bench_pacing_backtoback()),
    // Back-to-back mode IGNORES the rate, so it must not PARSE it either:
    // env_size is strict and exits 2 on a non-numeric value, which would
    // turn an inherited stale CER_BENCH_TARGET_RATE_HZ into a refusal of a
    // perfectly valid saturation run. (Same rule as the other two latency
    // nodes.)
    rate_hz_(bench_pacing_backtoback()
               ? 0 : env_size("CER_BENCH_TARGET_RATE_HZ", 0))
  {
    is_plain_check<Msg>(this->get_logger());
    // Validated ONCE here, so every later `target_ + warmup_` in this
    // class is wrap-free (see common.hpp::check_sample_budget). Without
    // it an oversized env pair wraps the reserve capacity AND the
    // completion threshold: an "successful" run that dumps nothing, or an
    // absurd allocation, instead of the promised exit-2 refusal.
    samples_.reserve(check_sample_budget("composed_rtt_node", target_, warmup_));

    rclcpp::QoS qos = bench_qos();
    kick_pub_ = this->create_publisher<std_msgs::msg::Empty>("kick", qos);
    echo_sub_ = this->create_subscription<Msg>(
      "echo", qos,
      [this](typename Msg::ConstSharedPtr msg) { on_echo(msg); });

    RCLCPP_INFO(
      this->get_logger(),
      "composed latency ready (payload=%zu target=%zu warmup=%zu chrt=%s "
      "ipc=%s loaned=0 qos=%s pacing=%s rate=%zuHz)",
      payload_size_, target_, warmup_,
      chrt_ ? "1" : "0",
      composed_ipc_label(),
      bench_qos_label().c_str(),
      backtoback_ ? "backtoback" : "quiescent",
      rate_hz_);
  }

  // Quiescent pacing, started only AFTER the bootstrap kick. Created in
  // the constructor it raced the whole-chain bootstrap below, which calls
  // exec.spin_some() while it waits: on_tick() could publish kicks before
  // the explicit bootstrap kick, dropped under be1 and retained as a
  // RELIABLE burst under rel10. Decoupled from the round trip for the
  // usual reason — an in-callback rate wait blocks the executor, and here
  // it would block ALL THREE nodes, which share one executor thread.
  // rcl's missed-slot catch-up clamp applies unchanged (F1.6).
  // Returns false iff the context shut down before the timer could be
  // created; the caller then exits the way the bootstrap-timeout arm
  // does. main()'s rclcpp::ok() check is NOT enough on its own: it is
  // separated from this call by publish_kick(), so a SIGINT can land in
  // between — and rclcpp entity creation after shutdown THROWS rather
  // than returning, turning a Ctrl-C into an uncaught exception. The
  // re-check here narrows that window; it cannot close it (the signal can
  // arrive between this check and the call), so the creation itself is
  // guarded too. The catch RE-CHECKS rather than swallowing: a genuine
  // RCL error with the context still up is rethrown, so only the shutdown
  // race is absorbed.
  bool start_pacing() {
    if (backtoback_) {
      return true;
    }
    if (!rclcpp::ok()) {
      return false;
    }
    auto period = std::chrono::nanoseconds(1'000'000'000ULL / rate_hz_);
    try {
      kick_timer_ = this->create_wall_timer(period, [this]() { on_tick(); });
    } catch (const rclcpp::exceptions::RCLError &) {
      if (!rclcpp::ok()) {
        return false;
      }
      throw;
    }
    return true;
  }

  void publish_kick() {
    ++kicks_sent_;
    kick_pub_->publish(std::make_unique<std_msgs::msg::Empty>());
  }

  size_t kick_matched() const {
    return kick_pub_->get_subscription_count() +
           kick_pub_->get_intra_process_subscription_count();
  }

private:
  void on_tick() {
    if (done_) return;
    publish_kick();
  }

  void on_echo(const typename Msg::ConstSharedPtr & msg) {
    if (done_) {
      return;
    }
    ++received_;

    uint64_t send_ns = msg->ts_ns;
    uint64_t now_ns = wall_ns();
    switch (classify_stamp_pair(send_ns, now_ns)) {
      case StampVerdict::kUnstamped:
        ++unstamped_;
        if (unstamped_ == 1) {
          RCLCPP_WARN(
            this->get_logger(),
            "echo carried NO stamp (send_ns=0) at now_ns=%llu — the "
            "sample cannot be taken. Logged once; the running count is "
            "reported as unstamped= in the DELIVERY line (printed at "
            "finalize, or after the spin on a cell that never finalizes).",
            static_cast<unsigned long long>(now_ns));
        }
        return;
      case StampVerdict::kNonPositiveRtt:
        ++nonpositive_rtt_;
        if (nonpositive_rtt_ == 1) {
          RCLCPP_WARN(
            this->get_logger(),
            "echo's receive instant is not after its send instant "
            "(send_ns=%llu now_ns=%llu) — a duplicate or non-monotone "
            "stamp; now_ns - send_ns is unsigned and would wrap. Logged "
            "once; the running count is reported as nonpositive_rtt= in "
            "the DELIVERY line at finalize.",
            static_cast<unsigned long long>(send_ns),
            static_cast<unsigned long long>(now_ns));
        }
        return;
      case StampVerdict::kUsable:
        // INSIDE the arm — see latency_node.cpp: with the push after the
        // switch, a future verdict nobody added an arm for falls through
        // and records the unsigned subtraction this gate exists to
        // prevent (the colcon build has no `-Werror`, so `-Wswitch` is
        // only a warning).
        samples_.push_back(now_ns - send_ns);
        break;
    }

    size_t total_needed = target_ + warmup_;
    if (samples_.size() >= total_needed) {
      done_ = true;
      if (kick_timer_) {
        kick_timer_->cancel();
      }
      finalize();
      rclcpp::shutdown();
      return;
    }

    if (backtoback_) {
      publish_kick();
    }
  }

  void finalize() {
    if (samples_.size() <= warmup_) {
      RCLCPP_WARN(
        this->get_logger(),
        "finalize: only %zu samples (need > %zu warmup) — skipping",
        samples_.size(), warmup_);
      return;
    }
    samples_.erase(samples_.begin(), samples_.begin() + warmup_);
    // Chronological order preserved; compile_csv.py owns percentiles.
    raw_dump_samples(this->get_logger(), samples_, payload_size_);
    report_delivery();
  }

public:
  // The DELIVERY receipt, split OUT of finalize() and callable again
  // after the spin returns — at most once, whichever path gets there
  // first. See latency_node.cpp's copy for the reasoning: finalize()
  // runs only on a cell that collected its full sample population, and
  // on_echo ends the spin right after it — so in the all-unusable-stamps
  // shape the counters exist for, finalize() alone never prints the receipt.
  // ping and pong report after the spin too (below), so every
  // role leaves a receipt. Reaching this on a timed-out
  // cell also needs run_bench.sh's watchdog to SIGTERM before it
  // SIGKILLs, which it does and the parity checker pins.
  void report_delivery() {
    if (delivery_reported_) {
      return;
    }
    delivery_reported_ = true;
    // ...and the echoes that arrived but yielded no sample, one count
    // per reason (see sample_gate.hpp): received == samples + warmup +
    // unstamped + nonpositive_rtt, and without these counts the last two terms are
    // invisible.
    // `samples` there is the POST-warmup count the .bin carries, which is
    // why the warmup term is separate — finalize erases the prefix before
    // this runs. On the after-the-spin path samples_ is un-erased and the
    // identity is the oracle's two-term form instead.
    std::fprintf(
      stderr,
      "DELIVERY role=latency received=%zu kicks_sent=%zu unstamped=%zu "
      "nonpositive_rtt=%zu\n",
      received_, kicks_sent_, unstamped_, nonpositive_rtt_);
    std::fflush(stderr);
  }

private:
  size_t payload_size_;
  size_t target_;
  size_t warmup_;
  bool chrt_;
  bool backtoback_;
  size_t rate_hz_;
  bool done_{false};
  // At-most-once for the DELIVERY receipt: finalize() prints it on a
  // healthy cell, run_composed() prints it after the spin on a cell that
  // never finalized.
  bool delivery_reported_{false};
  size_t received_{0};
  size_t kicks_sent_{0};
  // Echoes the stamp gate declined, one counter per verdict (see
  // sample_gate.hpp). Logged once each, counted always, reported in
  // DELIVERY — the same discipline the loaned-take sink uses for its
  // take/return failures. Kept APART because the remedies differ: an
  // unstamped echo is a publisher wiring fault, a non-positive round
  // trip is clock behaviour.
  // The log-once WARN each of these arms fires sits ON the receive path,
  // so its first call — which pays one-off costs the later ones do not
  // (the first logger-level lookup, the first write to stderr; NOT the
  // logging stack's initialisation, which rclcpp::init already did in
  // main) — lands in the gap before the NEXT echo and can inflate that
  // one sample's measured round trip.
  //
  // CONSIDERED AND KEPT, with its real cost stated: it fires at most once
  // per condition per process, only on a cell that has already dropped a
  // sample, and the contamination direction is to OVER-report latency,
  // never to flatter it (the stall lands after this echo's now_ns is
  // read, inside a LATER echo's window). What it does NOT buy is a mark
  // on the published row: unstamped/nonpositive_rtt stop at the cell's
  // _delivery.txt and never reach the CSV, so on a cell with ONE sporadic
  // drop the spike can land in max_ns (or p99_9_ns at small n) with
  // nothing on the row saying a drop occurred. Deferring the log off the
  // receive path — or carrying the counts onto the row — would close
  // that; neither is implemented, which is stated here rather than left implicit.
  size_t unstamped_{0};
  size_t nonpositive_rtt_{0};
  std::vector<uint64_t> samples_;
  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr kick_pub_;
  typename rclcpp::Subscription<Msg>::SharedPtr echo_sub_;
  rclcpp::TimerBase::SharedPtr kick_timer_;
};

template <typename Msg>
int run_composed() {
  auto ping = std::make_shared<ComposedPingNode<Msg>>();
  auto pong = std::make_shared<ComposedPongNode<Msg>>();
  auto latency = std::make_shared<ComposedLatencyNode<Msg>>();

  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(latency);
  exec.add_node(pong);
  exec.add_node(ping);

  // Whole-chain bootstrap (see the file header): every hop matched
  // before the first kick. Counts sum inter- and intra-process
  // matches, so both IPC shades bootstrap through the same predicate.
  auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(15);
  while (rclcpp::ok() &&
         (latency->kick_matched() == 0 || ping->ping_matched() == 0 ||
          pong->echo_matched() == 0)) {
    if (std::chrono::steady_clock::now() > deadline) {
      RCLCPP_ERROR(
        latency->get_logger(),
        "bootstrap timeout: chain not fully matched within 15s "
        "(kick=%zu ping=%zu echo=%zu subscriptions)",
        latency->kick_matched(), ping->ping_matched(),
        pong->echo_matched());
      // The receipt BEFORE the exit. A cell can die after the node is
      // constructed and before the spin — a bootstrap-discovery timeout,
      // a SIGTERM landing mid-bootstrap (run_bench.sh's watchdog
      // guarantees TERM before KILL, not that the spin was ever
      // entered), or a pacing start that loses the same race — and every
      // one of those is a path on which the latency sink would otherwise vanish
      // without a word. The counts are zero there, and that is the point: an
      // explicit zero receipt says the sink came up and saw nothing,
      // which "no DELIVERY receipts at all" does not. Harmless to the
      // coherence gate, which only evaluates receipts on a cell whose
      // run_rc is 0.
      latency->report_delivery();
      return 2;
    }
    exec.spin_some();
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }

  // ONE explicit bootstrap kick, THEN normal pacing.
  // A SIGINT during the bootstrap wait breaks that loop through
  // !rclcpp::ok(), and entity creation after shutdown THROWS rather than
  // returning — so kicking and starting the timer here would turn a
  // Ctrl-C into an exception. Exit the way the timeout arm does.
  if (!rclcpp::ok()) {
    latency->report_delivery();
    return 2;
  }

  latency->publish_kick();
  if (!latency->start_pacing()) {
    // SIGINT landed between main's check and timer creation.
    latency->report_delivery();
    return 2;
  }
  exec.spin();  // returns when finalize() calls rclcpp::shutdown()

  // Delivery accounting (F1.4): all three roles report here, after the
  // spin ends — the same three lines, same strings, as the 3-process
  // lane, so run_bench.sh's "DELIVERY role=" grep and the _delivery.txt
  // artifacts are lane-agnostic. The latency line is a no-op when
  // finalize() already printed it; it is the ONLY print on a cell that
  // never finalized, which is the shape the stamp-gate counters exist
  // for (see latency_node.cpp's LatencyNode::report_delivery) — and it
  // is reached there only because run_bench.sh's watchdog SIGTERMs
  // before it SIGKILLs.
  latency->report_delivery();
  std::fprintf(
    stderr, "DELIVERY role=ping published=%zu\n", ping->published_count());
  std::fprintf(
    stderr, "DELIVERY role=pong echoed=%zu\n", pong->echoed_count());
  std::fflush(stderr);
  return 0;
}

}  // namespace ros2_rtt_bench

int main(int argc, char ** argv) {
  [[maybe_unused]] ros2_rtt_bench::CpuDmaLock dma_lock;

  rclcpp::init(argc, argv);
  // TYPE-CLASS guard (defense-in-depth beside run_bench.sh's up-front
  // rejection): this binary is pod-only — the image class is
  // structurally excluded from its lane (unbounded types cannot loan;
  // the usage lanes are pod-only by enumeration). Refuse loudly rather
  // than silently measure Pod<N> under an image-labeled cell.
  // bench_msg_class(), not a raw env_str compare: it VALIDATES the value
  // (and exits 2 naming the two legal classes), so a typo'd CER_BENCH_MSG
  // dies with "must be 'pod' or 'image'" instead of this lane's message,
  // which would send the reader looking for an image lane they did not
  // ask for. It also reads the env once instead of twice. Its refusal is
  // a bare std::exit(2), so on that one path rclcpp::shutdown() is not
  // called — reachable only on a DIRECT binary invocation with a typo'd
  // class (run_bench.sh validates and exports the value first), and the
  // process exits 2 either way.
  if (ros2_rtt_bench::bench_msg_class() != "pod") {
    std::fprintf(
      stderr,
      "%s: CER_BENCH_MSG='%s' is not runnable by this binary (pod-only "
      "lane) — image cells run the rclcpp trio only\n",
      argv[0], ros2_rtt_bench::bench_msg_class().c_str());
    rclcpp::shutdown();
    return 2;
  }
  size_t size = ros2_rtt_bench::env_size("CER_BENCH_PAYLOAD_SIZE", 64);

  // Validate the lane's env BEFORE constructing nodes (fail-fast,
  // matching latency_node.cpp's discipline): the IPC shade (also
  // forces the loud exit-2 on unset/garbage), the raw-dump
  // destination, and the quiescent rate.
  (void)ros2_rtt_bench::composed_ipc_on();

  if (ros2_rtt_bench::env_str("CER_BENCH_RAW_DUMP_DIR", "").empty() ||
      ros2_rtt_bench::env_str("CER_BENCH_RAW_NAME", "").empty()) {
    fprintf(stderr,
      "composed_rtt_node: CER_BENCH_RAW_DUMP_DIR and CER_BENCH_RAW_NAME "
      "must both be set — refusing to run a measurement whose samples "
      "would be discarded.\n");
    rclcpp::shutdown();
    return 2;
  }

  if (!ros2_rtt_bench::bench_pacing_backtoback() &&
      ros2_rtt_bench::env_size("CER_BENCH_TARGET_RATE_HZ", 0) == 0) {
    fprintf(stderr,
      "composed_rtt_node: CER_BENCH_TARGET_RATE_HZ must be set under "
      "paced CER_BENCH_PACING (quiescent/fixed100; typical robotics rates: 1000 @ small "
      "payload down to 10 @ 16 MB). Set CER_BENCH_PACING=backtoback for "
      "saturation pacing.\n");
    rclcpp::shutdown();
    return 2;
  }

  // ...and a rate whose wall period floors to ZERO is refused too: every
  // pacer here derives its period as `1'000'000'000ULL / rate_hz`, so a
  // rate above 1 GHz yields a 0 ns period, and
  // `create_wall_timer(0ns)` fires as fast as the executor allows.
  // the run becomes a saturation run wearing the requested rate as its
  // label — in the .rate sidecar, the CSV and the plot.
  //
  // Gated on the pacing mode for the same reason the class refuses to even
  // PARSE the rate under backtoback (see the member initializer): an
  // inherited stale CER_BENCH_TARGET_RATE_HZ must not refuse a saturation
  // run that never consults it.
  if (!ros2_rtt_bench::bench_pacing_backtoback()) {
    ros2_rtt_bench::check_rate_hz(
      "composed_rtt_node", ros2_rtt_bench::env_size("CER_BENCH_TARGET_RATE_HZ", 0));
  }

  int rc;
  ROS2_RTT_BENCH_DISPATCH_BY_SIZE(size, ros2_rtt_bench::run_composed);
  // unreachable — the macro ends every case with `return`
  rc = 2;
  rclcpp::shutdown();
  return rc;
}
