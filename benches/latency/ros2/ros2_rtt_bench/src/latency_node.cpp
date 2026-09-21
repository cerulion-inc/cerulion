// latency_node.cpp — round-trip benchmark, latency sink (process 3 of 3).
//
// ONE source tree, TWO pacing modes (CER_BENCH_PACING — replaces the
// old duplicated cerulion_round_trip{,_quiescent} sibling folders):
//
//   quiescent (default) — rate-limited at CER_BENCH_TARGET_RATE_HZ via
//     a wall-clock kick timer. Queues drain between iterations; this
//     measures latency in an unsaturated steady state at typical
//     robotics sensor rates. CER_BENCH_TARGET_RATE_HZ is REQUIRED.
//   backtoback — the next /kick is published from the echo callback
//     itself (saturation; the old cerulion_round_trip loop body).
//     CER_BENCH_TARGET_RATE_HZ is ignored.
//
// Subscribes to /echo, accumulates samples (now_ns - send_ns) read
// from the message's ts_ns field, and publishes /kick to sustain the
// chain. Once CER_BENCH_TARGET_SAMPLES + CER_BENCH_WARMUP samples are
// collected, drops the warmup prefix and writes the raw samples as
// a binary LE u64 array to a .bin file under CER_BENCH_RAW_DUMP_DIR
// for post-processing by compile_csv.py. CER_BENCH_RAW_DUMP_DIR and
// CER_BENCH_RAW_NAME are REQUIRED (fail-fast in main() — a run whose
// samples would be silently discarded must not start).
//
// The /echo message type is the same type ping/pong use — chosen by
// the type-class axis (CER_BENCH_MSG: Pod<N> templated over
// CER_BENCH_PAYLOAD_SIZE, or sensor_msgs/msg/Image — see
// msg_class_dispatch.hpp).
//
// Bootstrap: main() blocks until kick_pub_->get_subscription_count() > 0
// (ping_node has subscribed to /kick), then publishes one kick to start
// the chain.
//
// Why the quiescent kick timer lives OUT of the echo callback: an
// in-callback rate.wait_next() (1–100 ms hold) blocked the executor
// and dropped echoes at large payloads — under best_effort +
// KeepLast(1) a missed echo was unrecoverable and the bench hung.
// With a wall timer the kick rate is decoupled from the round-trip;
// a missed echo just shows up as a longer gap in the sample stream.

#include <chrono>
#include <cstddef>
#include <cstdio>
#include <memory>
#include <thread>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/exceptions.hpp>
#include <std_msgs/msg/empty.hpp>

#include "common.hpp"
#include "msg_class_dispatch.hpp"
#include "sample_gate.hpp"

namespace ros2_rtt_bench {

template <typename Msg>
class LatencyNode : public rclcpp::Node {
public:
  LatencyNode()
  : rclcpp::Node("ros2_rtt_bench_latency", bench_node_options()),
    // The .bin filename's size label: sizeof(Msg) for Pod<N> (exactly N
    // by construction), the sweep point itself for the image class
    // (sizeof(Image) is a small vector-bearing struct — meaningless as
    // a wire label). See MsgAdapter::payload_bytes.
    payload_size_(MsgAdapter<Msg>::payload_bytes(
      env_size("CER_BENCH_PAYLOAD_SIZE", 64))),
    target_(env_size("CER_BENCH_TARGET_SAMPLES",
                     bench_pacing_backtoback() ? 10000 : 9000)),
    warmup_(env_size("CER_BENCH_WARMUP", 1000)),
    chrt_(env_bool("CER_BENCH_CHRT", false)),
    backtoback_(bench_pacing_backtoback()),
    // Back-to-back mode IGNORES the rate (header contract), so it must not
    // PARSE it either: env_size is strict and exits 2 on a non-numeric
    // value, which would turn an inherited stale CER_BENCH_TARGET_RATE_HZ
    // into a refusal of a perfectly valid saturation run. Quiescent keeps
    // the strict parse — and main() still requires a non-zero rate there.
    rate_hz_(bench_pacing_backtoback()
               ? 0 : env_size("CER_BENCH_TARGET_RATE_HZ", 0))
  {
    is_plain_check<Msg>(this->get_logger());
    // Validated ONCE here, so every later `target_ + warmup_` in this
    // class is wrap-free (see common.hpp::check_sample_budget).
    samples_.reserve(check_sample_budget("latency_node", target_, warmup_));

    // QoS axis — see common.hpp::bench_qos (be1 = zero-copy
    // pin, rel10 = real-stack pin; uniform across the chain,
    // loaned-probe included, so the `loaned` column reflects the
    // cell's actual QoS).
    rclcpp::QoS qos = bench_qos();

    kick_pub_ = this->create_publisher<std_msgs::msg::Empty>("kick", qos);

    // Probe a Msg-typed publisher to determine can_loan_messages
    // for this RMW + transport. Drop after sampling — we don't actually
    // publish Msg from latency_node.
    {
      auto probe = this->create_publisher<Msg>("__loaned_probe_latency", qos);
      loaned_ = probe->can_loan_messages();
    }
    if (env_bool("CER_BENCH_DISABLE_LOAN", false)) {
      loaned_ = false;
    }
    if (MsgAdapter<Msg>::kVariable) {
      // TYPE-CLASS axis (CER_BENCH_MSG=image): the SAME override
      // ping_node and pong_node apply, and it belongs here too even
      // though this node never publishes Msg. `loaned=` is a CELL
      // label: all three nodes write it into one <cell>_<size>_node.log
      // and compile_csv's loaned_from_log takes the FIRST match, so an
      // RMW that answered can_loan_messages()=true for an unbounded
      // type would have this probe stamp loaned=1 on a cell whose
      // publishers force loans OFF — a mislabeled row decided by which
      // node happened to log first.
      loaned_ = false;
    }

    echo_sub_ = this->create_subscription<Msg>(
      "echo", qos,
      [this](typename Msg::ConstSharedPtr msg) { on_echo(msg); });

    RCLCPP_INFO(
      this->get_logger(),
      "latency_node ready (msg=%s payload=%zu target=%zu warmup=%zu chrt=%s "
      "loaned=%s qos=%s pacing=%s rate=%zuHz readiness=%s)",
      bench_msg_class().c_str(), payload_size_, target_, warmup_,
      chrt_ ? "1" : "0",
      loaned_ ? "1" : "0",
      bench_qos_label().c_str(),
      backtoback_ ? "backtoback" : "quiescent",
      rate_hz_,
      bench_readiness_label());
  }

  // Public so main() can poll get_subscription_count() during
  // bootstrap. Kick publishes go through publish_kick() so every kick
  // (timer, backtoback, bootstrap) is counted for delivery accounting.
  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr kick_pub_;

  size_t kick_subscriber_count() const {
    return kick_pub_ ? kick_pub_->get_subscription_count() : 0;
  }

  size_t echo_publisher_count() const {
    return echo_sub_ ? echo_sub_->get_publisher_count() : 0;
  }

  // Deliberately non-const (and so is chain_endpoints_discovered): rclcpp's graph
  // accessors have been const for a long time, but a const method here
  // would be a COMPILE error rather than a runtime one if that ever
  // changed, and this file is built only inside the ROS 2 image.
  size_t ping_subscriber_count() { return this->count_subscribers("ping"); }

  // Full-chain readiness for the bootstrap kick. Waiting only on the
  // /kick subscriber proves ping_node is up — NOT that pong_node has
  // matched, and under the best-effort/volatile lane a /ping sample
  // published before pong's subscription matches is simply DROPPED. In
  // back-to-back mode the chain is driven by echoes, so that one lost
  // sample means no echo, no further kick, and the cell hangs to its
  // timeout. Two more conditions observe pong from here — both are pure
  // GRAPH queries, so nothing extra is created on the measured topics:
  //   - echo_sub_->get_publisher_count(): pong's /echo writer matched us;
  //   - count_subscribers("ping"): pong's /ping reader EXISTS.
  // Scope, hence the name: the first two are real matches against
  // THIS node, the third is an existence query — the link this protects,
  // ping's /ping writer against pong's reader, is inferred from pong being
  // demonstrably up and already discovered by us, not proved. That makes
  // the race far less likely, not impossible.
  // A bootstrap RETRY is deliberately not used instead: in back-to-back a
  // second kick would start a SECOND concurrent chain and change the
  // saturation shape being measured.
  // The two conditions that are real MATCHES against this node. Both are
  // endpoint counts carried by the transport, not ROS graph metadata, so
  // they answer correctly across process boundaries on every RMW.
  bool endpoints_matched() {
    return kick_subscriber_count() > 0 &&
           echo_publisher_count() > 0;
  }

  bool chain_endpoints_discovered() {
    if (bench_readiness_is_probe()) {
      // rmw_cerulion: the third condition is a graph query this RMW
      // answers per process, so it can never pass here. The probe phase
      // in run_latency() supplies the proof instead, out of DATA. See
      // bench_readiness_is_probe() in common.hpp for the whole argument.
      return endpoints_matched();
    }
    return endpoints_matched() && ping_subscriber_count() > 0;
  }

  // --- probe readiness (rmw_cerulion only) ----------------------------
  // begin_probe() puts on_echo into a mode where an arriving echo proves
  // the chain and is then DROPPED: not stamped, not pushed, not counted
  // toward the measured window. It is warm up in the strict sense.
  void begin_probe() { probing_ = true; }
  void end_probe() { probing_ = false; }
  bool chain_live() const { return chain_live_; }
  size_t probe_kicks() const { return probe_kicks_; }
  size_t probe_echoes() const { return probe_echoes_; }

  void publish_probe_kick() {
    ++probe_kicks_;
    // Counted in kicks_sent_ as well, because the kick really was sent
    // and ping really will publish for it. run_bench.sh's receipt chain
    // asserts kicks_sent >= published, so a probe kick hidden from that
    // counter would read as ping publishing more than it was asked to.
    publish_kick();
  }

  // Quiescent pacing, started only AFTER the bootstrap kick — see the
  // bootstrap loop in run_latency(). Creating this timer in the
  // constructor raced the bootstrap: that loop calls spin_some() while
  // it waits for the chain to match, so on_tick() could publish kicks
  // BEFORE the explicit bootstrap kick as soon as /kick was discovered,
  // while the ping->pong->echo path was not matched yet. Those kicks are
  // dropped under be1, and under rel10 (RELIABLE/KEEP_LAST(10)) they are
  // retained and delivered as a burst the moment the peer matches —
  // corrupting both startup and the pacing the row is labelled with.
  // Creating it here also anchors the wall grid on the first kick rather
  // than on construction, i.e. on discovery time.
  //
  // Missed-slot policy (fix F1.6 parity): rcl timers jump
  // next_call_time ahead in WHOLE periods when a callback fires late
  // (rcl_timer_call's catch-up clamp), so a stalled executor never
  // produces a catch-up BURST — the same skip-missed-slots pacing
  // latency_node_rcl.cpp implements explicitly.
  //
  // rate_hz_ > 0 is validated in main() before the node is constructed;
  // backtoback drives itself from on_echo and starts no timer.
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
    std_msgs::msg::Empty kick;
    kick_pub_->publish(kick);
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

    if (probing_) {
      // Probe readiness: this echo is the proof the chain carries data,
      // and that is ALL it is used for. Returning here before the stamp
      // is read keeps it out of samples_ entirely, so no probe round
      // trip can reach the .bin or any percentile. received_ still
      // counts it: that field means echoes this node saw, and the
      // receipt chain in run_bench.sh reads it as an upper bound on
      // samples, which stays true.
      ++probe_echoes_;
      chain_live_ = true;
      return;
    }

    uint64_t send_ns = MsgAdapter<Msg>::read_stamp(*msg);
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
        // The push lives INSIDE the arm, not after the switch. The
        // switch carries no `default:` so `-Wswitch` names a call site
        // that has not grown an arm for a new verdict — but the colcon
        // build has no `-Werror` (CMakeLists: -Wall -Wextra -Wpedantic),
        // so that is a WARNING and the build succeeds. With the push
        // after the switch, an unhandled verdict falls straight through
        // and records `now_ns - send_ns` — the UNSIGNED subtraction this
        // gate exists to prevent — in the percentile array, under a
        // label that says latency. Inside the arm it records nothing.
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
      // Saturation: sustain the chain from the callback — the next
      // round-trip starts as soon as this one is recorded.
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
    // No sort + no inline percentile compute — preserve the chronological
    // sample order so HDR / latency-over-time analysis can run post-hoc
    // from the .bin. compile_csv.py owns the percentile derivation.
    raw_dump_samples(this->get_logger(), samples_, payload_size_);
    report_delivery();
  }

public:
  // The DELIVERY receipt, split OUT of finalize() and callable again
  // after the spin returns — at most once, whichever path gets there
  // first.
  //
  // WHY: finalize() runs only once `samples_.size() >= target_ +
  // warmup_`, and on_echo calls rclcpp::shutdown() immediately after it
  // to end the spin. So in the shape this receipt exists for — a
  // cell whose stamps are ALL unusable — the sink never finalizes, the
  // spin ends only when run_bench.sh's per-cell watchdog signals, and
  // without this call the counters die with the process: the operator
  // learns "at least one" from the log-once WARN and never learns "all
  // 10 000 of them".
  // ping_node and pong_node report after their spin ends the same way
  // (composed_rtt_node.cpp does it for both), so every role
  // leaves a receipt.
  //
  // A watchdog that sends this pid a straight SIGKILL defeats it:
  // SIGKILL is uncatchable — the spin never returns and this method
  // cannot run. The watchdog sends SIGTERM with a bounded grace first,
  // the same discipline it applies to ping and pong, and
  // check_percentile_parity.py::check_sample_gate_accounting pins that
  // ordering so the two halves cannot drift apart.
  //
  // run_bench.sh collects every `DELIVERY role=` line into the cell's
  // _delivery.txt and then reads each counter with `receipt_count`, whose
  // grep IS `-m1` — so a duplicate would be harmless to the gate. The
  // flag keeps the receipt file itself exact anyway.
  void report_delivery() {
    if (delivery_reported_) {
      return;
    }
    delivery_reported_ = true;
    // Delivery accounting: kicks_sent vs received
    // quantifies loss (a kick whose echo never returned — expected
    // under be1 at large payloads). run_bench.sh greps this into
    // _logs/<cell>_<size>_delivery.txt — REPORTED, never gated.
    // ...and the echoes that arrived but yielded no sample, one count
    // per reason (see sample_gate.hpp). Without them `received_` and the
    // sample count disagree with nothing to explain the gap: the
    // identity is received == samples + warmup + unstamped +
    // nonpositive_rtt, and without these counts the last two terms are invisible.
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
  bool loaned_{false};
  bool done_{false};
  // At-most-once for the DELIVERY receipt: finalize() prints it on a
  // healthy cell, run_latency() prints it after the spin on a cell that
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
  bool probing_{false};
  bool chain_live_{false};
  size_t probe_kicks_{0};
  size_t probe_echoes_{0};
  std::vector<uint64_t> samples_;
  typename rclcpp::Subscription<Msg>::SharedPtr echo_sub_;
  rclcpp::TimerBase::SharedPtr kick_timer_;
};

template <typename Msg>
int run_latency() {
  auto node = std::make_shared<LatencyNode<Msg>>();

  // Bootstrap: wait until the WHOLE ping -> pong -> echo path is matched
  // (see LatencyNode::chain_endpoints_discovered), then send one kick to start it.
  auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(15);
  while (rclcpp::ok() && !node->chain_endpoints_discovered()) {
    if (std::chrono::steady_clock::now() > deadline) {
      RCLCPP_ERROR(
        node->get_logger(),
        "bootstrap timeout: the ping->pong->echo endpoints were not all "
        "discovered within 15s (kick subscribers=%zu, echo publishers=%zu, ping "
        "subscribers=%zu) — are ping_node and pong_node up?",
        node->kick_subscriber_count(),
        node->echo_publisher_count(),
        node->ping_subscriber_count());
      // The receipt BEFORE the exit. A cell can die after the node is
      // constructed and before the spin — a bootstrap-discovery timeout,
      // a SIGTERM landing mid-bootstrap (run_bench.sh's watchdog
      // guarantees TERM before KILL, not that the spin was ever
      // entered), or a pacing start that loses the same race — and every
      // one of those is a path on which this sink would otherwise vanish
      // without a word. The counts are zero there, and that is the point: an
      // explicit zero receipt says the sink came up and saw nothing,
      // which "no DELIVERY receipts at all" does not. Harmless to the
      // coherence gate, which only evaluates receipts on a cell whose
      // run_rc is 0.
      node->report_delivery();
      return 2;
    }
    rclcpp::spin_some(node);
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }

  // Probe readiness (rmw_cerulion only): the graph query the third
  // condition used is unanswerable across processes on this RMW, so the
  // chain proves itself with DATA instead. Bounded on both axes: a
  // per-probe wait and a wall deadline of its own, so a genuinely dead
  // chain still fails in seconds with a message that says what it tried.
  if (bench_readiness_is_probe()) {
    node->begin_probe();
    const auto probe_deadline =
      std::chrono::steady_clock::now() + std::chrono::seconds(15);
    const auto per_probe = std::chrono::milliseconds(200);
    while (rclcpp::ok() && !node->chain_live()) {
      if (std::chrono::steady_clock::now() > probe_deadline) {
        RCLCPP_ERROR(
          node->get_logger(),
          "probe readiness timeout: both endpoint matches held (kick "
          "subscribers=%zu, echo publishers=%zu) but no echo came back "
          "after %zu probe kicks in 15s. The graph says the chain is "
          "wired and no data crossed it. ping_node or pong_node is up "
          "but not forwarding.",
          node->kick_subscriber_count(),
          node->echo_publisher_count(),
          node->probe_kicks());
        node->report_delivery();
        return 2;
      }
      node->publish_probe_kick();
      const auto wait_until = std::chrono::steady_clock::now() + per_probe;
      while (rclcpp::ok() && !node->chain_live() &&
             std::chrono::steady_clock::now() < wait_until) {
        rclcpp::spin_some(node);
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
      }
    }
    if (!rclcpp::ok()) {
      node->report_delivery();
      return 2;
    }
    // Settle while STILL in probe mode: an earlier probe whose echo was
    // slower than its own 200 ms window would otherwise land after the
    // flip and be measured. Probe round trips must not reach the data,
    // so the straggler is absorbed here and dropped like the rest.
    const auto settle =
      std::chrono::steady_clock::now() + std::chrono::milliseconds(50);
    while (rclcpp::ok() && std::chrono::steady_clock::now() < settle) {
      rclcpp::spin_some(node);
      std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
    node->end_probe();
    RCLCPP_INFO(
      node->get_logger(),
      "readiness=matched+probe: chain proven by data (%zu probe kicks, "
      "%zu probe echoes). Those round trips are warm up and are not "
      "measured; the measured window starts at the bootstrap kick below.",
      node->probe_kicks(), node->probe_echoes());
  }

  // ONE explicit bootstrap kick, THEN normal pacing — never a timer
  // running during the bootstrap spin_some above.
  // A SIGINT during the bootstrap wait breaks that loop through
  // !rclcpp::ok(), and entity creation after shutdown THROWS rather than
  // returning — so kicking and starting the timer here would turn a
  // Ctrl-C into an exception. Exit the way the timeout arm does.
  if (!rclcpp::ok()) {
    node->report_delivery();
    return 2;
  }

  node->publish_kick();
  if (!node->start_pacing()) {
    // SIGINT landed between main's check and timer creation.
    node->report_delivery();
    return 2;
  }

  rclcpp::spin(node);
  // The spin ends EITHER because on_echo called rclcpp::shutdown() after
  // finalize() — in which case the receipt is already out and this is a
  // no-op — or because a signal the process can ACT on reached it
  // without the cell ever finalizing, which is exactly the
  // all-unusable-stamps shape the counters exist for. "Can act on" is
  // load-bearing: under SIGKILL rclcpp's spin never returns and this
  // line cannot run, which is why run_bench.sh's per-cell watchdog now
  // sends SIGTERM with a bounded grace first.
  node->report_delivery();
  return 0;
}

}  // namespace ros2_rtt_bench

int main(int argc, char ** argv) {
  [[maybe_unused]] ros2_rtt_bench::CpuDmaLock dma_lock;

  rclcpp::init(argc, argv);
  size_t size = ros2_rtt_bench::env_size("CER_BENCH_PAYLOAD_SIZE", 64);

  // Fail-fast on a run whose samples could not land anywhere. The old
  // tree soft-warned and DISCARDED the samples — a full measurement
  // burned with nothing written (Principle: loud errors over silent
  // fallbacks).
  if (ros2_rtt_bench::env_str("CER_BENCH_RAW_DUMP_DIR", "").empty() ||
      ros2_rtt_bench::env_str("CER_BENCH_RAW_NAME", "").empty()) {
    fprintf(stderr,
      "latency_node: CER_BENCH_RAW_DUMP_DIR and CER_BENCH_RAW_NAME must "
      "both be set — refusing to run a measurement whose samples would "
      "be discarded.\n");
    rclcpp::shutdown();
    return 2;
  }

  // Quiescent pacing requires an explicit rate; backtoback ignores it.
  if (!ros2_rtt_bench::bench_pacing_backtoback() &&
      ros2_rtt_bench::env_size("CER_BENCH_TARGET_RATE_HZ", 0) == 0) {
    fprintf(stderr,
      "latency_node: CER_BENCH_TARGET_RATE_HZ must be set under "
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
      "latency_node", ros2_rtt_bench::env_size("CER_BENCH_TARGET_RATE_HZ", 0));
  }

  int rc;
  ROS2_RTT_BENCH_DISPATCH_BY_CLASS(size, ros2_rtt_bench::run_latency);
  // unreachable — the macro ends every case with `return`
  rc = 2;
  rclcpp::shutdown();
  return rc;
}
