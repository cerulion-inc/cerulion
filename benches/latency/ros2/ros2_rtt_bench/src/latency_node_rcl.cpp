// latency_node_rcl.cpp — latency sink with rcl-direct receive.
//
// Same shape as latency_node.cpp; the /echo subscription is taken via
// rcl_take_loaned_message + rcl_return_loaned_message_from_subscription
// instead of rclcpp::Subscription's typed callback (which memcpy's the
// inbound payload from the RMW buffer into a heap Msg before the
// callback fires). This is the `loan` recv lane of the cell matrix.
//
// The wait loop is driven by `rclcpp::WaitSet` rather than raw
// `rcl_wait_set_t` for cleaner shutdown + type-safety (the `performance_test`
// performance_test convention). Same `rcl_take_loaned_message`
// underneath.
//
// Pacing (CER_BENCH_PACING, see latency_node.cpp for the full story):
//   quiescent (default) — kicks driven off a wall-clock deadline at
//     CER_BENCH_TARGET_RATE_HZ (required), decoupled from echo arrival.
//   backtoback — the next kick is published right after each recorded
//     sample (saturation).
//
// /kick is published via the rclcpp Publisher unchanged (Empty msg, tiny).
//
// Raw samples land in CER_BENCH_RAW_DUMP_DIR/<CER_BENCH_RAW_NAME>_<size>.bin
// for compile_csv.py to post-process. No inline percentile compute,
// no inline CSV write. Both env vars are REQUIRED (fail-fast in main()).

#include <sched.h>
#include <chrono>
#include <cstddef>
#include <cstdio>
#include <cstring>
#include <memory>
#include <thread>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/exceptions.hpp>
#include <rclcpp/wait_set.hpp>
#include <rcl/error_handling.h>  // rcl_get_error_string / rcl_reset_error
#include <rcl/subscription.h>
#include <rmw/rmw.h>  // rmw_get_implementation_identifier
#include <rmw/types.h>
#include <std_msgs/msg/empty.hpp>

#include "common.hpp"
#include "pod_dispatch.hpp"
#include "sample_gate.hpp"

namespace ros2_rtt_bench {

template <typename Msg>
class LatencyNodeRcl : public rclcpp::Node {
public:
  LatencyNodeRcl()
  : rclcpp::Node("ros2_rtt_bench_latency_rcl", bench_node_options()),
    payload_size_(sizeof(Msg)),
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
    samples_.reserve(check_sample_budget("latency_node_rcl", target_, warmup_));

    // QoS axis — see common.hpp::bench_qos. Uniform across
    // the chain, loaned-probe included.
    rclcpp::QoS qos = bench_qos();

    kick_pub_ = this->create_publisher<std_msgs::msg::Empty>("kick", qos);

    // Probe loaned-publish capability. We don't use this on the kick
    // (Empty msg), but record it for the log.
    {
      auto probe = this->create_publisher<Msg>("__loaned_probe_latency_rcl", qos);
      loaned_ = probe->can_loan_messages();
    }
    if (env_bool("CER_BENCH_DISABLE_LOAN", false)) {
      loaned_ = false;
    }

    sub_ = this->create_subscription<Msg>(
      "echo", qos,
      [](typename Msg::ConstSharedPtr) { /* never called */ });

    rcl_sub_ = sub_->get_subscription_handle().get();

    // rmw_take_loaned_message is a NO-OP stub on rmw_zenoh (every
    // released distro — re-verify on new rmw_zenoh versions). It
    // returns RMW_RET_UNSUPPORTED and rcutils repeatedly truncates the
    // resulting error string at the 240 B buffer, generating multi-GiB
    // log spew if the bench loops. The capability is exposed via
    // rclcpp::Subscription::can_loan_messages(). Probe at startup and
    // bail cleanly if false. Upstream: ros2/rmw_zenoh#175 and #893.
    //
    // Audit fix C4 — what this bit actually reads: rcl gates
    // SUBSCRIPTION-side loans OFF by default (rclcpp#2335 / rcl#1110,
    // Nov 2023, backported to Humble), so can_loan_messages() is
    // `!rcl_gate && rmw_capability` — with ROS_DISABLE_LOANED_MESSAGES
    // unset it read FALSE on every RMW even where
    // rcl_take_loaned_message (which bypasses the rcl gate) takes
    // loans fine, making the `rcl_take_loaned_recv=` line contradict
    // what this lane measured on rmw_fastrtps. The harness exports
    // ROS_DISABLE_LOANED_MESSAGES=0 on this lane (run_bench.sh) so
    // the published capability line reports the RMW's real capability.
    loan_recv_supported_ = sub_->can_loan_messages();
    // Diagnostic only: which core each take ran on, one per sample.
    cpu_probe_ = env_bool("CER_BENCH_LAT_CPU_PROBE", false);

    RCLCPP_INFO(
      this->get_logger(),
      "latency_node_rcl ready (payload=%zu target=%zu warmup=%zu chrt=%s "
      "loaned=%s qos=%s pacing=%s rcl_take_loaned_recv=%s readiness=%s)",
      payload_size_, target_, warmup_,
      chrt_ ? "1" : "0",
      loaned_ ? "1" : "0",
      bench_qos_label().c_str(),
      backtoback_ ? "backtoback" : "quiescent",
      loan_recv_supported_ ? "1" : "0",
      bench_readiness_label());

    if (!loan_recv_supported_) {
      // Grep-friendly sentinel. Include the active RMW so the runner
      // can distinguish a structural skip (rmw_zenoh ships a NO-OP
      // stub on every distro) from a transient "iox-roudi not up yet"
      // case on cyclonedds (which should still retry so RouDi has
      // time to settle). NOTE (pitfall-11 caveat, re-scoped by C4):
      // the "reads false even when rcl_take_loaned_message works"
      // observation was made with ROS_DISABLE_LOANED_MESSAGES unset —
      // i.e. the rcl env gate forcing false on EVERY rmw, not a
      // cyclonedds quirk. With =0 exported on this lane the bit
      // reflects the rmw; whether cyclonedds still reports a
      // conservative false on some builds (0.10 without DDS_HAS_SHM)
      // needs a re-check on the target machine. Only rmw_zenoh / rmw_cerulion
      // short-circuit on this probe.
      const char * rmw = std::getenv("RMW_IMPLEMENTATION");
      const char * loan_env = std::getenv("ROS_DISABLE_LOANED_MESSAGES");
      fprintf(stderr,
        "RMW_LOAN_RECV_UNSUPPORTED rmw=%s "
        "rclcpp::Subscription::can_loan_messages() returned false "
        "(ROS_DISABLE_LOANED_MESSAGES=%s; rcl gates sub-side loans OFF "
        "unless it is '0' — rclcpp#2335 / rcl#1110 — and run_bench.sh "
        "exports =0 on the loan lane, so on this lane false means the "
        "RMW itself). rmw_zenoh: see ros2/rmw_zenoh#175 #893.\n",
        rmw ? rmw : "<unset>",
        loan_env ? loan_env : "<unset>");
      fflush(stderr);
    }
  }

  bool loan_recv_supported() const { return loan_recv_supported_; }

  size_t kick_subscriber_count() const {
    return kick_pub_ ? kick_pub_->get_subscription_count() : 0;
  }

  size_t echo_publisher_count() const {
    return sub_ ? sub_->get_publisher_count() : 0;
  }

  // Deliberately non-const (and so is chain_endpoints_discovered): rclcpp's graph
  // accessors have been const for a long time, but a const method here
  // would be a COMPILE error rather than a runtime one if that ever
  // changed, and this file is built only inside the ROS 2 image.
  size_t ping_subscriber_count() { return this->count_subscribers("ping"); }

  // Full-chain readiness for the bootstrap kick (same rule as
  // latency_node.cpp): a /kick subscriber proves only that ping_node is
  // up. Under the best-effort/volatile lane a /ping sample published
  // before pong's subscription matches is DROPPED, and in back-to-back
  // mode the chain is echo-driven — one lost sample and the cell hangs to
  // its timeout. Both extra conditions are pure GRAPH queries, so nothing
  // extra is created on the measured topics. Scope, hence the name:
  // the /kick and /echo counts are real matches against THIS node, the
  // /ping one is an existence query — the link this protects is inferred
  // from pong being demonstrably up, not proved. A bootstrap RETRY is
  // deliberately not used: a second kick would start a SECOND concurrent
  // chain and change the saturation shape being measured.
  // The two conditions that are real MATCHES against this node; both are
  // transport-carried endpoint counts, correct across processes on every
  // RMW. Same split, same reason, as latency_node.cpp.
  bool endpoints_matched() {
    return kick_subscriber_count() > 0 &&
           echo_publisher_count() > 0;
  }

  bool chain_endpoints_discovered() {
    if (bench_readiness_is_probe()) {
      // rmw_cerulion: see bench_readiness_is_probe() in common.hpp. The
      // graph query cannot answer across processes on this RMW, so
      // probe_for_first_echo() below supplies the proof out of data.
      return endpoints_matched();
    }
    return endpoints_matched() && ping_subscriber_count() > 0;
  }

  size_t probe_kicks() const { return probe_kicks_; }
  size_t probe_echoes() const { return probe_echoes_; }

  // Probe readiness (rmw_cerulion only). Publish a kick, poll the
  // subscription for a bounded slice, repeat until an echo comes back or
  // the deadline passes. Every message taken here is RETURNED and
  // DISCARDED: nothing is stamped, nothing reaches samples_, so no probe
  // round trip can appear in a .bin or in any percentile. The queue is
  // drained on the successful pass, so no probe echo survives into the
  // measured window either.
  //
  // DELIBERATELY POLLED, no wait set. Building a
  // second rclcpp::WaitSet over sub_ for the probe makes run() throw
  // `subscription already associated with a wait set`, and the node dies
  // with SIGABRT the moment readiness succeeds (measured on box-x86,
  // 2026-09-18: rc=134 on every cerulion loan cell). A subscription can
  // belong to one wait set, run()'s owns it, and a probe has no business
  // competing for that. Polling costs a 2 ms sleep during a warm up that
  // normally ends after one kick, and it cannot interact with the
  // measured loop's wait set at all.
  bool probe_for_first_echo(std::chrono::seconds timeout) {
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    while (rclcpp::ok() && std::chrono::steady_clock::now() < deadline) {
      ++probe_kicks_;
      // Counted in kicks_sent_ too: run_bench.sh's receipt chain asserts
      // kicks_sent >= published, and ping really does publish for these.
      publish_kick();
      const auto slice_end =
        std::chrono::steady_clock::now() + std::chrono::milliseconds(200);
      while (rclcpp::ok() && std::chrono::steady_clock::now() < slice_end) {
        bool took_one = false;
        while (rclcpp::ok()) {
          void * loaned_msg = nullptr;
          rmw_message_info_t info;
          rcl_ret_t ret = rcl_take_loaned_message(
            rcl_sub_, &loaned_msg, &info, nullptr);
          if (ret == RCL_RET_SUBSCRIPTION_TAKE_FAILED) break;
          if (ret != RCL_RET_OK) {
            // Not counted into take_failures_: that counter describes the
            // MEASURED drain, and a probe-phase failure here would
            // inflate a number the published receipt attributes to the
            // run.
            rcl_reset_error();
            break;
          }
          ++probe_echoes_;
          took_one = true;
          rcl_ret_t rret = rcl_return_loaned_message_from_subscription(
            rcl_sub_, loaned_msg);
          if (rret != RCL_RET_OK) {
            rcl_reset_error();
          }
        }
        if (took_one) {
          return true;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
      }
    }
    return false;
  }

  // Bootstrap: wait for the whole ping -> pong -> echo path to match.
  bool wait_for_chain(std::chrono::seconds timeout) {
    auto deadline = std::chrono::steady_clock::now() + timeout;
    while (rclcpp::ok() && !chain_endpoints_discovered()) {
      if (std::chrono::steady_clock::now() > deadline) {
        return false;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    return rclcpp::ok();
  }

  void send_bootstrap_kick() {
    publish_kick();
  }

  // Drive the rcl-direct receive loop. Returns 0 when target_+warmup_
  // samples collected and raw .bin written.
  int run() {
    rclcpp::WaitSet wait_set;
    wait_set.add_subscription(sub_);

    // Quiescent: kicks driven off a wall clock instead of inside the
    // receive loop — a missed echo becomes a longer sample gap, not a
    // chain deadlock. Backtoback: no schedule; the kick is published
    // after each recorded sample in the drain loop below.
    auto period = std::chrono::nanoseconds(
      backtoback_ ? 0 : 1'000'000'000ULL / rate_hz_);
    auto next_kick = std::chrono::steady_clock::now() + period;

    while (rclcpp::ok() && !done_) {
      std::chrono::nanoseconds wait_budget(0);
      if (!backtoback_) {
        // Fire a due kick, then SKIP MISSED SLOTS: next_kick anchors
        // to now + period, never += period. A late
        // wait_set.wait would otherwise owe several back-to-back
        // catch-up kicks — and the kick rides bench_qos(), so under
        // rel10 (RELIABLE/KEEP_LAST(10)) every one of them would be
        // delivered as a burst, corrupting the quiescent pacing.
        // (KEEP_LAST(1) overwrite does not make the
        // burst harmless in general: that only holds for be1.)
        auto now = std::chrono::steady_clock::now();
        if (now >= next_kick) {
          publish_kick();
          next_kick = now + period;
        }
        wait_budget = std::chrono::duration_cast<std::chrono::nanoseconds>(
          next_kick - now);
        if (wait_budget < std::chrono::nanoseconds(0)) {
          wait_budget = std::chrono::nanoseconds(0);
        }
      } else {
        // Saturation: the chain is self-sustaining once bootstrapped;
        // 100 ms cap keeps Ctrl+C / shutdown responsive.
        wait_budget = std::chrono::milliseconds(100);
      }

      auto wait_result = wait_set.wait(wait_budget);
      if (wait_result.kind() == rclcpp::WaitResultKind::Timeout) continue;
      if (wait_result.kind() != rclcpp::WaitResultKind::Ready) break;

      // Drain queue.
      while (rclcpp::ok() && !done_) {
        void * loaned_msg = nullptr;
        rmw_message_info_t info;
        rcl_ret_t ret = rcl_take_loaned_message(
          rcl_sub_, &loaned_msg, &info, nullptr);
        if (ret == RCL_RET_SUBSCRIPTION_TAKE_FAILED) break;
        if (ret != RCL_RET_OK) {
          // LOUD persistent-take-failure diagnostic (measured
          // on latency_bench:lyrical — same arm as pong_node_rcl.cpp):
          // a silent `break` here discards the wake WITHOUT
          // consuming the sample, so the wait set stays hot and the loop
          // spins Ready→take-fail invisibly. Known cause on lyrical:
          // rmw_cyclonedds 4.1.4's uninitialized
          // ArrayValueType::m_is_self_contained (UB — see PITFALLS.md
          // #11, "the lyrical inversion"). The bench.py structural skip
          // is the remedy; this names the cause in minutes instead of a
          // 300s timeout. Control flow stays a plain count,
          // log once, break.
          ++take_failures_;
          if (take_failures_ == 1) {
            RCLCPP_ERROR(
              this->get_logger(),
              "rcl_take_loaned_message failed: ret=%d rmw=%s err='%s' — "
              "if this repeats, take-loan is likely unsupported by this "
              "rmw (the loan lane spins Ready→take-fail without "
              "consuming; e.g. rmw_cyclonedds 4.1.4 on lyrical). "
              "Failure count is reported in the DELIVERY "
              "line at finalize.",
              static_cast<int>(ret),
              rmw_get_implementation_identifier(),
              rcl_get_error_string().str);
            rcl_reset_error();
          }
          break;
        }
        ++received_;

        auto * in = static_cast<const Msg *>(loaned_msg);
        uint64_t send_ns = in->ts_ns;
        uint64_t now_ns = wall_ns();

        // Return-code CHECKED
        // (same arm as pong_node_rcl.cpp): a failed
        // return silently LEAKS the loan. Count always, log once,
        // report in DELIVERY. Sits between the ts read and the sample
        // push but costs one compare on the happy path — no effect on
        // any recorded number (now_ns is already captured above).
        rcl_ret_t rret = rcl_return_loaned_message_from_subscription(
          rcl_sub_, loaned_msg);
        if (rret != RCL_RET_OK) {
          ++return_failures_;
          if (return_failures_ == 1) {
            RCLCPP_ERROR(
              this->get_logger(),
              "rcl_return_loaned_message_from_subscription failed: ret=%d "
              "rmw=%s err='%s' — the loan is leaked (the RMW cannot "
              "reclaim the chunk). Failure count is reported in the "
              "DELIVERY line at finalize.",
              static_cast<int>(rret),
              rmw_get_implementation_identifier(),
              rcl_get_error_string().str);
          }
          rcl_reset_error();
        }

        switch (classify_stamp_pair(send_ns, now_ns)) {
          case StampVerdict::kUnstamped:
            ++unstamped_;
            if (unstamped_ == 1) {
              RCLCPP_WARN(
                this->get_logger(),
                "echo carried NO stamp (send_ns=0) at now_ns=%llu — the "
                "sample cannot be taken. Logged once; the running count is "
                "reported as unstamped= in the DELIVERY line (printed at "
                "finalize, or when the drain loop ends on a cell that "
                "never finalizes).",
                static_cast<unsigned long long>(now_ns));
            }
            continue;
          case StampVerdict::kNonPositiveRtt:
            ++nonpositive_rtt_;
            if (nonpositive_rtt_ == 1) {
              RCLCPP_WARN(
                this->get_logger(),
                "echo's receive instant is not after its send instant "
                "(send_ns=%llu now_ns=%llu) — a duplicate or non-monotone "
                "stamp; now_ns - send_ns is unsigned and would wrap. "
                "Logged once; the running count is reported as "
                "nonpositive_rtt= in the DELIVERY line at finalize.",
                static_cast<unsigned long long>(send_ns),
                static_cast<unsigned long long>(now_ns));
            }
            continue;
          case StampVerdict::kUsable:
            // INSIDE the arm — see latency_node.cpp: with the push after
            // the switch, a future verdict nobody added an arm for falls
            // through and records the unsigned subtraction this gate
            // exists to prevent (the colcon build has no `-Werror`, so
            // `-Wswitch` is only a warning).
            samples_.push_back(now_ns - send_ns);
            if (cpu_probe_) {
              a_cpu_take_.push_back(sched_getcpu());
            }
            break;
        }

        size_t total_needed = target_ + warmup_;
        if (samples_.size() >= total_needed) {
          done_ = true;
          finalize();
          break;
        }

        if (backtoback_) {
          publish_kick();
        }
      }
    }

    // The drain loop ends EITHER because finalize() ran (the receipt is
    // already out and this is a no-op) or because rclcpp::ok() went
    // false / the wait set stopped being Ready without the sample
    // population ever filling — which is exactly the all-unusable-stamps
    // shape the stamp-gate counters exist for. Before this call the
    // counters died with the process in that shape.
    report_delivery();
    // wait_set destructor handles cleanup (RAII).
    return 0;
  }

private:
  void publish_kick() {
    ++kicks_sent_;
    std_msgs::msg::Empty kick;
    kick_pub_->publish(kick);
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
    raw_dump_samples(this->get_logger(), samples_, payload_size_);
    // Delivery accounting: kicks_sent vs received
    // quantifies loss (a kick whose echo never returned — expected
    // under be1 at large payloads). run_bench.sh greps this into
    // _logs/<cell>_<size>_delivery.txt — REPORTED, never gated.
    // take_failures counts non-OK/non-TAKE_FAILED rets from
    // rcl_take_loaned_message (NB a take-loan
    // that ALWAYS fails never reaches finalize — the in-flight signal
    // for that shape is the log-once RCLCPP_ERROR in the drain loop).
    report_delivery();
  }

public:
  // The DELIVERY receipt, split OUT of finalize() and callable again
  // when the drain loop ends — at most once, whichever path gets there
  // first. PUBLIC because run_latency_rcl() calls it on the exits that
  // happen after this node is constructed and before its drain loop is
  // entered (bootstrap failure, a SIGTERM landing mid-bootstrap, an
  // unsupported take-loan skip); each of those would otherwise leave the cell
  // with no receipt at all. See latency_node.cpp's copy for the reasoning: finalize()
  // runs only on a cell that collected its full sample population (it
  // sets done_, which is what ends the drain loop), so in the
  // all-unusable-stamps shape the counters exist for, finalize() alone
  // never prints the receipt. Reaching this on a timed-out cell also needs
  // run_bench.sh's watchdog to SIGTERM before it SIGKILLs, which it
  // does and the parity checker pins.
  void dump_cpu_probe() {
    if (!cpu_probe_ || a_cpu_take_.empty()) return;
    const char * out = std::getenv("CER_BENCH_LAT_CPU_OUT");
    if (!out || !*out) return;
    std::FILE * f = std::fopen(out, "wb");
    if (!f) return;
    std::fwrite(a_cpu_take_.data(), sizeof(int32_t), a_cpu_take_.size(), f);
    std::fclose(f);
  }

  void report_delivery() {
    if (delivery_reported_) {
      return;
    }
    delivery_reported_ = true;
    dump_cpu_probe();
    // unstamped / nonpositive_rtt: echoes that arrived but yielded no
    // sample, one count per reason (see sample_gate.hpp). Same
    // discipline as the two failure counters above — logged once,
    // counted always — and the identity received == samples + warmup +
    // unstamped + nonpositive_rtt is what makes the gap explainable.
    // `samples` there is the POST-warmup count the .bin carries, which is
    // why the warmup term is separate — finalize erases the prefix before
    // this runs. On the after-the-spin path samples_ is un-erased and the
    // identity is the oracle's two-term form instead.
    std::fprintf(
      stderr,
      "DELIVERY role=latency received=%zu kicks_sent=%zu take_failures=%zu "
      "return_failures=%zu unstamped=%zu nonpositive_rtt=%zu\n",
      received_, kicks_sent_, take_failures_, return_failures_,
      unstamped_, nonpositive_rtt_);
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
  bool loan_recv_supported_{false};
  bool done_{false};
  // At-most-once for the DELIVERY receipt: finalize() prints it on a
  // healthy cell, run()'s tail prints it on a cell that never finalized.
  bool delivery_reported_{false};
  size_t received_{0};
  size_t kicks_sent_{0};
  // Non-OK/non-TAKE_FAILED rets from rcl_take_loaned_message (see the
  // drain loop) — logged once, counted always, reported in DELIVERY.
  size_t take_failures_{0};
  // Non-OK rets from rcl_return_loaned_message_from_subscription — a
  // failed return leaks the loan. Same discipline:
  // logged once, counted always, reported in DELIVERY.
  size_t return_failures_{0};
  // Echoes the stamp gate declined, one counter per verdict (see
  // sample_gate.hpp). Kept APART because the remedies differ: an
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
  bool cpu_probe_{false};
  std::vector<int32_t> a_cpu_take_;
  size_t probe_kicks_{0};
  size_t probe_echoes_{0};
  std::vector<uint64_t> samples_;
  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr kick_pub_;
  typename rclcpp::Subscription<Msg>::SharedPtr sub_;
  const rcl_subscription_t * rcl_sub_{nullptr};
};

template <typename Msg>
int run_latency_rcl() {
  auto node = std::make_shared<LatencyNodeRcl<Msg>>();

  // Structural-skip path: rmw_take_loaned_message is a NO-OP
  // RMW_RET_UNSUPPORTED stub in rmw_zenoh (every released distro:
  // ros2/rmw_zenoh#175 #893). rmw_cerulion is kept in the same arm as a
  // guard, but it is not the reason: measured 2026-09-18 on this
  // harness, rmw_cerulion reports can_loan_messages()=1 and the lane's
  // ready line prints rcl_take_loaned_recv=1, so this branch does not
  // fire for it. Re-verify on new versions of either. If we
  // entered the take loop, the queued-but-never-consumed data would
  // keep the wait set hot and rcutils would generate multi-GiB of
  // error-string truncation warnings. Detect at startup and exit
  // with rc=77 (autotools SKIP) so the bench driver (bench.py) treats it as a
  // no-retry structural SKIP. Re-verify on new versions of either.
  //
  // For other RMWs where can_loan_messages() returns false, this is
  // typically transient (e.g. cyclonedds before iox-roudi has fully
  // engaged) or conservative (cyclonedds reports false even when the
  // take works — pitfall-11 caveat). The take loop is a clean failure
  // mode there — let it run and let the bench driver retry.
  if (!node->loan_recv_supported()) {
    const char * rmw = std::getenv("RMW_IMPLEMENTATION");
    if (rmw && (std::strcmp(rmw, "rmw_zenoh_cpp") == 0 ||
                std::strcmp(rmw, "rmw_cerulion") == 0)) {
      // The receipt BEFORE the exit — see latency_node.cpp's copy: a
      // cell can die after the node is constructed and before the drain
      // loop is entered, and every such path would otherwise lose the receipt.
      node->report_delivery();
      return 77;  // structural skip — no retry helps
    }
    // Fall through for other RMWs; the bootstrap or take loop will
    // fail naturally and the bench driver can retry.
  }

  if (!node->wait_for_chain(std::chrono::seconds(15))) {
    RCLCPP_ERROR(
      node->get_logger(),
      "bootstrap timeout: the ping->pong->echo endpoints were not all "
      "discovered within 15s (kick subscribers=%zu, echo publishers=%zu, ping "
      "subscribers=%zu) — are ping_node and pong_node up?",
      node->kick_subscriber_count(),
      node->echo_publisher_count(),
      node->ping_subscriber_count());
    // The receipt BEFORE the exit — see latency_node.cpp's copy: a cell
    // can die after the node is constructed and before the drain loop is
    // entered, and every such path would otherwise lose the receipt entirely.
    node->report_delivery();
    return 2;
  }
  // Probe readiness (rmw_cerulion only): the graph condition the gate
  // above drops on this RMW is replaced by data. See common.hpp.
  if (bench_readiness_is_probe()) {
    if (!node->probe_for_first_echo(std::chrono::seconds(15))) {
      RCLCPP_ERROR(
        node->get_logger(),
        "probe readiness timeout: both endpoint matches held (kick "
        "subscribers=%zu, echo publishers=%zu) but no echo came back "
        "after %zu probe kicks in 15s. The graph says the chain is "
        "wired and no data crossed it. ping_node or pong_node is up but "
        "not forwarding.",
        node->kick_subscriber_count(),
        node->echo_publisher_count(),
        node->probe_kicks());
      node->report_delivery();
      return 2;
    }
    RCLCPP_INFO(
      node->get_logger(),
      "readiness=matched+probe: chain proven by data (%zu probe kicks, "
      "%zu probe echoes). Those round trips are warm up and are not "
      "measured; the measured window starts at the bootstrap kick below.",
      node->probe_kicks(), node->probe_echoes());
  }

  node->send_bootstrap_kick();
  // Same shutdown race the two rclcpp nodes guard in start_pacing: this
  // node creates no wall timer, but run() constructs an rclcpp::WaitSet,
  // which is rcl entity creation and THROWS on a context that has shut
  // down. wait_for_chain already returns rclcpp::ok(), so the window is
  // just this gap — narrowed by the check, closed by the catch. The catch
  // RE-CHECKS: a genuine RCL error with the context still up is rethrown.
  if (!rclcpp::ok()) {
    node->report_delivery();
    return 2;
  }
  try {
    // NOT an early return: run() prints the receipt at its own tail.
    return node->run();
  } catch (const rclcpp::exceptions::RCLError &) {
    if (!rclcpp::ok()) {
      node->report_delivery();
      return 2;
    }
    throw;
  }
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

  // Fail-fast on a run whose samples could not land anywhere (see
  // latency_node.cpp).
  if (ros2_rtt_bench::env_str("CER_BENCH_RAW_DUMP_DIR", "").empty() ||
      ros2_rtt_bench::env_str("CER_BENCH_RAW_NAME", "").empty()) {
    fprintf(stderr,
      "latency_node_rcl: CER_BENCH_RAW_DUMP_DIR and CER_BENCH_RAW_NAME "
      "must both be set — refusing to run a measurement whose samples "
      "would be discarded.\n");
    rclcpp::shutdown();
    return 2;
  }

  // Quiescent pacing requires an explicit rate; backtoback ignores it.
  if (!ros2_rtt_bench::bench_pacing_backtoback() &&
      ros2_rtt_bench::env_size("CER_BENCH_TARGET_RATE_HZ", 0) == 0) {
    fprintf(stderr,
      "latency_node_rcl: CER_BENCH_TARGET_RATE_HZ must be set under "
      "paced CER_BENCH_PACING (quiescent/fixed100). Set CER_BENCH_PACING=backtoback for "
      "saturation pacing.\n");
    rclcpp::shutdown();
    return 2;
  }

  // ...and a rate whose wall period floors to ZERO is refused too: every
  // pacer here derives its period as `1'000'000'000ULL / rate_hz`, so a
  // rate above 1 GHz yields a 0 ns period, and
  // a 0 ns period leaves `next_kick` permanently already-due, so this
  // node publishes a kick on every WaitSet iteration and its wait budget
  // collapses to 0 — a saturation loop.
  // the run becomes a saturation run wearing the requested rate as its
  // label — in the .rate sidecar, the CSV and the plot.
  //
  // Gated on the pacing mode for the same reason the class refuses to even
  // PARSE the rate under backtoback (see the member initializer): an
  // inherited stale CER_BENCH_TARGET_RATE_HZ must not refuse a saturation
  // run that never consults it.
  if (!ros2_rtt_bench::bench_pacing_backtoback()) {
    ros2_rtt_bench::check_rate_hz(
      "latency_node_rcl", ros2_rtt_bench::env_size("CER_BENCH_TARGET_RATE_HZ", 0));
  }

  int rc;
  ROS2_RTT_BENCH_DISPATCH_BY_SIZE(size, ros2_rtt_bench::run_latency_rcl);
  rc = 2;
  rclcpp::shutdown();
  return rc;
}
