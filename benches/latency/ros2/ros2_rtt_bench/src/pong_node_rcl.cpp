// pong_node_rcl.cpp — pong with rcl-direct receive (zero-copy *receive*).
//
// rclcpp::Subscription's typed callback API memcpy's the inbound payload
// from the RMW buffer into a fresh heap Msg before invoking the user
// callback. That memcpy is the dominant cost in the multi-process bench
// at large payloads — see the suite's METHODOLOGY.md (why ROS2-SHM
// still scales linearly with payload on the rclcpp recv lane).
//
// This binary is identical in shape to pong_node.cpp but replaces the
// rclcpp callback with the `performance_test` pattern:
//
//   rclcpp::WaitSet  +  rcl_take_loaned_message  +  rcl_return_loaned_message_from_subscription
//
// rcl_take_loaned_message hands back a void* into the RMW-managed
// (typically SHM) buffer. We cast to Msg*, read ts_ns, return the
// loan immediately. There is no memcpy on the receive side.
//
// The wait loop is driven by `rclcpp::WaitSet`
// rather than raw `rcl_wait_set_t`. The two have equivalent semantics
// (rclcpp::WaitSet wraps the same rcl_wait_set_t underneath) but
// rclcpp::WaitSet adds RAII shutdown + type-safe `WaitResult.kind()`,
// matching the `performance_test` convention.
//
// rclcpp::Publisher (with borrow_loaned_message) is kept on the publish
// side — the publisher hot path is already zero-copy when the RMW
// supports it, so no parallel rcl-direct send path is needed.
//
// Mode-A discipline: the non-loan outbound message
// is PREALLOCATED ONCE at node construction (rosidl's default ctor
// zero-fills the whole payload — the old per-echo make_unique paid
// that inside the timed window). Per iteration the ONLY work between
// receive and publish is the 8-byte ts copy. This deliberately DIFFERS
// from the May-2026 campaign's per-echo construction shape — see
// METHODOLOGY.md.

#include <algorithm>
#include <chrono>
#include <cstddef>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <deque>
#include <sched.h>
#include <memory>
#include <optional>
#include <utility>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/wait_set.hpp>
#include <rcl/error_handling.h>  // rcl_get_error_string / rcl_reset_error
#include <rcl/subscription.h>
#include <rmw/rmw.h>  // rmw_get_implementation_identifier
#include <rmw/types.h>

#include "common.hpp"
#include "pod_dispatch.hpp"

namespace ros2_rtt_bench {

template <typename Msg>
class PongNodeRcl : public rclcpp::Node {
public:
  ~PongNodeRcl() override
  {
    // rclcpp::LoanedMessage's destructor asks its publisher whether it
    // can still loan (rcl_publisher_can_loan_messages), so a
    // loan that outlives its publisher dereferences freed rcl memory and
    // the process dies in librcl. A destructor BODY runs BEFORE member
    // destruction, so clearing here returns every held loan while
    // publisher_ is still alive, on every exit path, whatever the
    // member declaration order happens to be.
    //
    // This is a CALLER-side ordering hazard: measured on x86 Linux,
    // a minimal program that borrows and never publishes exits 0
    // when the loan dies first and segfaults when the publisher does,
    // with the fault inside librcl either way. It applies to any node
    // that holds a prefetched loan across the wait, as this one
    // does.
    loan_pool_.clear();
  }

  PongNodeRcl()
  : rclcpp::Node("ros2_rtt_bench_pong_rcl", bench_node_options())
  {
    is_plain_check<Msg>(this->get_logger());

    // QoS axis — see common.hpp::bench_qos. Uniform across
    // the chain.
    rclcpp::QoS qos = bench_qos();

    publisher_ = this->create_publisher<Msg>("echo", qos);
    loan_supported_ = publisher_->can_loan_messages();
    if (env_bool("CER_BENCH_DISABLE_LOAN", false)) {
      loan_supported_ = false;
    }
    // Diagnostic only, and loud about it: a campaign must never set this.
    refill_defer_ns_ =
      env_size("CER_BENCH_PONG_REFILL_DEFER_US", 1000) * 1000ULL;
    attrib_ = env_bool("CER_BENCH_PONG_ATTRIB", false);
    if (attrib_) {
      RCLCPP_WARN(
        this->get_logger(),
        "CER_BENCH_PONG_ATTRIB=1: per-echo phase timing is ON. The clock "
        "reads sit INSIDE the window this node contributes to the "
        "round trip, so any latency measured in this run is a "
        "diagnostic, not a published number.");
    }

    if (!loan_supported_) {
      // Mode-A preallocation (G3): pay the rosidl zero-fill ONCE,
      // outside the measurement loop.
      out_ = std::make_unique<Msg>();
    }

    // Create a typed Subscription for setup (entity creation,
    // discovery, QoS negotiation), then bypass rclcpp's callback
    // dispatch by reaching for the underlying rcl_subscription_t.
    // The empty callback is never invoked because we don't add the
    // node to an rclcpp executor — we drive the wait loop ourselves.
    sub_ = this->create_subscription<Msg>(
      "ping", qos,
      [](typename Msg::ConstSharedPtr) { /* never called */ });

    rcl_sub_ = sub_->get_subscription_handle().get();

    // rmw_take_loaned_message is a NO-OP stub on rmw_zenoh (every
    // released distro — re-verify on new rmw_zenoh versions). Probe
    // the capability bit at startup so we don't
    // spin in a loop where every rcl_take_loaned_message returns
    // RMW_RET_UNSUPPORTED and rcutils truncates the resulting error
    // string at the 240 B buffer (multi-GiB of node.log spam).
    // Upstream: ros2/rmw_zenoh#175 and #893.
    //
    // Audit fix C4 — what this bit actually reads: rcl gates
    // SUBSCRIPTION-side loans OFF by default (rclcpp#2335 / rcl#1110,
    // Nov 2023, backported to Humble), so can_loan_messages() is
    // `!rcl_gate && rmw_capability` — with ROS_DISABLE_LOANED_MESSAGES
    // unset it reads FALSE on every RMW even where
    // rcl_take_loaned_message (which bypasses the rcl gate) takes
    // loans fine. The harness exports ROS_DISABLE_LOANED_MESSAGES=0
    // on this lane (run_bench.sh) so the bit — and the `rcl_take_
    // loaned_recv=` line below — reports the RMW's real capability
    // instead of the env default.
    loan_recv_supported_ = sub_->can_loan_messages();

    // G3 ON THE ECHO SIDE. G3 is the rule that payload sized work stays
    // outside the timed window. The loaned reply for the FIRST echo is
    // borrowed here, before this node says it is ready, so the very
    // first round trip is measured under the same rule as every later
    // one. See refill_loan_pool() for why the borrow may not sit
    // between the take and the publish.
    if (loan_supported_) {
      prefetch_depth_ = env_size("CER_BENCH_PONG_PREFETCH", kPrefetchDefault);
      if (prefetch_depth_ < 1 || prefetch_depth_ > kPrefetchMax) {
        std::fprintf(stderr,
                     "CER_BENCH_PONG_PREFETCH must be 1..%zu, got %zu\n",
                     kPrefetchMax, prefetch_depth_);
        std::exit(2);
      }
      refill_loan_pool();
    }

    RCLCPP_INFO(
      this->get_logger(),
      "pong_node_rcl ready (msg_size=%zu loaned_pub=%s qos=%s "
      "rcl_take_loaned_recv=%s echo_rule=%s prefetch_depth=%zu refill_defer_us=%llu)",
      sizeof(Msg),
      loan_supported_ ? "1" : "0",
      bench_qos_label().c_str(),
      loan_recv_supported_ ? "1" : "0",
      loan_supported_ ? "prefetched_loan" : "preallocated_copy",
      loan_supported_ ? prefetch_depth_ : 0,
      (unsigned long long)(loan_supported_ ? refill_defer_ns_ / 1000 : 0));

    if (!loan_recv_supported_) {
      // Grep-friendly sentinel. Include the active RMW so
      // run_full_sweep.sh can distinguish structural skip
      // (rmw_zenoh) from transient cap=false (e.g. cyclonedds when
      // iox-roudi hasn't engaged yet).
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

  // Drive the rcl-direct wait+take loop. Returns when rclcpp::ok()
  // becomes false (Ctrl+C or shutdown).
  //
  // rclcpp::WaitSet wraps the rcl_wait_set_t with RAII shutdown and a
  // type-safe `WaitResult.kind()` discriminator (Ready / Timeout /
  // Empty). The `performance_test` pattern uses this rather
  // than raw rcl, matching that established ROS2 convention.
  // Same `rcl_take_loaned_message` underneath — the change is purely
  // about the wait-side ergonomics.
  int run() {
    rclcpp::WaitSet wait_set;
    wait_set.add_subscription(sub_);

    while (rclcpp::ok()) {
      // 100 ms timeout so Ctrl+C / rclcpp::shutdown is responsive.
      // Liveness budget is unchanged at 100 ms. When a refill is
      // pending we simply wake EARLIER, at its deadline, and never
      // later: the wait is the shorter of the two.
      // A pending refill shortens the wait to 1 ms so we wake at or just
      // after its deadline. Deliberately NOT the exact remainder: a
      // sub millisecond remainder rounds to a 0 ms wait, which is a busy
      // spin, and waking a few hundred microseconds late costs nothing
      // when the next ping is 10 ms away. A larger deferral simply loops
      // here, sleeping 1 ms at a time, until the deadline passes.
      const auto budget = refill_pending_
        ? std::chrono::milliseconds(1)
        : std::chrono::milliseconds(100);
      auto wait_result = wait_set.wait(budget);
      if (wait_result.kind() == rclcpp::WaitResultKind::Timeout) {
        // THE refill site. Nothing is in flight here: the wait timed out,
        // so no echo is outstanding and no reader is waiting to be woken.
        if (refill_pending_ && wall_ns() >= refill_at_) {
          const uint64_t t_ref0 = attrib_ ? wall_ns() : 0;
          if (attrib_) {
            a_cpu_ref_.push_back(sched_getcpu());
          }
          refill_loan_pool();
          if (attrib_) {
            const uint64_t t_ref1 = wall_ns();
            a_refill_.push_back(t_ref1 - t_ref0);
            last_refill_end_ = t_ref1;
          }
          refill_pending_ = false;
        }
        continue;
      }
      if (wait_result.kind() != rclcpp::WaitResultKind::Ready) {
        // Empty (no entities — shouldn't happen post-add) or Error.
        break;
      }

      // Drain. The RMW may deliver multiple messages between waits;
      // take_loaned in a loop until SUBSCRIPTION_TAKE_FAILED.
      while (rclcpp::ok()) {
        void * loaned_msg = nullptr;
        rmw_message_info_t info;
        const uint64_t t_take0 = attrib_ ? wall_ns() : 0;
        if (attrib_ && last_refill_end_ != 0) {
          // Positive: the chain was idle this long before the ping
          // arrived. Zero or negative: the ping was ALREADY waiting when
          // the refill finished, which is the collision this probe is
          // looking for. One entry per take attempt; the empty-queue
          // attempt that ends a drain is dropped below.
          a_gap_.push_back(static_cast<int64_t>(t_take0) -
                           static_cast<int64_t>(last_refill_end_));
        }
        rcl_ret_t ret = rcl_take_loaned_message(
          rcl_sub_, &loaned_msg, &info, nullptr);
        const uint64_t t_take1 = attrib_ ? wall_ns() : 0;
        if (ret == RCL_RET_SUBSCRIPTION_TAKE_FAILED) {
          // Not an echo: drop the gap this attempt just pushed so the
          // array stays 1:1 with echoes, and therefore 1:1 with the
          // latency node's samples.
          if (attrib_ && !a_gap_.empty()) {
            a_gap_.pop_back();
          }
          break;  // queue empty
        }
        if (ret != RCL_RET_OK) {
          // LOUD persistent-take-failure diagnostic (measured
          // on latency_bench:lyrical): a silent
          // `break` here discards the wake WITHOUT consuming the sample,
          // so the wait set stays hot and the loop spins Ready→take-fail
          // at 100% of a core — pong echoes 0 of 85k pings and the only
          // symptom is a 300s cell timeout. Known cause on lyrical:
          // rmw_cyclonedds 4.1.4's uninitialized
          // ArrayValueType::m_is_self_contained (UB — see PITFALLS.md
          // #11, "the lyrical inversion") makes take-loan fail per call
          // or SIGABRT. The structural skip in bench.py is the remedy; this
          // logging is the diagnostic that names the cause in
          // minutes instead of a timeout. Control flow stays a plain
          // count, log once, break.
          ++take_failures_;
          if (take_failures_ == 1) {
            RCLCPP_ERROR(
              this->get_logger(),
              "rcl_take_loaned_message failed: ret=%d rmw=%s err='%s' — "
              "if this repeats, take-loan is likely unsupported by this "
              "rmw (the loan lane spins Ready→take-fail without "
              "consuming; e.g. rmw_cyclonedds 4.1.4 on lyrical). "
              "Failure count is reported in the DELIVERY "
              "line at exit.",
              static_cast<int>(ret),
              rmw_get_implementation_identifier(),
              rcl_get_error_string().str);
            rcl_reset_error();
          }
          break;
        }

        // Zero-copy receive: cast to Msg*, read ts_ns inline.
        auto * in = static_cast<const Msg *>(loaned_msg);
        uint64_t ts = in->ts_ns;
        if (attrib_) {
          // The FORWARD leg, measured rather than halved: pong holds
          // ping's stamp, and both processes read the same
          // CLOCK_MONOTONIC. The suite's one_way_p50_ns is round trip
          // over two and carries no information about which leg is slow.
          a_fwd_.push_back(static_cast<int64_t>(t_take0) -
                           static_cast<int64_t>(ts));
        }

        // Return the loan as soon as we've extracted what we need —
        // RMW buffer ownership returns to the RMW. The return code is
        // CHECKED:
        // a failed return silently LEAKS the loan — the RMW never
        // reclaims the SHM chunk — so count always, log once, report in
        // the DELIVERY line. Does not affect any recorded latency.
        const uint64_t t_ret0 = attrib_ ? wall_ns() : 0;
        rcl_ret_t rret = rcl_return_loaned_message_from_subscription(
          rcl_sub_, loaned_msg);
        const uint64_t t_ret1 = attrib_ ? wall_ns() : 0;
        if (rret != RCL_RET_OK) {
          ++return_failures_;
          if (return_failures_ == 1) {
            RCLCPP_ERROR(
              this->get_logger(),
              "rcl_return_loaned_message_from_subscription failed: ret=%d "
              "rmw=%s err='%s' — the loan is leaked (the RMW cannot "
              "reclaim the chunk). Failure count is reported in the "
              "DELIVERY line at exit.",
              static_cast<int>(rret),
              rmw_get_implementation_identifier(),
              rcl_get_error_string().str);
          }
          rcl_reset_error();
        }

        ++echoed_;
        // Forward the timestamp on the publish side (rclcpp::Publisher,
        // loaned where supported). G3: the ts copy is the ONLY work
        // between receive and publish — the non-loan outbound is
        // preallocated at construction, never rebuilt per echo.
        if (loan_supported_) {
          // G3: the 8 byte stamp copy is the ONLY work between the take
          // and the publish. Every loan here was borrowed while the
          // chain was idle, outside every timed window.
          uint64_t t_bor0 = 0, t_bor1 = 0;
          if (loan_pool_.empty()) {
            // Burst deeper than the pool. Correctness over purity: borrow
            // inline rather than drop the echo, and count it, because an
            // inline borrow is the very thing this design removes from
            // the window and a silent one would make the row a lie.
            ++inline_borrows_;
            t_bor0 = attrib_ ? wall_ns() : 0;
            loan_pool_.emplace_back(publisher_->borrow_loaned_message());
            t_bor1 = attrib_ ? wall_ns() : 0;
          }
          auto loan = std::move(loan_pool_.front());
          loan_pool_.pop_front();
          loan.get().ts_ns = ts;
          const uint64_t t_pub0 = attrib_ ? wall_ns() : 0;
          publisher_->publish(std::move(loan));
          if (attrib_) {
            a_cpu_pub_.push_back(sched_getcpu());
            const uint64_t t_pub1 = wall_ns();
            record_attrib(t_take1 - t_take0, t_ret1 - t_ret0,
                          t_bor1 - t_bor0, t_pub1 - t_pub0);
          }
        } else {
          out_->ts_ns = ts;
          const uint64_t t_pub0 = attrib_ ? wall_ns() : 0;
          publisher_->publish(*out_);
          if (attrib_) {
            const uint64_t t_pub1 = wall_ns();
            record_attrib(t_take1 - t_take0, t_ret1 - t_ret0, 0,
                          t_pub1 - t_pub0);
          }
        }
      }
      // The queue is drained. Do NOT refill here: the echo published a
      // moment ago and is still IN FLIGHT, and the payload sized borrow
      // would hold this core while the kernel is deciding where to wake
      // the reader (METHODOLOGY section 21). Arm a deadline instead and let
      // the wait above expire on it, so the borrow runs when nothing is
      // outstanding.
      if (loan_supported_) {
        refill_at_ = wall_ns() + refill_defer_ns_;
        refill_pending_ = true;
      }
    }

    // Delivery accounting: run_bench.sh SIGTERMs
    // this node after the latency sink finishes; report what was
    // echoed. REPORTED into _logs/<cell>_<size>_delivery.txt — never
    // gated. take_failures counts non-OK/non-TAKE_FAILED rets from
    // rcl_take_loaned_message (2026-08-12: a broken take-loan rmw —
    // rmw_cyclonedds 4.1.4 on lyrical — spins here invisibly; a nonzero
    // count with echoed=0 IS that signature).
    report_attrib();
    if (loan_supported_ && inline_borrows_ > 0) {
      // Loud, because it means the pool was too shallow for the burst
      // and some echo paid a payload sized borrow inside its window.
      RCLCPP_WARN(
        this->get_logger(),
        "pong borrowed INLINE %zu time(s): the prefetch pool (depth %zu) "
        "ran dry during a burst, so those echoes paid the borrow inside "
        "the timed window. Their round trips are inflated by it.",
        inline_borrows_, prefetch_depth_);
    }
    std::fprintf(stderr,
                 "DELIVERY role=pong echoed=%zu take_failures=%zu "
                 "return_failures=%zu\n",
                 echoed_, take_failures_, return_failures_);
    std::fflush(stderr);

    // wait_set destructor handles cleanup automatically (RAII).
    return 0;
  }

private:
  bool loan_supported_{false};

  // G3 ON THE ECHO SIDE. The reply loan for the NEXT echo,
  // borrowed after the current one is published.
  //
  // WHY: rmw_borrow_loaned_message is not a pointer handout on every
  // RMW. On rmw_cerulion the loanable path loans an uninitialised slot,
  // zeroes the 32 byte wire header, and then runs the typesupport
  // init_function over the payload (crates/rmw_cerulion/src/api/
  // pubsub.rs, via type_bridge_cpp.rs init_loaned_payload). The C++
  // introspection init writes EVERY member, and this bench's Pod<N> body
  // is a fixed uint8 array of N minus 8 bytes, so the borrow carries a
  // payload sized write. Held between the take and the publish, that
  // write lands inside the timed round trip and grows with the payload,
  // which is exactly what G3 exists to keep out (ping already borrows
  // before it stamps, and the copy path branch below preallocates once
  // at construction for the same reason). Moving the borrow after the
  // publish costs nothing and puts it where the other two already are.
  //
  // SCOPE: outside the window means outside under a PACED
  // variant, where the chain is idle between echoes. Under backtoback
  // the chain is saturated and the next ping can arrive while this
  // borrow is still running, so the cost moves rather than vanishing.
  // It is still never between the take and the publish.
  //
  // WHY A POOL AND NOT ONE LOAN. The alternative holds exactly one
  // prefetched loan and refills it immediately after each publish,
  // inside the drain loop. Measured on box-x86 2026-09-18 (PONG_ATTRIB):
  // the borrow costs 669 us at 16 MiB against 192 ns at 64 B, so it is
  // the one phase that scales with the payload, and a ping that arrived
  // during that 669 us queues BEHIND it and carries the wait into its
  // own round trip. That shows up as a bimodal round trip: p75 16.9 us,
  // p90 626.7 us. The pool plus a refill AFTER the drain loop means a
  // burst up to kPrefetchDepth is served from loans already paid for,
  // and the refill happens with nothing waiting behind it.
  //
  // DEPTH 3 against the rmw's budget of 4
  // (rmw_cerulion RMW_LOANED_SAMPLES_BUDGET, api/pubsub.rs:92, set as
  // publisher_max_loaned_samples): three held plus one transient, which
  // is the shape that constant documents. Asking for more would earn a
  // BAD_ALLOC from the borrow, since that path errors rather than
  // blocking when the budget is gone (api/pubsub.rs:1371).
  // DEPTH IS A KNOB, because deeper is not simply better. A deeper pool
  // absorbs longer bursts, but it also means the slot a subscriber reads
  // was last touched `depth` borrows ago instead of one, so it is colder
  // when the read happens. Measured at 1 MiB the median walked 22.4 us
  // (borrow in the window) to 27.0 (depth 1) to 31.5 (depth 3), which is
  // the shape that trade off predicts. CER_BENCH_PONG_PREFETCH picks it;
  // the ceiling is the rmw's own loaned budget minus the transient
  // (RMW_LOANED_SAMPLES_BUDGET = 4, api/pubsub.rs:92).
  // DEFAULT 1, from the depth sweep (box-x86, 2026-09-18, k=1 x 2000 per
  // point). What removed the tail was refilling AFTER the drain loop,
  // not the pool: 16 MiB p90 read 26.7 us at depth 1, 26.7 at depth 2
  // and 29.9 at depth 3, against 626.7 us when the refill sat between
  // messages. Depth beyond 1 bought nothing and cost a little at 1 MiB
  // (p50 27.6 us at depth 1 against 31.3 at depths 2 and 3), which is
  // the cold slot effect the note above predicts. The pool stays as the
  // burst guard, one deep by default.
  static constexpr size_t kPrefetchMax = 3;
  static constexpr size_t kPrefetchDefault = 1;
  size_t prefetch_depth_{kPrefetchDefault};
  size_t inline_borrows_{0};

  void refill_loan_pool() {
    while (loan_pool_.size() < prefetch_depth_) {
      loan_pool_.emplace_back(publisher_->borrow_loaned_message());
    }
  }

  // --- phase attribution (diagnostic; OFF unless CER_BENCH_PONG_ATTRIB=1)
  // Answers one question: of the time pong spends between taking a ping
  // and publishing its echo, how much is the take, how much the loan
  // return, how much the BORROW, and how much the publish. It exists
  // because rmw_borrow_loaned_message is not a pointer handout on every
  // RMW: rmw_cerulion runs the typesupport init_function over the loaned
  // payload (crates/rmw_cerulion/src/type_bridge_cpp.rs
  // init_loaned_payload), and for a Pod<N> whose body is a fixed uint8
  // array that constructor writes every member, so the borrow carries a
  // payload-sized write INSIDE the measured window. Whether that term
  // dominates is a measurement, not a deduction, and this is the
  // measurement. Never on during a campaign: the timing calls would sit
  // in the very window the campaign reports.
  bool attrib_{false};
  std::vector<uint64_t> a_take_, a_return_, a_borrow_, a_publish_;
  std::vector<uint64_t> a_refill_;
  std::vector<int64_t> a_gap_;      // take_start - prev refill_end
  std::vector<int64_t> a_fwd_;      // take_start - ping's stamp
  std::vector<int32_t> a_cpu_pub_, a_cpu_ref_;
  uint64_t last_refill_end_{0};
  bool refill_pending_{false};
  uint64_t refill_at_{0};
  uint64_t refill_defer_ns_{1000000};  // 1 ms

  void record_attrib(uint64_t take, uint64_t ret, uint64_t borrow,
                     uint64_t publish) {
    a_take_.push_back(take);
    a_return_.push_back(ret);
    a_borrow_.push_back(borrow);
    a_publish_.push_back(publish);
  }

  static uint64_t median_of(std::vector<uint64_t> v) {
    if (v.empty()) return 0;
    std::sort(v.begin(), v.end());
    return v[v.size() / 2];
  }

  static uint64_t pct_of(std::vector<uint64_t> v, double q) {
    if (v.empty()) return 0;
    std::sort(v.begin(), v.end());
    size_t idx = static_cast<size_t>(q * static_cast<double>(v.size() - 1));
    return v[idx];
  }

  // Written at exit, beside the DELIVERY receipt, so the numbers live in
  // the artifacts rather than in a terminal someone has to still have
  // open. Discards the first tenth as warm up, the same discipline the
  // measured lanes use.
  void report_attrib() {
    if (!attrib_ || a_take_.empty()) return;
    size_t warm = a_take_.size() / 10;
    auto trim = [warm](const std::vector<uint64_t> & v) {
      return std::vector<uint64_t>(v.begin() + static_cast<long>(warm), v.end());
    };
    auto take = trim(a_take_), ret = trim(a_return_);
    auto bor = trim(a_borrow_), pub = trim(a_publish_);
    const char * out = std::getenv("CER_BENCH_PONG_ATTRIB_OUT");
    std::FILE * f = (out && *out) ? std::fopen(out, "a") : stderr;
    if (!f) f = stderr;
    std::fprintf(f,
                 "PONG_ATTRIB payload=%zu echoes=%zu warmup_discarded=%zu "
                 "take_p50=%llu take_p90=%llu return_p50=%llu return_p90=%llu "
                 "borrow_p50=%llu borrow_p90=%llu publish_p50=%llu "
                 "publish_p90=%llu refill_n=%zu refill_p50=%llu "
                 "refill_p90=%llu refill_max=%llu (ns)\n",
                 sizeof(Msg), take.size(), warm,
                 (unsigned long long)median_of(take),
                 (unsigned long long)pct_of(take, 0.90),
                 (unsigned long long)median_of(ret),
                 (unsigned long long)pct_of(ret, 0.90),
                 (unsigned long long)median_of(bor),
                 (unsigned long long)pct_of(bor, 0.90),
                 (unsigned long long)median_of(pub),
                 (unsigned long long)pct_of(pub, 0.90),
                 a_refill_.size(),
                 (unsigned long long)median_of(a_refill_),
                 (unsigned long long)pct_of(a_refill_, 0.90),
                 (unsigned long long)(a_refill_.empty()
                                      ? 0
                                      : *std::max_element(a_refill_.begin(),
                                                          a_refill_.end())));
    std::fflush(f);
    if (f != stderr) std::fclose(f);
    // Per echo gaps, 1:1 with the latency node's samples, so a slow
    // sample can be matched to the window it fell in.
    const char * gd = std::getenv("CER_BENCH_PONG_GAP_OUT");
    if (gd && *gd && !a_gap_.empty()) {
      std::FILE * g = std::fopen(gd, "wb");
      if (g) {
        // 16 bytes per echo: gap, fwd, cpu_publish, cpu_refill.
        size_t n = a_gap_.size();
        for (size_t i = 0; i < n; ++i) {
          int64_t gp = a_gap_[i];
          int64_t fw = (i < a_fwd_.size()) ? a_fwd_[i] : 0;
          int32_t cp = (i < a_cpu_pub_.size()) ? a_cpu_pub_[i] : -1;
          int32_t cr = (i < a_cpu_ref_.size()) ? a_cpu_ref_[i] : -1;
          std::fwrite(&gp, sizeof gp, 1, g);
          std::fwrite(&fw, sizeof fw, 1, g);
          std::fwrite(&cp, sizeof cp, 1, g);
          std::fwrite(&cr, sizeof cr, 1, g);
        }
        std::fclose(g);
      }
    }
  }

  bool loan_recv_supported_{false};
  size_t echoed_{0};
  // Non-OK/non-TAKE_FAILED rets from rcl_take_loaned_message (see the
  // take loop) — logged once, counted always, reported in DELIVERY.
  size_t take_failures_{0};
  // Non-OK rets from rcl_return_loaned_message_from_subscription — a
  // failed return leaks the loan. Same discipline:
  // logged once, counted always, reported in DELIVERY.
  size_t return_failures_{0};
  std::unique_ptr<Msg> out_;  // preallocated non-loan outbound (Mode-A/G3)
  typename rclcpp::Publisher<Msg>::SharedPtr publisher_;
  // DECLARED AFTER publisher_ on purpose: members are
  // destroyed in reverse declaration order, so the loans here die BEFORE
  // the publisher they came from. The destructor above already returns
  // them explicitly; this ordering is the backstop that keeps the class
  // correct if that line is ever removed.
  std::deque<rclcpp::LoanedMessage<Msg>> loan_pool_;
  typename rclcpp::Subscription<Msg>::SharedPtr sub_;
  const rcl_subscription_t * rcl_sub_{nullptr};
};

template <typename Msg>
int run_pong_rcl() {
  auto node = std::make_shared<PongNodeRcl<Msg>>();

  // Structural-skip path: short-circuit for rmw_zenoh AND
  // rmw_cerulion, whose rmw_take_loaned_message are both NO-OP
  // RMW_RET_UNSUPPORTED stubs (see latency_node_rcl.cpp for the full
  // story) — the take loop would generate multi-GiB of error-string
  // spam from rcutils. For other RMWs, can_loan_messages=false is
  // typically a transient setup issue (e.g. iox-roudi not yet
  // engaged) — let the run loop attempt take and exit on its own.
  if (!node->loan_recv_supported()) {
    const char * rmw = std::getenv("RMW_IMPLEMENTATION");
    if (rmw && (std::strcmp(rmw, "rmw_zenoh_cpp") == 0 ||
                std::strcmp(rmw, "rmw_cerulion") == 0)) {
      return 77;  // structural skip — no retry helps
    }
  }
  return node->run();
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

  int rc;
  ROS2_RTT_BENCH_DISPATCH_BY_SIZE(size, ros2_rtt_bench::run_pong_rcl);
  rc = 2;
  rclcpp::shutdown();
  return rc;
}
