// pong_node.cpp — round-trip benchmark, echo node (process 2 of 3).
//
// Subscribes to /ping, publishes to /echo. Same message type as
// ping_node — chosen by the type-class axis (CER_BENCH_MSG: Pod<N> via
// pod_dispatch.hpp, or sensor_msgs/msg/Image — see
// msg_class_dispatch.hpp). Forwards the stamp from the inbound to the
// outbound message; payload bytes are not memcpy'd from in → out — the
// bench measures transport cost (which depends on serialized size),
// not user-side memcpy cost.
//
// borrow_loaned_message() is used on the publish side where the RMW
// supports it; the inbound message is always whatever the subscriber
// allocates (may also be a loan in zero-copy paths).
//
// Mode-A discipline: the non-loan outbound message
// is PREALLOCATED ONCE at node construction and reused — rosidl's
// default ctor zero-fills the whole payload array, and the old
// per-echo make_unique<Msg>() paid that inside the timed window (the
// round trip is being clocked while this node runs). Per iteration the
// ONLY work between receive and publish is the 8-byte ts_ns copy. This
// deliberately DIFFERS from the May-2026 campaign's per-echo
// construction shape — see METHODOLOGY.md.

#include <cstddef>
#include <cstdio>
#include <memory>
#include <utility>

#include <rclcpp/rclcpp.hpp>

#include "common.hpp"
#include "msg_class_dispatch.hpp"

namespace ros2_rtt_bench {

template <typename Msg>
class PongNode : public rclcpp::Node {
public:
  PongNode()
  : rclcpp::Node("ros2_rtt_bench_pong", bench_node_options()),
    payload_bytes_(MsgAdapter<Msg>::payload_bytes(
      env_size("CER_BENCH_PAYLOAD_SIZE", 64)))
  {
    is_plain_check<Msg>(this->get_logger());

    // QoS axis — see common.hpp::bench_qos (be1 = zero-copy
    // pin, rel10 = real-stack pin; uniform across the chain).
    rclcpp::QoS qos = bench_qos();

    publisher_ = this->create_publisher<Msg>("echo", qos);
    loan_supported_ = publisher_->can_loan_messages();
    if (env_bool("CER_BENCH_DISABLE_LOAN", false)) {
      loan_supported_ = false;
    }
    if (MsgAdapter<Msg>::kVariable) {
      // TYPE-CLASS axis: an unbounded type cannot ride a loaned
      // publish — a borrowed Image would echo an EMPTY data vector
      // under an N-byte label. See ping_node.cpp's twin comment.
      loan_supported_ = false;
    }

    if (!loan_supported_) {
      // Mode-A preallocation (G3): pay the rosidl zero-fill ONCE,
      // outside the measurement loop — the image class also resizes
      // its unbounded data array to the sweep point here.
      out_ = std::make_unique<Msg>();
      MsgAdapter<Msg>::prepare(*out_, payload_bytes_);
    }

    sub_ = this->create_subscription<Msg>(
      "ping", qos,
      [this](typename Msg::ConstSharedPtr msg) { echo(msg); });

    RCLCPP_INFO(
      this->get_logger(),
      "pong_node ready (msg=%s payload=%zu loaned=%s qos=%s)",
      bench_msg_class().c_str(), payload_bytes_,
      loan_supported_ ? "1" : "0",
      bench_qos_label().c_str());
  }

  size_t echoed_count() const { return echoed_; }

private:
  void echo(const typename Msg::ConstSharedPtr & in) {
    ++echoed_;
    // G3 invariant (both branches): the stamp copy is the ONLY work
    // between receive and publish. No payload writes, no per-echo
    // message construction (the non-loan outbound is preallocated; the
    // image class's outbound data vector was sized once at prealloc —
    // the serialize + delivery copy publish() performs on it IS the
    // transport cost this class exists to measure).
    if (loan_supported_) {
      auto loan = publisher_->borrow_loaned_message();
      MsgAdapter<Msg>::write_stamp(
        loan.get(), MsgAdapter<Msg>::read_stamp(*in));
      publisher_->publish(std::move(loan));
    } else {
      MsgAdapter<Msg>::write_stamp(
        *out_, MsgAdapter<Msg>::read_stamp(*in));
      publisher_->publish(*out_);
    }
  }

  size_t payload_bytes_;
  bool loan_supported_{false};
  size_t echoed_{0};
  std::unique_ptr<Msg> out_;  // preallocated non-loan outbound (Mode-A/G3)
  typename rclcpp::Publisher<Msg>::SharedPtr publisher_;
  typename rclcpp::Subscription<Msg>::SharedPtr sub_;
};

template <typename Msg>
int run_pong() {
  auto node = std::make_shared<PongNode<Msg>>();
  rclcpp::spin(node);
  // Delivery accounting: run_bench.sh SIGTERMs
  // this node after the latency sink finishes; report what was echoed.
  // REPORTED into _logs/<cell>_<size>_delivery.txt — never gated.
  std::fprintf(
    stderr, "DELIVERY role=pong echoed=%zu\n", node->echoed_count());
  std::fflush(stderr);
  return 0;
}

}  // namespace ros2_rtt_bench

int main(int argc, char ** argv) {
  [[maybe_unused]] ros2_rtt_bench::CpuDmaLock dma_lock;

  rclcpp::init(argc, argv);
  size_t size = ros2_rtt_bench::env_size("CER_BENCH_PAYLOAD_SIZE", 64);

  int rc;
  ROS2_RTT_BENCH_DISPATCH_BY_CLASS(size, ros2_rtt_bench::run_pong);
  rc = 2;
  rclcpp::shutdown();
  return rc;
}
