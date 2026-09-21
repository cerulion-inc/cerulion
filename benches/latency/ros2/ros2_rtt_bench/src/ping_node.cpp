// ping_node.cpp — round-trip benchmark, ping initiator (process 1 of 3).
//
// Subscribes to /kick (std_msgs/Empty), publishes to /ping. The /ping
// message type is chosen at startup by the TYPE-CLASS axis
// (CER_BENCH_MSG, see msg_class_dispatch.hpp): pod (default) = a
// fixed-size POD from CER_BENCH_PAYLOAD_SIZE via pod_dispatch.hpp;
// image = sensor_msgs/msg/Image with its unbounded data array resized
// to the sweep point at prealloc (loans structurally off — the class
// hypothesis under test). POD ("plain") layout
// is the necessary condition for loaned publishes on every RMW, but
// what a loaned=1 bit PROVES differs per stack (audit fix,
// steelman rclcpp-layer + fastdds3 findings):
//   - rmw_fastrtps (jazzy + lyrical): can_loan_messages() = is_plain
//     since Iron (PR ros2/rmw_fastrtps#568) — the loan is an
//     API-boundary borrow, INDEPENDENT of DataSharing, which
//     rmw_fastrtps forces OFF unless RMW_FASTRTPS_USE_QOS_FROM_XML=1
//     (the `zc` lane). loaned=1 on a stock fastdds cell is NOT proof
//     of a zero-copy data path.
//   - rmw_cyclonedds: version/build-dependent — jazzy's Cyclone 0.10
//     needs a DDS_HAS_SHM (iceoryx) build + engaged SHM; lyrical's
//     Cyclone 11 maps rmw_publish_loaned_message onto plain publish,
//     so loaned=1 there does not prove zero-copy either.
//   - rmw_zenoh: loan APIs are unimplemented stubs on every released
//     distro (ros2/rmw_zenoh#175 #893) — loaned=0 is structural.
// Variable-length types (sensor_msgs/Image with .data[]) would force
// every RMW into the heap-allocate-and-copy fallback; the per-row
// loaned= column records what each cell actually did.
//
// On every kick callback, stamps wall_ns() into the outbound msg's
// ts_ns field and publishes. The first /ping is published reactively
// in response to the bootstrap /kick that latency_node sends once it
// sees ping_node subscribe — no Timer, no WallRate. This keeps the
// chain transport-paced (matches Cerulion's --no-real-time methodology
// in graph_rtt_bench).
//
// Mode-A discipline (fix G3 — payload fill excluded from the
// timed window): the non-loan outbound message is PREALLOCATED ONCE at
// node construction and reused every iteration. rosidl's default ctor
// zero-fills the entire payload array, so the old per-iteration
// make_unique<Msg>() (a) put a payload-sized write inside the timed
// window and (b) sat BETWEEN the stamp and the publish. This
// deliberately DIFFERS from the May-2026 campaign's per-echo
// construction shape — see METHODOLOGY.md. On the loan path the loan
// is necessarily borrowed per iteration (publish() consumes it — that
// IS the loaned-publish middleware cost), but the stamp is written
// AFTER the borrow: the timed window opens at the stamp, and the ONLY
// work between stamp and publish is the 8-byte ts_ns write.

#include <cstddef>
#include <cstdio>
#include <memory>
#include <utility>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/empty.hpp>

#include "common.hpp"
#include "msg_class_dispatch.hpp"

namespace ros2_rtt_bench {

template <typename Msg>
class PingNode : public rclcpp::Node {
public:
  PingNode()
  : rclcpp::Node("ros2_rtt_bench_ping", bench_node_options()),
    payload_bytes_(MsgAdapter<Msg>::payload_bytes(
      env_size("CER_BENCH_PAYLOAD_SIZE", 64)))
  {
    is_plain_check<Msg>(this->get_logger());

    // QoS axis: CER_BENCH_QOS ∈ {be1 (default), rel10}.
    // See common.hpp::bench_qos() for the two profiles and why be1
    // (BEST_EFFORT/VOLATILE/KEEP_LAST(1)) is the zero-copy pin and
    // rel10 (RELIABLE/KEEP_LAST(10)) the real-stack pin.
    rclcpp::QoS qos = bench_qos();

    publisher_ = this->create_publisher<Msg>("ping", qos);
    loan_supported_ = publisher_->can_loan_messages();
    if (env_bool("CER_BENCH_DISABLE_LOAN", false)) {
      loan_supported_ = false;
    }
    if (MsgAdapter<Msg>::kVariable) {
      // TYPE-CLASS axis (CER_BENCH_MSG=image): an unbounded type
      // cannot ride a loaned publish — a loan slot is fixed-size, and
      // a borrowed Image would publish an EMPTY data vector under an
      // N-byte label (a mislabeled cell). Every current RMW already
      // answers can_loan_messages()=false for non-plain types; this
      // override makes the mislabel impossible even if a future RMW
      // claims otherwise (such a claim would deserve its own lane, not
      // a silent flip of this one). The printed loaned= line records
      // the post-override truth the cell measured.
      loan_supported_ = false;
    }

    if (!loan_supported_) {
      // Mode-A preallocation (G3): construct — and pay the rosidl
      // zero-fill for — the outbound message ONCE, outside the
      // measurement loop. The image class additionally resizes its
      // unbounded data array to the sweep point here (its O(N)
      // zero-fill, paid at the same prealloc site). Per iteration only
      // the stamp is written.
      msg_ = std::make_unique<Msg>();
      MsgAdapter<Msg>::prepare(*msg_, payload_bytes_);
    }

    kick_sub_ = this->create_subscription<std_msgs::msg::Empty>(
      "kick", qos,
      [this](std_msgs::msg::Empty::ConstSharedPtr) { publish_ping(); });

    RCLCPP_INFO(
      this->get_logger(),
      "ping_node ready (msg=%s payload=%zu loaned=%s qos=%s)",
      bench_msg_class().c_str(), payload_bytes_,
      loan_supported_ ? "1" : "0",
      bench_qos_label().c_str());
  }

  size_t published_count() const { return published_; }

private:
  void publish_ping() {
    ++published_;

    // G3 invariant (both branches): the wall_ns() stamp is the LAST
    // write before publish, and NOTHING else sits between stamp and
    // publish. Payload bytes are never written in the timed window
    // (Mode-A): the loan buffer holds whatever it holds, the
    // preallocated message holds its construction-time zeros — the
    // benchmark measures the transport cost of moving N bytes, not
    // the cost of writing them.
    if (loan_supported_) {
      auto loan = publisher_->borrow_loaned_message();
      auto & m = loan.get();
      MsgAdapter<Msg>::write_stamp(m, wall_ns());
      publisher_->publish(std::move(loan));
    } else {
      MsgAdapter<Msg>::write_stamp(*msg_, wall_ns());
      publisher_->publish(*msg_);
    }
  }

  size_t payload_bytes_;
  bool loan_supported_{false};
  size_t published_{0};
  std::unique_ptr<Msg> msg_;  // preallocated non-loan outbound (Mode-A/G3)
  typename rclcpp::Publisher<Msg>::SharedPtr publisher_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr kick_sub_;
};

template <typename Msg>
int run_ping() {
  auto node = std::make_shared<PingNode<Msg>>();
  rclcpp::spin(node);
  // Delivery accounting: run_bench.sh SIGTERMs
  // this node once the latency sink finishes; rclcpp's signal handler
  // ends the spin and we report what was published. The runner greps
  // this into _logs/<cell>_<size>_delivery.txt — REPORTED, never gated
  // (be1 loss at large payloads is a finding, not a harness failure).
  std::fprintf(
    stderr, "DELIVERY role=ping published=%zu\n", node->published_count());
  std::fflush(stderr);
  return 0;
}

}  // namespace ros2_rtt_bench

int main(int argc, char ** argv) {
  [[maybe_unused]] ros2_rtt_bench::CpuDmaLock dma_lock;

  rclcpp::init(argc, argv);
  size_t size = ros2_rtt_bench::env_size("CER_BENCH_PAYLOAD_SIZE", 64);

  int rc;
  ROS2_RTT_BENCH_DISPATCH_BY_CLASS(size, ros2_rtt_bench::run_ping);
  // unreachable — the macro ends every case with `return`
  rc = 2;
  rclcpp::shutdown();
  return rc;
}
