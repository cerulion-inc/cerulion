// SPDX-License-Identifier: AGPL-3.0-only
// ACCEPT (unique_ptr) + ACCEPT (stack) — the fixture that closes
// the corpus gap which hid the initialization defect.
//
// Every other fixture publishes std_msgs/String, which is NOT a plain type, so
// Fast DDS never loans it and the rewritten bytes always ran on rclcpp's
// CONSTRUCTING fallback (run_matrix.sh:9 says as much). geometry_msgs/Twist is
// plain and fixed, so it is the shape a loaning rmw actually hands back
// uninitialized — and it is PARTIALLY written here, which is the only shape
// where the difference is observable:
//
//   make_unique<Twist>()  ->  linear.y/z and angular.x/y are ZERO
//   an uninitialized loan ->  they are whatever the pool last held
//
// So the rewrite must value-initialize the loaned message. `tick_full_fill`
// writes every field and is the control: it must be rewritten the SAME way
// (the store is emitted unconditionally — see the emit comment for why a
// per-field elision would need a whole write-coverage analysis).
#include <memory>
#include <utility>

#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"

class SafePlainPartialFill : public rclcpp::Node {
 public:
  SafePlainPartialFill() : rclcpp::Node("safe_plain_partial_fill") {
    pub_ = create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
  }

  // PARTIAL: only two of the six fields are written.
  void tick_partial() {
    auto msg = std::make_unique<geometry_msgs::msg::Twist>();
    msg->linear.x = 1.0;
    msg->angular.z = 0.5;
    pub_->publish(std::move(msg));
  }

  // CONTROL: every field written. Rewritten identically — the value-init is
  // unconditional.
  void tick_full_fill() {
    geometry_msgs::msg::Twist msg;
    msg.linear.x = 1.0;
    msg.linear.y = 0.0;
    msg.linear.z = 0.0;
    msg.angular.x = 0.0;
    msg.angular.y = 0.0;
    msg.angular.z = 0.5;
    pub_->publish(msg);
  }

 private:
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_;
};
