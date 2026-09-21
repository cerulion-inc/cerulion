// SPDX-License-Identifier: AGPL-3.0-only
// ACCEPT (stack): a default-constructed stack local, filled through `.`,
// published by const-ref. The rewrite binds `auto& msg = loaned.get();` so
// every fill line stays untouched.
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

class SafeStack : public rclcpp::Node {
 public:
  SafeStack() : rclcpp::Node("safe_stack") {
    pub_ = create_publisher<std_msgs::msg::String>("chatter_stack", 10);
  }

  void tick() {
    std_msgs::msg::String msg;
    msg.data = "stack hello";
    pub_->publish(msg);
  }

 private:
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_;
};
