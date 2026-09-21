// SPDX-License-Identifier: AGPL-3.0-only
// REFUSE (retained-member): the message is a class member rewritten in place
// across ticks — its lifetime is the node's, not the call's, so a loan
// cannot replace it.
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

class UnsafeRetainedMember : public rclcpp::Node {
 public:
  UnsafeRetainedMember() : rclcpp::Node("unsafe_retained_member") {
    pub_ = create_publisher<std_msgs::msg::String>("retained", 10);
  }

  void tick() {
    msg_.data += "x";
    pub_->publish(msg_);
  }

 private:
  std_msgs::msg::String msg_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_;
};
