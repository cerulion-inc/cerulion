// SPDX-License-Identifier: AGPL-3.0-only
// REFUSE (conditional-publish): the declaration and the publish sit in
// different statement scopes — on the not-taken path the message is built
// and dropped, and proving loan-return equivalence is out of scope.
#include <memory>
#include <utility>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

class UnsafeConditional : public rclcpp::Node {
 public:
  UnsafeConditional() : rclcpp::Node("unsafe_conditional") {
    pub_ = create_publisher<std_msgs::msg::String>("conditional", 10);
  }

  void tick(bool ready) {
    auto msg = std::make_unique<std_msgs::msg::String>();
    msg->data = "maybe";
    if (ready) {
      pub_->publish(std::move(msg));
    }
  }

 private:
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_;
};
