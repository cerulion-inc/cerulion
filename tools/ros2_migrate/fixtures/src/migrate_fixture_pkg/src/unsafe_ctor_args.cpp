// SPDX-License-Identifier: AGPL-3.0-only
// REFUSE (constructor-args): make_shared with a constructor argument — the
// loaned message is typesupport-default-initialized, so constructor
// arguments cannot be honored.
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

class UnsafeCtorArgs : public rclcpp::Node {
 public:
  UnsafeCtorArgs() : rclcpp::Node("unsafe_ctor_args") {
    pub_ = create_publisher<std_msgs::msg::String>("ctor_args", 10);
  }

  void tick() {
    auto msg =
        std::make_shared<std_msgs::msg::String>(std::allocator<void>());
    msg->data = "ctor args";
    pub_->publish(*msg);
  }

 private:
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_;
};
