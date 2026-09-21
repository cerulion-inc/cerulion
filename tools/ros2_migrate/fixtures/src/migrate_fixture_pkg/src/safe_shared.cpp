// SPDX-License-Identifier: AGPL-3.0-only
// ACCEPT (shared_ptr): local make_shared, fill, publish(*msg) — the shared
// pointer never escapes and nothing uses it after the publish.
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

class SafeShared : public rclcpp::Node {
 public:
  SafeShared() : rclcpp::Node("safe_shared") {
    pub_ = create_publisher<std_msgs::msg::String>("chatter_shared", 10);
  }

  void tick() {
    auto msg = std::make_shared<std_msgs::msg::String>();
    msg->data = "shared hello";
    pub_->publish(*msg);
  }

 private:
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_;
};
