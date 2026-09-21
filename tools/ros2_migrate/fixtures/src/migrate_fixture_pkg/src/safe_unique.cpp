// SPDX-License-Identifier: AGPL-3.0-only
// ACCEPT (unique_ptr): the canonical safe pattern — local make_unique, fill,
// publish(std::move(msg)) on the same publisher, nothing else touches msg.
#include <memory>
#include <utility>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

class SafeUnique : public rclcpp::Node {
 public:
  SafeUnique() : rclcpp::Node("safe_unique") {
    pub_ = create_publisher<std_msgs::msg::String>("chatter", 10);
  }

  void tick() {
    auto msg = std::make_unique<std_msgs::msg::String>();
    msg->data = "hello";
    pub_->publish(std::move(msg));
  }

 private:
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_;
};

// Cross-TU handles for matrix_runner's behavior pin (publish-free; the
// analyzer proposes nothing for these).
#include "matrix_api.hpp"

std::shared_ptr<rclcpp::Node> matrix_make_safe_unique() {
  return std::make_shared<SafeUnique>();
}

void matrix_tick_safe_unique(rclcpp::Node *n) {
  static_cast<SafeUnique *>(n)->tick();
}
