// SPDX-License-Identifier: AGPL-3.0-only
// REFUSE (message-built-elsewhere + not-a-local): the message is built in one
// function and published in another — the borrow site cannot be proven.
#include <memory>
#include <utility>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

class UnsafeCrossFunction : public rclcpp::Node {
 public:
  UnsafeCrossFunction() : rclcpp::Node("unsafe_cross_function") {
    pub_ = create_publisher<std_msgs::msg::String>("cross", 10);
  }

  // message-built-elsewhere: the local is another function's return value.
  void tick() {
    auto msg = build_message();
    pub_->publish(std::move(msg));
  }

  // not-a-local: the published pointer arrives as a parameter.
  void forward(std::unique_ptr<std_msgs::msg::String> msg) {
    pub_->publish(std::move(msg));
  }

 private:
  std::unique_ptr<std_msgs::msg::String> build_message() {
    auto m = std::make_unique<std_msgs::msg::String>();
    m->data = "built elsewhere";
    return m;
  }

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_;
};
