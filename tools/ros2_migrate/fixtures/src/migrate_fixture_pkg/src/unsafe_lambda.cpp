// SPDX-License-Identifier: AGPL-3.0-only
// REFUSE (captured-by-lambda): the message variable is used inside a lambda
// body — the closure could outlive the publish.
#include <memory>
#include <utility>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

class UnsafeLambda : public rclcpp::Node {
 public:
  UnsafeLambda() : rclcpp::Node("unsafe_lambda") {
    pub_ = create_publisher<std_msgs::msg::String>("lambda", 10);
  }

  void tick() {
    auto msg = std::make_unique<std_msgs::msg::String>();
    auto fill = [&msg]() { msg->data = "from lambda"; };
    fill();
    pub_->publish(std::move(msg));
  }

  // The capture-LIST arm: a by-copy capture retains the
  // shared message even though the lambda body never spells the name — the
  // loaned rewrite would turn the retained shared_ptr into a raw pointer
  // into returned loan memory.
  void tick_copy_capture() {
    auto msg = std::make_shared<std_msgs::msg::String>();
    msg->data = "retained by copy";
    auto keep = [msg]() {};
    keep();
    pub_->publish(*msg);
  }

 private:
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_;
};
