// SPDX-License-Identifier: AGPL-3.0-only
// ACCEPT x2 (unique_ptr, one function): two independent safe sites in ONE
// function body — the tool mints `loaned` for the first and `loaned2` for
// the second (deterministic, source order).
#include <memory>
#include <utility>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

class SafeTwoSites : public rclcpp::Node {
 public:
  SafeTwoSites() : rclcpp::Node("safe_two_sites") {
    pub_a_ = create_publisher<std_msgs::msg::String>("two_a", 10);
    pub_b_ = create_publisher<std_msgs::msg::String>("two_b", 10);
  }

  void tick() {
    auto first = std::make_unique<std_msgs::msg::String>();
    first->data = "site one";
    pub_a_->publish(std::move(first));
    auto second = std::make_unique<std_msgs::msg::String>();
    second->data = "site two";
    pub_b_->publish(std::move(second));
  }

 private:
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_a_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_b_;
};
