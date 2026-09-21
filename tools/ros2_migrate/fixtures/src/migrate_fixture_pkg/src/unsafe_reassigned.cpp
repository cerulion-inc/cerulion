// SPDX-License-Identifier: AGPL-3.0-only
// REFUSE (publisher-reassigned): the publisher field is assigned inside the
// function — the borrow source could differ from the publish target.
#include <memory>
#include <utility>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

class UnsafeReassigned : public rclcpp::Node {
 public:
  UnsafeReassigned() : rclcpp::Node("unsafe_reassigned") {
    pub_ = create_publisher<std_msgs::msg::String>("reassigned_a", 10);
  }

  void tick(bool rotate) {
    auto msg = std::make_unique<std_msgs::msg::String>();
    msg->data = "which publisher?";
    if (rotate) {
      pub_ = create_publisher<std_msgs::msg::String>("reassigned_b", 10);
    }
    pub_->publish(std::move(msg));
  }

 private:
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_;
};

// The CHAIN arm: the publisher itself is never assigned,
// but the object HOLDING it is — `holder_.pub` after `holder_ = spare_;`
// is a different publisher than at borrow time, so the whole chain must be
// scanned, not just the terminal field.
struct ReassignedPubHolder {
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub;
};

class UnsafeReassignedChain : public rclcpp::Node {
 public:
  UnsafeReassignedChain() : rclcpp::Node("unsafe_reassigned_chain") {
    holder_.pub = create_publisher<std_msgs::msg::String>("chain_a", 10);
    spare_.pub = create_publisher<std_msgs::msg::String>("chain_b", 10);
  }

  void tick(bool rotate) {
    auto msg = std::make_unique<std_msgs::msg::String>();
    msg->data = "holder chain";
    if (rotate) {
      holder_ = spare_;
    }
    holder_.pub->publish(std::move(msg));
  }

 private:
  ReassignedPubHolder holder_;
  ReassignedPubHolder spare_;
};
