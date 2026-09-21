// SPDX-License-Identifier: AGPL-3.0-only
// REFUSE (publisher-expr-not-trivial): the publish target is chosen at
// runtime from several publishers — the borrow source cannot be proven to be
// the publish target, and the publisher expression is evaluated twice.
// This is the mutation fixture for
// CERULION_MIGRATE_MUTANT_DROP_PUBLISHER_TRIVIALITY.
#include <memory>
#include <utility>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

class UnsafeWrongPublisher : public rclcpp::Node {
 public:
  UnsafeWrongPublisher() : rclcpp::Node("unsafe_wrong_publisher") {
    pub_a_ = create_publisher<std_msgs::msg::String>("wrong_a", 10);
    pub_b_ = create_publisher<std_msgs::msg::String>("wrong_b", 10);
  }

  void tick(bool left) {
    auto msg = std::make_unique<std_msgs::msg::String>();
    msg->data = "one of two";
    (left ? pub_a_ : pub_b_)->publish(std::move(msg));
  }

 private:
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_a_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_b_;
};
