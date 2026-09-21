// SPDX-License-Identifier: AGPL-3.0-only
// REFUSE (publisher-reassigned, both sites): the publisher is MUTATED
// through a call between borrow and publish — `std::swap(pub_, spare_)`
// and a helper receiving the publisher by non-const reference. The
// assignment/reset/swap member scans cannot see either; the
// call-mutation arm refuses any call that can receive a publisher-chain
// decl by non-const reference or pointer (an unresolvable callee proves
// nothing and also refuses). One publish per FUNCTION so each site's
// refusal is attributable. This is the mutation fixture for
// CERULION_MIGRATE_MUTANT_DROP_CALL_MUTATION: with the arm compiled out,
// both sites flip to rewrites — the kill.
#include <memory>
#include <utility>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

// A helper that MAY replace the publisher — non-const reference.
void retune(rclcpp::Publisher<std_msgs::msg::String>::SharedPtr &pub);

class UnsafeSwapPublisher : public rclcpp::Node {
 public:
  UnsafeSwapPublisher() : rclcpp::Node("unsafe_swap_publisher") {
    pub_ = create_publisher<std_msgs::msg::String>("swap_a", 10);
    spare_ = create_publisher<std_msgs::msg::String>("swap_b", 10);
  }

  void tick_swapped() {
    auto msg = std::make_unique<std_msgs::msg::String>();
    msg->data = "borrowed from one, published on the other";
    std::swap(pub_, spare_);
    pub_->publish(std::move(msg));
  }

  void tick_helper() {
    auto msg = std::make_unique<std_msgs::msg::String>();
    msg->data = "the helper may have rebound the publisher";
    retune(pub_);
    pub_->publish(std::move(msg));
  }

 private:
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr spare_;
};
