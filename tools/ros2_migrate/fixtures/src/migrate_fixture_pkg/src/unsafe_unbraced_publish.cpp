// SPDX-License-Identifier: AGPL-3.0-only
// REFUSE (conditional-publish, both sites): the publish sits under
// UNBRACED control flow — `if (ready_) pub_->publish(...)` and an
// unbraced for-loop body. Neither body is a CompoundStmt, so the publish
// shares the FUNCTION block with the declaration and the old
// nearest-compound compare accepted it — while the rewrite would borrow
// UNCONDITIONALLY and publish on a branch (leaking the loan on the
// not-taken path) or repeatedly (reusing one loan across iterations).
// The control-flow walk refuses any control construct between
// the publish and the shared compound. One publish per FUNCTION so each
// refusal is attributable. This is the mutation fixture for
// CERULION_MIGRATE_MUTANT_DROP_CONTROL_FLOW_WALK: with the walk compiled
// out, both safe_unique-shaped sites flip to rewrites — the kill.
#include <memory>
#include <utility>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

class UnsafeUnbracedPublish : public rclcpp::Node {
 public:
  UnsafeUnbracedPublish() : rclcpp::Node("unsafe_unbraced_publish") {
    pub_ = create_publisher<std_msgs::msg::String>("unbraced", 10);
  }

  // clang-format off
  void tick_if() {
    auto msg = std::make_unique<std_msgs::msg::String>();
    msg->data = "published only when ready";
    if (ready_) pub_->publish(std::move(msg));
  }

  void tick_loop() {
    auto msg = std::make_unique<std_msgs::msg::String>();
    msg->data = "published per iteration from one loan";
    for (int i = 0; i < 2; ++i) pub_->publish(std::move(msg));
  }
  // clang-format on

 private:
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_;
  bool ready_ = false;
};
