// SPDX-License-Identifier: AGPL-3.0-only
// REFUSE (reuse-after-move + use-after-publish): one function publishes the
// same message twice; the other reads a field after the publish consumed it.
// This is the mutation fixture for
// CERULION_MIGRATE_MUTANT_DROP_USE_AFTER_PUBLISH (the use-after-publish
// half).
#include <memory>
#include <utility>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

class UnsafeReuseAfterMove : public rclcpp::Node {
 public:
  UnsafeReuseAfterMove() : rclcpp::Node("unsafe_reuse_after_move") {
    pub_ = create_publisher<std_msgs::msg::String>("reuse", 10);
  }

  // reuse-after-move: the shared message is published twice.
  void tick_twice() {
    auto msg = std::make_shared<std_msgs::msg::String>();
    msg->data = "twice";
    pub_->publish(*msg);
    pub_->publish(*msg);
  }

  // use-after-publish: a fill-shaped use lands after the publish consumed
  // the message. Deliberately fill-SHAPED (not an escape), so the
  // DROP_USE_AFTER_PUBLISH variant rewrites it and the matrix catches
  // the change.
  void tick_then_touch() {
    auto msg = std::make_unique<std_msgs::msg::String>();
    msg->data = "touch later";
    pub_->publish(std::move(msg));
    msg->data = "touched after publish";
  }

 private:
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_;
};
