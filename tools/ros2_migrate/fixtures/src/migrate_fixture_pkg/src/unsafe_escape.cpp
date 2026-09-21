// SPDX-License-Identifier: AGPL-3.0-only
// REFUSE (pointer-escapes): the raw pointer leaves the function through
// helper(msg.get()) — .get() is a member FUNCTION use, not a field fill.
// This is the mutation fixture for CERULION_MIGRATE_MUTANT_DROP_ESCAPE_CHECK:
// with the escape check compiled out, this file is (incorrectly) rewritten
// and the matrix fails.
#include <memory>
#include <utility>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

namespace {
void remember_for_later(std_msgs::msg::String *m) { (void)m; }
}  // namespace

class UnsafeEscape : public rclcpp::Node {
 public:
  UnsafeEscape() : rclcpp::Node("unsafe_escape") {
    pub_ = create_publisher<std_msgs::msg::String>("escape", 10);
  }

  void tick() {
    auto msg = std::make_unique<std_msgs::msg::String>();
    msg->data = "escapes";
    remember_for_later(msg.get());
    pub_->publish(std::move(msg));
  }

 private:
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_;
};
