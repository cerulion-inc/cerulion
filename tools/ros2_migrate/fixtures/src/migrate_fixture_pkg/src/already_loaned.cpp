// SPDX-License-Identifier: AGPL-3.0-only
// SILENT (idempotence): already the migrated shape — the tool proposes NO
// rewrite and NO candidate for it, so a second migration pass over migrated
// code produces an empty diff.
#include <utility>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

class AlreadyLoaned : public rclcpp::Node {
 public:
  AlreadyLoaned() : rclcpp::Node("already_loaned") {
    pub_ = create_publisher<std_msgs::msg::String>("loaned_out", 10);
  }

  void tick() {
    auto loaned = pub_->borrow_loaned_message();
    auto msg = &loaned.get();
    msg->data = "already migrated";
    pub_->publish(std::move(loaned));
  }

 private:
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_;
};
