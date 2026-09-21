// SPDX-License-Identifier: AGPL-3.0-only
// REFUSE (unsupported-publisher-type): the publisher OBJECT's static type
// is a class DERIVED from rclcpp::Publisher. The publish-site gate proves
// only that the resolved publish() method's parent is rclcpp::Publisher —
// an INHERITED publish satisfies it — but the generated
// `borrow_loaned_message()` resolves on the WRITTEN static type, where the
// derived class hides the API with a different return type: the migrated
// source would fail to compile (`.get()` on a raw pointer), or worse. The
// exact-type gate refuses anything whose declared type (smart-
// pointer pointee for arrow forms) is not exactly rclcpp::Publisher. The
// exact-type ACCEPT controls are the standing safe fixtures (all
// `Publisher<T>::SharedPtr`). This is the mutation fixture for
// CERULION_MIGRATE_MUTANT_DROP_EXACT_PUBLISHER: with the gate compiled
// out, the safe_unique-shaped message side flips the site to a rewrite —
// the kill. Never constructed at runtime.
#include <memory>
#include <utility>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

class HidingPublisher : public rclcpp::Publisher<std_msgs::msg::String> {
 public:
  // Hides the base loaned-message API with a different return type.
  std_msgs::msg::String *borrow_loaned_message();
};

class UnsafeDerivedPublisher : public rclcpp::Node {
 public:
  UnsafeDerivedPublisher() : rclcpp::Node("unsafe_derived_publisher") {}

  void tick() {
    auto msg = std::make_unique<std_msgs::msg::String>();
    msg->data = "published through the derived static type";
    pub_->publish(std::move(msg));
  }

 private:
  // Deliberately never initialized — the fixture only has to COMPILE and
  // be ANALYZED (matrix_runner never ticks it).
  std::shared_ptr<HidingPublisher> pub_;
};
