// SPDX-License-Identifier: AGPL-3.0-only
// ACCEPT (both sites): the two verdicts the publisher-root analysis
// reaches for a QUALIFIED publisher, each in the shadow position that
// refuses when the publisher is written unqualified.
//
// The distinction is the whole point: treating "qualified" as a single
// answer is wrong in the dangerous direction. A qualified-id's TAIL is
// immune to block-scope declarations; its
// HEAD is an ordinary name and is not. So:
//
//   tick_global_scope_qualified     `::g_qualified_pub` — the head is global
//       scope, which is not nameable, so nothing in the expression is
//       resolved by unqualified lookup at all. This is the ONLY arm in the
//       matrix that reaches that verdict, and the arm whose consequence is
//       to skip the scan entirely and REWRITE.
//   tick_namespace_head_unshadowed  `qualified_pub::pub_` — the head IS a
//       name, so it is scanned like any other; here it is not re-declared,
//       so the site rewrites. The refusing twin, where a block-scope
//       `namespace … = …;` rebinds exactly that head, is in
//       unsafe_shadowed_publisher.cpp.
//
// Both arms put a local named `pub_` between the two edits — the position
// that REFUSES in unsafe_shadowed_publisher.cpp's first arm — because the
// terminal `pub_` is reached by member/qualified lookup in both spellings
// and is deliberately not scanned.
//
// (The third unshadowable spelling, an explicit `this->pub_`, is not a
// fixture: assert_matrix.py pins every rewrite replacement at exactly one
// `->`, calibrated to the arrow-form publishers every fixture uses, and
// `this->pub_->borrow_loaned_message()` carries two. Loosening a
// load-bearing oracle to host one fixture is the worse trade, and the
// global-scope arm above reaches the same verdict.)
#include <memory>
#include <utility>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

rclcpp::Publisher<std_msgs::msg::String>::SharedPtr g_qualified_pub;

namespace qualified_pub {
rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_;
}  // namespace qualified_pub

class SafeQualifiedPublisher : public rclcpp::Node {
 public:
  SafeQualifiedPublisher() : rclcpp::Node("safe_qualified_publisher") {
    g_qualified_pub =
        create_publisher<std_msgs::msg::String>("qualified_global", 10);
    qualified_pub::pub_ =
        create_publisher<std_msgs::msg::String>("qualified_ns", 10);
    other_ = create_publisher<std_msgs::msg::String>("qualified_other", 10);
  }

  void tick_global_scope_qualified() {
    auto msg = std::make_unique<std_msgs::msg::String>();
    msg->data = "global scope is not a name a declaration can rebind";
    auto pub_ = other_;
    (void)pub_;
    ::g_qualified_pub->publish(std::move(msg));
  }

  void tick_namespace_head_unshadowed() {
    auto msg = std::make_unique<std_msgs::msg::String>();
    msg->data = "the head is a name, and this one is not re-declared";
    auto pub_ = other_;
    (void)pub_;
    qualified_pub::pub_->publish(std::move(msg));
  }

 private:
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr other_;
};
