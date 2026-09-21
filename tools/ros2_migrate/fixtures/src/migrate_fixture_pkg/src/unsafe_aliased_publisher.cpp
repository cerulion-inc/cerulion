// SPDX-License-Identifier: AGPL-3.0-only
// REFUSE (publisher-reassigned, both sites): a local REFERENCE alias
// (`auto &alias = pub_;`) and a POINTER alias (`auto *p = &pub_;`) to the
// publisher. A write through the alias (`alias = spare_;`) names the
// ALIAS's decl, not `pub_`'s, so the per-decl reassignment scan cannot
// see it — the generated borrow would run before the aliased write and
// the publish after it, sending one publisher's loan through another.
// The alias scan refuses the alias BINDING itself
// (conservative: tracking writes through aliases soundly is an escape
// analysis; refused-not-guessed is the contract). One publish per
// FUNCTION so each refusal is attributable. This is the mutation fixture
// for CERULION_MIGRATE_MUTANT_DROP_ALIAS_SCAN: with the scan compiled
// out, the aliased writes are invisible to the per-decl scan and both
// safe_unique-shaped sites flip to rewrites — the kill.
#include <memory>
#include <utility>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

class UnsafeAliasedPublisher : public rclcpp::Node {
 public:
  UnsafeAliasedPublisher() : rclcpp::Node("unsafe_aliased_publisher") {
    pub_ = create_publisher<std_msgs::msg::String>("alias_a", 10);
    spare_ = create_publisher<std_msgs::msg::String>("alias_b", 10);
  }

  void tick_ref_alias() {
    auto &alias = pub_;
    auto msg = std::make_unique<std_msgs::msg::String>();
    msg->data = "rebound through the reference alias";
    alias = spare_;
    pub_->publish(std::move(msg));
  }

  void tick_ptr_alias() {
    auto *p = &pub_;
    auto msg = std::make_unique<std_msgs::msg::String>();
    msg->data = "rebound through the pointer alias";
    *p = spare_;
    pub_->publish(std::move(msg));
  }

 private:
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr spare_;
};
