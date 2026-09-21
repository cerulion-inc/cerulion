// SPDX-License-Identifier: AGPL-3.0-only
// REFUSE (message-built-elsewhere): the loan is borrowed from ONE
// publisher and published on ANOTHER — a real defect (the loan belongs to
// pub_a_'s allocator; publishing it on pub_b_ is not idempotence), so the
// site must land in the manual-candidate report, never be silently
// skipped as "already migrated". The same-publisher proof
// compares the borrow target's decl chain with the publish target's; a
// mismatch falls through to normal decl classification. The genuine
// idempotence ACCEPT control is already_loaned.cpp (same shape, ONE
// publisher, still silent). This is the mutation fixture for
// CERULION_MIGRATE_MUTANT_DROP_LOAN_TARGET_PROOF: with the proof compiled
// out the site classifies already-migrated and VANISHES from the report —
// the kill oracle is the vanished report ("silent" mode).
#include <utility>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

class UnsafeCrossLoan : public rclcpp::Node {
 public:
  UnsafeCrossLoan() : rclcpp::Node("unsafe_cross_loan") {
    pub_a_ = create_publisher<std_msgs::msg::String>("cross_a", 10);
    pub_b_ = create_publisher<std_msgs::msg::String>("cross_b", 10);
  }

  void tick() {
    auto loaned = pub_a_->borrow_loaned_message();
    auto msg = &loaned.get();
    msg->data = "borrowed from a, published on b";
    pub_b_->publish(std::move(loaned));
  }

 private:
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_a_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_b_;
};
