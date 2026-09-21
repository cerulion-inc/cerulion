// SPDX-License-Identifier: AGPL-3.0-only
// REFUSE (message-built-elsewhere): the publisher is a DERIVED class of
// rclcpp::Publisher whose own `borrow_loaned_message` HIDES the base API
// and returns a plain message pointer. This is the shape that proves the
// CLASS proof is NOT subsumed by the same-publisher
// TARGET proof (the matrix12 finding): the borrow and the publish share
// ONE decl chain (`pub_`), so the target proof passes, and the publish
// resolves on the BASE rclcpp::Publisher, so the site gate passes — only
// the class proof (the borrow method's DECLARING class must be
// rclcpp::Publisher itself) refuses the hider. This is therefore the
// isolating mutation fixture for CERULION_MIGRATE_MUTANT_DROP_LOAN_PROOF:
// with the class proof compiled out the site classifies already-migrated
// and VANISHES from the report ("silent" kill mode). The cross-chain
// impostor (a pool class merely named like the API) is
// unsafe_fake_loan.cpp, which the TARGET proof also refuses — that is why
// it can no longer isolate this scenario. Never constructed at runtime —
// the analyzer and the compiler are the only consumers.
#include <memory>
#include <utility>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

class SneakyPublisher : public rclcpp::Publisher<std_msgs::msg::String> {
 public:
  // Hides rclcpp::Publisher's loaned-message API; returns a plain heap
  // message instead of a LoanedMessage.
  std_msgs::msg::String *borrow_loaned_message();
};

class UnsafeDerivedLoan : public rclcpp::Node {
 public:
  UnsafeDerivedLoan() : rclcpp::Node("unsafe_derived_loan") {}

  void tick() {
    auto msg = pub_->borrow_loaned_message();
    msg->data = "borrowed through the hider";
    pub_->publish(*msg);
  }

 private:
  // Deliberately never initialized/constructed — the fixture only has to
  // COMPILE and be ANALYZED (matrix_runner never ticks it).
  std::shared_ptr<SneakyPublisher> pub_;
};
