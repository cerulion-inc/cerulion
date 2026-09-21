// SPDX-License-Identifier: AGPL-3.0-only
// REFUSE (message-built-elsewhere): the message variable is initialized
// from a CUSTOM method that merely SHARES the name `borrow_loaned_message`
// — it is not rclcpp::Publisher's loaned-message API (it returns a plain
// heap message from a pool-like holder). "Already migrated" may only be
// claimed on PROOF (the callee's class is rclcpp::Publisher); an impostor
// falls through to normal analysis and is REPORTED, never silently
// skipped as idempotent. The genuine-idempotence ACCEPT control is
// already_loaned.cpp (real `pub_->borrow_loaned_message()`, expected
// silent: no rewrite, no candidate). NOTE (matrix12): this fixture no
// longer isolates CERULION_MIGRATE_MUTANT_DROP_LOAN_PROOF — the
// same-publisher TARGET proof (a later review addition) independently
// refuses this cross-chain
// shape (pool_ != pub_), so with the class proof compiled out the site
// STILL reports. The isolating mutation fixture is
// unsafe_derived_loan.cpp (same chain, derived hider — only the class
// proof refuses it); this file stays as the plain cross-chain-impostor
// matrix row.
#include <memory>
#include <utility>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

class FakeLoanPool {
 public:
  // NOT rclcpp's API — a pool handing out plain messages.
  std_msgs::msg::String *borrow_loaned_message() { return &slot_; }

 private:
  std_msgs::msg::String slot_;
};

class UnsafeFakeLoan : public rclcpp::Node {
 public:
  UnsafeFakeLoan() : rclcpp::Node("unsafe_fake_loan") {
    pub_ = create_publisher<std_msgs::msg::String>("fake_loan", 10);
  }

  void tick() {
    auto msg = pool_.borrow_loaned_message();
    msg->data = "from the impostor pool";
    pub_->publish(*msg);
  }

 private:
  FakeLoanPool pool_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_;
};
