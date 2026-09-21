// SPDX-License-Identifier: AGPL-3.0-only
// ACCEPT (unique_ptr) with a MINT constraint: an object-like preprocessor
// macro named `loaned` is ACTIVE in the translation unit — from a header in
// real life; defined here after the includes, which is the same
// preprocessor state at the function. The name appears NOWHERE in the
// function: no local, no AST reference, not in the raw body text, so the
// three name scans all clear it — yet an emitted `auto loaned = ...`
// would be macro-expanded into `auto 42 = ...` and the migration would not
// compile (the MACRO_COLLISION class). The preprocessor macro scan steers
// the mint to `loaned2` (asserted per-replacement by assert_matrix.py).
// Mutation fixture for CERULION_MIGRATE_MUTANT_DROP_MACRO_NAME_SCAN under
// the "shadow" kill mode: with the scan compiled out the site still
// rewrites but mints the colliding `loaned` — the kill is the colliding
// name in the output. (The body text deliberately never spells the macro's
// name, or the raw-text scan alone would steer the mint and a dropped
// scan would go undetected.)
#include <memory>
#include <utility>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

#define loaned 42

class SafeMacroLoanedDefined : public rclcpp::Node {
 public:
  SafeMacroLoanedDefined() : rclcpp::Node("safe_macro_loaned_defined") {
    pub_ = create_publisher<std_msgs::msg::String>("macro_defined", 10);
  }

  void tick() {
    auto msg = std::make_unique<std_msgs::msg::String>();
    msg->data = "an object-like macro is active in this translation unit";
    pub_->publish(std::move(msg));
  }

 private:
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_;
};
