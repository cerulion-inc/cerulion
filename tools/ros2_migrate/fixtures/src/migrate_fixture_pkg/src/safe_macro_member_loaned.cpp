// SPDX-License-Identifier: AGPL-3.0-only
// ACCEPT (unique_ptr) with a MINT constraint: the body invokes a macro
// whose replacement list references a member named `loaned` — the raw
// body text shows only `CERULION_FIXTURE_RECORD()`, so a text-only shadow
// scan cannot see the collision, and a local minted as `loaned` would
// SHADOW the member inside the rewritten body (the
// MACRO_SHADOWING class). The AST name scan sees through the expansion and
// steers the mint to `loaned2` (asserted per-replacement by
// assert_matrix.py). This is the mutation fixture for
// CERULION_MIGRATE_MUTANT_DROP_AST_NAME_SCAN under the "shadow" kill
// mode: with the scan compiled out, the site still rewrites but mints the
// colliding `loaned` — the kill is the colliding name in the output.
#include <memory>
#include <utility>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

#define CERULION_FIXTURE_RECORD() (loaned += 1)

class SafeMacroMemberLoaned : public rclcpp::Node {
 public:
  SafeMacroMemberLoaned() : rclcpp::Node("safe_macro_member_loaned") {
    pub_ = create_publisher<std_msgs::msg::String>("macro_member", 10);
  }

  void tick() {
    auto msg = std::make_unique<std_msgs::msg::String>();
    msg->data = "the macro references the member the text hides";
    CERULION_FIXTURE_RECORD();
    pub_->publish(std::move(msg));
  }

 private:
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_;
  int loaned = 0;
};
