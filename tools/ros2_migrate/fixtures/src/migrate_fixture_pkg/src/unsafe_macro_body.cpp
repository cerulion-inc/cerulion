// SPDX-License-Identifier: AGPL-3.0-only
// REFUSE (macro-expansion): the function BODY's closing brace comes from a
// macro, so the body's source range is macro-tainted and its text is
// unavailable — while the publish SITE's own decl/call/arg ranges are
// clean interior spans. The minted-name shadow scan reads the body text;
// with none, a minted `loaned` could silently shadow a member or global
// the invisible text references, so every site in such a body refuses
// loudly (NAME_SHADOWING) instead of proceeding on a guessed
// name. This is the mutation fixture for
// CERULION_MIGRATE_MUTANT_DROP_BODY_TEXT_GUARD: with the guard compiled
// out the interior site proceeds on an empty shadow scan and flips to a
// rewrite — the kill.
#include <memory>
#include <utility>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

#define CERULION_FIXTURE_TICK_END }

class UnsafeMacroBody : public rclcpp::Node {
 public:
  UnsafeMacroBody() : rclcpp::Node("unsafe_macro_body") {
    pub_ = create_publisher<std_msgs::msg::String>("macro_body", 10);
  }

  // clang-format off
  void tick() {
    auto msg = std::make_unique<std_msgs::msg::String>();
    msg->data = "the body ends in a macro";
    pub_->publish(std::move(msg));
  CERULION_FIXTURE_TICK_END
  // clang-format on

 private:
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_;
};
