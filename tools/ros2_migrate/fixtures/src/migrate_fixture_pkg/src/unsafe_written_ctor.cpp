// SPDX-License-Identifier: AGPL-3.0-only
// REFUSE (unsupported-publish-shape, both sites): the publish argument is a
// WRITTEN construction — an explicit copy the author asked for. The rewrite
// evaluates its argument into a loaned slot, so unwrapping a written
// constructor would silently DELETE the construction (and any behavior it
// performs). Only COMPILER-INSERTED copy/move wraps are transparent
// (`stripWrappers`); a written `T{*msg}` (CXXTemporaryObjectExpr) and a
// written `T(*msg)` (functional cast) both stay wrapped and report as
// manual candidates. One publish per FUNCTION on purpose: the braced site
// is the mutation discriminator, and a second same-function use of `msg`
// would let the use-after-publish scan refuse it for the wrong reason
// under the mutant. This is the mutation fixture for
// CERULION_MIGRATE_MUTANT_DROP_CTOR_WRITTEN_GUARD: with the written-ctor
// guard compiled out, the braced construction is unwrapped to `*msg` and
// that site flips to a shared_ptr rewrite — the kill.
#include <memory>
#include <utility>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

class UnsafeWrittenCtor : public rclcpp::Node {
 public:
  UnsafeWrittenCtor() : rclcpp::Node("unsafe_written_ctor") {
    pub_ = create_publisher<std_msgs::msg::String>("written_ctor", 10);
  }

  void tick_braced() {
    auto msg = std::make_shared<std_msgs::msg::String>();
    msg->data = "explicitly copied (braced)";
    // Braced written construction — the mutant discriminator.
    pub_->publish(std_msgs::msg::String{*msg});
  }

  void tick_paren() {
    auto msg = std::make_shared<std_msgs::msg::String>();
    msg->data = "explicitly copied (paren)";
    // Paren written construction (functional cast) — same class, spelled
    // the other way.
    pub_->publish(std_msgs::msg::String(*msg));
  }

 private:
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_;
};
