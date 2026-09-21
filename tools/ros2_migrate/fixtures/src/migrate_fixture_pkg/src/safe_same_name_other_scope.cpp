// SPDX-License-Identifier: AGPL-3.0-only
// ACCEPT (all six sites): the ANTI-BLANKET control for the
// publisher-shadow refusal (unsafe_shadowed_publisher.cpp). Every site must
// still rewrite, so a "refuse whenever the name appears anywhere" fix fails
// right here.
//
// FOUR of the six arms do that the same way: a local spelled like the
// publisher, in a position where it shadows NEITHER edit. The last two are
// the accept twins of the mechanisms that are not a local at all — a
// using-DIRECTIVE (which declares no name) placed where it is in effect at
// both edits, and a namespace-qualified member access whose object root is
// simply not shadowed.
//
// The two positions are the two boundaries of the window the proof looks
// at, one on each side:
//
//   tick_shadow_after_publish  — the local is declared AFTER the publish.
//       Both the spliced borrow and the publish precede it, so both name
//       the member; the local rebinds nothing that either edit reads.
//   tick_same_name_inner_scope — the local lives in a nested block that is
//       CLOSED before the publish. It is textually between the two edits
//       and visible at neither, because a block-scope declaration dies with
//       its block.
//
// Two further arms pin what the refusals cannot: `tick_shadow_before_the_
// declaration` holds the window's LOWER bound (a scan widened to start at
// the top of the block would refuse it), and `tick_chain_root_unshadowed`
// holds the ROOT-only rule (a scan widened to every name in the publisher
// chain would refuse it). Both widenings are the natural over-fix, and
// before these arms existed both passed the whole matrix.
//
// Together with the refusals in unsafe_shadowed_publisher.cpp these pin the
// window as an open interval over the SHARED block's own declarations —
// not "the name appears somewhere in the function", not "the name appears
// somewhere between the two source offsets", and not "any name the
// publisher expression mentions".
#include <memory>
#include <utility>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

namespace acceptns {
rclcpp::Publisher<std_msgs::msg::String>::SharedPtr accepted_pub_;
}  // namespace acceptns

namespace qualns {
struct Held {
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub;
};
}  // namespace qualns

class SafeSameNameOtherScope : public rclcpp::Node {
 public:
  struct Holder {
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub;
  };

  SafeSameNameOtherScope() : rclcpp::Node("safe_same_name_other_scope") {
    pub_ = create_publisher<std_msgs::msg::String>("same_name_a", 10);
    other_ = create_publisher<std_msgs::msg::String>("same_name_b", 10);
    holder_.pub = pub_;
    held_.pub = pub_;
  }

  void tick_shadow_after_publish() {
    auto msg = std::make_unique<std_msgs::msg::String>();
    msg->data = "the shadow is declared after both edits";
    pub_->publish(std::move(msg));
    auto pub_ = other_;
    (void)pub_;
  }

  // The window's LOWER bound. The local IS the publisher and is declared
  // BEFORE the message, so both edits name it and nothing is shadowed —
  // this must rewrite. Without this arm, widening the scan to start at the
  // top of the block instead of at the declaration passes the whole matrix:
  // every other fixture either breaks at the publish before reaching its
  // shadow, or has no DeclStmt child to find.
  void tick_shadow_before_the_declaration() {
    auto pub_ = other_;
    auto msg = std::make_unique<std_msgs::msg::String>();
    msg->data = "declared before the message: the borrow names it too";
    pub_->publish(std::move(msg));
  }

  // ROOT, not every name in the chain. `holder_` is the only part of
  // `holder_.pub` that unqualified lookup resolves; `pub` is found by member
  // lookup inside holder_'s type, which no block-scope declaration can
  // change. So an unrelated local spelled like the TERMINAL member is not a
  // shadow and must not be treated as one — this is the arm the
  // "ROOT only, deliberately" rationale is written for, and the one that
  // fails if someone widens the scan to every publisherChainDecls name.
  void tick_chain_root_unshadowed() {
    auto msg = std::make_unique<std_msgs::msg::String>();
    msg->data = "only the terminal member's spelling collides";
    auto pub = other_;
    (void)pub;
    holder_.pub->publish(std::move(msg));
  }

  // ACCEPT: the using-DIRECTIVE sits BEFORE the message declaration, so both
  // edits are below it and resolve the name identically. This is the control
  // for the refusal next door: a fix that refused on any using-directive
  // ANYWHERE in the function, rather than in the window between the edits,
  // fails right here.
  void tick_using_directive_before_the_declaration() {
    using namespace acceptns;
    auto msg = std::make_unique<std_msgs::msg::String>();
    msg->data = "the directive precedes both edits";
    accepted_pub_->publish(std::move(msg));
  }

  // ACCEPT: a namespace-headed qualifier whose OBJECT root is not shadowed.
  // The refusing twin next door shadows `holder_`; here only an unrelated
  // name is declared between the edits, so both names resolve identically at
  // both points and the site must still rewrite. Without this arm, refusing
  // on any qualified member access would pass the whole matrix.
  void tick_ns_qualified_member_unshadowed() {
    auto msg = std::make_unique<std_msgs::msg::String>();
    msg->data = "only an unrelated name is declared between the edits";
    auto unrelated = other_;
    (void)unrelated;
    held_.qualns::Held::pub->publish(std::move(msg));
  }

  void tick_same_name_inner_scope() {
    auto msg = std::make_unique<std_msgs::msg::String>();
    {
      auto pub_ = other_;
      (void)pub_;
    }
    msg->data = "the same name in a block that closes before the publish";
    pub_->publish(std::move(msg));
  }

 private:
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr other_;
  Holder holder_;
  qualns::Held held_;
};
