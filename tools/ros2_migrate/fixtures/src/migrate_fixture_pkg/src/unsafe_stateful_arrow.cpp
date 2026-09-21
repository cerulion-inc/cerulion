// SPDX-License-Identifier: AGPL-3.0-only
// REFUSE (unsupported-publisher-type): the publisher is reached through a
// CUSTOM overloaded operator-> whose result changes between calls. The
// generated rewrite evaluates the publisher expression TWICE (borrow +
// publish), so a stateful operator-> hands the borrow and the publish
// DIFFERENT publishers — a silent routing change, and a loan published on
// a publisher that never issued it. Only std::shared_ptr/std::unique_ptr
// (the rclcpp Publisher SharedPtr/UniquePtr forms) qualify as arrow sugar;
// the standing ACCEPT controls for that are safe_shared.cpp/safe_unique.cpp
// (same message shape, std smart-pointer publisher, still rewritten).
// HISTORY: the dedicated arrow-identity
// refusal and its DROP_ARROW_IDENTITY mutant were DELETED — the
// exact-publisher gate provably subsumes them (a stateful arrow passing
// the exact-pointee test is unconstructible: no free operator-> exists
// and rclcpp::Publisher declares none — proof at the strip site in the
// prover). This fixture stays as the identity CLASS's row, now refused
// by that gate (the custom arrow does not strip, so the core's type is
// the wrapper's raw-pointer result — never exactly rclcpp::Publisher);
// its load-bearing guard is the DROP_EXACT_PUBLISHER mutant.
#include <memory>
#include <utility>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

class RotatingPublisherHandle {
 public:
  using Pub = rclcpp::Publisher<std_msgs::msg::String>;
  RotatingPublisherHandle(Pub::SharedPtr a, Pub::SharedPtr b)
      : a_(std::move(a)), b_(std::move(b)) {}
  // Stateful: each dereference yields the OTHER publisher.
  Pub *operator->() {
    flip_ = !flip_;
    return (flip_ ? a_ : b_).get();
  }

 private:
  Pub::SharedPtr a_;
  Pub::SharedPtr b_;
  bool flip_ = false;
};

class UnsafeStatefulArrow : public rclcpp::Node {
 public:
  UnsafeStatefulArrow()
      : rclcpp::Node("unsafe_stateful_arrow"),
        handle_(create_publisher<std_msgs::msg::String>("stateful_a", 10),
                create_publisher<std_msgs::msg::String>("stateful_b", 10)) {}

  void tick() {
    auto msg = std::make_unique<std_msgs::msg::String>();
    msg->data = "whichever publisher the arrow lands on";
    handle_->publish(std::move(msg));
  }

 private:
  RotatingPublisherHandle handle_;
};
