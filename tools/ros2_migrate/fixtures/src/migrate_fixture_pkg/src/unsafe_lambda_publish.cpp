// SPDX-License-Identifier: AGPL-3.0-only
// REFUSE (publish-inside-lambda) x2.
//
// A `collectPublishCalls` that prunes at any LambdaExpr, on the premise that "a
// lambda's call operator is analysed as its own function context when the
// visitor reaches it", is wrong for this visitor: MigrateVisitor is a
// plain RecursiveASTVisitor with only a VisitFunctionDecl override, and clang
// reaches a lambda's operator() as its own FunctionDecl only via
// TraverseDecl(getLambdaClass()), gated behind shouldVisitImplicitCode()
// (false by default, not overridden). So a publish written inside a lambda
// would be collected by NOBODY: no edit AND no candidate — silence, in the single
// most common ROS 2 publisher idiom, contradicting this tool's own contract
// that everything unprovable is refused into the candidates list.
//
// Two sites: the timer callback (the idiom this exists for) and a plain
// immediately-invoked lambda (the same prune, minimal machinery).
#include <chrono>
#include <memory>
#include <utility>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

class UnsafeLambdaPublish : public rclcpp::Node {
 public:
  UnsafeLambdaPublish() : rclcpp::Node("unsafe_lambda_publish") {
    pub_ = create_publisher<std_msgs::msg::String>("lambda_publish", 10);
    timer_ = create_wall_timer(std::chrono::seconds(1), [this]() {
      auto msg = std::make_unique<std_msgs::msg::String>();
      msg->data = "tick";
      pub_->publish(std::move(msg));
    });
  }

  void tick() {
    auto emit = [this]() {
      auto msg = std::make_unique<std_msgs::msg::String>();
      msg->data = "immediate";
      pub_->publish(std::move(msg));
    };
    emit();
  }

 private:
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};
