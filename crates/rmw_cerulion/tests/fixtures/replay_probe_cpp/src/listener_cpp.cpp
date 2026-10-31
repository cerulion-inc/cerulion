// SPDX-License-Identifier: AGPL-3.0-only
// Node "listener_cpp": subscribes /out, folds every input in arrival order and
// republishes the fold on /heard (its output, compared on replay).
#include "probe_common.hpp"
#include "std_msgs/msg/string.hpp"

class Listener : public rclcpp::Node
{
public:
  Listener()
  : Node("listener_cpp"), log_("listener_cpp")
  {
    pub_ = create_publisher<std_msgs::msg::String>("/heard", 10);
    sub_ = create_subscription<std_msgs::msg::String>(
      "/out", 10, [this](const std_msgs::msg::String & m) {
        state_ = probe::fold(state_, "o", m.data);
        std_msgs::msg::String h;
        h.data = state_;
        pub_->publish(h);
        log_.log("out", "out", h.data);
      });
  }

private:
  probe::CbLog log_;
  std::string state_ = "0";
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  probe::spin(std::make_shared<Listener>());
  rclcpp::shutdown();
  return 0;
}
