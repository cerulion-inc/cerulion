// SPDX-License-Identifier: AGPL-3.0-only
// A ROS-time timer at 10 Hz publishes a counter "a:N" on /a (run with
// use_sim_time:=true). Node name talker_cpp.
#include <chrono>
#include "probe_common.hpp"
#include "std_msgs/msg/string.hpp"

class Talker : public rclcpp::Node
{
public:
  Talker()
  : Node("talker_cpp"), log_("talker_cpp")
  {
    pub_ = create_publisher<std_msgs::msg::String>("/a", 10);
    timer_ = rclcpp::create_timer(this, get_clock(), std::chrono::milliseconds(100), [this] {tick();});
  }

private:
  void tick()
  {
    int64_t now = get_clock()->now().nanoseconds();
    if (now == 0) {return;}
    ++n_;
    std_msgs::msg::String m;
    m.data = "a:" + std::to_string(n_);
    pub_->publish(m);
    log_.log("timer", "out", m.data, now);
  }
  probe::CbLog log_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  int n_ = 0;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  probe::spin(std::make_shared<Talker>());
  rclcpp::shutdown();
  return 0;
}
