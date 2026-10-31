// SPDX-License-Identifier: AGPL-3.0-only
// Node "mixer": subscribes /a; a ROS-time timer at 7 Hz publishes the fold of
// every input in arrival order and every fire's ROS time on /out, so the
// output is a pure function of the callback order and the inputs.
//   MIXER_WALLCLOCK=1  also folds std::chrono::system_clock at each fire (the wall-clock control)
//   PROBE_MT=1         spins a 2-thread MultiThreadedExecutor instead
//   PROBE_CB_SLEEP_US  sleeps that long in each /a callback (a callback that does work)
#include <chrono>
#include <cstdlib>
#include <thread>
#include "probe_common.hpp"
#include "std_msgs/msg/string.hpp"

class Mixer : public rclcpp::Node
{
public:
  Mixer()
  : Node("mixer"), log_("mixer"), wall_(probe::env_on("MIXER_WALLCLOCK"))
  {
    pub_ = create_publisher<std_msgs::msg::String>("/out", 10);
    sub_ = create_subscription<std_msgs::msg::String>(
      "/a", 10, [this](const std_msgs::msg::String & m) {
        state_ = probe::fold(state_, "a", m.data);
        log_.log("a", "inp", m.data);
        if (sleep_us_ > 0) {
          std::this_thread::sleep_for(std::chrono::microseconds(sleep_us_));
        }
      });
    timer_ = rclcpp::create_timer(
      this, get_clock(), std::chrono::nanoseconds(1000000000LL / 7), [this] {tick();});
  }

private:
  void tick()
  {
    int64_t now = get_clock()->now().nanoseconds();
    if (now == 0) {return;}
    std::string extra;
    if (wall_) {
      extra = std::to_string(
        std::chrono::system_clock::now().time_since_epoch().count());
    }
    state_ = probe::fold(state_, "t", std::to_string(now) + extra);
    std_msgs::msg::String m;
    m.data = state_;
    pub_->publish(m);
    log_.log("timer", "out", m.data, now);
  }
  probe::CbLog log_;
  bool wall_;
  long sleep_us_ = std::getenv("PROBE_CB_SLEEP_US") ? std::atol(std::getenv("PROBE_CB_SLEEP_US")) : 0;
  std::string state_ = "0";
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  probe::spin(std::make_shared<Mixer>());
  rclcpp::shutdown();
  return 0;
}
