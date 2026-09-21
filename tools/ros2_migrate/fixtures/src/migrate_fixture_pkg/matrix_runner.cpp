// SPDX-License-Identifier: AGPL-3.0-only
// The behavior pin: drive safe_unique's tick and require the "hello" payload
// to arrive at a plain subscription. run_matrix.sh runs this BEFORE and
// AFTER migration under rmw_fastrtps_cpp (a non-loaning path for
// std_msgs/String, so rclcpp's borrow_loaned_message falls back to
// allocate+copy) — the migrated code must behave identically.
#include <chrono>
#include <cstdio>
#include <memory>
#include <string>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

#include "src/matrix_api.hpp"

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = matrix_make_safe_unique();
  bool got = false;
  std::string payload;
  auto probe = std::make_shared<rclcpp::Node>("matrix_probe");
  auto sub = probe->create_subscription<std_msgs::msg::String>(
      "chatter", 10, [&](std_msgs::msg::String::ConstSharedPtr m) {
        got = true;
        payload = m->data;
      });
  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(node);
  exec.add_node(probe);
  auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(10);
  while (!got && std::chrono::steady_clock::now() < deadline) {
    matrix_tick_safe_unique(node.get());
    exec.spin_some();
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }
  rclcpp::shutdown();
  if (!got || payload != "hello") {
    std::fprintf(stderr, "matrix_runner: FAIL (got=%d payload='%s')\n",
                 static_cast<int>(got), payload.c_str());
    return 1;
  }
  std::printf("matrix_runner: received '%s'\n", payload.c_str());
  return 0;
}
