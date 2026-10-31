// SPDX-License-Identifier: AGPL-3.0-only
// Shared helpers of the rclcpp replay probe: a deterministic fold (64-bit
// FNV-1a, hex) and a per-node callback log, one JSON line per callback in
// callback order, at <PROBE_OUT>/<node>.jsonl.
#pragma once
#include <cinttypes>
#include <cstdio>
#include <cstdlib>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"

namespace probe
{
inline std::string fold(const std::string & state, const std::string & tag, const std::string & v)
{
  uint64_t h = 0xcbf29ce484222325ULL;
  auto eat = [&h](const std::string & s) {
      for (unsigned char c : s) {
        h ^= c;
        h *= 0x100000001b3ULL;
      }
    };
  eat(state);
  eat(tag);
  eat(v);
  char buf[17];
  std::snprintf(buf, sizeof buf, "%016" PRIx64, h);
  return buf;
}

inline bool env_on(const char * name)
{
  const char * v = std::getenv(name);
  return v != nullptr && std::string(v) == "1";
}

class CbLog
{
public:
  explicit CbLog(const std::string & node)
  {
    const char * d = std::getenv("PROBE_OUT");
    std::string path = std::string(d ? d : ".") + "/" + node + ".jsonl";
    f_ = std::fopen(path.c_str(), "w");
  }
  ~CbLog()
  {
    if (f_) {std::fclose(f_);}
  }
  // One line: {"cb": .., "i": .., "<key>": "<val>"} (plus "now" when given).
  void log(const std::string & cb, const std::string & key, const std::string & val, int64_t now = -1)
  {
    if (!f_) {return;}
    if (now >= 0) {
      std::fprintf(
        f_, "{\"cb\": \"%s\", \"i\": %d, \"now\": %" PRId64 ", \"%s\": \"%s\"}\n",
        cb.c_str(), i_, now, key.c_str(), val.c_str());
    } else {
      std::fprintf(
        f_, "{\"cb\": \"%s\", \"i\": %d, \"%s\": \"%s\"}\n", cb.c_str(), i_, key.c_str(),
        val.c_str());
    }
    std::fflush(f_);
    ++i_;
  }

private:
  std::FILE * f_ = nullptr;
  int i_ = 0;
};

// Spin `node` on a SingleThreadedExecutor, or on a 2-thread
// MultiThreadedExecutor when PROBE_MT=1.
inline void spin(const rclcpp::Node::SharedPtr & node)
{
  if (env_on("PROBE_MT")) {
    rclcpp::executors::MultiThreadedExecutor ex(rclcpp::ExecutorOptions(), 2);
    ex.add_node(node);
    ex.spin();
  } else {
    rclcpp::executors::SingleThreadedExecutor ex;
    ex.add_node(node);
    ex.spin();
  }
}
}  // namespace probe
