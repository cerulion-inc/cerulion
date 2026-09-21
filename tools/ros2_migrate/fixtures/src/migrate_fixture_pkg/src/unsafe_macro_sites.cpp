// SPDX-License-Identifier: AGPL-3.0-only
// REFUSE (macro-expansion) x2.
//
// TWO distinct publish call sites, both spelled inside ONE macro body. The
// body-text guard refuses both (the body range is macro-tainted), so both are
// reported as candidates — and THAT is the point of this fixture: a
// candidate anchored at its SPELLING location makes both sites
// resolve to the identical (file, line) inside the `#define`, and the Rust
// side's (file, line, reason) dedup collapses them into ONE. The operator sees
// a single manual-review entry, pointing at a `#define` they may not own,
// while the second real site vanishes from the report AND the manifest.
//
// Anchoring at the EXPANSION location gives each invocation its own line, so
// the matrix expects TWO. (The other half of the fix — a per-site COLUMN, for
// two invocations that share one line — is pinned by the engine unit arm
// `candidates_on_one_line_are_distinct_sites_when_their_columns_differ`,
// where two columns can be constructed exactly.)
#include <memory>
#include <utility>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

#define CERULION_FIXTURE_PUBLISH(WHAT)                             \
  {                                                                \
    auto msg = std::make_unique<std_msgs::msg::String>();          \
    msg->data = WHAT;                                              \
    pub_->publish(std::move(msg));                                 \
  }

class UnsafeMacroSites : public rclcpp::Node {
 public:
  UnsafeMacroSites() : rclcpp::Node("unsafe_macro_sites") {
    pub_ = create_publisher<std_msgs::msg::String>("macro_sites", 10);
  }

  void tick() {
    CERULION_FIXTURE_PUBLISH("first")
    CERULION_FIXTURE_PUBLISH("second")
  }

 private:
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_;
};
