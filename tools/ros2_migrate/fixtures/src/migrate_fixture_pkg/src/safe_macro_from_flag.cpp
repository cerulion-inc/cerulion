// SPDX-License-Identifier: AGPL-3.0-only
// ACCEPT (unique_ptr) with a MINT constraint — the twin of
// safe_macro_loaned_defined.cpp: the object-like macro `loaned` arrives
// from the COMPILE COMMAND (`-Dloaned=42`, set per-source in
// CMakeLists.txt so it lands in this TU's compile_commands.json entry), so
// it lives in the preprocessor's PREDEFINES buffer rather than in any
// source line. The prover's PPCallbacks observer is registered in
// CreateASTConsumer, which runs BEFORE the predefines are lexed at
// EnterMainSourceFile (measured on a real tooling run: a callback
// registered there reports -D macros, __STDC__, __cplusplus and in-TU
// defines alike), so the mint must still steer to `loaned2` (asserted
// per-replacement by assert_matrix.py). The body text never spells the
// macro's name, so the raw-text scan cannot steer the mint on its own.
#include <memory>
#include <utility>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

class SafeMacroFromFlag : public rclcpp::Node {
 public:
  SafeMacroFromFlag() : rclcpp::Node("safe_macro_from_flag") {
    pub_ = create_publisher<std_msgs::msg::String>("macro_from_flag", 10);
  }

  void tick() {
    auto msg = std::make_unique<std_msgs::msg::String>();
    msg->data = "an object-like macro arrives from the compile command";
    pub_->publish(std::move(msg));
  }

 private:
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_;
};
