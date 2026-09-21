// SPDX-License-Identifier: AGPL-3.0-only
// REFUSE (pointer-escapes) x2, ACCEPT (stack) x1.
//
// A field access is only a FILL if the glvalue it produces does not ESCAPE.
// An `isFillUse` that accepts ANY use whose parent is a field MemberExpr and
// stops there, while `collectUses` tracks only direct DeclRefExprs naming the
// message, leaves an alias derived from a field invisible to the
// use-after-publish scan. Paired with a `kStack` local published BY COPY
// (`publish(msg)` binds `publish(const T&)`), that lets the prover rewrite
// code in which the alias was NEVER dangling into `publish(std::move(loaned))`
// — moving the loan out from under a live pointer, i.e. turning well-defined
// code into a use-after-move.
//
// The two refusing functions are the two escape shapes: address-of, and a
// reference binding. `tick_plain_fill` is the ANTI-BLANKET control in the
// same file — an ordinary field fill must STILL rewrite, or the fix is just
// a refusal of everything.
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

namespace {
std::string *g_sink = nullptr;
// Keeps the reference it is handed — the callee this fixture exists for.
void register_sink(std::string &s) { g_sink = &s; }
// Takes a COPY: nothing of the message outlives the call.
void take_by_value(std::string s) { (void)s; }
}  // namespace

class UnsafeFieldAlias : public rclcpp::Node {
 public:
  UnsafeFieldAlias() : rclcpp::Node("unsafe_field_alias") {
    pub_ = create_publisher<std_msgs::msg::String>("field_alias", 10);
  }

  // The address of a field outlives the publish.
  void tick_addr_of() {
    std_msgs::msg::String msg;
    msg.data = "aliased";
    std::string *p = &msg.data;
    pub_->publish(msg);
    p->append("!");  // well-defined BEFORE a rewrite; UB after one
  }

  // A reference bound to a field is the same escape by another spelling.
  void tick_ref_bind() {
    std_msgs::msg::String msg;
    msg.data = "aliased";
    std::string &r = msg.data;
    pub_->publish(msg);
    r.append("!");  // well-defined BEFORE a rewrite; UB after one
  }

  // The THIRD spelling of the same escape — the field handed to a
  // callee that keeps the reference. A reference parameter binds with no cast
  // node, so the earlier walk saw the call as its parent, matched no arm, and
  // fell through as "does not escape".
  void tick_ref_argument() {
    std_msgs::msg::String msg;
    msg.data = "aliased";
    register_sink(msg.data);  // binds std::string& and stores it
    pub_->publish(msg);
    g_sink->append("!");  // well-defined BEFORE a rewrite; UB after one
  }

  // CONTROL: an ordinary fill, a COPY of a field (not an alias), and a member
  // CALL on a field (the field is the OBJECT, not an argument) must still be
  // provable — the refusal is targeted, not blanket.
  void tick_plain_fill() {
    std_msgs::msg::String msg;
    msg.data = "plain";
    std::string copy = msg.data;
    (void)copy;
    msg.data.push_back('!');   // object position — not an argument
    take_by_value(msg.data);   // a by-VALUE parameter copies; no alias
    pub_->publish(msg);
  }

 private:
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_;
};
