// SPDX-License-Identifier: AGPL-3.0-only
// REFUSE (two publisher-expr-not-trivial): the publisher expression carries a
// TEMPLATE ARGUMENT LIST, and a template argument's names are resolved by
// ordinary UNQUALIFIED lookup at the publish site — so a block-scope
// declaration written between the two edits changes which publisher the SAME
// written text names.
//
// This is the seventh mechanism in the shadowing class, and the one
// the "ROOT only" rule does not model. That proof resolves the publisher to
// the identifiers unqualified lookup resolves: a chain ROOT, and a
// qualifier's OUTERMOST specifier. A template argument is neither. It is not
// reached by MEMBER lookup (which is what makes a chain's tail immune) and it
// is not reached by QUALIFIED lookup (which is what makes a qualified-id's
// tail immune): `ns::Holder<Tag>` does not find `ns::Tag`, it needs
// `ns::Holder<ns::Tag>`.
//
// MEASURED, clang++ -std=c++17 -Wall -Wextra: the identical written text
// `ns::Holder<TagA>::id` reads 11 above a block-scope `using TagA = Beta;`
// and 22 below it, neither carrying a diagnostic. Migrated, the borrow is
// spliced ABOVE the alias and names a different specialization than the
// publish does — a silent cross-publisher loan out of code that was
// well-defined. The migrated source compiles too, so the matrix's own build
// check does not catch it either.
//
// Both spellings are pinned, because they reach the gap through different
// parts of the analysis:
//
//   tick_template_args_on_the_id    `g_pub_v<TagA>` — a VARIABLE TEMPLATE.
//       The DeclRefExpr's written range spans `g_pub_v<TagA>` (AST-dump
//       verified), so the argument really is in the text that gets spliced,
//       while the root collects only `g_pub_v`.
//   tick_template_arg_in_qualifier  `tmplq::Holder<TagA>::pub` — the argument
//       rides the qualifier's TAIL, which the classifier walks past on its
//       way to the outermost specifier `tmplq`. That head is then scanned,
//       and is clean, so the site passed.
//
// The refusal is taken on the TEXT (`pubText` carrying a `<`) rather than on
// either AST shape, deliberately: a text test is TOTAL over the class, and
// every earlier false accept here was reached by an AST shape nobody had
// enumerated. This is the mutation fixture for
// CERULION_MIGRATE_MUTANT_DROP_PUBLISHER_TEMPLATE_ARGS — with the belt
// compiled out BOTH sites flip to rewrites, which is the kill.
//
// The ACCEPT control is safe_qualified_publisher.cpp: the same qualified and
// namespace-scope publisher shapes WITHOUT template arguments, which must
// still rewrite. Without it, a belt that simply refused every qualified
// publisher would pass here.
#include <memory>
#include <utility>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

// The two tags the block-scope alias swaps between. Both specializations are
// assigned in the constructor: nothing in the matrix CALLS these arms (the
// runner drives one safe tick), but a fixture that would null-deref if it
// were called is not a sound fixture.
struct TagA {};
struct TagB {};

template <class Tag>
rclcpp::Publisher<std_msgs::msg::String>::SharedPtr g_pub_v;

namespace tmplq {
template <class Tag>
struct Holder {
  static rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub;
};
template <class Tag>
rclcpp::Publisher<std_msgs::msg::String>::SharedPtr Holder<Tag>::pub;
}  // namespace tmplq

class UnsafeTemplateArgumentPublisher : public rclcpp::Node {
 public:
  UnsafeTemplateArgumentPublisher()
      : rclcpp::Node("unsafe_template_argument_publisher") {
    g_pub_v<TagA> = create_publisher<std_msgs::msg::String>("tmpl_var_a", 10);
    g_pub_v<TagB> = create_publisher<std_msgs::msg::String>("tmpl_var_b", 10);
    tmplq::Holder<TagA>::pub =
        create_publisher<std_msgs::msg::String>("tmpl_static_a", 10);
    tmplq::Holder<TagB>::pub =
        create_publisher<std_msgs::msg::String>("tmpl_static_b", 10);
  }

  // REFUSE (publisher-expr-not-trivial): explicit template arguments on the
  // ID-EXPRESSION itself. `g_pub_v` is not re-declared anywhere, so the root
  // scan is clean and says so; `TagA` is the name that moved, and nothing
  // collected it.
  void tick_template_args_on_the_id() {
    auto msg = std::make_unique<std_msgs::msg::String>();
    msg->data = "a template argument is looked up unqualified at the publish";
    using TagA = TagB;  // rebinds the ARGUMENT, not the publisher's own name
    g_pub_v<TagA>->publish(std::move(msg));
  }

  // REFUSE (publisher-expr-not-trivial): a template argument inside a
  // NAMESPACE-headed qualifier. The head `tmplq` IS scanned — the shadowing
  // fix made sure of that — and it is clean. The argument in the tail is
  // what changes meaning, and the tail is deliberately not scanned because
  // for a NAME it would be an over-refusal.
  void tick_template_arg_in_qualifier() {
    auto msg = std::make_unique<std_msgs::msg::String>();
    msg->data = "the qualifier tail carries a name the head scan cannot see";
    using TagA = TagB;
    tmplq::Holder<TagA>::pub->publish(std::move(msg));
  }
};
