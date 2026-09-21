// SPDX-License-Identifier: AGPL-3.0-only
// REFUSE (seven publisher-shadowed, two publisher-expr-not-trivial,
// one publisher-lookup-changed):
// the publisher's name is
// RE-DECLARED between the message declaration and the publish, so the
// borrow the rewrite splices at the DECLARATION would resolve that name to
// a different publisher than the publish does.
//
// This is a NAME-LOOKUP defect, which is why every AST-level publisher
// proof walks past it: triviality, the exact-type gate and the
// reassignment/alias scans all reason about the decl the publish site
// RESOLVED, and each of them is satisfied here. What is not satisfied is
// the thing the rewrite actually relies on — that `pubText`, the publisher
// as written at the publish, still names that publisher after it is
// spliced ABOVE the shadowing declaration. Migrated, `tick_shadowed()`
// borrows a loan from `this->pub_` and publishes it on `other_`: a
// cross-publisher loan, silently, out of code that was well-defined.
//
// `tick_chain_root_shadowed()` is the same defect through a member CHAIN,
// and it is what pins that the proof keys on the chain's ROOT: `holder.pub`
// resolves `pub` by MEMBER lookup on holder's type (no declaration can
// shadow that), while `holder` is an ordinary unqualified name that the
// local `Holder holder = spare_;` rebinds. (The member is spelled without a
// trailing underscore ONLY so the local can shadow it — a shadow needs the
// two spellings to be equal.)
//
// The ACCEPT controls live in safe_same_name_other_scope.cpp — same names,
// positions where no shadow exists, and they must still rewrite; without
// them a proof that simply refused every function carrying the name would
// pass here. This is the mutation fixture for
// CERULION_MIGRATE_MUTANT_DROP_PUBLISHER_SHADOW: with the check compiled
// out all TEN sites flip to rewrites (the whole #ifndef block goes,
// including the qualifier refusals) — the kill.
#include <memory>
#include <utility>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

// The target of the third arm's block-scope using-declaration. DEFINED here,
// not `extern`: an undefined data symbol in the fixture shared library would
// fail to resolve when the matrix's runner loads it.
namespace shadow_ns {
rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_;
}  // namespace shadow_ns

// The namespace-ALIAS arm's two targets. A qualified-id's tail is immune to
// block-scope declarations; its HEAD is an ordinary name and is not.
namespace alias_target {
rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_;
}  // namespace alias_target
namespace alias_other {
rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_;
}  // namespace alias_other

// A TYPE-headed qualifier's target for the last arm.
struct QualBase {
  static rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_;
};
rclcpp::Publisher<std_msgs::msg::String>::SharedPtr QualBase::pub_;

// The using-DIRECTIVE arm's namespace, and the namespace-qualified-member
// arm's type. Both arms are built from code that compiles, so the
// hazard is real on well-formed input.
namespace lookup_ns {
rclcpp::Publisher<std_msgs::msg::String>::SharedPtr widened_pub_;
}  // namespace lookup_ns
namespace nsq {
struct Held {
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub;
};
}  // namespace nsq

// A base holding a publisher, so the last arm can qualify by CLASS name.
class QualifiedBase {
 public:
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr base_pub_;
};

class UnsafeShadowedPublisher : public rclcpp::Node, public QualifiedBase {
 public:
  struct Holder {
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub;
  };

  UnsafeShadowedPublisher() : rclcpp::Node("unsafe_shadowed_publisher") {
    base_pub_ = create_publisher<std_msgs::msg::String>("shadow_base", 10);
    pub_ = create_publisher<std_msgs::msg::String>("shadow_outer", 10);
    other_ = create_publisher<std_msgs::msg::String>("shadow_inner", 10);
    holder.pub = pub_;
    spare_.pub = other_;
  }

  void tick_shadowed() {
    auto msg = std::make_unique<std_msgs::msg::String>();
    msg->data = "borrowed from the member, published on the local";
    auto pub_ = other_;
    pub_->publish(std::move(msg));
  }

  // The same divergence reached WITHOUT declaring a variable: a block-scope
  // using-declaration introduces `pub_` into this block, so the publish
  // resolves to `shadow_ns::pub_` while the borrow spliced above it still
  // resolves to the member. This is the arm that makes the scan's choice of
  // `NamedDecl` over `VarDecl` a GATE rather than a comment — a `UsingDecl`
  // is not a `VarDecl`, so narrowing the scan silently re-opens exactly this.
  void tick_using_declaration_shadow() {
    auto msg = std::make_unique<std_msgs::msg::String>();
    msg->data = "borrowed from the member, published on the namespace one";
    using shadow_ns::pub_;
    pub_->publish(std::move(msg));
  }

  // A block-scope namespace ALIAS rebinds the qualified name's HEAD, so the
  // publish resolves to `alias_other::pub_` while the borrow spliced above
  // it still resolves to `alias_target::pub_`. This is the arm that refutes
  // "a qualified name cannot be shadowed" — a tempting assumption,
  // and the reason the accept control next door is a
  // qualified publisher whose head is NOT rebound.
  void tick_namespace_alias_shadow() {
    auto msg = std::make_unique<std_msgs::msg::String>();
    msg->data = "the alias rebinds the qualified name's head";
    namespace alias_target = alias_other;
    alias_target::pub_->publish(std::move(msg));
  }

  // A STRUCTURED BINDING declares `pub_` without any declaration of that
  // name appearing in the statement's own decl list: the DeclStmt carries
  // one unnamed DecompositionDecl and the BindingDecls hang off it. A scan
  // that walks only the decl list finds nothing and rewrites.
  void tick_structured_binding_shadow() {
    auto msg = std::make_unique<std_msgs::msg::String>();
    msg->data = "the binding shadows the member without being in decls()";
    auto [pub_, tag] = std::make_pair(other_, 0);
    (void)tag;
    pub_->publish(std::move(msg));
  }

  // The same declaration behind a statement WRAPPER. A labeled-statement
  // takes a statement and a declaration-statement is one, so the block's
  // child is a LabelStmt and a bare cast to DeclStmt walks straight past it.
  void tick_label_wrapped_shadow() {
    auto msg = std::make_unique<std_msgs::msg::String>();
    msg->data = "the shadow hides behind a label";
    // Deliberately no `goto` to consume the label: a jump between the two
    // edits is the OPEN reachability question filed as item 6 of this
    // issue, and coupling this fixture's expected REASON to a future change
    // there would make it fail for something it is not about. The label is
    // unused on purpose; this fixture package sets no -W flags.
  shadow_here:
    auto pub_ = other_;
    pub_->publish(std::move(msg));
  }

  // REFUSE (publisher-expr-not-trivial): a TYPE-headed qualifier. The analysis
  // resolves a qualified publisher's head so it can be scanned, and a type
  // head is the one it declines to resolve: the WRITTEN spelling can be an
  // alias (`using QualBase = Other;`) whose identifier differs from the
  // resolved type's name, and it is the written text that gets spliced, so
  // there is nothing here the analysis can reliably scan for.
  //
  // This arm is in the fixture because the refusal is a real BEHAVIOUR
  // CHANGE — before this refusal a type-qualified publisher was rewritten — and
  // because it is one of only two things in the matrix that reach the
  // "a qualifier head this analysis cannot name" verdict. Its reason differs from this file's other
  // arms deliberately; the file's subject is the publisher-root analysis,
  // not one reason string.
  void tick_type_qualified_publisher() {
    auto msg = std::make_unique<std_msgs::msg::String>();
    msg->data = "a type-headed qualifier is refused, not resolved";
    QualBase::pub_->publish(std::move(msg));
  }

  // REFUSE (publisher-expr-not-trivial): the SECOND spelling of a
  // type-headed qualifier, and a different AST node from the arm above.
  // `QualifiedBase::base_pub_` inside a member function is a MemberExpr over
  // an IMPLICIT `this` that carries the specifier — measured — whereas
  // `QualBase::pub_` (a static member of an unrelated class) is a
  // DeclRefExpr. The classifier handles a qualifier in both places, and
  // before this arm only the DeclRefExpr one was pinned: deleting the
  // MemberExpr branch made the root resolve to the MEMBER name `base_pub_`
  // while the spliced text still reads `QualifiedBase::base_pub_`, whose
  // head a block-scope `using QualifiedBase = Other;` rebinds — the round's
  // own false accept, one node shape further out, with every fixture green.
  void tick_qualified_inherited_member() {
    auto msg = std::make_unique<std_msgs::msg::String>();
    msg->data = "a class-qualified inherited member is a MemberExpr";
    QualifiedBase::base_pub_->publish(std::move(msg));
  }

  // REFUSE (publisher-lookup-changed): a using-DIRECTIVE does not DECLARE
  // the publisher's name, so a scan that only compares declared names walks
  // past it — and it still changes what the name means from that point on.
  // The borrow is spliced ABOVE it, where `widened_pub_` does not resolve at
  // all: MEASURED, the original compiles and the migrated source does not.
  // It can look harmless because stage 3b builds the
  // applied bytes; a USER has no stage 3b, and gets an invalid patch or a
  // commit that fails the build.
  void tick_using_directive_between() {
    auto msg = std::make_unique<std_msgs::msg::String>();
    msg->data = "the directive makes the name visible only from here";
    using namespace lookup_ns;
    widened_pub_->publish(std::move(msg));
  }

  // REFUSE (publisher-shadowed): a NAMESPACE-headed qualifier on a member
  // access whose OBJECT is shadowed. The publisher expression resolves two
  // names unqualified — `nsq` (the qualifier head) and `holder_` (the object
  // root) — and a walk that returns the first and stops leaves a local
  // `holder_` between the edits invisible. Such a walk closes the same
  // shape for a TYPE-headed qualifier by refusing it, and opens it here.
  void tick_ns_qualified_member_on_shadowed_holder() {
    auto msg = std::make_unique<std_msgs::msg::String>();
    msg->data = "the qualifier head is not the only name looked up";
    nsq::Held holder_ = spare_held_;
    holder_.nsq::Held::pub->publish(std::move(msg));
  }

  void tick_chain_root_shadowed() {
    auto msg = std::make_unique<std_msgs::msg::String>();
    msg->data = "borrowed through the member holder, published through a local";
    Holder holder = spare_;
    holder.pub->publish(std::move(msg));
  }

 private:
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr other_;
  Holder holder;
  Holder spare_;
  nsq::Held holder_;
  nsq::Held spare_held_;
};
