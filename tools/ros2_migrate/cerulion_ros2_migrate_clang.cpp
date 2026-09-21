// SPDX-License-Identifier: AGPL-3.0-only
//
// cerulion-ros2-migrate-clang — the AST prover/rewriter behind
// `cerulion ros2 migrate`.
//
// A standalone clang LibTooling tool (clang-tidy-check style). It consumes a
// colcon workspace's compile_commands.json (`-p <build dir>`), analyses ONE
// translation unit per invocation, and emits a JSON analysis on stdout:
// proposed byte-range EDITS for every publish call site the AST PROVES safe
// to rewrite to the upstream loaned-message API, plus a manual-candidates
// list carrying a fixed reason string for every publish site it refuses.
//
// The tool never touches any file. Edit application, diff rendering, consent,
// git, and colcon orchestration all live in the Rust verb
// (`cerulion_cli_engine::ros2_migrate`) — one code path serves dry-run and
// write, so the printed diff and the applied bytes cannot diverge.
//
// EDIT RANGE CONTRACT: each rewrite emits exactly two edits. The DECL edit
// replaces the whole declaration statement (semicolon included); the PUBLISH
// edit replaces the WHOLE member call expression — publisher, operator,
// member, arguments. Replacement text splices the publisher's WRITTEN CORE
// (no trailing operator; the operator is re-added explicitly). Beware: a
// smart-pointer publisher's callee base is the `operator->` DESUGARING
// node, whose source range SPANS the written `->` token — taking its text
// verbatim doubles the arrow (a defect seen in the container). An emitted core text
// ending in an operator is refused at emission (internal invariant), and
// run_matrix.sh stage 3b builds the APPLIED bytes so a range regression can
// never pass the matrix again.
//
// # The safe pattern (rewrite ONLY when every condition is proven)
//
// A locally-constructed message —
//
//   auto msg = std::make_unique<T>();      // or std::make_shared<T>(), or T msg;
//   msg->field = ...;                      // any number of fill lines
//   pub_->publish(std::move(msg));         // or publish(*msg) / publish(msg)
//
// — is rewritten to the chosen 3-line shape (every fill line untouched):
//
//   auto loaned = pub_->borrow_loaned_message();
//   auto msg = &loaned.get();              // `auto& msg = loaned.get();` for the stack shape
//   msg->field = ...;                      // unchanged
//   pub_->publish(std::move(loaned));
//
// Proven from the AST, all of: the message is a local variable of the same
// function body as the publish; its declaration carries no constructor
// arguments (the loaned message is typesupport-default-initialized, which
// matches make_unique exactly); every use of the variable is a member-access
// fill or the one publish; nothing uses it after the publish; it is published
// exactly once; the publish target is an `rclcpp::Publisher<T>` whose T is
// the variable's T; the publisher expression is side-effect-free (a plain
// variable/member chain, no calls) so evaluating it twice — once for the
// borrow, once for the publish — is the same publisher; and the publisher
// field/variable is never reassigned in the function (the borrow source must
// BE the publish target); and — because the publisher is spliced as TEXT into
// the declaration's replacement, above every statement between the two — the
// publisher's root NAME is not re-declared in between, so the spliced borrow
// resolves it to the publisher the publish uses.
//
// Everything unprovable is refused into the candidates list with one of the
// fixed reasons below — never rewritten, never silent.
//
// # Candidate reason vocabulary (fixed strings; the Rust verb renders them)
//
//   pointer-escapes           the variable is used other than as a fill/publish
//   captured-by-lambda        a use sits inside a lambda body
//   use-after-publish         any use after the publish call
//   reuse-after-move          the variable is published more than once
//   conditional-publish       decl and publish are in different statement scopes
//   publisher-expr-not-trivial  the publisher expression contains a
//                             call/conditional, or is qualified by a name
//                             whose written spelling cannot be recovered
//   publisher-reassigned      the publisher field/variable is assigned in this function
//   publisher-shadowed        the publisher's name is re-declared between the
//                             declaration and the publish, so the spliced borrow
//                             would resolve it to a different publisher
//   publisher-lookup-changed  a using-directive between the two widens
//                             unqualified lookup, so the spliced borrow may not
//                             resolve the publisher's name at all
//   constructor-args          make_unique/make_shared/T(...) with arguments
//   not-a-local               the published pointer is a parameter/outer/global
//   retained-member           the published message is a class member
//   message-built-elsewhere   the message comes from another function's return
//   unsupported-publisher-type  publish() on a non-rclcpp::Publisher publisher class
//   unsupported-publish-shape the publish argument shape is not recognized
//   unsupported-decl-shape    the declaration shape is not recognized
//   type-mismatch             declared message type != publisher's message type
//   name-collision            no free `loaned`/`loanedN` name in the function
//   macro-expansion           the site is (partly) inside a macro expansion
//
// A using-DIRECTIVE between the two edits
// is REFUSED (`publisher-lookup-changed`), not tolerated. It can look
// harmless because the spliced borrow would fail to compile and
// the matrix's stage 3b builds the applied bytes — true of the MATRIX, and
// irrelevant to a user, who has no stage 3b. There, dry-run prints an invalid
// patch and `--write` commits source that does not build. "Wrong bytes" is not
// an acceptable outcome for a verb whose contract is to refuse anything it
// cannot prove.
//
// Known v1 limits: dependent (template) function bodies are not analysed;
// generic lambdas likewise; a reference alias extracted from a field
// (`auto& r = msg->data;`) is an ordinary fill use, so a use of the ALIAS
// after publish is not tracked (it was equally dangling in the original
// `std::move(msg)` form) — those fail toward REFUSAL or no-report, never a
// rewrite. ONE limit can produce a behavior-changing rewrite and is stated
// in the README ("Known v1 limits"): the reassignment scan keys
// on arguments NAMING the publisher chain, so a callee that reaches the
// publisher without naming it — any member call on the node, a helper
// handed `this`/`*this`/a node alias, any call at all for a global
// publisher — can swap the publisher unseen between borrow and publish; the
// rewritten site then borrows from the old publisher and publishes on the
// replacement. Planned hardening: refuse non-analyzable callees receiving
// `this`/a node alias while the chain roots in a member (deliberately not a
// fix-round change — it refuses most real callbacks, e.g. any logging call,
// and needs its own accept-envelope review).
//
// # Determinism
//
// Output is a single JSON object with fixed key order; rewrites and
// candidates are sorted by (file, offset/line). Two runs over the same
// sources produce byte-identical output.
//
// Build + test: see tools/ros2_migrate/README.md (the ros2-bench Jazzy container
// carries the clang/LLVM toolchain; this file is NOT part of the Rust
// workspace build).
//
// Mutation seams (compile-time, for tools/ros2_migrate/run_matrix.sh ONLY —
// never define these in a shipped build; run_matrix.sh asserts each mutant
// flips exactly its fixture):
//   CERULION_MIGRATE_MUTANT_DROP_ESCAPE_CHECK
//   CERULION_MIGRATE_MUTANT_DROP_USE_AFTER_PUBLISH
//   CERULION_MIGRATE_MUTANT_DROP_PUBLISHER_TRIVIALITY
//   CERULION_MIGRATE_MUTANT_DROP_MACRO_NAME_SCAN
//   CERULION_MIGRATE_MUTANT_DROP_PUBLISHER_SHADOW
//   CERULION_MIGRATE_MUTANT_DROP_PUBLISHER_TEMPLATE_ARGS

#include <algorithm>
#include <cstdio>
#include <cstring>
#include <memory>
#include <set>
#include <string>
#include <tuple>
#include <vector>

#include "clang/AST/ASTConsumer.h"
#include "clang/AST/ASTContext.h"
#include "clang/AST/Decl.h"
#include "clang/AST/DeclCXX.h"
#include "clang/AST/DeclTemplate.h"
#include "clang/AST/Expr.h"
#include "clang/AST/ExprCXX.h"
#include "clang/AST/NestedNameSpecifier.h"
#include "clang/AST/ParentMapContext.h"
#include "clang/AST/RecursiveASTVisitor.h"
#include "clang/Basic/SourceManager.h"
#include "clang/Frontend/CompilerInstance.h"
#include "clang/Frontend/FrontendAction.h"
#include "clang/Lex/Lexer.h"
#include "clang/Lex/MacroInfo.h"
#include "clang/Lex/PPCallbacks.h"
#include "clang/Lex/Preprocessor.h"
#include "clang/Lex/Token.h"
#include "clang/Tooling/CommonOptionsParser.h"
#include "clang/Tooling/Tooling.h"
#include "llvm/Support/CommandLine.h"
#include "llvm/Support/FileSystem.h"
#include "llvm/Support/Path.h"
#include "llvm/Support/raw_ostream.h"

using namespace clang;

// Macro collision guard: every macro name the
// preprocessor DEFINES anywhere in the translation unit (headers and the
// main file alike, builtins and command-line -D included), recorded as the
// preprocessor sees each #define. A later #undef does NOT free the name —
// a name that was a macro anywhere in the TU is not worth the ambiguity —
// and the minted loaned local must avoid every one of them
// (FunctionAnalyzer::mintLoanedName). Populated per TU by MacroNameRecorder.
std::set<std::string> g_macroNames;

namespace {

llvm::cl::OptionCategory MigrateCategory("cerulion-ros2-migrate-clang options");
llvm::cl::opt<std::string> SrcRoot(
    "src-root",
    llvm::cl::desc("Workspace source root; only code written in files under "
                   "this directory is analysed (required)"),
    llvm::cl::Required, llvm::cl::cat(MigrateCategory));

constexpr unsigned kFormatVersion = 1;
constexpr const char *kToolVersion = "1.0.0";

// ---------------------------------------------------------------------------
// Result model (mirrors the JSON contract the Rust verb parses).
// ---------------------------------------------------------------------------

struct Edit {
  unsigned offset = 0;
  unsigned length = 0;
  std::string original;
  std::string replacement;
};

struct RewriteResult {
  std::string file;      // real path of the file holding the edits
  std::string function;  // qualified name, for the report
  std::string kind;      // "unique_ptr" | "shared_ptr" | "stack"
  std::string message_type;
  std::string publisher;  // verbatim publisher expression text
  unsigned line = 0;      // publish call line (1-based)
  std::vector<Edit> edits;
};

struct CandidateResult {
  std::string file;
  std::string function;
  unsigned line = 0;
  // The per-call-site discriminator. Every
  // invocation of a macro whose body spells the publish resolves to the SAME
  // (file, line) — the `#define` — so without a column N distinct sites
  // collapse into one reported candidate and N-1 vanish from the report and
  // the manifest.
  unsigned column = 0;
  std::string reason;
  std::string detail;
};

struct Results {
  std::vector<RewriteResult> rewrites;
  std::vector<CandidateResult> candidates;
};

// ---------------------------------------------------------------------------
// Small helpers.
// ---------------------------------------------------------------------------

std::string realPathOf(llvm::StringRef p) {
  llvm::SmallString<256> real;
  if (llvm::sys::fs::real_path(p, real, /*expand_tilde=*/false)) {
    return p.str();  // fall back to the spelled path
  }
  return std::string(real.str());
}

bool pathUnder(llvm::StringRef path, llvm::StringRef root) {
  if (!path.starts_with(root)) {
    return false;
  }
  if (path.size() == root.size()) {
    return true;
  }
  return path[root.size()] == '/';
}

std::string jsonEscape(llvm::StringRef s) {
  std::string out;
  out.reserve(s.size() + 8);
  for (unsigned char c : s) {
    switch (c) {
      case '"':
        out += "\\\"";
        break;
      case '\\':
        out += "\\\\";
        break;
      case '\n':
        out += "\\n";
        break;
      case '\r':
        out += "\\r";
        break;
      case '\t':
        out += "\\t";
        break;
      default:
        if (c < 0x20) {
          char buf[8];
          std::snprintf(buf, sizeof(buf), "\\u%04x", c);
          out += buf;
        } else {
          out += static_cast<char>(c);
        }
    }
  }
  return out;
}

// ---------------------------------------------------------------------------
// Per-function analysis.
// ---------------------------------------------------------------------------

// One publish call site, pre-classified.
struct PublishSite {
  QualType msgType;  // the Publisher's message T (canonical)
  enum ArgKind { kMove, kRef, kDeref, kOther } argKind = kOther;
  const VarDecl *var = nullptr;  // the message variable, when arg resolves
};

class FunctionAnalyzer {
 public:
  FunctionAnalyzer(ASTContext &ctx, const FunctionDecl *fn,
                   const std::string &srcRootReal, Results &results)
      : ctx_(ctx),
        sm_(ctx.getSourceManager()),
        lo_(ctx.getLangOpts()),
        fn_(fn),
        srcRootReal_(srcRootReal),
        results_(results) {}

  void run() {
    const Stmt *body = fn_->getBody();
    if (!body) {
      return;
    }
    collectPublishCalls(body);
    if (publishCalls_.empty()) {
      return;
    }
    collectLocalNames(body);
    collectReferencedNames(body);
    // The function body's SOURCE TEXT, for the minted-name shadow scan.
    // Name shadowing: if the BODY's own range is macro-tainted
    // or unreadable, the shadow scan sees NOTHING — a minted `loaned`
    // could silently shadow a member/global the invisible text references
    // — so every site in this body REFUSES loudly instead of proceeding
    // on a guessed name. The per-site range checks cannot stand in: a
    // macro at the body BOUNDARY taints the body range while an interior
    // site's own decl/call/arg ranges stay clean.
    RangeInfo bodyRange = rangeInfo(body->getSourceRange());
#ifndef CERULION_MIGRATE_MUTANT_DROP_BODY_TEXT_GUARD
    if (bodyRange.macro || bodyRange.text.empty()) {
      for (const CXXMemberCallExpr *call : publishCalls_) {
        candidate(call->getBeginLoc(), "macro-expansion",
                  "the function body's source text is unavailable (macro "
                  "at the body boundary) — a minted loaned name cannot be "
                  "proven collision-free");
      }
      return;
    }
#endif
    bodyText_ = bodyRange.text;
    for (const CXXMemberCallExpr *call : publishCalls_) {
      analyzePublish(call);
    }
  }

 private:
  ASTContext &ctx_;
  SourceManager &sm_;
  const LangOptions &lo_;
  const FunctionDecl *fn_;
  const std::string &srcRootReal_;
  Results &results_;

  std::vector<const CXXMemberCallExpr *> publishCalls_;
  std::set<std::string> localNames_;
  std::set<std::string> referencedNames_;
  std::string bodyText_;
  // A double-published variable is ONE defect: each publish site of the
  // pair refuses, but the reuse-after-move candidate is emitted once per
  // VARIABLE (anchored at the first refusing site), not once per site —
  // the container matrix caught the duplicate.
  std::set<const VarDecl *> reuseReported_;
  std::set<std::string> mintedNames_;

  // ---- collection -------------------------------------------------------

  // A publish written INSIDE a lambda body is
  // REPORTED, never dropped.
  //
  // A prune at the lambda would rest on "a lambda's call operator is analysed
  // as its own function context when the visitor reaches it". That is not
  // true of this visitor: `MigrateVisitor` is a plain `RecursiveASTVisitor`
  // with only a `VisitFunctionDecl` override, and clang reaches a lambda's
  // `operator()` as its own `FunctionDecl` only via
  // `TraverseDecl(getLambdaClass())`, which is gated behind
  // `shouldVisitImplicitCode()` — false by default, and not overridden here.
  // So `publishCalls_` would stay empty and `run()` return with NO edit and NO
  // candidate for the single most common ROS 2 publisher idiom:
  //
  //     timer_ = create_wall_timer(1s, [this]() { pub_->publish(...); });
  //
  // which contradicts this tool's own contract that everything unprovable is
  // refused into the candidates list, never silent. This report is deliberately
  // ADDITIVE — nothing that rewrites stops rewriting; a site that would be
  // invisible becomes a reported candidate.
  void reportLambdaPublishes(const Stmt *s) {
    if (!s) {
      return;
    }
    if (const auto *call = dyn_cast<CXXMemberCallExpr>(s)) {
      if (const CXXMethodDecl *md = call->getMethodDecl()) {
        if (md->getDeclName().isIdentifier() && md->getName() == "publish") {
          candidate(call->getBeginLoc(), "publish-inside-lambda",
                    "the publish call is written inside a lambda body, which "
                    "this version does not analyse — migrate it by hand or "
                    "move the publish into a named method");
        }
      }
    }
    // Nested lambdas are walked too: every publish in there is equally
    // unanalysed and equally owed a report.
    for (const Stmt *child : s->children()) {
      if (child) {
        reportLambdaPublishes(child);
      }
    }
  }

  // Collect every member call named `publish` in this function body, NOT
  // descending into lambda bodies (which this version cannot prove — their
  // publishes are reported instead, see `reportLambdaPublishes`).
  void collectPublishCalls(const Stmt *s) {
    if (const auto *le = dyn_cast<LambdaExpr>(s)) {
      reportLambdaPublishes(le->getBody());
      return;
    }
    if (const auto *call = dyn_cast<CXXMemberCallExpr>(s)) {
      if (const CXXMethodDecl *md = call->getMethodDecl()) {
        if (md->getDeclName().isIdentifier() && md->getName() == "publish") {
          publishCalls_.push_back(call);
        }
      }
    }
    for (const Stmt *child : s->children()) {
      if (child) {
        collectPublishCalls(child);
      }
    }
  }

  // Collect every variable/parameter name in the function (lambdas included:
  // a minted name must not shadow or be shadowed anywhere in the body).
  void collectLocalNames(const Stmt *s) {
    for (const ParmVarDecl *p : fn_->parameters()) {
      if (p->getDeclName().isIdentifier()) {
        localNames_.insert(p->getName().str());
      }
    }
    collectDeclNames(s);
  }

  // Macro shadowing: every identifier the body REFERENCES,
  // collected from the AST — which sees THROUGH macro invocations, unlike
  // `bodyText_` (the raw source shows `RECORD()`, never the `loaned` its
  // replacement list names). A minted name must collide with none of
  // these, or the generated local would shadow the member/global the
  // expanded code references and silently change its meaning.
  void collectReferencedNames(const Stmt *s) {
    if (const auto *dre = dyn_cast<DeclRefExpr>(s)) {
      if (dre->getDecl()->getDeclName().isIdentifier()) {
        referencedNames_.insert(dre->getDecl()->getName().str());
      }
    }
    if (const auto *refMe = dyn_cast<MemberExpr>(s)) {
      if (refMe->getMemberDecl()->getDeclName().isIdentifier()) {
        referencedNames_.insert(refMe->getMemberDecl()->getName().str());
      }
    }
    for (const Stmt *child : s->children()) {
      if (child) {
        collectReferencedNames(child);
      }
    }
  }

  void collectDeclNames(const Stmt *s) {
    if (const auto *ds = dyn_cast<DeclStmt>(s)) {
      for (const Decl *d : ds->decls()) {
        if (const auto *vd = dyn_cast<VarDecl>(d)) {
          if (vd->getDeclName().isIdentifier()) {
            localNames_.insert(vd->getName().str());
          }
        }
      }
    }
    for (const Stmt *child : s->children()) {
      if (child) {
        collectDeclNames(child);
      }
    }
  }

  // ---- expression shape helpers -----------------------------------------

  // Argument count EXCLUDING defaulted arguments. Load-bearing for the
  // stack shape: rosidl-generated message default constructors take a
  // defaulted `MessageInitialization` argument, so `T msg;` is a
  // CXXConstructExpr with ONE CXXDefaultArgExpr — counting raw arguments
  // misclassified the whole stack pattern (found by the container matrix).
  template <typename CallLike>
  static unsigned explicitArgCount(const CallLike *c) {
    unsigned n = 0;
    for (unsigned i = 0; i < c->getNumArgs(); ++i) {
      if (!isa<CXXDefaultArgExpr>(c->getArg(i))) {
        ++n;
      }
    }
    return n;
  }

  static const Expr *stripWrappers(const Expr *e) {
    while (true) {
      e = e->IgnoreParens();
      if (const auto *ice = dyn_cast<ImplicitCastExpr>(e)) {
        e = ice->getSubExpr();
        continue;
      }
      if (const auto *ewc = dyn_cast<ExprWithCleanups>(e)) {
        e = ewc->getSubExpr();
        continue;
      }
      if (const auto *mte = dyn_cast<MaterializeTemporaryExpr>(e)) {
        e = mte->getSubExpr();
        continue;
      }
      if (const auto *bte = dyn_cast<CXXBindTemporaryExpr>(e)) {
        e = bte->getSubExpr();
        continue;
      }
      if (const auto *ce = dyn_cast<CXXConstructExpr>(e)) {
        // Descend only through a genuine 1-argument wrap — a defaulted
        // argument is part of the CONSTRUCTOR, not a wrapped
        // sub-expression (see explicitArgCount) — and, to close an unsafe
        // rewrite, only through a COMPILER-INSERTED copy/move: a
        // WRITTEN construction (`T{*msg}` is a CXXTemporaryObjectExpr) or
        // a converting constructor performs behavior the rewrite would
        // silently delete, so it must stay wrapped and fall out of the
        // supported shapes as a manual candidate.
#ifndef CERULION_MIGRATE_MUTANT_DROP_CTOR_WRITTEN_GUARD
        const bool compilerInsertedCopy = !isa<CXXTemporaryObjectExpr>(ce) &&
                                          ce->getConstructor() &&
                                          ce->getConstructor()->isCopyOrMoveConstructor();
#else
        const bool compilerInsertedCopy = true;
#endif
        if (compilerInsertedCopy && ce->getNumArgs() >= 1 &&
            !isa<CXXDefaultArgExpr>(ce->getArg(0)) &&
            explicitArgCount(ce) == 1) {
          e = ce->getArg(0);
          continue;
        }
      }
      return e;
    }
  }

  // std::move(x) → x, else nullptr.
  static const Expr *stdMoveArg(const Expr *e) {
    const auto *call = dyn_cast<CallExpr>(e);
    if (!call || call->getNumArgs() != 1) {
      return nullptr;
    }
    const FunctionDecl *fd = call->getDirectCallee();
    if (!fd || !fd->getDeclName().isIdentifier() || fd->getName() != "move") {
      return nullptr;
    }
    if (!fd->isInStdNamespace()) {
      return nullptr;
    }
    return call->getArg(0);
  }

  // PUBLISHER IDENTITY: the generated rewrite evaluates
  // the publisher expression TWICE — once for `borrow_loaned_message()`,
  // once for `publish(std::move(..))` — so arrow sugar between the written
  // core and the member is only safe when both dereferences provably yield
  // the SAME object. That holds for the std smart pointers the rclcpp
  // `Publisher<T>::SharedPtr` / `::UniquePtr` aliases resolve to
  // (`operator->` returns the stored pointer, and the reassignment scan
  // below already refuses a chain that is written to between the two
  // evaluations). ANY other overloaded `operator->` — a stateful proxy, a
  // rotating router — can hand the borrow and the publish DIFFERENT
  // publishers (a silent routing change, and a loan published on a
  // publisher that never issued it). Refused, never assumed. The object
  // type is checked (not the method's parent) because libstdc++ inherits
  // `shared_ptr::operator->` from `__shared_ptr_access` while libc++
  // declares it directly — the written operand's type is the stable fact.
  static bool isStdSmartPointerArrow(const CXXOperatorCallExpr *oc) {
    const Expr *obj = oc->getArg(0)->IgnoreParenImpCasts();
    const CXXRecordDecl *rd = obj->getType()->getAsCXXRecordDecl();
    if (!rd || !rd->isInStdNamespace()) {
      return false;
    }
    const llvm::StringRef name = rd->getName();
    return name == "shared_ptr" || name == "unique_ptr";
  }

  // Unsafe-rewrite guard: is this type EXACTLY rclcpp::Publisher —
  // not a class derived from it? The site gate proves only that the
  // RESOLVED publish method's parent is rclcpp::Publisher, which an
  // INHERITED publish satisfies on a derived publisher — but the
  // generated `borrow_loaned_message()` resolves on the WRITTEN static
  // type, where a derived class can hide the API with a different return
  // type (the migrated source then fails to compile, or worse).
  static bool isExactRclcppPublisher(QualType t) {
    const CXXRecordDecl *rd = t.getCanonicalType()->getAsCXXRecordDecl();
    return rd && rd->getQualifiedNameAsString() == "rclcpp::Publisher";
  }

  // The publisher expression is side-effect-free: a plain variable, `this`,
  // or a member chain over those — no calls, no conditionals, no indexing.
  //
  // WIDENING this set requires revisiting `collectPublisherRoots`.
  // Its bottom fall-through answers `kRootShapeAlreadyRefused` — proceed
  // without a shadow scan — on the argument that anything reaching it was
  // already refused HERE. Widen this and that argument stops holding, and
  // the failure mode is a silent rewrite with the scan skipped, which no
  // fixture can catch (only the DROP_PUBLISHER_TRIVIALITY mutant exercises
  // that path at all).
  // Smart-pointer `operator->` sugar between the written expression and the
  // member is stripped by the CALLER (the written base range never includes
  // it; only std::shared_ptr/std::unique_ptr arrows qualify — see
  // isStdSmartPointerArrow), so this checks the WRITTEN core only.
  static bool trivialPublisherExpr(const Expr *e) {
    e = e->IgnoreParenImpCasts();
    if (isa<DeclRefExpr>(e) || isa<CXXThisExpr>(e)) {
      return true;
    }
    if (const auto *me = dyn_cast<MemberExpr>(e)) {
      return trivialPublisherExpr(me->getBase());
    }
    return false;
  }

  // EVERY ValueDecl named along a trivial publisher chain — `holder.pub_`
  // names BOTH `pub_` (the terminal) and `holder` (the base): an assignment
  // to ANY of them between borrow and publish can swap the publisher, so
  // the reassignment scan must cover the whole chain (a
  // terminal-only scan misses `holder = other;`).
  static void publisherChainDecls(const Expr *e,
                                  std::vector<const ValueDecl *> &out) {
    e = e->IgnoreParenImpCasts();
    if (const auto *dre = dyn_cast<DeclRefExpr>(e)) {
      out.push_back(dre->getDecl());
      return;
    }
    if (const auto *chainMe = dyn_cast<MemberExpr>(e)) {
      out.push_back(chainMe->getMemberDecl());
      publisherChainDecls(chainMe->getBase(), out);
    }
  }

  // Publisher shadowing: the publisher expression's ROOT NAME —
  // the ONE identifier in the spliced text that ordinary UNQUALIFIED name
  // lookup resolves at the point the text is spliced. Everything else in a
  // trivial publisher expression is resolved by MEMBER lookup on the type
  // of what precedes it (`holder.pub_` finds `pub_` in holder's class, so
  // only `holder` can be shadowed), which no block-scope declaration can
  // change.
  //
  // FOUR answers, as an enum rather than a bool plus an out-param's
  // emptiness: `kRootUnnameableRoot` and `kRootNamed` have OPPOSITE
  // consequences at the one call site — refuse vs scan-then-proceed — and a
  // bool invites the tidy-up that swaps them, scanning for a declaration
  // named "" which never matches. `kRoot*` follows this file's existing
  // local-enum spelling (kMove/kRef/kDeref, kUnique/kShared/kStack).
  //
  // kRootUnshadowable is the safe answer, not a missing one — but it is a
  // NARROWER set than "qualified", and treating the two as equal is
  // wrong in the dangerous direction. A qualified-id's TAIL is immune
  // to block-scope declarations; its HEAD is an ordinary unqualified name
  // and is not:
  //
  //     namespace ns = evil;              // a block-scope namespace ALIAS
  //     ns::g_pub->publish(std::move(msg));
  //
  // resolves to `evil::g_pub` at the publish and to whatever `ns` meant
  // above, where the borrow is spliced — the same silent cross-publisher
  // loan this round exists to refuse, arriving through the one lookup the
  // draft exempted by assertion. (`using Base = Other;` does it through a
  // type alias.) So a qualified root resolves to the NAME OF ITS OUTERMOST
  // SPECIFIER, which the between-scan then treats like any other name.
  //
  // What genuinely cannot be rebound, and is the whole of kRootUnshadowable:
  //   * an explicit `this->pub_` — `this` is not a name at all;
  //   * a `::`-rooted specifier (`::g_pub`) — global scope is not nameable;
  //   * `__super::` — a keyword.
  // A specifier headed by a TYPE is refused rather than resolved: the
  // WRITTEN spelling can be an alias whose name differs from the resolved
  // type's, so the text that gets spliced is not recoverable from the
  // resolved decl. `safe_qualified_publisher.cpp` is the accept control;
  // the refusals are `unsafe_shadowed_publisher.cpp`'s two type-headed arms
  // (its alias arm is a different verdict — a namespace alias resolves to a
  // name, so it is SCANNED and refused as `publisher-shadowed`).
  enum RootKind {
    // The written core is not a shape this analysis models. It is NOT a
    // refusal of its own, and that is deliberate: the shapes this classifier
    // walks — a DeclRefExpr, a CXXThisExpr, or a MemberExpr chain over
    // either — are the set `trivialPublisherExpr` accepts a few lines above,
    // so a shape reaching the FALL-THROUGH AT THE BOTTOM has already been
    // refused `publisher-expr-not-trivial` and cannot arrive in a shipped
    // build.
    //
    // That argument covers the bottom `return` and nothing else, which is
    // why the `isIdentifier()` guards above return the REFUSING verdict
    // instead: those are name-shape guards, not expression-shape guards
    // (`trivialPublisherExpr` accepts a DeclRefExpr or a MemberExpr whatever
    // its decl's DeclarationName kind), so falling through on them would
    // skip the scan and rewrite unchecked. They cannot re-mask
    // DROP_PUBLISHER_TRIVIALITY: that mutant's fixture is a
    // ConditionalOperator, which lands on the bottom fall-through.
    //
    // Refusing it a second time is not free, which is how this was found:
    // it made the pre-existing DROP_PUBLISHER_TRIVIALITY mutant SURVIVE in
    // the container. That mutant compiles triviality out so
    // `unsafe_wrong_publisher.cpp` (`(left ? pub_a_ : pub_b_)->publish(…)`)
    // must flip to a rewrite — and a redundant refusal here kept it refused,
    // so the mutant stopped isolating the check it exists to isolate. Same
    // decision as for DROP_ARROW_IDENTITY: a provably subsumed
    // check is deleted rather than kept as dead code that masks a mutant.
    kRootShapeAlreadyRefused,
    kRootUnnameableRoot,  // a qualifier head this analysis cannot name — REFUSE
    kRootUnshadowable,    // nothing scope-resolved — nothing to check
    kRootNamed,           // `out` holds the name to scan for
  };

  // The OUTERMOST component of a nested-name-specifier — the one written
  // first, and the only one unqualified lookup resolves. Everything after it
  // is looked up inside what it names.
  //
  // The kinds below are clang 18's (`build.sh` probes llvm-config-18).
  // Upstream has since rewritten NestedNameSpecifier into a value type whose
  // Kind drops Identifier/NamespaceAlias/TypeSpecWithTemplate and removes
  // getAsNamespace()/getAsNamespaceAlias() — so a toolchain bump lands here
  // first, as a compile error rather than a silent behaviour change, which
  // is the direction to want.
  static RootKind classifyQualifierHead(const NestedNameSpecifier *nns,
                                        std::string &out) {
    if (!nns) {
      return kRootUnnameableRoot;
    }
    for (int guard = 0; guard < 256; ++guard) {
      const NestedNameSpecifier *prefix = nns->getPrefix();
      if (!prefix) {
        break;
      }
      nns = prefix;
      if (guard == 255) {
        return kRootUnnameableRoot;  // pathological depth: refuse, as the
                                     // other walks in this file do
      }
    }
    switch (nns->getKind()) {
      case NestedNameSpecifier::Global:
      case NestedNameSpecifier::Super:
        return kRootUnshadowable;  // `::x`, `__super::x` — not nameable
      case NestedNameSpecifier::Namespace:
        // `namedRoot`, not an isIdentifier() test: an ANONYMOUS namespace
        // has no written head, and isIdentifier() would wave it through
        // with an empty name (see namedRoot).
        return namedRoot(nns->getAsNamespace(), out);
      case NestedNameSpecifier::NamespaceAlias:
        return namedRoot(nns->getAsNamespaceAlias(), out);
      default:
        // A type-headed specifier (`Base::`, `Alias::`) or a dependent one.
        // The written spelling can be an alias whose identifier differs from
        // the resolved type's name, and it is the WRITTEN text that gets
        // spliced, so there is nothing here this analysis can reliably scan
        // for. Refused, never guessed.
        return kRootUnnameableRoot;
    }
  }

  // The written identifier of a decl, or a refusal. `getIdentifier()`, NOT
  // `getDeclName().isIdentifier()`: the latter is a STORED-KIND test, and an
  // EMPTY DeclarationName stores as kind Identifier — so it answers true for
  // a decl with no identifier at all, `getName()` then returns "", and the
  // between-scan would search for a name that matches nothing (except a
  // DecompositionDecl, whose own name is also empty). That is an ACCEPT on
  // an expression this analysis cannot actually name. The `out.empty()`
  // belt is deliberate duplication: this is the one place a silent empty
  // name could turn the scan into a no-op, and it costs one comparison.
  static RootKind namedRoot(const NamedDecl *nd, std::string &out) {
    if (!nd || !nd->getIdentifier()) {
      return kRootUnnameableRoot;
    }
    out = nd->getName().str();
    if (out.empty()) {
      return kRootUnnameableRoot;
    }
    return kRootNamed;
  }

  // Collect EVERY identifier in the written publisher expression that
  // ordinary unqualified lookup resolves at the point the text is spliced.
  // There can be more than one. A walk that
  // returns the FIRST answer and exits the chain walk goes wrong here:
  //
  //     holder_.nsq::B::pub_->publish(...)
  //
  // yields only `nsq` (the qualifier head) and never looks at `holder_`.
  // A local `holder_` declared between the two edits is then invisible
  // — such a walk closes exactly this
  // shape for a TYPE-headed qualifier (by refusing it) while opening it for
  // a namespace-headed one. Verified on compiling code.
  //
  // The rule is per COMPONENT: a qualifier's outermost specifier is looked
  // up unqualified (so it is collected, or refused when unnameable); the
  // names AFTER a qualifier are found by qualified lookup and are not; the
  // object expression of a member access still has its own root, whatever
  // the member is qualified by.
  static RootKind collectPublisherRoots(const Expr *e,
                                        std::vector<std::string> &out) {
    out.clear();
    for (int guard = 0; guard < 256; ++guard) {
      e = e->IgnoreParenImpCasts();
      if (const auto *me = dyn_cast<MemberExpr>(e)) {
        if (me->hasQualifier()) {
          std::string qname;
          const RootKind qk = classifyQualifierHead(me->getQualifier(), qname);
          if (qk == kRootUnnameableRoot) {
            return kRootUnnameableRoot;
          }
          if (qk == kRootNamed) {
            out.push_back(qname);
          }
          // kRootUnshadowable (`::`, `__super`): nothing to scan, and the
          // object expression below still has to be walked.
        }
        if (me->getBase() && me->getBase()->isImplicitCXXThis()) {
          if (!me->hasQualifier()) {
            // Written bare: unqualified lookup found a member of the
            // enclosing class, and a local of that name shadows it.
            std::string n;
            if (namedRoot(me->getMemberDecl(), n) != kRootNamed) {
              return kRootUnnameableRoot;
            }
            out.push_back(n);
          }
          return out.empty() ? kRootUnshadowable : kRootNamed;
        }
        if (!me->getBase()) {
          return kRootUnnameableRoot;
        }
        e = me->getBase();
        continue;
      }
      if (isa<CXXThisExpr>(e)) {
        return out.empty() ? kRootUnshadowable : kRootNamed;
      }
      if (const auto *dre = dyn_cast<DeclRefExpr>(e)) {
        if (dre->hasQualifier()) {
          std::string qname;
          const RootKind qk = classifyQualifierHead(dre->getQualifier(), qname);
          if (qk == kRootUnnameableRoot) {
            return kRootUnnameableRoot;
          }
          if (qk == kRootNamed) {
            out.push_back(qname);
          }
          return out.empty() ? kRootUnshadowable : kRootNamed;
        }
        std::string n;
        if (namedRoot(dre->getDecl(), n) != kRootNamed) {
          return kRootUnnameableRoot;
        }
        out.push_back(n);
        return kRootNamed;
      }
      return out.empty() ? kRootShapeAlreadyRefused : kRootNamed;
    }
    return kRootUnnameableRoot;  // pathological depth: refuse, never assume
  }

  // The WRITTEN core of a publisher expression: at most one std
  // smart-pointer `operator->` desugaring stripped (the rule is that
  // only std::shared_ptr/std::unique_ptr arrows qualify).
  static const Expr *publisherWrittenCore(const Expr *e) {
    e = e->IgnoreParenImpCasts();
    if (const auto *oc = dyn_cast<CXXOperatorCallExpr>(e)) {
      if (oc->getOperator() == OO_Arrow && oc->getNumArgs() >= 1 &&
          isStdSmartPointerArrow(oc)) {
        e = oc->getArg(0)->IgnoreParenImpCasts();
      }
    }
    return e;
  }

  // False-idempotence guard: do two publisher expressions name the
  // SAME decl chain? Empty/unresolvable chains prove NOTHING and compare
  // unequal — conservative by construction. NOTE (settled by the matrix):
  // this target proof does NOT subsume the CLASS proof — a
  // derived publisher whose own `borrow_loaned_message` HIDES the base
  // API passes this chain comparison AND the publish-site gate (publish
  // resolves on the base), and only the class proof (the borrow method's
  // DECLARING class must be rclcpp::Publisher itself) refuses it. The
  // two proofs are independent; `unsafe_derived_loan.cpp` isolates the
  // class proof, `unsafe_cross_loan.cpp` isolates this one.
  static bool samePublisherChain(const Expr *a, const Expr *b) {
    if (!a || !b) {
      return false;
    }
    std::vector<const ValueDecl *> ca;
    std::vector<const ValueDecl *> cb;
    publisherChainDecls(publisherWrittenCore(a), ca);
    publisherChainDecls(publisherWrittenCore(b), cb);
    return !ca.empty() && ca == cb;
  }

  // The variable appears in a lambda CAPTURE LIST anywhere in `s` —
  // checked via the capture list directly (LambdaCapture::getCapturedVar),
  // never inferred from DeclRefExpr traversal: an explicit `[msg]` capture
  // whose body never spells the name still retains the message.
  bool capturedByAnyLambda(const Stmt *s, const VarDecl *vd) {
    if (const auto *lam = dyn_cast<LambdaExpr>(s)) {
      for (const LambdaCapture &cap : lam->captures()) {
        if (cap.capturesVariable() && cap.getCapturedVar() == vd) {
          return true;
        }
      }
    }
    for (const Stmt *child : s->children()) {
      if (child && capturedByAnyLambda(child, vd)) {
        return true;
      }
    }
    return false;
  }

  // ---- source range helpers ---------------------------------------------

  // (fileRealPath, offset, length, text) for a token range; empty file on
  // macro involvement or cross-file ranges.
  struct RangeInfo {
    std::string file;
    unsigned offset = 0;
    unsigned length = 0;
    std::string text;
    bool macro = false;
  };

  RangeInfo rangeInfo(SourceRange r) {
    RangeInfo out;
    if (r.getBegin().isMacroID() || r.getEnd().isMacroID()) {
      out.macro = true;
      return out;
    }
    SourceLocation b = sm_.getSpellingLoc(r.getBegin());
    SourceLocation e =
        Lexer::getLocForEndOfToken(sm_.getSpellingLoc(r.getEnd()), 0, sm_, lo_);
    FileID fb = sm_.getFileID(b);
    if (fb != sm_.getFileID(e)) {
      out.macro = true;  // treat cross-file as unrewritable
      return out;
    }
    out.file = realPathOf(sm_.getFilename(b));
    out.offset = sm_.getFileOffset(b);
    out.length = sm_.getFileOffset(e) - out.offset;
    bool invalid = false;
    llvm::StringRef buf = sm_.getBufferData(fb, &invalid);
    if (invalid || out.offset + out.length > buf.size()) {
      out.macro = true;
      return out;
    }
    out.text = buf.substr(out.offset, out.length).str();
    return out;
  }

  // The whitespace prefix of the line holding `offset` in `file`'s buffer.
  std::string lineIndentAt(FileID fid, unsigned offset) {
    bool invalid = false;
    llvm::StringRef buf = sm_.getBufferData(fid, &invalid);
    if (invalid) {
      return std::string();
    }
    size_t lineStart = buf.rfind('\n', offset);
    lineStart = (lineStart == llvm::StringRef::npos) ? 0 : lineStart + 1;
    size_t i = lineStart;
    while (i < buf.size() && (buf[i] == ' ' || buf[i] == '\t')) {
      ++i;
    }
    return buf.slice(lineStart, i).str();
  }

  unsigned lineOf(SourceLocation loc) {
    return sm_.getSpellingLineNumber(sm_.getSpellingLoc(loc));
  }

  // ---- parent walking ---------------------------------------------------

  // Nearest enclosing CompoundStmt of `s` (walking parents), or nullptr.
  // Control flow: any control construct between `node` and its
  // nearest enclosing CompoundStmt `scope` means the node executes
  // conditionally (or repeatedly) even though the compound compare sees
  // one shared block — the unbraced-body / conditional-expression /
  // short-circuit shapes. Walk parent-by-parent; stop at the scope.
  bool controlFlowAboveWithinScope(const Stmt *node, const CompoundStmt *scope) {
    DynTypedNode cur = DynTypedNode::create(*node);
    for (int guard = 0; guard < 256; ++guard) {
      auto parents = ctx_.getParents(cur);
      if (parents.empty()) {
        return false;
      }
      cur = parents[0];
      if (cur.get<CompoundStmt>() == scope) {
        return false;
      }
      if (cur.get<FunctionDecl>() || cur.get<LambdaExpr>()) {
        return false;
      }
      if (cur.get<IfStmt>() || cur.get<WhileStmt>() || cur.get<ForStmt>() ||
          cur.get<DoStmt>() || cur.get<CXXForRangeStmt>() ||
          cur.get<SwitchStmt>() || cur.get<ConditionalOperator>() ||
          cur.get<BinaryConditionalOperator>()) {
        return true;
      }
      if (const auto *bo = cur.get<BinaryOperator>()) {
        if (bo->isLogicalOp()) {
          return true;  // && / || short-circuit conditional execution
        }
      }
    }
    return true;  // pathological depth: refuse, never assume
  }

  // Publisher shadowing: the direct child of `scope` that
  // CARRIES `node` — the statement slot `node` occupies in the block it
  // shares with the other edit. nullptr if `node` is not under `scope` at
  // all, or if the chain leaves the Stmt hierarchy (the caller refuses).
  //
  // POSTCONDITION, load-bearing for `nameRedeclaredBetween`: a non-null
  // result's PARENT is `scope`, and a CompoundStmt's children ARE its body
  // statements — so the result is guaranteed to be found in `scope->body()`
  // by pointer identity. MEMBERSHIP only, never ORDER: which carrier the
  // walk meets first is a separate argument, so `nameRedeclaredBetween`
  // still fails closed if its window never opens.
  const Stmt *topLevelStmtIn(const CompoundStmt *scope, const Stmt *node) {
    DynTypedNode cur = DynTypedNode::create(*node);
    for (int guard = 0; guard < 256; ++guard) {
      auto parents = ctx_.getParents(cur);
      if (parents.empty()) {
        return nullptr;
      }
      if (parents[0].get<CompoundStmt>() == scope) {
        return cur.get<Stmt>();
      }
      if (cur.get<FunctionDecl>() || cur.get<LambdaExpr>()) {
        return nullptr;
      }
      cur = parents[0];
    }
    return nullptr;
  }

  // Publisher shadowing: is `name` re-declared in `scope`
  // strictly between the two statement slots the edits occupy?
  //
  // Both bounds arrive as CARRIERS the caller has already resolved through
  // `topLevelStmtIn` — never as the raw decl/publish nodes — so this
  // function answers exactly one question and its `true` means exactly one
  // thing: a shadow was found. "Could not locate an edit in the block it
  // shares with the other" is a DIFFERENT condition with a different remedy,
  // and it is refused at the call site under its own reason rather than
  // being folded in here, where it would be reported to the operator as a
  // re-declaration that does not exist.
  //
  // The proof is exact rather than approximate BECAUSE the caller has
  // already proven both edits sit in the SAME CompoundStmt: within one
  // block, the set of visible names only GROWS from one statement to the
  // next, and it grows by exactly the declarations written between them.
  // Declarations earlier in the block, or in any enclosing scope, are
  // visible at both points; declarations in an INNER scope between them (a
  // nested block, a `for` init) have died before either point and are
  // visible at neither — so only the block's own DeclStmts in the open
  // interval can make one spelling name two entities. Nothing here needs
  // source locations: a CompoundStmt's children are in source order, which
  // is also the order in which their declarations take effect.
  enum ScanResult {
    kScanNoShadow,       // walked the window, found nothing
    kScanShadowed,       // found a re-declaration of `name`
    kScanLookupWidened,  // a using-DIRECTIVE in the window — REFUSE
    // Two REFUSALS, deliberately not one: they are different conditions with
    // different messages. Collapsing them made the refusal for an unreadable
    // statement claim the two edits "could not be ordered", which was untrue
    // — the window was ordered fine, the walk just met a statement it could
    // not unwrap. That is the same "refusal message asserting something
    // untrue" this file separates `unsupported-decl-shape` from
    // `publisher-shadowed` to avoid.
    kScanWindowNeverOpened,    // the two edits never bounded a window
    kScanUnreadableStatement,  // a statement in the window could not be read
  };

  ScanResult nameRedeclaredBetween(const CompoundStmt *scope,
                                   const Stmt *afterCarrier,
                                   const Stmt *beforeCarrier,
                                   const std::string &name) {
    bool between = false;
    for (const Stmt *child : scope->body()) {
      if (child == beforeCarrier) {
        break;
      }
      if (child == afterCarrier) {
        between = true;
        continue;
      }
      if (!between) {
        continue;
      }
      // A declaration can sit behind a statement WRAPPER and still declare
      // into this block: `lbl: auto pub_ = other_;` is a LabelStmt whose
      // child is the DeclStmt (a labeled-statement takes a statement, and a
      // declaration-statement is one). A bare dyn_cast to DeclStmt walks
      // past it and the shadow is MISSED — a rewrite, in the dangerous
      // direction.
      //
      // NOT AttributedStmt, though it is tempting to unwrap it here on the
      // assumption that `[[maybe_unused]] auto pub_ = …;` produces one.
      // MEASURED: it does not — the attribute attaches to the VarDecl and
      // the statement stays a plain DeclStmt, which the ordinary path
      // already handles. The statement attributes that DO make an
      // AttributedStmt (`[[likely]]`, `[[fallthrough]]`) are ill-formed on a
      // declaration, so no reachable input needed that arm.
      const Stmt *inner = child;
      bool unwrapped = false;
      for (int guard = 0; guard < 16 && inner; ++guard) {
        if (const auto *lbl = dyn_cast<LabelStmt>(inner)) {
          inner = lbl->getSubStmt();
          continue;
        }
        // `case 1: auto pub_ = other_;` — believed unreachable here (for the
        // declaration and the publish to share a switch body, every case
        // label would cross the message variable's initialization, which is
        // ill-formed), and covered anyway because `controlFlowAboveWithinScope`
        // does NOT rescue it: walking up from a publish under a case label
        // reaches the switch-body CompoundStmt — which IS the shared scope —
        // before it ever sees the SwitchStmt.
        if (const auto *sc = dyn_cast<SwitchCase>(inner)) {
          inner = sc->getSubStmt();
          continue;
        }
        unwrapped = true;
        break;
      }
      if (!unwrapped) {
        // Pathological wrapper depth — refuse, never assume, the way every
        // other depth guard in this file does (`topLevelStmtIn`,
        // `enclosingCompound`, `controlFlowAboveWithinScope`). Falling
        // through to the `continue` below would skip a declaration this
        // walk could not read: an ACCEPT, and the only depth guard here
        // that resolved in that direction.
        return kScanUnreadableStatement;
      }
      const auto *ds = dyn_cast_or_null<DeclStmt>(inner);
      if (!ds) {
        continue;
      }
      // NamedDecl, NOT VarDecl, and that is load-bearing rather than
      // incidental breadth. A block-scope USING-DECLARATION between the two
      // edits — `using ns::pub_;` — introduces the name into THIS block, so
      // the publish resolves to `ns::pub_` while the borrow spliced above it
      // still resolves to the member: exactly the divergence this proof
      // exists to refuse, and a `UsingDecl` is not a `VarDecl`. Narrowing
      // this to VarDecl would silently re-open it, which is why
      // unsafe_shadowed_publisher.cpp carries a using-declaration arm.
      for (const Decl *d : ds->decls()) {
        // A using-DIRECTIVE does not DECLARE the publisher's name, so the
        // name comparison below can never see it — and it still changes what
        // that name means. `using namespace ns;` makes every name in `ns`
        // visible FROM THAT POINT, so a publisher written unqualified can
        // resolve to `ns::pub_` at the publish while the borrow, spliced
        // ABOVE the directive, names something else or nothing at all.
        //
        // MEASURED on the reported shape: the original compiles and the
        // migrated source does not (`use of undeclared identifier 'pub_'`).
        // That residual can look harmless on
        // the grounds that the matrix's stage 3b builds the applied bytes —
        // but a USER has no stage 3b. Dry-run hands them an invalid patch,
        // and `--write` commits code that fails to build. Refused, and given
        // its own reason because the remedy differs from a shadow's.
        if (isa<UsingDirectiveDecl>(d)) {
          return kScanLookupWidened;
        }
        const auto *nd = dyn_cast<NamedDecl>(d);
        if (!nd) {
          continue;
        }
        if (nd->getDeclName().isIdentifier() && nd->getName() == name) {
          return kScanShadowed;
        }
        // A structured binding does NOT declare its names at this level:
        // `auto [pub_, n] = …;` yields ONE DecompositionDecl whose own name
        // is EMPTY, with the BindingDecls hanging off it as children. So the
        // loop above can never match a structured-binding shadow, and
        // `auto [pub_, n]` between the two edits was rewritten.
        if (const auto *dd = dyn_cast<DecompositionDecl>(nd)) {
          for (const BindingDecl *bd : dd->bindings()) {
            if (bd && bd->getDeclName().isIdentifier() &&
                bd->getName() == name) {
              return kScanShadowed;
            }
          }
        }
      }
    }
    // `topLevelStmtIn`'s postcondition gives MEMBERSHIP — both carriers are
    // direct children of `scope` — but NOT order. That the declaration's
    // slot is reached first is a separate argument (a use cannot precede its
    // declaration in the same block, and `body()` is in source order), and
    // an argument is not a guarantee. If the window never opened, this walk
    // examined nothing, so "no shadow found" would be a claim it did not
    // earn: refuse, in the same defence-in-depth spelling the init-statement
    // guard above uses for its own believed-unreachable case.
    if (!between) {
      return kScanWindowNeverOpened;
    }
    return kScanNoShadow;
  }

  const CompoundStmt *enclosingCompound(const Stmt *s) {
    DynTypedNode cur = DynTypedNode::create(*s);
    for (int guard = 0; guard < 256; ++guard) {
      auto parents = ctx_.getParents(cur);
      if (parents.empty()) {
        return nullptr;
      }
      cur = parents[0];
      if (const auto *cs = cur.get<CompoundStmt>()) {
        return cs;
      }
      if (cur.get<FunctionDecl>() || cur.get<LambdaExpr>()) {
        return nullptr;
      }
    }
    return nullptr;
  }

  bool insideLambda(const Stmt *s) {
    DynTypedNode cur = DynTypedNode::create(*s);
    for (int guard = 0; guard < 256; ++guard) {
      auto parents = ctx_.getParents(cur);
      if (parents.empty()) {
        return false;
      }
      cur = parents[0];
      if (cur.get<LambdaExpr>()) {
        return true;
      }
      if (const auto *fd = cur.get<FunctionDecl>()) {
        return fd != fn_ && isa<CXXMethodDecl>(fd) &&
               cast<CXXMethodDecl>(fd)->getParent()->isLambda();
      }
    }
    return false;
  }

  // ---- candidates -------------------------------------------------------

  // A candidate is anchored at the EXPANSION
  // location, not the spelling location. `lineOf` (spelling) is right for a
  // REWRITE — an edit must land where the text actually is — but wrong for a
  // REPORT: when the publisher expression is spelled inside a macro body,
  // every invocation shares the `#define`'s (file, line), frequently in a
  // header outside `--src-root` that the operator does not own. The
  // expansion location names the call site they actually wrote. For a
  // location that is not macro-expanded the two are identical, so nothing
  // else in the report moves. The column comes along as the per-site key
  // that keeps two invocations on ONE source line distinct.
  void candidate(SourceLocation loc, llvm::StringRef reason,
                 llvm::StringRef detail) {
    SourceLocation at = sm_.getExpansionLoc(loc);
    CandidateResult c;
    c.file = realPathOf(sm_.getFilename(at));
    c.function = fn_->getQualifiedNameAsString();
    c.line = sm_.getExpansionLineNumber(loc);
    c.column = sm_.getExpansionColumnNumber(loc);
    c.reason = reason.str();
    c.detail = detail.str();
    results_.candidates.push_back(std::move(c));
  }

  // ---- the per-site prover ----------------------------------------------

  void analyzePublish(const CXXMemberCallExpr *call) {
    const CXXMethodDecl *md = call->getMethodDecl();
    const CXXRecordDecl *rd = md ? md->getParent() : nullptr;
    if (!rd) {
      return;
    }
    std::string cls = rd->getQualifiedNameAsString();
    const auto *spec = dyn_cast<ClassTemplateSpecializationDecl>(rd);
    if (cls != "rclcpp::Publisher" || !spec ||
        spec->getTemplateArgs().size() == 0) {
      // Report only publisher-flavored classes; a random `publish()` method
      // on an unrelated class is not a migration site.
      if (cls.find("Publisher") != std::string::npos) {
        candidate(call->getBeginLoc(), "unsupported-publisher-type", cls);
      }
      return;
    }
    QualType msgT = spec->getTemplateArgs()[0].getAsType().getCanonicalType();

    const auto *me = dyn_cast<MemberExpr>(call->getCallee());
    if (!me) {
      return;
    }

    if (call->getNumArgs() != 1) {
      candidate(call->getBeginLoc(), "unsupported-publish-shape",
                "publish() with argument count != 1");
      return;
    }

    // -- classify the argument -------------------------------------------
    const Expr *arg = stripWrappers(call->getArg(0));
    PublishSite site;
    site.msgType = msgT;

    const Expr *inner = nullptr;
    if (const Expr *mv = stdMoveArg(arg)) {
      site.argKind = PublishSite::kMove;
      inner = mv->IgnoreParenImpCasts();
    } else if (const auto *uo = dyn_cast<UnaryOperator>(arg)) {
      if (uo->getOpcode() == UO_Deref) {
        site.argKind = PublishSite::kDeref;
        inner = uo->getSubExpr()->IgnoreParenImpCasts();
      }
    } else if (const auto *oc = dyn_cast<CXXOperatorCallExpr>(arg)) {
      if (oc->getOperator() == OO_Star && oc->getNumArgs() == 1) {
        site.argKind = PublishSite::kDeref;
        inner = oc->getArg(0)->IgnoreParenImpCasts();
      }
    } else if (isa<DeclRefExpr>(arg)) {
      site.argKind = PublishSite::kRef;
      inner = arg;
    }

    if (!inner) {
      if (const auto *fieldRef = dyn_cast<MemberExpr>(arg)) {
        (void)fieldRef;
        candidate(call->getBeginLoc(), "retained-member",
                  "the published message is a class member — rewrite it as a "
                  "local, or migrate by hand");
      } else if (isa<CallExpr>(arg)) {
        candidate(call->getBeginLoc(), "message-built-elsewhere",
                  "the published message is another function's return value");
      } else {
        candidate(call->getBeginLoc(), "unsupported-publish-shape",
                  arg->getStmtClassName());
      }
      return;
    }

    const auto *dre = dyn_cast<DeclRefExpr>(inner);
    if (!dre) {
      if (const auto *fieldRef = dyn_cast<MemberExpr>(inner)) {
        (void)fieldRef;
        candidate(call->getBeginLoc(), "retained-member",
                  "the published message is a class member — rewrite it as a "
                  "local, or migrate by hand");
      } else if (isa<CallExpr>(inner)) {
        candidate(call->getBeginLoc(), "message-built-elsewhere",
                  "the published message is another function's return value");
      } else {
        candidate(call->getBeginLoc(), "unsupported-publish-shape",
                  inner->getStmtClassName());
      }
      return;
    }
    const auto *vd = dyn_cast<VarDecl>(dre->getDecl());
    if (!vd) {
      candidate(call->getBeginLoc(), "unsupported-publish-shape",
                "publish argument does not name a variable");
      return;
    }
    if (isa<ParmVarDecl>(vd)) {
      candidate(call->getBeginLoc(), "not-a-local",
                "the message arrives as a parameter — it is built in another "
                "function");
      return;
    }
    if (!vd->hasLocalStorage() ||
        vd->getDeclContext() != cast<DeclContext>(fn_)) {
      candidate(call->getBeginLoc(), "not-a-local",
                "the message variable is not a local of this function");
      return;
    }
    site.var = vd;

    // Capture-list check FIRST (independent of use-shape analysis): an
    // explicit `[msg]` capture retains the message even when the lambda
    // body never spells the name.
    if (capturedByAnyLambda(fn_->getBody(), vd)) {
      candidate(call->getBeginLoc(), "captured-by-lambda",
                "the message variable appears in a lambda capture list");
      return;
    }

    // -- classify the declaration shape ----------------------------------
    enum { kUnique, kShared, kStack, kAlreadyLoaned, kBad } declKind = kBad;
    std::string badReason, badDetail;
    QualType declMsgT;

    const Expr *init = vd->getInit();
    const Expr *strippedInit = init ? stripWrappers(init) : nullptr;
    if (strippedInit) {
      if (const auto *mc = dyn_cast<CXXMemberCallExpr>(strippedInit)) {
        const CXXMethodDecl *mmd = mc->getMethodDecl();
        if (mmd && mmd->getDeclName().isIdentifier() &&
            mmd->getName() == "borrow_loaned_message") {
          // False-skip guard: "already migrated" is claimed
          // ONLY when the callee is provably rclcpp::Publisher's own
          // loaned-message API — the same qualified-name proof as the
          // publish-site gate. A CUSTOM method that merely shares the
          // name is NOT idempotence: it falls through to normal decl
          // classification (a member-call initializer that is not
          // std::make_* reports message-built-elsewhere), so the site is
          // reported rather than silently skipped.
#ifndef CERULION_MIGRATE_MUTANT_DROP_LOAN_PROOF
          const CXXRecordDecl *owner = mmd->getParent();
          bool provenPublisherApi =
              owner && owner->getQualifiedNameAsString() == "rclcpp::Publisher";
#else
          bool provenPublisherApi = true;
#endif
          // False-idempotence guard: the borrow must also target the
          // SAME publisher the publish targets — pub_a_->borrow paired
          // with pub_b_->publish is a cross-publisher defect, not
          // idempotence, and must land in the manual-candidate report
          // (message-built-elsewhere via the fall-through), never be
          // silently skipped. The comparison is the decl CHAIN of each
          // side's written core (the std arrow strip), and an
          // unresolvable chain on either side proves nothing:
          // conservative, not-already-migrated.
#ifndef CERULION_MIGRATE_MUTANT_DROP_LOAN_TARGET_PROOF
          if (provenPublisherApi) {
            const auto *pubMe = dyn_cast<MemberExpr>(call->getCallee());
            provenPublisherApi =
                pubMe != nullptr &&
                samePublisherChain(mc->getImplicitObjectArgument(),
                                   pubMe->getBase());
          }
#endif
          if (provenPublisherApi) {
            declKind = kAlreadyLoaned;
          }
        }
      }
      if (declKind == kBad) {
        if (const auto *ce = dyn_cast<CallExpr>(strippedInit)) {
          const FunctionDecl *fd = ce->getDirectCallee();
          std::string fname;
          if (fd && fd->getDeclName().isIdentifier() &&
              fd->isInStdNamespace()) {
            fname = fd->getName().str();
          }
          if (fname == "make_unique" || fname == "make_shared") {
            if (explicitArgCount(ce) != 0) {
              declKind = kBad;
              badReason = "constructor-args";
              badDetail = "std::" + fname +
                          " with constructor arguments — the loaned message "
                          "is default-initialized";
            } else {
              declKind = (fname == "make_unique") ? kUnique : kShared;
              // T from unique_ptr<T> / shared_ptr<T>.
              QualType vt = vd->getType().getCanonicalType();
              if (const auto *vspec =
                      vt->getAsCXXRecordDecl()
                          ? dyn_cast<ClassTemplateSpecializationDecl>(
                                vt->getAsCXXRecordDecl())
                          : nullptr) {
                if (vspec->getTemplateArgs().size() > 0) {
                  declMsgT = vspec->getTemplateArgs()[0]
                                 .getAsType()
                                 .getCanonicalType();
                }
              }
            }
          } else {
            declKind = kBad;
            badReason = "message-built-elsewhere";
            badDetail =
                "the message is initialized from another function's return "
                "value";
          }
        } else if (const auto *cce = dyn_cast<CXXConstructExpr>(strippedInit)) {
          // Count EXPLICIT arguments only: `T msg;` on a rosidl message
          // resolves to `T_(MessageInitialization = ALL)`, i.e. one
          // CXXDefaultArgExpr — still a default construction.
          if (explicitArgCount(cce) == 0) {
            declKind = kStack;
            declMsgT = vd->getType().getCanonicalType();
          } else {
            declKind = kBad;
            badReason = "constructor-args";
            badDetail = "message constructed with arguments — the loaned "
                        "message is default-initialized";
          }
        } else if (isa<InitListExpr>(strippedInit)) {
          const auto *ile = cast<InitListExpr>(strippedInit);
          if (ile->getNumInits() == 0) {
            declKind = kStack;
            declMsgT = vd->getType().getCanonicalType();
          } else {
            declKind = kBad;
            badReason = "constructor-args";
            badDetail = "braced initializer with arguments";
          }
        } else {
          declKind = kBad;
          badReason = "unsupported-decl-shape";
          badDetail = strippedInit->getStmtClassName();
        }
      }
    } else if (!init && vd->getType()->getAsCXXRecordDecl()) {
      declKind = kStack;
      declMsgT = vd->getType().getCanonicalType();
    } else {
      declKind = kBad;
      badReason = "unsupported-decl-shape";
      badDetail = "uninitialized non-class declaration";
    }

    if (declKind == kAlreadyLoaned) {
      return;  // already the migrated shape — silently idempotent
    }
    if (declKind == kBad) {
      candidate(call->getBeginLoc(), badReason, badDetail);
      return;
    }

    // -- shape/argument consistency --------------------------------------
    bool consistent =
        (declKind == kUnique && site.argKind == PublishSite::kMove) ||
        (declKind == kShared && site.argKind == PublishSite::kDeref) ||
        (declKind == kStack && (site.argKind == PublishSite::kRef ||
                                site.argKind == PublishSite::kMove));
    if (!consistent) {
      candidate(call->getBeginLoc(), "unsupported-publish-shape",
                "declaration shape and publish argument shape do not pair");
      return;
    }

    if (declMsgT.isNull() || !ctx_.hasSameUnqualifiedType(declMsgT, msgT)) {
      candidate(call->getBeginLoc(), "type-mismatch",
                "declared message type is not the publisher's message type");
      return;
    }

    // -- publisher expression: written core + triviality ------------------
    // RANGE CONTRACT: the publish EDIT replaces the WHOLE
    // CXXMemberCallExpr — publisher, operator, member, arguments — and the
    // publisher text spliced into replacements is the WRITTEN CORE with NO
    // trailing operator (`op` re-adds it). For a smart-pointer publisher
    // the callee MemberExpr's base is the `operator->` DESUGARING CALL,
    // and — measured in the container — that node's source range SPANS the
    // written `->` token, so taking the base's text and re-appending `op`
    // doubled the arrow (`pub_->->borrow_loaned_message()`). Strip AT MOST
    // ONE operator-> desugaring to reach the core (its written arrow is
    // exactly what `op` re-adds); a core still holding any operator call
    // (`operator*`, a second `operator->`) is refused as non-trivial —
    // never rewritten through an untested text path.
    const Expr *pubCore = me->getBase()->IgnoreParenImpCasts();
    if (const auto *oc = dyn_cast<CXXOperatorCallExpr>(pubCore)) {
      if (oc->getOperator() == OO_Arrow && oc->getNumArgs() >= 1 &&
          isStdSmartPointerArrow(oc)) {
        // Strip ONLY a std smart-pointer arrow. The
        // REFUSAL that stood here (a loud reject of any non-std
        // arrow, mutant DROP_ARROW_IDENTITY) was DELETED
        // after a matrix kill-regression forced the subsumption
        // question, and this time subsumption HOLDS — proof, by cases on
        // a non-std arrow the old check would have refused:
        //   1. The arrow does not strip (this condition), so pubCore
        //      stays the OPERATOR CALL; the exact-type gate
        //      below sees the call's RESULT type (a raw pointer or a
        //      wrapper class, never exactly rclcpp::Publisher) and
        //      refuses unsupported-publisher-type. Were the gate ever
        //      to pass, triviality still refuses a call node.
        //   2. The one shape that reaches the gate as std-smart-pointer-
        //      of-exact-Publisher necessarily uses std's OWN stateless
        //      operator-> — C++ has no free operator-> overload and
        //      rclcpp::Publisher declares none, so no stateful arrow can
        //      coexist with that type.
        //   3. Multi-level arrows (a wrapper whose operator-> returns a
        //      shared_ptr) strip at most one level and leave an operator
        //      call in the core — refused by the gate/triviality, as
        //      before.
        // A stateful arrow passing the exact-pointee gate is therefore
        // unconstructible, no isolating fixture can exist, and the check
        // was dead code per policy. The identity CLASS keeps its fixture
        // (unsafe_stateful_arrow.cpp, now refused by the exact-type
        // gate) and its guard is the DROP_EXACT_PUBLISHER
        // mutant.
        pubCore = oc->getArg(0)->IgnoreParenImpCasts();
      }
    }
#ifndef CERULION_MIGRATE_MUTANT_DROP_EXACT_PUBLISHER
    // Unsafe-rewrite guard: the WRITTEN publisher's static type must
    // be EXACTLY rclcpp::Publisher. For an arrow form pubCore is the smart
    // pointer — check its POINTEE; for a direct object, the type itself.
    {
      QualType objT = pubCore->getType().getCanonicalType();
      QualType pointee = objT;
      if (const auto *rd = objT->getAsCXXRecordDecl()) {
        if (rd->isInStdNamespace() &&
            (rd->getName() == "shared_ptr" || rd->getName() == "unique_ptr")) {
          if (const auto *spec = dyn_cast<ClassTemplateSpecializationDecl>(rd)) {
            if (spec->getTemplateArgs().size() > 0) {
              pointee = spec->getTemplateArgs()[0].getAsType();
            }
          }
        }
      }
      if (!isExactRclcppPublisher(pointee)) {
        candidate(call->getBeginLoc(), "unsupported-publisher-type",
                  "the publisher's declared type is not exactly "
                  "rclcpp::Publisher — a derived class can hide "
                  "borrow_loaned_message(), so the generated borrow could "
                  "resolve to the wrong API");
        return;
      }
    }
#endif
#ifndef CERULION_MIGRATE_MUTANT_DROP_PUBLISHER_TRIVIALITY
    if (!trivialPublisherExpr(pubCore)) {
      candidate(call->getBeginLoc(), "publisher-expr-not-trivial",
                "the publisher expression must be a plain variable or "
                "member chain — it is evaluated twice (borrow + publish)");
      return;
    }
#endif
    // -- publisher reassignment scan (the WHOLE chain) --------------------
    {
      std::vector<const ValueDecl *> chain;
      publisherChainDecls(pubCore, chain);
      for (const ValueDecl *pubDecl : chain) {
        if (publisherReassignedIn(fn_->getBody(), pubDecl)) {
          candidate(call->getBeginLoc(), "publisher-reassigned",
                    "the publisher (or an object holding it) is assigned "
                    "in this function — the borrow source could differ "
                    "from the publish target");
          return;
        }
      }
#ifndef CERULION_MIGRATE_MUTANT_DROP_ALIAS_SCAN
      // Publisher aliasing: a local REFERENCE or POINTER bound
      // to any chain decl (`auto &alias = pub_;` / `auto *p = &pub_;`)
      // lets a later write go through the ALIAS's decl, which the
      // per-decl scan above cannot see. Conservative: the alias BINDING
      // itself refuses — tracking writes through aliases soundly would
      // be an escape analysis, and refused-not-guessed is the contract.
      if (publisherAliasedIn(fn_->getBody(), chain)) {
        candidate(call->getBeginLoc(), "publisher-reassigned",
                  "a reference/pointer alias to the publisher (or an "
                  "object holding it) is taken in this function — writes "
                  "through the alias cannot be tracked, so the borrow and "
                  "publish targets cannot be proven identical");
        return;
      }
#endif
    }

    // -- locate ranges ----------------------------------------------------
    const DeclStmt *declStmt = findDeclStmt(vd);
    if (!declStmt || !declStmt->isSingleDecl()) {
      candidate(call->getBeginLoc(), "unsupported-decl-shape",
                "multi-variable declaration statement");
      return;
    }
    // Defense-in-depth: the DeclStmt must sit directly in a block — an
    // if/for/switch INIT-STATEMENT position cannot host the two-statement
    // replacement (invalid C++). With the same-nearest-CompoundStmt gate
    // above this is believed unreachable (an init-statement declaration's
    // scope confines its uses to the control statement, whose bodies are
    // different compounds), but the guarantee is structural rather than a
    // reachability argument.
    {
      auto declParents = ctx_.getParents(*declStmt);
      if (declParents.empty() ||
          declParents[0].get<CompoundStmt>() == nullptr) {
        candidate(call->getBeginLoc(), "unsupported-decl-shape",
                  "declared in a control-statement initializer");
        return;
      }
    }

    RangeInfo declRange = rangeInfo(declStmt->getSourceRange());
    RangeInfo callRange = rangeInfo(call->getSourceRange());
    RangeInfo argRange = rangeInfo(call->getArg(0)->getSourceRange());
    if (declRange.macro || callRange.macro || argRange.macro) {
      candidate(call->getBeginLoc(), "macro-expansion",
                "the site is (partly) inside a macro expansion");
      return;
    }
    if (declRange.file != callRange.file) {
      candidate(call->getBeginLoc(), "macro-expansion",
                "declaration and publish are spelled in different files");
      return;
    }
    if (!pathUnder(declRange.file, srcRootReal_)) {
      return;  // outside the workspace source root — not ours to touch
    }
    // DeclStmt ranges normally include the trailing semicolon; verify, and
    // extend over trailing whitespace to a directly-following `;` if not.
    if (declRange.text.empty() || declRange.text.back() != ';') {
      FileID fid = sm_.getFileID(
          sm_.getSpellingLoc(declStmt->getSourceRange().getBegin()));
      bool invalid = false;
      llvm::StringRef buf = sm_.getBufferData(fid, &invalid);
      unsigned end = declRange.offset + declRange.length;
      while (!invalid && end < buf.size() &&
             (buf[end] == ' ' || buf[end] == '\t')) {
        ++end;
      }
      if (invalid || end >= buf.size() || buf[end] != ';') {
        candidate(call->getBeginLoc(), "unsupported-decl-shape",
                  "could not locate the declaration's terminating semicolon");
        return;
      }
      declRange.length = end + 1 - declRange.offset;
      declRange.text = buf.substr(declRange.offset, declRange.length).str();
    }

    RangeInfo pubRange = rangeInfo(pubCore->getSourceRange());
    if (pubRange.macro || pubRange.text.empty()) {
      candidate(call->getBeginLoc(), "macro-expansion",
                "the publisher expression is (partly) inside a macro");
      return;
    }
    std::string pubText = pubRange.text;
    std::string op = me->isArrow() ? "->" : ".";
    // Range-contract invariant: the core text must carry NO trailing
    // operator — `op` re-adds it. A regression here (the
    // doubled-arrow class) must REFUSE the site loudly, never emit bytes
    // that do not build.
    auto endsWith = [](const std::string &s, const char *suf) {
      size_t n = std::strlen(suf);
      return s.size() >= n && s.compare(s.size() - n, n, suf) == 0;
    };
    if (endsWith(pubText, "->") || endsWith(pubText, ".")) {
      candidate(call->getBeginLoc(), "publisher-expr-not-trivial",
                "internal range-contract violation: the publisher core text "
                "ends in an operator — refusing rather than emitting a "
                "doubled operator");
      return;
    }

    // A template-argument list written
    // inside the publisher core is refused, on the TEXT, before any of the
    // name analysis runs.
    //
    // The scope proof below resolves the core to the identifiers ordinary
    // unqualified lookup resolves: a chain ROOT, and a qualifier's OUTERMOST
    // specifier. A template argument is neither — and it is looked up
    // UNQUALIFIED at the point of use, so `ns::Holder<Tag>` does not find
    // `ns::Tag`. A block-scope `using Tag = Other;` written between the two
    // edits therefore changes which specialization the SAME written text
    // names, and the borrow — spliced above that alias — names a different
    // publisher than the publish does. Both versions compile, so nothing
    // downstream catches it either.
    //
    // MEASURED, clang++ -std=c++17 -Wall -Wextra: the identical written text
    // `ns::Holder<TagA>::id` reads 11 above a block-scope `using TagA = Beta;`
    // and 22 below it, with no diagnostic on either.
    //
    // Two spellings reach this: a template argument inside a QUALIFIER
    // (`ns::Holder<Tag>::pub_`, whose tail `classifyQualifierHead` walks past
    // on its way to the outermost specifier), and explicit arguments on the
    // ID-EXPRESSION itself (`pub_v<Tag>`, a variable template — its
    // DeclRefExpr's written range was AST-dump-verified to span `pub_v<Tag>`,
    // so the text carries `Tag` while the root collects only `pub_v`).
    //
    // The test is on the TEXT, and that is the point: it is TOTAL. Every one
    // of the seven false accepts already closed in this class was reached by
    // an AST shape nobody had enumerated, so the belt that closes this one
    // does not depend on enumerating shapes. A trivial core is only
    // identifiers, `.`, `->` and `::`; no construct that can reach this line
    // carries a `<` otherwise.
    //
    // Deliberately NOT paired with an AST test, and the reason is
    // soundness of the mutant suite rather than economy. `DeclRefExpr`/`MemberExpr::
    // hasExplicitTemplateArgs()` do exist and look like natural
    // braces — but this belt runs FIRST and is total, so such a test
    // could never fire in a shipped build. It would fire under a MUTANT
    // that compiles this belt out, silently keeping the fixture refused and
    // reporting the mutant killed when the check it isolates is gone. That
    // is exactly how a redundant check masks `DROP_PUBLISHER_TRIVIALITY`
    // (and `DROP_ARROW_IDENTITY`): a second, redundant
    // refusal is dead in production and alive only under the mutation that
    // matters. One check, one mutant, one kill.
    //
    // Known over-refusal, in the safe direction: a COMMENT written between
    // the core's own tokens is inside its source range, so `holder_ /*<*/
    // .pub_` is refused. No fixture produces one, and a refusal costs a
    // migration the tool declines to make rather than a wrong one.
#ifndef CERULION_MIGRATE_MUTANT_DROP_PUBLISHER_TEMPLATE_ARGS
    if (pubText.find('<') != std::string::npos) {
      candidate(call->getBeginLoc(), "publisher-expr-not-trivial",
                "the publisher expression carries a template argument list, "
                "whose names ordinary unqualified lookup resolves at the "
                "publish site — they can mean something else where the "
                "borrow is spliced");
      return;
    }
#endif

    // -- use analysis -----------------------------------------------------
    std::vector<const DeclRefExpr *> uses;
    collectUses(fn_->getBody(), vd, uses);
    unsigned publishUses = 0;
    unsigned callEnd = callRange.offset + callRange.length;
    for (const DeclRefExpr *u : uses) {
      RangeInfo ur = rangeInfo(u->getSourceRange());
      if (ur.macro) {
        candidate(call->getBeginLoc(), "macro-expansion",
                  "a use of the message variable is inside a macro");
        return;
      }
      // Skip the use that IS part of the declaration's own init (none: a
      // DeclRefExpr to vd cannot appear in its own init legally except via
      // weird self-reference; treat those as ordinary uses).
      if (ur.offset >= declRange.offset &&
          ur.offset < declRange.offset + declRange.length) {
        continue;
      }
      bool isPublishUse =
          ur.offset >= argRange.offset &&
          ur.offset < argRange.offset + argRange.length &&
          publishArgConsumesVar(site);
      // A use inside ANOTHER publish call's argument also counts as a
      // publish use (reuse-after-move detection below).
      if (!isPublishUse) {
        for (const CXXMemberCallExpr *other : publishCalls_) {
          if (other == call) {
            continue;
          }
          RangeInfo oar = other->getNumArgs() == 1
                              ? rangeInfo(other->getArg(0)->getSourceRange())
                              : RangeInfo{};
          if (!oar.macro && oar.length > 0 && ur.offset >= oar.offset &&
              ur.offset < oar.offset + oar.length) {
            ++publishUses;  // counted extra below via publishUses
            isPublishUse = true;
            break;
          }
        }
        if (isPublishUse) {
          // Fall through to the after-publish check for this use too.
          if (ur.offset >= callEnd) {
#ifndef CERULION_MIGRATE_MUTANT_DROP_USE_AFTER_PUBLISH
            if (reuseReported_.insert(vd).second) {
              candidate(call->getBeginLoc(), "reuse-after-move",
                        "the message variable is published more than once");
            }
            return;
#endif
          }
          continue;
        }
      } else {
        ++publishUses;
        continue;
      }
      // Non-publish use.
      if (insideLambda(u)) {
        candidate(call->getBeginLoc(), "captured-by-lambda",
                  "the message variable is used inside a lambda");
        return;
      }
#ifndef CERULION_MIGRATE_MUTANT_DROP_USE_AFTER_PUBLISH
      if (ur.offset >= callEnd) {
        candidate(call->getBeginLoc(), "use-after-publish",
                  "the message variable is used after the publish call");
        return;
      }
#endif
#ifndef CERULION_MIGRATE_MUTANT_DROP_ESCAPE_CHECK
      if (!isFillUse(u)) {
        candidate(call->getBeginLoc(), "pointer-escapes",
                  "the message variable is used other than as a field "
                  "fill or the publish");
        return;
      }
#endif
    }
    if (publishUses > 1) {
      if (reuseReported_.insert(vd).second) {
        candidate(call->getBeginLoc(), "reuse-after-move",
                  "the message variable is published more than once");
      }
      return;
    }

    // -- control-flow scope ----------------------------------------------
    const CompoundStmt *declScope = enclosingCompound(declStmt);
    const CompoundStmt *pubScope = enclosingCompound(call);
    if (!declScope || declScope != pubScope) {
      candidate(call->getBeginLoc(), "conditional-publish",
                "declaration and publish sit in different statement scopes");
      return;
    }
#ifndef CERULION_MIGRATE_MUTANT_DROP_CONTROL_FLOW_WALK
    // Control flow: the compound compare alone accepts an
    // UNBRACED control body — `if (ready) pub_->publish(std::move(msg));`
    // has no CompoundStmt of its own, so the publish shares the FUNCTION
    // block with the declaration while executing conditionally (or
    // repeatedly, in an unbraced loop): the rewrite would borrow
    // unconditionally and publish on a branch. Walk from the call up to
    // the shared compound; ANY control construct on the way refuses.
    if (controlFlowAboveWithinScope(call, pubScope)) {
      candidate(call->getBeginLoc(), "conditional-publish",
                "the publish executes under control flow (an unbraced "
                "if/loop/switch body or a conditional expression) — the "
                "borrow would run unconditionally while the publish may "
                "not, or may repeat");
      return;
    }
#endif

#ifndef CERULION_MIGRATE_MUTANT_DROP_PUBLISHER_SHADOW
    // The unsafe-rewrite guard: the publisher name must
    // mean the SAME entity where the borrow is spliced as it does at the
    // publish.
    //
    // The two edits do not sit in the same place: `pubText` is the
    // publisher as WRITTEN AT THE PUBLISH, and it is spliced into the
    // DECLARATION's replacement, above every statement between them. The
    // publisher-identity proofs so far all reason about the AST's resolved
    // decl (triviality, the exact-type gate, the reassignment/alias scans)
    // — none of them asks what the spliced TEXT resolves to at its new
    // position. A declaration written between the two points can therefore
    // rebind the name under the borrow:
    //
    //     auto msg = std::make_unique<T>();   // borrow spliced HERE
    //     auto pub_ = other_;                 // shadows the member pub_
    //     pub_->publish(std::move(msg));      // resolves to the LOCAL
    //
    // Migrated, that borrows a loan from `this->pub_` and publishes it on
    // `other_` — a cross-publisher loan, silently, from code that was
    // well-defined. It is the class the accept gate exists to prevent, and
    // it arrives through name lookup rather than through the AST.
    //
    // ROOT only, deliberately: a member chain resolves everything after
    // its root by member lookup, so refusing on the terminal member's name
    // would reject `this->pub_` beside an unrelated local `pub_` — an
    // over-refusal with no defect behind it.
    {
      std::vector<std::string> rootNames;
      const RootKind rootKind = collectPublisherRoots(pubCore, rootNames);
      if (rootKind == kRootUnnameableRoot) {
        candidate(call->getBeginLoc(), "publisher-expr-not-trivial",
                  "the publisher expression's root is a name this "
                  "analysis cannot recover as the identifier that was "
                  "WRITTEN — a qualifier head it cannot resolve, or a "
                  "declaration carrying no identifier of its own (an "
                  "anonymous union member) — so the spliced borrow cannot "
                  "be proven to name the same publisher");
        return;
      }
      // kRootShapeAlreadyRefused falls through on purpose — see RootKind.
      // Both edits must be locatable as STATEMENTS of the block they share
      // before the interval between them means anything. Resolved here, not
      // inside the walk, so that "could not locate an edit" is refused under
      // its own reason: folded into the walk it would reach the operator as
      // `publisher-shadowed`, whose detail names a re-declaration that was
      // never found — a refusal message asserting something untrue.
      const Stmt *declCarrier = topLevelStmtIn(pubScope, declStmt);
      const Stmt *pubCarrier = topLevelStmtIn(pubScope, call);
      if (!declCarrier || !pubCarrier) {
        candidate(call->getBeginLoc(), "unsupported-decl-shape",
                  "the declaration and the publish could not both be located "
                  "as statements of the block they share");
        return;
      }
      // EVERY collected name, not just the first: a member access can carry
      // both a qualifier head and an object-expression root, and either one
      // being re-declared between the two edits breaks the splice.
      for (const std::string &rootName : rootNames) {
        const ScanResult scan = nameRedeclaredBetween(pubScope, declCarrier,
                                                      pubCarrier, rootName);
        // A total `switch` with NO `default`, and build.sh carries
        // `-Werror=switch`: an if-chain fell THROUGH to the rewrite path for
        // any value nobody had added a branch for, which is the wrong
        // direction for a refusal gate. A sixth ScanResult now fails the
        // build instead of silently accepting.
        switch (scan) {
          case kScanNoShadow:
            break;  // this name is clean — go on to the next one
          case kScanShadowed: {
            // A named local for readability of the three-part concatenation.
            // NOT a lifetime requirement: `candidate` takes its StringRef BY
            // VALUE and copies with `.str()` inside, and a temporary argument
            // lives to the end of the full-expression — i.e. through the
            // whole call — so an inline temporary would be sound too.
            const std::string detail =
                "a declaration between the message declaration and the "
                "publish re-declares `" +
                rootName +
                "` — the generated borrow is spliced ABOVE it, where that "
                "name resolves to a different publisher than the publish uses";
            candidate(call->getBeginLoc(), "publisher-shadowed", detail);
            return;
          }
          case kScanLookupWidened: {
            const std::string detail =
                "a using-directive between the message declaration and the "
                "publish widens unqualified lookup — the generated borrow is "
                "spliced ABOVE it, where `" +
                rootName +
                "` can name something else, or nothing at all";
            candidate(call->getBeginLoc(), "publisher-lookup-changed", detail);
            return;
          }
          // Both refusals below are reported under their OWN reason rather
          // than as `publisher-shadowed`, whose detail would name a
          // re-declaration that was never found — a refusal message
          // asserting something untrue, which is what the scan's header
          // says this separation exists to avoid. They are separate from
          // each other for the same reason.
          case kScanWindowNeverOpened:
            candidate(call->getBeginLoc(), "unsupported-decl-shape",
                      "the declaration and the publish could not be ordered "
                      "within the block they share");
            return;
          case kScanUnreadableStatement:
            candidate(call->getBeginLoc(), "unsupported-decl-shape",
                      "a statement between the message declaration and the "
                      "publish is wrapped more deeply than this analysis "
                      "unwraps, so the window could not be read");
            return;
        }
      }
    }
#endif

    // -- mint the loaned variable name ------------------------------------
    std::string loaned = mintLoanedName();
    if (loaned.empty()) {
      candidate(call->getBeginLoc(), "name-collision",
                "no free `loaned`/`loanedN` name in this function");
      return;
    }

    // -- build the edits ---------------------------------------------------
    FileID fid =
        sm_.getFileID(sm_.getSpellingLoc(declStmt->getSourceRange().getBegin()));
    std::string indent = lineIndentAt(fid, declRange.offset);
    std::string varName = vd->getName().str();

    // VERIFIED against the pinned sources: the
    // loaned message is VALUE-INITIALIZED here, because a loan does not
    // arrive initialized on every rmw and the code being replaced was.
    //
    //   rclcpp jazzy loaned_message.hpp: the can-loan branch takes the
    //   pointer `rcl_borrow_loaned_message` returns and CASTS it; it
    //   placement-news `MessageT()` only in the cannot-loan branch.
    //   rmw_fastrtps jazzy rmw_publisher.cpp: `loan_sample(*ros_message)`
    //   with no `LoanInitializationKind`, and no construction after.
    //   Fast DDS DataWriter.hpp: that parameter defaults to
    //   NO_LOAN_INITIALIZATION — "Do not perform initialization of sample".
    //
    // So on the ROS 2 DEFAULT rmw the buffer is uninitialized, possibly
    // recycled memory, while every construction this tool accepts
    // (`make_unique<T>()`, `make_shared<T>()`, `T msg;` — all with no
    // constructor arguments, enforced above) VALUE-initializes. Without this
    // store, a field the site does not write changes from its declared
    // default to whatever the loan pool last held: no compile error, no
    // crash, wrong data on the wire.
    //
    // ASSIGNMENT, not placement-new, and the direction matters both ways.
    // On the cannot-loan path rclcpp has ALREADY constructed a real
    // `MessageT` — for the non-plain types that take that path it owns heap
    // members — so a placement-new over it would LEAK them; assignment is a
    // proper assignment of a live object. On the loaning path rmw_fastrtps
    // loans only `is_plain()` types, whose implicit copy-assignment is
    // trivial: it WRITES the destination without reading it, so no
    // indeterminate value is ever read. Assignment is therefore correct
    // where the object exists and safe where it does not; placement-new is
    // the reverse. On `rmw_cerulion`, which constructs the slot itself, the
    // store is a redundant reset.
    //
    // Emitted for EVERY accepted kind rather than elided on a per-field
    // proof: proving that a site writes every field (through nested
    // messages, arrays and aliases) is a whole analysis, and refusing every
    // site that does not would refuse nearly all real ones — almost nobody
    // writes every field of a Twist. The cost is one value-init store on a
    // path that has just taken a loan.
    std::string declRepl = "auto " + loaned + " = " + pubText + op +
                           "borrow_loaned_message();\n" + indent;
    PrintingPolicy initPp(lo_);
    initPp.SuppressTagKeyword = true;
    declRepl += loaned + ".get() = " + site.msgType.getAsString(initPp) +
                "();\n" + indent;
    if (declKind == kStack) {
      declRepl += "auto& " + varName + " = " + loaned + ".get();";
    } else {
      declRepl += "auto " + varName + " = &" + loaned + ".get();";
    }
    std::string callRepl =
        pubText + op + "publish(std::move(" + loaned + "))";

    RewriteResult rw;
    rw.file = declRange.file;
    rw.function = fn_->getQualifiedNameAsString();
    rw.kind = (declKind == kUnique)   ? "unique_ptr"
              : (declKind == kShared) ? "shared_ptr"
                                      : "stack";
    PrintingPolicy pp(lo_);
    pp.SuppressTagKeyword = true;
    rw.message_type = site.msgType.getAsString(pp);
    rw.publisher = pubText;
    rw.line = lineOf(call->getBeginLoc());
    Edit declEdit{declRange.offset, declRange.length, declRange.text, declRepl};
    Edit callEdit{callRange.offset, callRange.length, callRange.text, callRepl};
    rw.edits.push_back(declEdit);
    rw.edits.push_back(callEdit);
    results_.rewrites.push_back(std::move(rw));
    mintedNames_.insert(loaned);
  }

  // The classified publish argument consumed the variable (kMove/kRef/kDeref
  // all name it).
  static bool publishArgConsumesVar(const PublishSite &site) {
    return site.var != nullptr && site.argKind != PublishSite::kOther;
  }

  // A use is a fill use when, after stripping sugar, it is the base of a
  // member access (msg->field, msg.field, (*msg).field).
  // The further question: does the glvalue this field access
  // produces ESCAPE the statement — is its address taken, or is it bound to
  // a reference?
  //
  // An `isFillUse` that accepts ANY use whose parent is a field MemberExpr
  // and stops there, without asking what happens to the result, is unsound:
  // `collectUses` only tracks direct `DeclRefExpr`s naming the message
  // variable, so an alias derived from a field is invisible to the
  // use-after-publish scan. Combined with the shape table accepting a
  // `kStack` declaration published by ref/copy — `publish(msg)` binds
  // `publish(const T&)` and COPIES, so `msg` and any alias into it stay
  // valid afterwards — it would rewrite
  //
  //     std_msgs::msg::String msg;
  //     auto *p = &msg.data;      // accepted as a harmless "fill"
  //     pub_->publish(msg);        // a safe copy-publish
  //     p->append("!");            // WELL-DEFINED before the rewrite
  //
  // into `publish(std::move(loaned))`, moving the loan out from under `p`
  // and turning correct code into a use-after-move. The header's aliasing
  // limit does NOT excuse this shape: it excuses an alias that "was equally
  // dangling in the original `std::move` form", and here nothing was moved
  // and the alias was never dangling.
  //
  // Refusing is the conservative direction — the site becomes a reported
  // `pointer-escapes` candidate instead of a rewrite, which is what this
  // tool promises to do with anything it cannot prove.
  //
  // The walk passes through the sugar that can sit between the field and the
  // escape (parens, implicit casts, further field selection, subscripting)
  // and decides at the address-of or the reference binding. SCOPE: a
  // pointer obtained through a member CALL on the field (`msg.data.data()`)
  // is not covered here — that is the documented aliasing
  // limit, not this check.
  // The third escape spelling: is `arg`
  // sitting in a parameter slot of `ce` that binds BY REFERENCE?
  //
  // `&msg.field` and `auto &r = msg.field;` are the first two spellings; the
  // third spelling of the same escape is the field handed to a callee that
  // keeps the reference:
  //
  //     void register_sink(std::string &s) { g_sink = &s; }
  //     register_sink(msg.data);   // binds by reference, no cast node
  //     pub_->publish(msg);         // would be rewritten to publish(std::move(...))
  //     g_sink->append("!");        // into moved-from loan memory
  //
  // A reference parameter binds WITHOUT an `ImplicitCastExpr`, so the field's
  // parent is the call itself and every other arm of the walk misses it, falling
  // through to "does not escape". That is the exact use-after-move class
  // the walk exists to close, and the scope note only ever exempts member CALLS
  // on the field, not passing the field as an argument.
  //
  // The check is deliberately NARROW so it can only ADD refusals: it fires
  // only when the node the walk came from IS one of the call's arguments.
  // A field in the OBJECT position (`msg.data.push_back(c)` — the ordinary
  // fill idiom) is not an argument, so it is untouched and keeps behaving
  // as a fill. An unreadable callee (function pointer, variadic slot,
  // a parameter that cannot be matched) is treated as escaping, which is the
  // conservative direction this tool refuses in.
  bool callArgumentBindsByReference(const CallExpr *ce, const Stmt *arg) {
    const Expr *argE = dyn_cast_or_null<Expr>(arg);
    if (!argE) {
      return false;
    }
    const Expr *want = stripWrappers(argE);
    unsigned first = 0;
    const FunctionDecl *fd = ce->getDirectCallee();
    if (isa<CXXOperatorCallExpr>(ce) && fd && isa<CXXMethodDecl>(fd)) {
      first = 1;  // argument 0 of an operator call is the object
    }
    for (unsigned i = first; i < ce->getNumArgs(); ++i) {
      const Expr *a = ce->getArg(i);
      if (!a || (a != argE && stripWrappers(a) != want)) {
        continue;
      }
      if (!fd) {
        return true;  // the signature is not visible — assume it keeps it
      }
      unsigned pi = i - first;
      if (pi >= fd->getNumParams()) {
        return true;  // a variadic slot: nothing to inspect
      }
      return fd->getParamDecl(pi)->getType()->isReferenceType();
    }
    return false;  // not an argument (the object position) — unchanged
  }

  bool fieldGlvalueEscapes(const Expr *fieldExpr) {
    const Stmt *prev = fieldExpr;
    DynTypedNode cur = DynTypedNode::create(*fieldExpr);
    for (int guard = 0; guard < 32; ++guard) {
      auto parents = ctx_.getParents(cur);
      if (parents.empty()) {
        return false;
      }
      const DynTypedNode next = parents[0];
      // A call reached from an ARGUMENT position escapes when the
      // parameter binds by reference. Checked before the sugar arms below so
      // an operator call carrying the field as an argument is not mistaken
      // for subscript/deref sugar.
      if (const auto *ce = next.get<CallExpr>()) {
        if (callArgumentBindsByReference(ce, prev)) {
          return true;
        }
      }
      if (const Stmt *asStmt = next.get<Stmt>()) {
        prev = asStmt;
      }
      cur = next;
      if (cur.get<ParenExpr>() || cur.get<ImplicitCastExpr>() ||
          cur.get<MemberExpr>() || cur.get<ArraySubscriptExpr>()) {
        continue;  // still naming a place inside the message
      }
      if (const auto *oc = cur.get<CXXOperatorCallExpr>()) {
        if (oc->getOperator() == OO_Subscript ||
            oc->getOperator() == OO_Star || oc->getOperator() == OO_Arrow) {
          continue;  // `msg.data[0]`, `(*msg).f` — still a place
        }
        return false;
      }
      if (const auto *uo = cur.get<UnaryOperator>()) {
        // `&msg.field` — the address outlives the statement.
        return uo->getOpcode() == UO_AddrOf;
      }
      if (const auto *vd = cur.get<VarDecl>()) {
        // `auto &r = msg.field;` binds an alias; `auto v = msg.field;`
        // copies and is a perfectly ordinary fill.
        return vd->getType()->isReferenceType();
      }
      return false;
    }
    return false;
  }

  bool isFillUse(const DeclRefExpr *u) {
    DynTypedNode cur = DynTypedNode::create(*u);
    for (int guard = 0; guard < 32; ++guard) {
      auto parents = ctx_.getParents(cur);
      if (parents.empty()) {
        return false;
      }
      cur = parents[0];
      if (const auto *me = cur.get<MemberExpr>()) {
        // A fill is a FIELD access (msg->field / msg.field). A member
        // FUNCTION reference (msg.get(), msg.reset(), msg->set__x(...)) is
        // not a fill — .get() in particular is exactly how a pointer
        // escapes. A FIELD access is only a fill if the glvalue it
        // produces does not escape either (see `fieldGlvalueEscapes`).
        if (!isa<FieldDecl>(me->getMemberDecl())) {
          return false;
        }
        return !fieldGlvalueEscapes(me);
      }
      if (cur.get<ImplicitCastExpr>() || cur.get<ParenExpr>()) {
        continue;
      }
      if (const auto *oc = cur.get<CXXOperatorCallExpr>()) {
        if (oc->getOperator() == OO_Arrow || oc->getOperator() == OO_Star) {
          continue;  // smart-pointer sugar on the way to a MemberExpr
        }
        return false;
      }
      if (const auto *uo = cur.get<UnaryOperator>()) {
        if (uo->getOpcode() == UO_Deref) {
          continue;  // (*msg).field
        }
        return false;
      }
      return false;
    }
    return false;
  }

  void collectUses(const Stmt *s, const VarDecl *vd,
                   std::vector<const DeclRefExpr *> &out) {
    if (const auto *dre = dyn_cast<DeclRefExpr>(s)) {
      if (dre->getDecl() == vd) {
        out.push_back(dre);
      }
    }
    for (const Stmt *child : s->children()) {
      if (child) {
        collectUses(child, vd, out);
      }
    }
  }

  const DeclStmt *findDeclStmt(const VarDecl *vd) {
    auto parents = ctx_.getParents(*vd);
    if (parents.empty()) {
      return nullptr;
    }
    return parents[0].get<DeclStmt>();
  }

  // Publisher aliasing: does `init` (an alias initializer,
  // stripped of at most one address-of) name any decl in the publisher
  // chain?
  static bool aliasTargetsChain(const Expr *init,
                                const std::vector<const ValueDecl *> &chain) {
    if (!init) {
      return false;
    }
    const Expr *e = init->IgnoreParenImpCasts();
    if (const auto *uo = dyn_cast<UnaryOperator>(e)) {
      if (uo->getOpcode() == UO_AddrOf) {
        e = uo->getSubExpr()->IgnoreParenImpCasts();
      }
    }
    std::vector<const ValueDecl *> ds;
    publisherChainDecls(e, ds);
    for (const ValueDecl *d : ds) {
      if (std::find(chain.begin(), chain.end(), d) != chain.end()) {
        return true;
      }
    }
    return false;
  }

  // Any local REFERENCE/POINTER variable in `s` bound to a chain decl.
  bool publisherAliasedIn(const Stmt *s,
                          const std::vector<const ValueDecl *> &chain) {
    if (const auto *ds = dyn_cast<DeclStmt>(s)) {
      for (const Decl *d : ds->decls()) {
        if (const auto *avd = dyn_cast<VarDecl>(d)) {
          QualType t = avd->getType();
          if ((t->isReferenceType() || t->isPointerType()) &&
              aliasTargetsChain(avd->getInit(), chain)) {
            return true;
          }
        }
      }
    }
    for (const Stmt *child : s->children()) {
      if (child && publisherAliasedIn(child, chain)) {
        return true;
      }
    }
    return false;
  }

  bool publisherReassignedIn(const Stmt *s, const ValueDecl *pubDecl) {
    if (const auto *bo = dyn_cast<BinaryOperator>(s)) {
      if (bo->isAssignmentOp() && namesDecl(bo->getLHS(), pubDecl)) {
        return true;
      }
    }
    if (const auto *oc = dyn_cast<CXXOperatorCallExpr>(s)) {
      if (oc->getOperator() == OO_Equal && oc->getNumArgs() >= 1 &&
          namesDecl(oc->getArg(0), pubDecl)) {
        return true;
      }
    }
    if (const auto *mc = dyn_cast<CXXMemberCallExpr>(s)) {
      const CXXMethodDecl *mmd = mc->getMethodDecl();
      if (mmd && mmd->getDeclName().isIdentifier() &&
          (mmd->getName() == "reset" || mmd->getName() == "swap") &&
          namesDecl(mc->getImplicitObjectArgument(), pubDecl)) {
        return true;
      }
    }
#ifndef CERULION_MIGRATE_MUTANT_DROP_CALL_MUTATION
    // Publisher mutation: a call can mutate the publisher
    // through a non-const reference/pointer parameter —
    // `std::swap(pub_, spare_)`, a helper taking `Publisher::SharedPtr&`
    // — which the assignment/reset/swap arms above cannot see. Any
    // ARGUMENT naming a chain decl (directly, or by address) counts as a
    // potential mutation unless the callee PROVABLY takes it by value or
    // const reference; an unresolvable callee, an out-of-range parameter
    // (variadic slot), or an address-taken argument proves nothing and is
    // treated as mutating — refused, never assumed harmless.
    if (const auto *ce = dyn_cast<CallExpr>(s)) {
      const FunctionDecl *callee = ce->getDirectCallee();
      // A member OPERATOR call carries its object as arg 0 (the OO_Equal
      // arm above owns assignment); explicit arguments start after it.
      unsigned firstArg = 0;
      if (isa<CXXOperatorCallExpr>(ce) && callee && isa<CXXMethodDecl>(callee)) {
        firstArg = 1;
      }
      for (unsigned i = firstArg; i < ce->getNumArgs(); ++i) {
        const Expr *a = ce->getArg(i)->IgnoreParenImpCasts();
        bool addressed = false;
        if (const auto *uo = dyn_cast<UnaryOperator>(a)) {
          if (uo->getOpcode() == UO_AddrOf) {
            a = uo->getSubExpr()->IgnoreParenImpCasts();
            addressed = true;
          }
        }
        if (!namesDecl(a, pubDecl)) {
          continue;
        }
        bool provablyHarmless = false;
        const unsigned pi = i - firstArg;
        if (!addressed && callee && pi < callee->getNumParams()) {
          QualType pt = callee->getParamDecl(pi)->getType();
          if (!pt->isReferenceType() && !pt->isPointerType()) {
            provablyHarmless = true;  // by value
          } else if (pt->isReferenceType() &&
                     pt.getNonReferenceType().isConstQualified()) {
            provablyHarmless = true;  // const reference
          }
        }
        if (!provablyHarmless) {
          return true;
        }
      }
    }
#endif
    for (const Stmt *child : s->children()) {
      if (child && publisherReassignedIn(child, pubDecl)) {
        return true;
      }
    }
    return false;
  }

  static bool namesDecl(const Expr *e, const ValueDecl *d) {
    e = e->IgnoreParenImpCasts();
    if (const auto *dre = dyn_cast<DeclRefExpr>(e)) {
      return dre->getDecl() == d;
    }
    if (const auto *me = dyn_cast<MemberExpr>(e)) {
      return me->getMemberDecl() == d;
    }
    return false;
  }

  static bool isIdentChar(char c) {
    return (c >= 'a' && c <= 'z') || (c >= 'A' && c <= 'Z') ||
           (c >= '0' && c <= '9') || c == '_';
  }

  // Identifier-boundary occurrence of `name` in `text`. Deliberately a TEXT
  // scan over the function body: local declarations alone miss a MEMBER or
  // GLOBAL (or macro) named `loaned` that an unchanged fill line resolves —
  // the inserted local would shadow it and change the line's meaning.
  // Conservative on purpose (an occurrence in a comment or string also
  // counts as taken): the worst case is skipping to `loaned2`…`loaned9` or
  // an explicit name-collision refusal, never a shadow.
  static bool identifierAppears(const std::string &text,
                                const std::string &name) {
    size_t pos = 0;
    while ((pos = text.find(name, pos)) != std::string::npos) {
      bool left = pos == 0 || !isIdentChar(text[pos - 1]);
      size_t end = pos + name.size();
      bool right = end >= text.size() || !isIdentChar(text[end]);
      if (left && right) {
        return true;
      }
      pos += 1;
    }
    return false;
  }

  std::string mintLoanedName() {
    auto nameFree = [&](const std::string &n) {
      // referencedNames_ is the AST's view (sees through macro
      // invocations); bodyText_ is the raw source's (sees names in
      // nested constructs the walk may not attribute). Both must clear
      // the minted name. Macro collision: so must the translation
      // unit's PREPROCESSOR MACROS — an object-like `#define loaned 42`
      // from a header sits outside all three (the token never appears
      // in the function and no AST node references it), yet an emitted
      // `auto loaned = ...` would be macro-expanded into `auto 42 = ...`.
      // g_macroNames holds every name #defined anywhere in the TU.
      return localNames_.count(n) == 0 && mintedNames_.count(n) == 0 &&
#ifndef CERULION_MIGRATE_MUTANT_DROP_AST_NAME_SCAN
             referencedNames_.count(n) == 0 &&
#endif
#ifndef CERULION_MIGRATE_MUTANT_DROP_MACRO_NAME_SCAN
             g_macroNames.count(n) == 0 &&
#endif
             !identifierAppears(bodyText_, n);
    };
    if (nameFree("loaned")) {
      return "loaned";
    }
    for (int i = 2; i <= 9; ++i) {
      std::string n = "loaned" + std::to_string(i);
      if (nameFree(n)) {
        return n;
      }
    }
    return std::string();
  }
};

// ---------------------------------------------------------------------------
// AST traversal: find function definitions written under --src-root.
// ---------------------------------------------------------------------------

class MigrateVisitor : public RecursiveASTVisitor<MigrateVisitor> {
 public:
  MigrateVisitor(ASTContext &ctx, const std::string &srcRootReal,
                 Results &results)
      : ctx_(ctx), srcRootReal_(srcRootReal), results_(results) {}

  bool VisitFunctionDecl(FunctionDecl *fn) {
    if (!fn->doesThisDeclarationHaveABody() || !fn->hasBody()) {
      return true;
    }
    if (fn->isDependentContext() || fn->isTemplated()) {
      return true;  // dependent bodies cannot resolve the publisher type
    }
    SourceManager &sm = ctx_.getSourceManager();
    SourceLocation loc = sm.getSpellingLoc(fn->getBody()->getBeginLoc());
    std::string file = realPathOf(sm.getFilename(loc));
    if (!pathUnder(file, srcRootReal_)) {
      return true;
    }
    FunctionAnalyzer(ctx_, fn, srcRootReal_, results_).run();
    return true;
  }

 private:
  ASTContext &ctx_;
  const std::string &srcRootReal_;
  Results &results_;
};

Results g_results;
std::string g_srcRootReal;
std::string g_mainFile;

class MigrateConsumer : public ASTConsumer {
 public:
  void HandleTranslationUnit(ASTContext &ctx) override {
    MigrateVisitor v(ctx, g_srcRootReal, g_results);
    v.TraverseDecl(ctx.getTranslationUnitDecl());
  }
};

// The PPCallbacks observer feeding g_macroNames. Registered in
// CreateASTConsumer, which BeginSourceFile runs BEFORE Execute enters the
// main file — and the predefines buffer (builtins, -D macros) is LEXED at
// EnterMainSourceFile, so a callback registered here reports them too.
// A macro-scan-order concern was refuted by measurement: a probe tool with
// this exact registration point reported `-Dloaned=42`, `-DFROM_D=7`,
// `__STDC__`, `__cplusplus` and an in-TU #define alike (537 names on a
// one-line TU); the matrix pins it with safe_macro_from_flag.cpp, whose
// colliding macro arrives from the compile command.
class MacroNameRecorder : public PPCallbacks {
 public:
  void MacroDefined(const Token &macroNameTok,
                    const MacroDirective *) override {
    if (const IdentifierInfo *ii = macroNameTok.getIdentifierInfo()) {
      g_macroNames.insert(ii->getName().str());
    }
  }
};

class MigrateAction : public ASTFrontendAction {
 public:
  std::unique_ptr<ASTConsumer> CreateASTConsumer(CompilerInstance &ci,
                                                 llvm::StringRef file) override {
    g_mainFile = realPathOf(file);
    g_macroNames.clear();
    ci.getPreprocessor().addPPCallbacks(std::make_unique<MacroNameRecorder>());
    return std::make_unique<MigrateConsumer>();
  }
};

// ---------------------------------------------------------------------------
// JSON emission (fixed key order; sorted lists — deterministic by
// construction).
// ---------------------------------------------------------------------------

void emitJson(llvm::raw_ostream &os, const Results &r) {
  auto rewrites = r.rewrites;
  auto candidates = r.candidates;
  std::stable_sort(rewrites.begin(), rewrites.end(),
                   [](const RewriteResult &a, const RewriteResult &b) {
                     unsigned ao = a.edits.empty() ? 0 : a.edits[0].offset;
                     unsigned bo = b.edits.empty() ? 0 : b.edits[0].offset;
                     return std::tie(a.file, ao) < std::tie(b.file, bo);
                   });
  std::stable_sort(candidates.begin(), candidates.end(),
                   [](const CandidateResult &a, const CandidateResult &b) {
                     return std::tie(a.file, a.line, a.column, a.reason) <
                            std::tie(b.file, b.line, b.column, b.reason);
                   });

  os << "{\n";
  os << "  \"format\": " << kFormatVersion << ",\n";
  os << "  \"tool_version\": \"" << kToolVersion << "\",\n";
  os << "  \"file\": \"" << jsonEscape(g_mainFile) << "\",\n";
  os << "  \"rewrites\": [";
  for (size_t i = 0; i < rewrites.size(); ++i) {
    const RewriteResult &rw = rewrites[i];
    os << (i ? ",\n" : "\n");
    os << "    {\n";
    os << "      \"file\": \"" << jsonEscape(rw.file) << "\",\n";
    os << "      \"function\": \"" << jsonEscape(rw.function) << "\",\n";
    os << "      \"kind\": \"" << jsonEscape(rw.kind) << "\",\n";
    os << "      \"message_type\": \"" << jsonEscape(rw.message_type)
       << "\",\n";
    os << "      \"publisher\": \"" << jsonEscape(rw.publisher) << "\",\n";
    os << "      \"line\": " << rw.line << ",\n";
    os << "      \"edits\": [";
    for (size_t j = 0; j < rw.edits.size(); ++j) {
      const Edit &e = rw.edits[j];
      os << (j ? ",\n" : "\n");
      os << "        {\"offset\": " << e.offset
         << ", \"length\": " << e.length << ", \"original\": \""
         << jsonEscape(e.original) << "\", \"replacement\": \""
         << jsonEscape(e.replacement) << "\"}";
    }
    os << "\n      ]\n";
    os << "    }";
  }
  os << "\n  ],\n";
  os << "  \"candidates\": [";
  for (size_t i = 0; i < candidates.size(); ++i) {
    const CandidateResult &c = candidates[i];
    os << (i ? ",\n" : "\n");
    os << "    {\"file\": \"" << jsonEscape(c.file) << "\", \"function\": \""
       << jsonEscape(c.function) << "\", \"line\": " << c.line
       << ", \"column\": " << c.column
       << ", \"reason\": \"" << jsonEscape(c.reason) << "\", \"detail\": \""
       << jsonEscape(c.detail) << "\"}";
  }
  os << "\n  ]\n";
  os << "}\n";
}

}  // namespace

int main(int argc, const char **argv) {
  auto expectedParser =
      tooling::CommonOptionsParser::create(argc, argv, MigrateCategory);
  if (!expectedParser) {
    llvm::errs() << llvm::toString(expectedParser.takeError()) << "\n";
    return 2;
  }
  tooling::CommonOptionsParser &op = *expectedParser;
  if (op.getSourcePathList().size() != 1) {
    llvm::errs() << "cerulion-ros2-migrate-clang: exactly ONE translation "
                    "unit per invocation\n";
    return 2;
  }
  g_srcRootReal = realPathOf(SrcRoot);

  tooling::ClangTool tool(op.getCompilations(), op.getSourcePathList());
  int rc =
      tool.run(tooling::newFrontendActionFactory<MigrateAction>().get());
  if (rc != 0) {
    llvm::errs() << "cerulion-ros2-migrate-clang: analysis failed (compile "
                    "errors in the translation unit?)\n";
    return 1;
  }
  emitJson(llvm::outs(), g_results);
  return 0;
}
