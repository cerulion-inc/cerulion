#!/usr/bin/env python3
# SPDX-License-Identifier: AGPL-3.0-only
"""Assert the cerulion-ros2-migrate-clang accept/reject matrix.

Usage: assert_matrix.py <analysis-dir>

<analysis-dir> holds one `<fixture>.cpp.json` per fixture translation unit
(produced by run_matrix.sh). Every expectation below is a HAND-WRITTEN
oracle over the fixture sources — never derived from tool output.
"""
import json
import re
import sys
from pathlib import Path

# fixture -> (expected rewrite kinds in source order, expected candidate
#             reasons as a multiset)
EXPECTED = {
    "safe_unique.cpp": (["unique_ptr"], []),
    "safe_shared.cpp": (["shared_ptr"], []),
    "safe_stack.cpp": (["stack"], []),
    "safe_two_sites.cpp": (["unique_ptr", "unique_ptr"], []),
    "unsafe_escape.cpp": ([], ["pointer-escapes"]),
    "unsafe_cross_function.cpp": ([], ["message-built-elsewhere",
                                       "not-a-local"]),
    "unsafe_wrong_publisher.cpp": ([], ["publisher-expr-not-trivial"]),
    # A CUSTOM stateful operator-> between
    # the written publisher and the member — the two generated evaluations
    # (borrow + publish) would land on different publishers. Refused by
    # the exact-publisher gate (the dedicated identity
    # refusal was deleted as provably subsumed);
    # the std smart-pointer ACCEPT controls are safe_shared.cpp /
    # safe_unique.cpp above.
    "unsafe_stateful_arrow.cpp": ([], ["unsupported-publisher-type"]),
    # WRITTEN constructions in the publish argument (braced
    # CXXTemporaryObjectExpr + paren functional cast) stay wrapped — the
    # rewrite would silently delete the explicit copy. Two sites, one per
    # function.
    "unsafe_written_ctor.cpp": ([], ["unsupported-publish-shape",
                                     "unsupported-publish-shape"]),
    # A custom method merely NAMED borrow_loaned_message is not
    # idempotence — reported (message-built-elsewhere), never silently
    # skipped. The genuine-idempotence control is already_loaned.cpp.
    "unsafe_fake_loan.cpp": ([], ["message-built-elsewhere"]),
    # Publisher MUTATED through a call between borrow and publish
    # (std::swap; a non-const-ref helper) — two sites, one per function.
    "unsafe_swap_publisher.cpp": ([], ["publisher-reassigned",
                                       "publisher-reassigned"]),
    # Borrowed from pub_a_, published on pub_b_ — a cross-
    # publisher loan is a DEFECT to report, not idempotence to skip. The
    # single-publisher ACCEPT control is already_loaned.cpp.
    "unsafe_cross_loan.cpp": ([], ["message-built-elsewhere"]),
    # A DERIVED publisher whose own
    # borrow_loaned_message HIDES the base API — same decl chain as the
    # publish, so the same-publisher target proof passes and ONLY the
    # loan-class proof refuses. Isolates DROP_LOAN_PROOF.
    "unsafe_derived_loan.cpp": ([], ["message-built-elsewhere"]),
    # UNSAFE_REWRITE: a derived-publisher OBJECT at the REWRITE
    # site — publish() is inherited so the method-parent gate passes, but
    # the generated borrow would resolve on the DERIVED static type (which
    # can hide the API with a wrong return type). Exact-type refusal.
    "unsafe_derived_publisher.cpp": ([], ["unsupported-publisher-type"]),
    # MACRO_SHADOWING: a macro invocation expands to a use of a
    # member named `loaned` that raw body text cannot show — the site still
    # REWRITES, but the AST name scan must steer the mint to `loaned2`
    # (asserted below beside the two-site check).
    "safe_macro_member_loaned.cpp": (["unique_ptr"], []),
    "safe_macro_loaned_defined.cpp": (["unique_ptr"], []),
    "safe_macro_from_flag.cpp": (["unique_ptr"], []),
    # CONTROL_FLOW: publishes under UNBRACED control bodies
    # (if; for) share the function block with the declaration, so the
    # compound compare alone accepted them — the control-flow walk
    # refuses. Two sites, one per function.
    "unsafe_unbraced_publish.cpp": ([], ["conditional-publish",
                                         "conditional-publish"]),
    # PUBLISHER_ALIASING: reference and pointer aliases to the
    # publisher; writes through the alias evade the per-decl scan, so the
    # alias BINDING refuses. Two sites, one per function.
    "unsafe_aliased_publisher.cpp": ([], ["publisher-reassigned",
                                          "publisher-reassigned"]),
    # The body's closing brace comes from a macro — body text
    # unavailable, minted names cannot be proven collision-free.
    "unsafe_macro_body.cpp": ([], ["macro-expansion"]),
    # Code-review bug: a field access is only a FILL if the
    # glvalue it produces does not ESCAPE. Two escape shapes refuse
    # (address-of, reference binding); the third function is the
    # ANTI-BLANKET control — an ordinary fill, and a COPY of a field, must
    # still rewrite, so a fix that simply refused everything fails here.
    # Without the escape check this file is rewritten THREE times, and the two
    # aliased ones are a use-after-move.
    # The third escape spelling (a field bound to a reference
    # PARAMETER) refuses too, and the control carries the two shapes that must
    # still rewrite: a member call where the field is the OBJECT, and a
    # by-VALUE parameter.
    "unsafe_field_alias.cpp": (["stack"], ["pointer-escapes",
                                           "pointer-escapes",
                                           "pointer-escapes"]),
    # A publish written inside a lambda body must not be
    # collected by nobody — no edit and no candidate, in the most common ROS 2
    # publisher idiom. Both sites (a create_wall_timer callback and an
    # immediately-invoked lambda) must be REPORTED. A prover reporting zero of
    # each is what the matrix's own totality check catches.
    "unsafe_lambda_publish.cpp": ([], ["publish-inside-lambda",
                                       "publish-inside-lambda"]),
    # TWO publish sites spelled inside ONE macro
    # body. Anchored at the SPELLING location they share the `#define`'s
    # (file, line) and the (file, line, reason) dedup collapses them into
    # one, silently dropping a real site from the report and the manifest.
    # Anchored at the EXPANSION location they are two.
    "unsafe_macro_sites.cpp": ([], ["macro-expansion", "macro-expansion"]),
    # geometry_msgs/Twist is PLAIN, so a loaning rmw really does
    # hand back an uninitialized buffer for it — the shape std_msgs/String
    # (every other fixture) can never reach. Partially-written first, then a
    # fully-written control; both rewrite, and both must carry the
    # value-init (the store is unconditional).
    "safe_plain_partial_fill.cpp": (["unique_ptr", "stack"], []),
    # The unsafe-rewrite guard (UNSAFE_REWRITE): the publisher's name is
    # RE-DECLARED between the message declaration and the publish, so the
    # borrow spliced at the declaration resolves it to a DIFFERENT publisher
    # than the publish uses — a silent cross-publisher loan out of
    # well-defined code. One arm per mechanism; without the guard, every one of
    # them is rewritten into a cross-publisher borrow.
    # TEN refusals: seven shadow mechanisms, two type-headed qualifiers and
    # one using-directive. Each shadow mechanism reaches the scan through a
    # different part of the AST, and SIX of the nine lookup mechanisms in
    # this class are ones a narrower scan misses:
    #   local           a plain block-scope variable shadowing the member
    #   using-decl      a using-DECLARATION (why the scan matches NamedDecl,
    #                   not VarDecl)
    #   namespace alias rebinding a QUALIFIED name's head (why "qualified"
    #                   is not by itself an unshadowable answer)
    #   structured bind names that are BindingDecls hanging off an unnamed
    #                   DecompositionDecl, absent from DeclStmt::decls()
    #   label-wrapped   a declaration behind a LabelStmt, which a bare cast
    #                   to DeclStmt walks straight past
    #   chain root      the same defect through a member CHAIN (why the
    #                   proof keys on the chain's ROOT)
    #   qualified member a NAMESPACE-headed qualifier on a member access,
    #                   which resolves TWO names unqualified — the qualifier
    #                   head AND the object root; a walk that returns
    #                   the first and stops misses the second
    # Plus ONE lookup-CHANGED refusal: a using-DIRECTIVE, which declares no
    # name at all, so a scan comparing declared names walks past it while it
    # still changes what the publisher's name means from that point on.
    # Plus TWO non-shadow refusals, both TYPE-headed qualifiers that the
    # classifier declines to resolve (the written spelling can be an alias whose
    # identifier differs from the resolved type's name) — and they are
    # DIFFERENT AST nodes, which is why both are here: a static member of an
    # unrelated class is a DeclRefExpr, while a class-qualified INHERITED
    # member is a MemberExpr over an implicit `this`. The classifier tests
    # the qualifier in one place for both; before that hoist, the MemberExpr
    # spelling walked past its own specifier and was ACCEPTED.
    "unsafe_shadowed_publisher.cpp": ([], ["publisher-expr-not-trivial",
                                           "publisher-expr-not-trivial",
                                           "publisher-lookup-changed",
                                           "publisher-shadowed",
                                           "publisher-shadowed",
                                           "publisher-shadowed",
                                           "publisher-shadowed",
                                           "publisher-shadowed",
                                           "publisher-shadowed",
                                           "publisher-shadowed"]),
    # The ANTI-BLANKET control for the refusal above — the same
    # name declared where it shadows the publisher at NEITHER edit (after
    # the publish; in a block that closes before it). Both must still
    # rewrite, so a fix that refused on the NAME rather than on its
    # position fails here.
    # SIX accept arms: the two window boundaries (a shadow after the
    # publish; a shadow in a block that closes before it), the two the
    # natural OVER-fixes would break — a shadow declared BEFORE the message
    # (a scan widened to start at the top of the block refuses it) and a
    # local spelled like a member chain's TERMINAL name (a scan widened to
    # every chain name refuses it) — and the two accept twins of the
    # mechanisms added most recently: a using-directive written BEFORE the
    # declaration (in effect at both edits, so it changes nothing) and a
    # namespace-qualified member whose object root is NOT shadowed.
    "safe_same_name_other_scope.cpp": (["unique_ptr", "unique_ptr",
                                        "unique_ptr", "unique_ptr",
                                        "unique_ptr", "unique_ptr"], []),
    # A template argument in the publisher expression.
    # Its names are resolved by ordinary UNQUALIFIED lookup at the publish
    # site — not by the member lookup that makes a chain's tail immune, and
    # not by the qualified lookup that makes a qualified-id's tail immune
    # (`ns::Holder<Tag>` does not find `ns::Tag`) — so a block-scope alias
    # written between the two edits changes which publisher the same written
    # text names. MEASURED: the migrated form compiles clean and borrows
    # from one specialization while publishing on another.
    # TWO refusals, one per spelling, because they reach the gap through
    # different parts of the analysis: explicit arguments on the
    # ID-EXPRESSION (a variable template, whose DeclRefExpr's written range
    # spans the argument list) and an argument inside a NAMESPACE-headed
    # qualifier's TAIL (which the classifier walks past on its way to the
    # outermost specifier). Both are refused on the TEXT rather than on
    # either AST shape, so the refusal is total over the class.
    "unsafe_template_argument_publisher.cpp": ([],
                                               ["publisher-expr-not-trivial",
                                                "publisher-expr-not-trivial"]),
    # The two verdicts a QUALIFIED publisher can reach, each in
    # the shadow position that refuses when written unqualified — a
    # `::`-rooted head (unshadowable: the only arm in the matrix that skips
    # the scan and rewrites) and a namespace head that is simply not
    # re-declared (scanned like any other name).
    "safe_qualified_publisher.cpp": (["unique_ptr", "unique_ptr"], []),
    "unsafe_reuse_after_move.cpp": ([], ["reuse-after-move",
                                         "use-after-publish"]),
    "unsafe_retained_member.cpp": ([], ["retained-member"]),
    "unsafe_ctor_args.cpp": ([], ["constructor-args"]),
    "unsafe_conditional.cpp": ([], ["conditional-publish"]),
    # Two arms each: the body-use / terminal-field originals plus the
    # capture-LIST and holder-CHAIN arms.
    "unsafe_lambda.cpp": ([], ["captured-by-lambda", "captured-by-lambda"]),
    "unsafe_reassigned.cpp": ([], ["publisher-reassigned",
                                   "publisher-reassigned"]),
    "already_loaned.cpp": ([], []),
}


def main() -> int:
    if len(sys.argv) != 2:
        print("usage: assert_matrix.py <analysis-dir>", file=sys.stderr)
        return 2
    outdir = Path(sys.argv[1])
    failures = []
    seen = set()
    for path in sorted(outdir.glob("*.json")):
        fixture = path.name[: -len(".json")]
        if fixture not in EXPECTED:
            failures.append(f"{fixture}: no expectation declared — declare "
                            "it in assert_matrix.py")
            continue
        seen.add(fixture)
        doc = json.loads(path.read_text())
        # `type(...) is int`, not `== 1`: Python's bool subclasses int, so
        # `True == 1` — a malformed document with `"format": true` must
        # fail the version gate, not slide through it.
        if type(doc.get("format")) is not int or doc["format"] != 1:
            failures.append(f"{fixture}: format {doc.get('format')!r} != 1")
            continue
        want_kinds, want_reasons = EXPECTED[fixture]
        got_kinds = [r["kind"] for r in doc.get("rewrites", [])]
        got_reasons = sorted(c["reason"] for c in doc.get("candidates", []))
        if got_kinds != want_kinds:
            failures.append(
                f"{fixture}: rewrite kinds {got_kinds} != {want_kinds}")
        if got_reasons != sorted(want_reasons):
            failures.append(
                f"{fixture}: candidate reasons {got_reasons} != "
                f"{sorted(want_reasons)}")
        # Structural pins on the accepted shapes.
        for rw in doc.get("rewrites", []):
            edits = rw.get("edits", [])
            if len(edits) != 2:
                failures.append(f"{fixture}: rewrite has {len(edits)} edits "
                                "(want 2: decl + publish)")
                continue
            decl, call = edits
            if "borrow_loaned_message()" not in decl["replacement"]:
                failures.append(f"{fixture}: decl replacement lacks "
                                "borrow_loaned_message()")
            if "publish(std::move(" not in call["replacement"]:
                failures.append(f"{fixture}: publish replacement lacks "
                                "publish(std::move(...))")
            # VERIFIED against rclcpp/rmw_fastrtps/Fast DDS: a loan
            # is NOT initialized on every rmw, while every construction this
            # tool accepts value-initializes — so the decl replacement must
            # value-initialize the loaned message or a field the site does not
            # write silently becomes recycled pool memory on the wire. Checked
            # for EVERY rewrite, not one fixture: the store is unconditional,
            # so deleting it must fail loudly everywhere.
            msg_type = rw.get("message_type", "")
            want_init = f".get() = {msg_type}();"
            if msg_type and want_init not in decl["replacement"]:
                failures.append(
                    f"{fixture}: decl replacement does not value-initialize "
                    f"the loaned message (want {want_init!r}); an unwritten "
                    "field would publish uninitialized loan memory on a "
                    f"loaning rmw: {decl['replacement']!r}")
            # A class found on a real build: the publisher text once carried the
            # smart-pointer arrow, so re-adding the operator doubled it —
            # bytes that pass substring oracles but do not BUILD. Every
            # fixture publisher is an arrow-form `pub_…`, so each
            # replacement must carry EXACTLY one arrow per spliced
            # publisher occurrence and never a doubled operator.
            for which, e in (("decl", decl), ("publish", call)):
                repl = e["replacement"]
                if "->->" in repl or ".." in repl.replace("...", ""):
                    failures.append(f"{fixture}: doubled operator in {which} "
                                    f"replacement: {repl!r}")
                if repl.count("->") != 1:
                    failures.append(f"{fixture}: {which} replacement must "
                                    f"carry exactly ONE '->' (got "
                                    f"{repl.count('->')}): {repl!r}")
        # Code-review bug: the macro fixture's TWO sites must
        # be anchored at their EXPANSION locations, not at the `#define`
        # they share. This needs its own check because the (file, line,
        # reason) DEDUP that collapsed them lives on the Rust side, over
        # the MERGED analysis — this file reads the prover's raw per-TU
        # JSON, where both candidates are present either way. What the
        # anchoring changes, and what is asserted here, is their LINES:
        # spelling-anchored they are one and the same line (and it is the
        # macro's, in a header the operator may not own); expansion-anchored
        # they are the two call sites actually written.
        # Six of the refusing fixture's eight candidates carry the
        # SAME reason, and it produces no rewrites, so stage 3b compiles
        # nothing for it — a prover that anchored two of them at one location
        # would satisfy both the count and the multiset. Distinct lines are
        # what separates "N sites refused" from "one site reported N times",
        # the same hole unsafe_macro_sites.cpp closes for the macro class.
        if fixture == "unsafe_shadowed_publisher.cpp":
            lines = sorted(c.get("line", 0) for c in doc.get("candidates", []))
            if len(set(lines)) != len(lines):
                failures.append(
                    f"{fixture}: refusals share a line {lines} — one site "
                    "reported more than once is indistinguishable from "
                    "several sites refused, under a reason multiset")
        # The same question for the accept side: N rewrites must be N SITES,
        # so their decl edits must start at N distinct offsets. Applied to
        # BOTH accept fixtures — the qualified one has two rewrites of the
        # same kind, which is the shape an ordered kind list cannot separate.
        if fixture in ("safe_same_name_other_scope.cpp",
                       "safe_qualified_publisher.cpp"):
            offsets = sorted(r["edits"][0]["offset"]
                             for r in doc.get("rewrites", [])
                             if len(r.get("edits", [])) == 2)
            if len(set(offsets)) != len(offsets):
                failures.append(
                    f"{fixture}: two rewrites share a decl-edit offset "
                    f"{offsets} — they are not distinct sites")
        if fixture == "unsafe_macro_sites.cpp":
            lines = sorted(c.get("line", 0) for c in doc.get("candidates", []))
            if len(set(lines)) != len(lines):
                failures.append(
                    f"{fixture}: the two macro-expanded sites share a line "
                    f"{lines} — candidates are anchored at the macro's "
                    "SPELLING location, so the Rust dedup collapses them "
                    "into one and a real site vanishes from the report")
            # The fixture writes its two invocations adjacent in `tick()`,
            # far below the `#define`; spelling-anchored BOTH collapse onto
            # the macro body's line, which the distinct-line check above is
            # what catches.

        # The two-site fixture mints `loaned` THEN `loaned2`,
        # deterministically and in source order — asserted PER SITE (a
        # concatenated substring search for `loaned2` alone passed
        # an output naming the sites loaned2/loaned3).
        # The macro-shadow fixture's mint must AVOID the member
        # name a macro expansion references — `loaned2`, never `loaned`
        # (only the AST name scan can see the collision; raw body text
        # shows just the macro invocation).
        if fixture == "safe_macro_member_loaned.cpp":
            repl = "".join(e.get("replacement", "")
                           for r in doc.get("rewrites", [])
                           for e in r.get("edits", []))
            if re.search(r"\bloaned\b", repl):
                failures.append("safe_macro_member_loaned.cpp: minted the "
                                "member-colliding name `loaned`: "
                                f"{repl!r}")
            if not re.search(r"\bloaned2\b", repl):
                failures.append("safe_macro_member_loaned.cpp: must mint "
                                f"`loaned2`: {repl!r}")
        # An object-like `#define loaned` ACTIVE in the TU is a
        # collision none of the name scans can see (the token is absent
        # from the function); only the preprocessor macro scan steers the
        # mint to `loaned2`.
        # The same constraint applies when the macro arrives from the
        # COMPILE COMMAND (-D, the predefines buffer) instead of a source
        # line — safe_macro_from_flag.cpp.
        if fixture in ("safe_macro_loaned_defined.cpp",
                       "safe_macro_from_flag.cpp"):
            repl = "".join(e.get("replacement", "")
                           for r in doc.get("rewrites", [])
                           for e in r.get("edits", []))
            if re.search(r"\bloaned\b", repl):
                failures.append(f"{fixture}: minted the macro-colliding "
                                f"name `loaned`: {repl!r}")
            if not re.search(r"\bloaned2\b", repl):
                failures.append(f"{fixture}: must mint `loaned2`: {repl!r}")
        if fixture == "safe_two_sites.cpp":
            per_site = ["".join(e.get("replacement", "")
                                for e in r.get("edits", []))
                        for r in doc.get("rewrites", [])]
            for idx, (site, name) in enumerate(zip(per_site,
                                                   ["loaned", "loaned2"])):
                if not re.search(rf"\b{name}\b", site):
                    failures.append(f"safe_two_sites.cpp: site {idx} must "
                                    f"mint `{name}`: {site!r}")
                for other in ("loaned", "loaned2", "loaned3"):
                    if other != name and re.search(rf"\b{other}\b", site):
                        failures.append(f"safe_two_sites.cpp: site {idx} "
                                        f"minted `{other}` where `{name}` "
                                        f"is the contract: {site!r}")
    missing = set(EXPECTED) - seen
    for fixture in sorted(missing):
        failures.append(f"{fixture}: no analysis output found")
    if failures:
        for f in failures:
            print(f"MATRIX FAIL: {f}")
        return 1
    print(f"matrix OK ({len(seen)} fixtures)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
