// SPDX-License-Identifier: AGPL-3.0-only
//! Every read-outcome MINT names its read-site role as a
//! compile-time constant, and the set of mint sites is a DECLARED inventory.
//!
//! # What this guards, and what it deliberately cannot
//!
//! The role's whole contract is that the CALL SITE declares it (see
//! [`cerulion_core::read_outcome::ReadSiteRole`]) — a site that inferred one
//! from state would be the silent inversion the declared constant exists to
//! prevent. Behaviourally that is pinned per site by the `role_view` arms in
//! `read_outcome_capture_iox2_test.rs`, which read the role back off a real
//! run; but a NEW mint arm added tomorrow is covered by no arm at all, and the
//! failure mode is silence (a record whose site nothing on the wire names, on a
//! bag whose format PROMISES the bits are readable).
//!
//! So this walk is a STRUCTURAL floor, not a correctness proof, and the
//! distinction is worth stating plainly: it can prove a mint NAMES a role and
//! that the number of mints has not changed unnoticed. **It cannot prove the
//! role named is the RIGHT one** — a site that stamps `Body` where the
//! scheduler is really draining passes this walk and is caught only by a
//! capture arm that reads the record back.
//!
//! # The rule
//!
//! Over a COMMENT-STRIPPED view (`code_only`, cribbed from
//! `cdylib_iox2_log_level_test.rs` — the module docs of the walked files name
//! these functions in prose to explain them, so a raw-text walk would count
//! sentences as call sites), every call to
//!
//! - `stage_read_outcome(` / `stage_read_outcome_with_producer(` — the
//!   subscriber's two mint helpers,
//! - `.record(` / `.record_with_producer(` — the stage's own entry points, and
//! - `.drain_samples(` — the ONE intermediate that carries a
//!   `role: ReadSiteRole` parameter of its own, so its callers are where the
//!   batch path's constant is really named,
//!
//! must carry a `ReadSiteRole::` LITERAL as the FINAL argument of its
//! paren-matched argument list. The FINAL argument specifically, because the
//! role is the last parameter of every mint entry point: scanning the whole
//! list for the substring is satisfied by a literal sitting in an EARLIER
//! argument while the role itself is computed, i.e. by exactly the shape this
//! walk exists to fail. The only sanctioned exception is a FORWARDING call
//! whose last argument is the bare `role` parameter (the two helper bodies,
//! which pass their caller's constant through); those are counted separately,
//! so turning a literal site into a forwarding one moves a number rather than
//! passing silently.
//!
//! Both counts are checked against declared inventories, so a new mint arm
//! fails LOUDLY until somebody classifies it — which is the point: the failure
//! message is where a contributor is told to add a `role_view` arm for it.

use std::path::{Path, PathBuf};

/// The block-comment markers, spelled with an escaped `*` for the reason
/// `cdylib_iox2_log_level_test.rs` documents: a raw opener in THIS file's
/// source would open a block comment in any walk that ever reads this file.
const BLOCK_OPEN: &str = concat!("/", "*");
const BLOCK_CLOSE: &str = concat!("*", "/");

/// The files that MINT read outcomes. Not a hand list of SITES — a hand list of
/// FILES, which is the coarser thing a walk can keep accurate.
const WALKED: [&str; 3] = [
    "src/transport/subscriber.rs",
    "src/graph/runtime.rs",
    "src/scheduler/mod.rs",
];

/// Calls that must name a `ReadSiteRole::` literal.
///
/// 29 in `subscriber.rs` (22 mint arms + the 7 `drain_samples` call sites,
/// which are where the batch path's constant is named), 1 in `runtime.rs` (the
/// `stage_read_outcomes_for_test` seam) and 2 in `scheduler/mod.rs` (its
/// in-module merge tests). Re-count with the failure message, never by editing
/// this number to match.
///
/// It went 30 -> 32 with the format-5 peek/head mark: the per-set
/// Sync matcher's PROMOTION (`sync_discard_head`) now stages a record of its
/// own — a `Drain`-role `DrainedBatch` at the promoted sequence with
/// `popped: 0` — where before the hand-off was silent, and it does so on both
/// the plain and the producer-annotated arm. Its capture arms are
/// `read_outcome_capture_iox2_test`'s
/// `the_sync_matchers_descent_pops_are_drain_site_reads` (which now
/// asserts the promotion row on BOTH legs) and
/// `a_non_improving_peek_is_marked_a_peek_not_the_head`.
const DECLARED_LITERAL_MINTS: usize = 32;

/// Calls that FORWARD a `role` parameter rather than naming one — all four in
/// `subscriber.rs`: the two `stage_read_outcome*` helper bodies, and the two
/// `DrainedBatch` mints inside `drain_samples`, which pass that function's own
/// `role` parameter through to them. Every one of those four is reached only
/// from a site this walk ALSO checks, so the chain bottoms out in a literal.
const DECLARED_FORWARDING_MINTS: usize = 4;

fn crate_root() -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR"))
}

/// Strip Rust comments, keeping the line structure so a probe can never match
/// across a line boundary it did not span. Block comments NEST; an unterminated
/// one fails CLOSED (stripped to EOF), so a mint sitting after it reads as
/// absent and its file fails this guard loudly rather than passing on text the
/// compiler would never see.
fn code_only(src: &str) -> String {
    let mut out = String::with_capacity(src.len());
    let mut rest = src;
    let mut depth = 0usize;
    while let Some(ch) = rest.chars().next() {
        if depth == 0 {
            if rest.starts_with("//") {
                match rest.find('\n') {
                    Some(nl) => rest = &rest[nl..],
                    None => break,
                }
            } else if rest.starts_with(BLOCK_OPEN) {
                depth = 1;
                rest = &rest[BLOCK_OPEN.len()..];
            } else {
                out.push(ch);
                rest = &rest[ch.len_utf8()..];
            }
        } else if rest.starts_with(BLOCK_OPEN) {
            depth += 1;
            rest = &rest[BLOCK_OPEN.len()..];
        } else if rest.starts_with(BLOCK_CLOSE) {
            depth -= 1;
            rest = &rest[BLOCK_CLOSE.len()..];
        } else {
            if ch == '\n' {
                out.push(ch);
            }
            rest = &rest[ch.len_utf8()..];
        }
    }
    out
}

/// 1-based line number of `offset` in `code`.
fn line_of(code: &str, offset: usize) -> usize {
    code[..offset].matches('\n').count() + 1
}

/// The paren-matched argument list starting at `open` (the byte index of the
/// `(`), or `None` if the parens never close. Deliberately unaware of string
/// literals: a mint's argument list holds identifiers and paths, and a walk
/// that guessed at quoting would be a second thing to get wrong.
fn arg_list(code: &str, open: usize) -> Option<&str> {
    let bytes = code.as_bytes();
    debug_assert_eq!(bytes[open], b'(');
    let mut depth = 0usize;
    for (i, b) in bytes.iter().enumerate().skip(open) {
        match b {
            b'(' => depth += 1,
            b')' => {
                depth -= 1;
                if depth == 0 {
                    return Some(&code[open + 1..i]);
                }
            }
            _ => {}
        }
    }
    None
}

/// Byte offsets of the `(` of every CALL to `name` in `code`.
///
/// `require_dot` demands the name be reached through a `.` — the stage's
/// methods, whose bare spellings (`record`) are ordinary words a free function
/// could carry. The trailing `(` in the needle already separates `record` from
/// `record_with_producer`; the character BEFORE separates a name from the tail
/// of a longer identifier. A `fn` DEFINITION is skipped, so a helper's own
/// signature is not read as a call to itself.
fn call_sites(code: &str, name: &str, require_dot: bool) -> Vec<usize> {
    let needle = format!("{name}(");
    let mut out = Vec::new();
    let mut from = 0usize;
    while let Some(rel) = code[from..].find(&needle) {
        let start = from + rel;
        from = start + needle.len();
        let prev = code[..start].chars().next_back();
        if require_dot && prev != Some('.') {
            continue;
        }
        if let Some(c) = prev {
            if c.is_alphanumeric() || c == '_' {
                continue;
            }
        }
        // Skip the definition: `fn <name>(`.
        if code[..start].trim_end().ends_with("fn") {
            continue;
        }
        out.push(start + name.len());
    }
    out
}

/// A call's classification.
enum Mint {
    /// Names a `ReadSiteRole::` variant inline.
    Literal,
    /// Forwards a `role` parameter (the two helper bodies).
    Forwarding,
}

/// The call's arguments, split at TOP-LEVEL commas.
///
/// A comma inside nested `()` / `[]` / `{}` belongs to a nested expression, not
/// to this call's argument list — `match r { A => x, B => y }` passed as ONE
/// argument must not read as two. A trailing empty segment (the trailing comma
/// a rustfmt'd multi-line call always carries) is dropped.
///
/// Deliberately unaware of string and char literals, for the reason
/// [`arg_list`] gives: a mint's argument list holds identifiers, paths and
/// numeric literals, and a walk that guessed at quoting would be a second thing
/// to get wrong.
fn top_level_args(args: &str) -> Vec<&str> {
    let mut out = Vec::new();
    let mut depth = 0usize;
    let mut start = 0usize;
    for (i, c) in args.char_indices() {
        match c {
            '(' | '[' | '{' => depth += 1,
            ')' | ']' | '}' => depth = depth.saturating_sub(1),
            ',' if depth == 0 => {
                out.push(args[start..i].trim());
                start = i + c.len_utf8();
            }
            _ => {}
        }
    }
    let tail = args[start..].trim();
    if !tail.is_empty() {
        out.push(tail);
    }
    out
}

/// Is `arg` — a WHOLE argument — a `ReadSiteRole` variant PATH?
///
/// The path may be qualified (`crate::read_outcome::ReadSiteRole::Body`) but it
/// must be the entire argument: a `ReadSiteRole::` reached through an enclosing
/// expression (`match tier { .. => ReadSiteRole::Drain, .. }`,
/// `ReadSiteRole::from(stage_role)`) is a role DERIVED from state, which is
/// precisely what the declared-constant contract forbids and what this walk
/// must refuse to classify.
fn is_role_literal(arg: &str) -> bool {
    let Some((qualifier, variant)) = arg.split_once("ReadSiteRole::") else {
        return false;
    };
    // Anything before the path must be a module qualifier (`a::b::`), never an
    // enclosing expression.
    let qualifier_ok = qualifier.is_empty()
        || (qualifier.ends_with("::")
            && qualifier
                .chars()
                .all(|c| c.is_alphanumeric() || c == '_' || c == ':'));
    // …and the variant must be the WHOLE tail: a bare identifier, nothing else.
    // `from(stage_role)` fails on the `(`.
    let variant_ok =
        !variant.is_empty() && variant.chars().all(|c| c.is_alphanumeric() || c == '_');
    qualifier_ok && variant_ok
}

/// Is `arg` — a WHOLE argument — a read-outcome KIND expression? Bare `kind`
/// (the forwarding helper bodies) or a `ReadOutcomeKind::` variant path, with
/// the same whole-argument discipline as [`is_role_literal`].
///
/// This is the walk's RECEIVER discriminator for the generic dotted names:
/// `.record(` and `.record_with_producer(` are names other types also use —
/// `NodeDeathLedger::record(node_id, cause)` (a node-death ledger producer)
/// is exactly such a collision, and a textual walk cannot type-resolve
/// the receiver. Every READ-OUTCOME record's FIRST parameter is its kind, so
/// the first argument is what separates a mint from a stranger wearing the
/// same name. Residual (the same limit as a renamed wrapper): a future
/// mint passing a COMPUTED kind through a generic name would leave the walk
/// silently — the `DECLARED_*_MINTS` count pins are what would notice the
/// inventory shrinking.
fn is_kind_expr(arg: &str) -> bool {
    if arg == "kind" {
        return true;
    }
    let Some((qualifier, variant)) = arg.split_once("ReadOutcomeKind::") else {
        return false;
    };
    let qualifier_ok = qualifier.is_empty()
        || (qualifier.ends_with("::")
            && qualifier
                .chars()
                .all(|c| c.is_alphanumeric() || c == '_' || c == ':'));
    let variant_ok =
        !variant.is_empty() && variant.chars().all(|c| c.is_alphanumeric() || c == '_');
    qualifier_ok && variant_ok
}

fn classify(args: &str) -> Option<Mint> {
    // The role is the LAST parameter of every mint entry point, so
    // classification reads THAT argument and nothing else. Scanning the whole
    // list for a `ReadSiteRole::` substring — which is what this did before —
    // is satisfied by a literal sitting in ANY earlier argument while the role
    // itself is computed, i.e. it passes exactly the shape the walk exists to
    // fail.
    let last = *top_level_args(args).last()?;
    if is_role_literal(last) {
        return Some(Mint::Literal);
    }
    (last == "role").then_some(Mint::Forwarding)
}

/// `classify` reads the FINAL argument, so a
/// `ReadSiteRole::` literal sitting in an EARLIER one cannot vouch for a
/// computed role.
///
/// Hand vectors, never a self-compare. The refused shapes are the ones a real
/// regression takes: a role picked by a `match`, a role built by a `From`
/// conversion, and a role that is a literal in the wrong argument slot.
#[test]
fn classify_reads_the_final_argument_not_any_literal_in_the_list() {
    let literal = |args: &str| matches!(classify(args), Some(Mint::Literal));
    let forwarding = |args: &str| matches!(classify(args), Some(Mint::Forwarding));
    let refused = |args: &str| classify(args).is_none();

    // ACCEPTED: the two sanctioned shapes, bare and module-qualified, on one
    // line and rustfmt'd across several with a trailing comma.
    assert!(literal("kind, Some(seq), 1, ReadSiteRole::Peek"));
    assert!(literal(
        "kind, Some(seq), 1, crate::read_outcome::ReadSiteRole::Body"
    ));
    assert!(literal(
        "\n    ReadOutcomeKind::DrainedBatch,\n    Some(4),\n    3,\n    ReadSiteRole::Drain,\n"
    ));
    assert!(forwarding("kind, served_seq, popped, role"));
    assert!(forwarding(
        "kind,\n    served_seq,\n    popped,\n    role,\n"
    ));

    // THE HEADLINE: an earlier argument names a role, the FINAL one is
    // computed. A bare `args.contains(..)` would classify every one of these
    // as `Literal`.
    assert!(refused(
        "kind, fallback(ReadSiteRole::Body), 1, self.derive_role()"
    ));
    assert!(refused(
        "ReadSiteRole::Peek, Some(seq), 1, if drained { a } else { b }"
    ));

    // A computed FINAL argument, in the two shapes a regression really takes —
    // and note the `match` arms' commas, which is why the split must be
    // top-level (a naive `rsplit(',')` would read `ReadSiteRole::Body }` here).
    assert!(refused(
        "kind, Some(seq), 1, match tier { A => ReadSiteRole::Drain, B => ReadSiteRole::Body }"
    ));
    assert!(refused(
        "kind, Some(seq), 1, ReadSiteRole::from(stage_role)"
    ));

    // A near-miss identifier is not the forwarding parameter.
    assert!(refused("kind, Some(seq), 1, role_hint"));
    assert!(refused("kind, Some(seq), 1, self.role"));

    // …and an argument list the walk cannot read at all is refused, not
    // guessed at.
    assert!(refused(""));
}

/// The generic-name RECEIVER discriminator: a `.record(` whose first argument
/// is not a read-outcome kind is another type's method (a real shape:
/// `NodeDeathLedger::record(node_id, cause)`, which makes a naive walk
/// flag two death-ledger calls as roleless mints), and the walk
/// must SKIP it rather than demand a role from it.
#[test]
fn a_foreign_record_call_is_out_of_scope_not_an_unclassified_mint() {
    // The two sanctioned first-argument shapes of a REAL mint.
    assert!(is_kind_expr("kind"));
    assert!(is_kind_expr("ReadOutcomeKind::DrainedBatch"));
    assert!(is_kind_expr(
        "crate::read_outcome::ReadOutcomeKind::Producer"
    ));

    // The real collision, plus the near-miss shapes: a computed kind
    // is NOT admitted (it would be the same derived-from-state inversion the
    // role rule refuses — such a mint leaves the walk and the count pins are
    // what notice).
    assert!(!is_kind_expr("node_id"));
    assert!(!is_kind_expr("id"));
    assert!(!is_kind_expr("self.kind_for(slot)"));
    assert!(!is_kind_expr("ReadOutcomeKind::from(raw)"));
    assert!(!is_kind_expr(""));
}

#[test]
fn every_read_outcome_mint_names_its_read_site_role() {
    let root = crate_root();
    let mut literal = Vec::new();
    let mut forwarding = Vec::new();
    let mut unclassified = Vec::new();

    for rel in WALKED {
        let path: &Path = &root.join(rel);
        let src = std::fs::read_to_string(path)
            .unwrap_or_else(|e| panic!("read-site role walk: cannot read {}: {e}", path.display()));
        let code = code_only(&src);

        let calls = [
            ("stage_read_outcome", false),
            ("stage_read_outcome_with_producer", false),
            ("record", true),
            ("record_with_producer", true),
            ("drain_samples", true),
        ];
        for (name, require_dot) in calls {
            for open in call_sites(&code, name, require_dot) {
                let Some(args) = arg_list(&code, open) else {
                    panic!(
                        "read-site role walk: unbalanced parentheses at {}:{} — the walk cannot read this \
                         mint's argument list, so it cannot vouch for its role",
                        rel,
                        line_of(&code, open)
                    );
                };
                // The generic dotted names collide with other types' methods
                // (see `is_kind_expr`): a call whose FIRST argument is not a
                // read-outcome kind is a different method wearing the same
                // name, out of this walk's scope — not an unclassified mint.
                if matches!(name, "record" | "record_with_producer")
                    && !top_level_args(args)
                        .first()
                        .copied()
                        .is_some_and(is_kind_expr)
                {
                    continue;
                }
                let site = format!("{rel}:{}", line_of(&code, open));
                match classify(args) {
                    Some(Mint::Literal) => literal.push(site),
                    Some(Mint::Forwarding) => forwarding.push(site),
                    None => unclassified.push(site),
                }
            }
        }
    }

    assert!(
        unclassified.is_empty(),
        "read-site role walk: these read-outcome mints name NO `ReadSiteRole::` literal and do not forward \
         a `role` parameter: {unclassified:?}\n\n  The role is the CALL SITE's, declared as a \
         compile-time constant — a site that derives one from state is the silent inversion the \
         constant exists to prevent. Name the role at the site, and add a `role_view` arm in \
         `read_outcome_capture_iox2_test.rs` that reads it back off a real run (this walk can \
         only prove a role was NAMED, never that it is the right one)."
    );

    assert_eq!(
        literal.len(),
        DECLARED_LITERAL_MINTS,
        "read-site role walk: the read-outcome mint inventory changed — {} literal sites, {DECLARED_LITERAL_MINTS} \
         declared.\n  sites: {literal:?}\n\n  A NEW mint arm is covered by no capture arm until \
         somebody writes one, so update DECLARED_LITERAL_MINTS in this file AND add the arm.",
        literal.len()
    );
    assert_eq!(
        forwarding.len(),
        DECLARED_FORWARDING_MINTS,
        "read-site role walk: the forwarding-mint count changed — {} sites, {DECLARED_FORWARDING_MINTS} \
         declared.\n  sites: {forwarding:?}\n\n  A forwarding call passes its caller's constant \
         through; turning a literal site into one hides the constant from this walk.",
        forwarding.len()
    );
}

/// ANTI-TAUTOLOGY: a broken stripper would make every assertion above vacuous
/// (nothing found, nothing unclassified, and the counts merely wrong) — so pin
/// that the stripped view still holds the code the walk is about.
#[test]
fn the_stripped_view_still_contains_the_code_the_walk_reads() {
    let src = std::fs::read_to_string(crate_root().join("src/transport/subscriber.rs"))
        .expect("read subscriber.rs");
    let code = code_only(&src);
    assert!(
        code.contains("fn stage_read_outcome("),
        "the stripped view lost the mint helper's own definition — every absence assertion in \
         this file would be vacuous"
    );
    assert!(
        code.contains("ReadSiteRole::Drain"),
        "the stripped view lost the role literals the walk classifies on"
    );
}

#[test]
fn the_walk_helpers_answer_their_hand_written_vectors() {
    let (open, close, line) = (BLOCK_OPEN, BLOCK_CLOSE, "//");

    // `code_only`: line, block, nesting, and fail-closed.
    assert_eq!(code_only(&format!("a {line} b\nc")), "a \nc");
    assert_eq!(code_only(&format!("a {open} b {close} c")), "a  c");
    assert_eq!(
        code_only(&format!("a {open} {open} b {close} c {close} d")),
        "a  d"
    );
    assert_eq!(
        code_only(&format!("a {open} b\nstage_read_outcome(")),
        "a \n"
    );

    // `call_sites`: a definition is not a call; a longer identifier is not a
    // match; the dotted/undotted split holds.
    let src = "fn stage_read_outcome(a) {}\nself.stage_read_outcome(X);\nx.record(Y);\n\
               foo_record(Z);\nother.record_with_producer(W);\nstage_read_outcome_x(Q);";
    // The DEFINITION is not a call; the dotted call is; a longer identifier
    // ending in the name is not.
    assert_eq!(call_sites(src, "stage_read_outcome", false).len(), 1);
    // `foo_record(` is rejected by the dot requirement AND by the preceding
    // identifier character, so it is counted under neither spelling.
    assert_eq!(call_sites(src, "record", true).len(), 1);
    assert_eq!(call_sites(src, "record", false).len(), 1);
    assert_eq!(call_sites(src, "record_with_producer", true).len(), 1);

    // `arg_list`: paren matching, including a nested call.
    let s = "f(a, g(b, c), d)";
    let open = s.find('(').expect("paren");
    assert_eq!(arg_list(s, open), Some("a, g(b, c), d"));
    assert_eq!(arg_list("f(a", 1), None);

    // `classify`: a literal, a forward, and neither.
    assert!(matches!(
        classify("Kind::Served, Some(1), 1, ReadSiteRole::Drain"),
        Some(Mint::Literal)
    ));
    assert!(matches!(
        classify("kind, served_seq, popped, role"),
        Some(Mint::Forwarding)
    ));
    assert!(classify("kind, served_seq, popped, chosen_role").is_none());
}
