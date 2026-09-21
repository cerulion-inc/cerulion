// SPDX-License-Identifier: AGPL-3.0-only
//! The `resim_cmd` platform-gating contract, pinned STRUCTURALLY.
//!
//! # Why a source walk and not a build
//!
//! The bug this file exists to prevent is a compile error on a platform a
//! Unix host cannot build. `cerulion_cli_engine::replay_cmd` is `#[cfg(unix)]`
//! (the bag reader is), and a `resim_cmd` that inherits that cfg
//! wholesale from its callee makes `main`'s UNCONDITIONAL references into
//! it fail to resolve on non-Unix (E0433). Worse than the
//! build break is the shape behind it: a user on a non-Unix
//! host gets no usage diagnostic at all from a verb that is gated away.
//!
//! No ordinary build catches that. Without a Windows target and a
//! cross-compiler, `cargo check` on a Unix host exercises
//! exactly the `cfg(unix)` arm — a cfg blind spot.
//! So the invariant is pinned by READING the two files:
//!
//! 1. the module declaration in `crates/cerulion_cli_engine/src/lib.rs` carries NO
//!    `#[cfg(...)]` (so every item in it exists on every platform unless the
//!    item itself is gated), and
//! 2. every `resim_cmd::` item `main.rs` names OUTSIDE a `#[cfg(unix)]`
//!    function is an item `resim_cmd` does NOT gate.
//!
//! Together those two are exactly "the references' cfg matches the
//! definition's", which is what a cross-build would have proved.
//!
//! Reading source is a weaker oracle than compiling — a `cfg` written in a
//! shape this walk does not model would slip past — so the walk is deliberately
//! literal about what it recognises, and its own anti-tautology arm proves it
//! still sees the code it claims to be reading.

use std::path::{Path, PathBuf};

/// Repo root (the parent of `cerulion_cli/`).
fn repo_root() -> PathBuf {
    Path::new(env!("CARGO_MANIFEST_DIR"))
        .parent()
        .expect("repo root")
        .to_path_buf()
}

fn read(rel: &str) -> String {
    let p = repo_root().join(rel);
    std::fs::read_to_string(&p).unwrap_or_else(|e| panic!("read {}: {e}", p.display()))
}

/// Strip `//`-to-end-of-line comments so a `#[cfg(unix)]` QUOTED in prose (this
/// module's own docs do it repeatedly) is never mistaken for a real attribute.
///
/// Line comments only, which is enough here: neither file uses block comments
/// around the items this walk reads, and the anti-tautology arm below proves
/// the stripped view still contains the real code.
fn code_only(src: &str) -> String {
    src.lines()
        .map(|line| match line.find("//") {
            Some(i) => &line[..i],
            None => line,
        })
        .collect::<Vec<_>>()
        .join("\n")
}

/// Is this attribute line a `cfg` that can make its item ABSENT on some target?
///
/// Deliberately CONSERVATIVE: any `#[cfg(...)]` counts, not just the literal
/// `#[cfg(unix)]` a naive matcher looks for. Two reasons, and the second is why the
/// conservative rule beats a smarter matcher.
///
/// 1. `#[cfg(any(unix))]` and `#[cfg(all(unix))]` are exactly equivalent to
///    `#[cfg(unix)]` and evade a `starts_with("#[cfg(unix)]")` test entirely —
///    the item reads as UNGATED, so `main` could name it unconditionally and the
///    walk would say the split was sound. There is no non-Unix CI job, so this
///    walk is the ONLY guard; an evadable guard is worse than none, because it
///    reports success.
/// 2. Enumerating the equivalent spellings is a losing game (`any(all(unix))`,
///    `not(not(unix))`, a `target_family = "unix"`, a future `cfg` on a
///    different axis). What the invariant actually needs is not "is this
///    predicate unix?" but "can this item be missing when `main` names it?" —
///    and for THAT question every `cfg` answers yes. An unconditionally-present
///    item carries no `cfg` at all.
///
/// A `cfg_attr` counts too — but only when the attribute it ADDS is a `cfg`.
///
/// It is tempting to say that `cfg_attr` "conditions
/// another attribute, never the item's existence". That is wrong, and the
/// counter-example is short: `#[cfg_attr(not(unix), cfg(unix))]` adds
/// `#[cfg(unix)]` exactly when `not(unix)` holds, so OFF Unix the item is
/// gated on `unix` and vanishes, while ON Unix no attribute is added at all.
/// The item therefore exists only on Unix — a unix gate wearing a different
/// hat, which a bare `#[cfg(` prefix test reads as UNGATED.
///
/// So the rule is the same conservative one, applied one level deeper: an added
/// attribute list containing `cfg(` anywhere makes the item conditionally
/// absent. An ordinary `#[cfg_attr(unix, allow(dead_code))]` adds a lint
/// attribute, never a `cfg`, and stays ungated — pinned both ways below.
///
/// The residual is a `cfg_attr` whose added attribute merely MENTIONS `cfg(` in
/// a string (`#[cfg_attr(unix, doc = "see cfg(unix)")]`), which reads as gated.
/// That is the safe direction: it can only make this walk demand that `main`
/// stop naming the item unconditionally, never let a real gate through.
fn is_gating_cfg(attr: &str) -> bool {
    let squashed: String = attr.chars().filter(|c| !c.is_whitespace()).collect();
    if squashed.starts_with("#[cfg(") {
        return true;
    }
    squashed
        .strip_prefix("#[cfg_attr(")
        .is_some_and(|added| added.contains("cfg("))
}

/// The `pub` items of `resim_cmd` that carry a gating `cfg`, by name.
///
/// The rule is one line long: an item is gated iff a gating `cfg` attribute
/// (see [`is_gating_cfg`]) appears in the attribute block immediately above it,
/// with nothing but other attributes and doc comments between.
fn unix_gated_items(src: &str) -> Vec<String> {
    let code = code_only(src);
    let mut gated = Vec::new();
    let mut cfg_pending = false;
    for line in code.lines() {
        let t = line.trim();
        if t.is_empty() {
            continue;
        }
        if is_gating_cfg(t) {
            cfg_pending = true;
            continue;
        }
        if t.starts_with('#') {
            // Another attribute — the block continues, the cfg still applies.
            continue;
        }
        if cfg_pending {
            // The first non-attribute line after the cfg is the item it gates.
            // Names are enough: the walk compares them against what `main` uses.
            if let Some(name) = item_name(t) {
                gated.push(name);
            }
            cfg_pending = false;
        }
    }
    gated.sort();
    gated
}

/// Record each source name in a grouped import's body (`a, b as c, self`).
///
/// `X as Y` and `self` both appear in real imports: take the SOURCE name (that
/// is what must exist in `resim_cmd`), and skip `self`, which names the module
/// rather than any item the walk compares against.
fn push_group_names(group: &str, refs: &mut Vec<String>) {
    for item in group.split(',') {
        let name: String = item
            .trim()
            .chars()
            .take_while(|c| c.is_alphanumeric() || *c == '_')
            .collect();
        if !name.is_empty() && name != "self" {
            refs.push(name);
        }
    }
}

/// The declared name of an item line (`pub fn foo(`, `const _: () = {`, …).
fn item_name(line: &str) -> Option<String> {
    let after_kw = |kw: &str| -> Option<String> {
        let rest = line.split_once(kw)?.1;
        let name: String = rest
            .trim_start()
            .chars()
            .take_while(|c| c.is_alphanumeric() || *c == '_')
            .collect();
        (!name.is_empty()).then_some(name)
    };
    for kw in ["fn ", "struct ", "enum ", "const ", "static ", "use "] {
        if let Some(n) = after_kw(kw) {
            return Some(n);
        }
    }
    None
}

/// Every `resim_cmd::<ITEM>` mentioned in `main.rs` OUTSIDE a `#[cfg(unix)]`
/// function, plus every `resim_cmd` item pulled in by an ungated `use`.
///
/// The scan is line-based and tracks one thing: whether the current line sits
/// inside a function whose attribute block carried `#[cfg(unix)]`. That is the
/// only gating shape `main.rs` uses for these references (verified by the
/// anti-tautology arm, which requires the gated set to be non-empty).
fn unconditional_resim_refs(src: &str) -> Vec<String> {
    let code = code_only(src);
    let mut refs = Vec::new();
    let mut cfg_pending = false;
    let mut in_gated_fn = false;
    let mut depth: i32 = 0;
    // The body of a grouped import still waiting for its closing brace.
    let mut pending_group: Option<String> = None;
    for line in code.lines() {
        let t = line.trim();
        if in_gated_fn {
            depth += line.matches('{').count() as i32;
            depth -= line.matches('}').count() as i32;
            if depth <= 0 {
                in_gated_fn = false;
            }
            continue;
        }
        if is_gating_cfg(t) {
            cfg_pending = true;
            continue;
        }
        if t.starts_with('#') || t.starts_with("///") {
            continue;
        }
        if cfg_pending && !t.is_empty() {
            cfg_pending = false;
            // A gated item: skip its whole body.
            in_gated_fn = true;
            depth = line.matches('{').count() as i32 - line.matches('}').count() as i32;
            if depth <= 0 {
                in_gated_fn = false;
            }
            continue;
        }
        // A grouped import may WRAP across source lines — rustfmt does exactly
        // that once the list outgrows the width, so it is the shape a long
        // import ENDS UP in rather than an exotic one. An open group therefore
        // carries into the following lines until its closing brace.
        if let Some(open) = pending_group.as_mut() {
            match line.split_once('}') {
                Some((head, _)) => {
                    open.push_str(head);
                    let group = std::mem::take(open);
                    pending_group = None;
                    push_group_names(&group, &mut refs);
                }
                None => {
                    // Each wrapped line is one or more whole items; the added
                    // comma keeps a trailing-comma-less last item separate from
                    // the next line's first.
                    open.push_str(line);
                    open.push(',');
                }
            }
            continue;
        }

        // Ungated line — record every name it reaches through `resim_cmd::`.
        let mut rest = line;
        while let Some(i) = rest.find("resim_cmd::") {
            rest = &rest[i + "resim_cmd::".len()..];
            // A GROUPED import (`resim_cmd::{run_resim, PlayFlags}`) reaches
            // several items at once. Taking `chars().take_while(is
            // alphanumeric)` straight off `rest` yields, on a `{`, the
            // EMPTY string and records NOTHING — so a grouped import would evade
            // the symmetry assertion entirely, and grouping is the form rustfmt
            // and ordinary style both produce. Each braced name is recorded on
            // its own.
            if let Some(inner) = rest.strip_prefix('{') {
                match inner.split_once('}') {
                    // Closed on this line: record it and keep scanning the
                    // remainder, so a second `resim_cmd::` after it is seen.
                    Some((group, tail)) => {
                        push_group_names(group, &mut refs);
                        rest = tail;
                        continue;
                    }
                    // Wraps: hand the rest of the line to the carry buffer.
                    None => {
                        pending_group = Some(format!("{inner},"));
                        break;
                    }
                }
            }
            let name: String = rest
                .chars()
                .take_while(|c| c.is_alphanumeric() || *c == '_')
                .collect();
            if !name.is_empty() {
                refs.push(name);
            }
        }
    }
    refs.sort();
    refs.dedup();
    refs
}

const ENGINE_LIB: &str = "cerulion_cli_engine/src/lib.rs";
const RESIM: &str = "cerulion_cli_engine/src/resim_cmd.rs";
const MAIN: &str = "cerulion_cli/src/main.rs";

#[test]
fn the_resim_module_declaration_carries_no_cfg() {
    // Half 1 of the invariant. If the MODULE is gated, nothing inside it exists
    // off Unix and every ungated reference in `main` breaks — which is exactly
    // the defect this test pins.
    let lib = code_only(&read(ENGINE_LIB));
    let (before, _) = lib
        .split_once("pub mod resim_cmd;")
        .expect("`pub mod resim_cmd;` must be declared in the engine's lib.rs");
    let tail = before
        .lines()
        .rev()
        .take_while(|l| {
            let t = l.trim();
            t.starts_with('#') || t.is_empty()
        })
        .collect::<Vec<_>>();
    assert!(
        !tail.iter().any(|l| l.contains("cfg(")),
        "`resim_cmd` must NOT be cfg-gated — `main` reaches its removal notice \
         on every platform. Gate the individual engine-touching functions \
         instead. Attributes found: {tail:?}"
    );
}

#[test]
fn every_unconditional_reference_names_an_ungated_item() {
    // Half 2. Anything `main` names outside a `#[cfg(unix)]` fn must be an item
    // `resim_cmd` does not gate, or the non-Unix build cannot resolve it.
    let gated = unix_gated_items(&read(RESIM));
    let refs = unconditional_resim_refs(&read(MAIN));

    let broken: Vec<&String> = refs.iter().filter(|r| gated.contains(r)).collect();
    assert!(
        broken.is_empty(),
        "`main.rs` names {broken:?} unconditionally, but `resim_cmd` gates \
         it with `#[cfg(unix)]` — that is an E0433 on every non-Unix build, \
         and a Unix host's build cannot catch it. Either un-gate the \
         item or move the reference into a `#[cfg(unix)]` function with a \
         `#[cfg(not(unix))]` twin.\n  gated: {gated:?}\n  unconditional refs: {refs:?}"
    );
}

#[test]
fn the_walk_still_sees_the_code_it_claims_to_read() {
    // ANTI-TAUTOLOGY, and it is not optional: both arms above PASS vacuously if
    // the walk finds nothing — a broken comment stripper, a renamed module, a
    // reworked `main` would each turn this file into decoration. So require
    // that each half found real material.
    let gated = unix_gated_items(&read(RESIM));
    assert!(
        gated.contains(&"run_resim".to_string()) && gated.contains(&"run_play_resim".to_string()),
        "the two engine-touching functions must be seen as gated; got {gated:?}"
    );

    // The arm names what holds the invariant: `main`'s PRE-AUTH usage
    // refusal, which asks the pure
    // `resolve_play_mode` oracle above the login gate and returns `EXIT_USAGE`.
    // All three are ungated in `resim_cmd` and named unconditionally in `main`.
    let refs = unconditional_resim_refs(&read(MAIN));
    for want in ["resolve_play_mode", "PlayFlags", "EXIT_USAGE"] {
        assert!(
            refs.contains(&want.to_string()),
            "the pre-auth usage refusal must be seen as an UNCONDITIONAL \
             reference to {want:?} — it is what this contract now protects; \
             got {refs:?}"
        );
    }

    // And the stripper must not be eating live code: `main.rs`'s docs quote
    // `#[cfg(unix)]` in prose, so a stripper that missed line comments would
    // report phantom gates.
    let code = code_only("let x = 1; // #[cfg(unix)] in a comment\n#[cfg(unix)]\nfn real() {}");
    assert!(!code.contains("in a comment"));
    assert!(code.contains("#[cfg(unix)]"));
    assert!(code.contains("fn real()"));
}

/// The gate matcher is not evadable by rewriting
/// the cfg PREDICATE into an equivalent spelling.
///
/// `#[cfg(any(unix))]` and `#[cfg(all(unix))]` mean exactly `#[cfg(unix)]`, and
/// a `starts_with("#[cfg(unix)]")` matcher read all three items as UNGATED —
/// which would let `main` name them unconditionally with this walk still
/// reporting success. There is no non-Unix CI job, so a false success here is
/// the whole failure: nothing else would catch the E0433.
///
/// The last two inputs are the point of the CONSERVATIVE rule: a gate on a
/// different axis, and a whitespace-padded one, are both still gates.
#[test]
fn a_cfg_predicate_cannot_be_respelled_to_evade_the_gate_walk() {
    for attr in [
        "#[cfg(unix)]",
        "#[cfg(any(unix))]",
        "#[cfg(all(unix))]",
        "#[cfg(any(all(unix)))]",
        "#[cfg(target_family = \"unix\")]",
        "#[cfg( unix )]",
        "#[cfg(not(windows))]",
        // A `cfg_attr` that ADDS a `cfg`. Off Unix `not(unix)` holds, so this
        // expands to `#[cfg(unix)]` and the item vanishes; on Unix nothing is
        // added. The item therefore exists only on Unix — a real gate, which
        // the `#[cfg(` prefix test read as ungated.
        "#[cfg_attr(not(unix), cfg(unix))]",
        "#[cfg_attr(feature = \"x\", cfg(unix))]",
        "#[cfg_attr(a, cfg_attr(b, cfg(unix)))]",
    ] {
        let src = format!("{attr}\npub fn gated_item() {{}}\n");
        assert_eq!(
            unix_gated_items(&src),
            vec!["gated_item".to_string()],
            "{attr} must be seen as a gate"
        );
    }

    // ANTI-TAUTOLOGY: an UNGATED item must still read as ungated, and
    // `cfg_attr` conditions another ATTRIBUTE, never the item's existence — so
    // neither may be swept up by the conservative rule.
    for src in [
        "pub fn plain() {}\n",
        "#[derive(Debug)]\npub struct Plain;\n",
        "#[cfg_attr(unix, allow(dead_code))]\npub fn conditioned() {}\n",
    ] {
        assert!(
            unix_gated_items(src).is_empty(),
            "must read as UNGATED: {src:?} -> {:?}",
            unix_gated_items(src)
        );
    }
}

/// A grouped import reaches its items, and the walk
/// must see every one of them.
///
/// `use ...::resim_cmd::{run_resim, PlayFlags};` yields ZERO refs to a scan
/// that reads the character after `resim_cmd::`, finds `{`, and collects the
/// empty string. Grouping is what rustfmt and ordinary style produce, so the
/// symmetry assertion is one `use` away from being silently inert. (The
/// `resim_usage_refusal` in `main.rs` uses exactly that form.)
#[test]
fn a_grouped_import_does_not_hide_its_items_from_the_symmetry_walk() {
    let src = "use cerulion_cli_engine::resim_cmd::{run_resim, PlayFlags};\n";
    assert_eq!(
        unconditional_resim_refs(src),
        vec!["PlayFlags".to_string(), "run_resim".to_string()],
        "both grouped names must be recorded"
    );

    // `self` names the module, not an item — and an `as` rename must record the
    // SOURCE name, since that is what has to exist in `resim_cmd`.
    let src = "use cerulion_cli_engine::resim_cmd::{self, run_play_resim as go};\n";
    assert_eq!(
        unconditional_resim_refs(src),
        vec!["run_play_resim".to_string()]
    );

    // A group closed on the same line must not swallow the REST of that line:
    // a second `resim_cmd::` after it is still a reference.
    let src = "use cerulion_cli_engine::resim_cmd::{EXIT_USAGE, PlayFlags};\n\
               let c = resim_cmd::EXIT_PASS;\n";
    let refs = unconditional_resim_refs(src);
    for want in ["EXIT_USAGE", "PlayFlags", "EXIT_PASS"] {
        assert!(refs.contains(&want.to_string()), "missing {want}: {refs:?}");
    }

    // ANTI-TAUTOLOGY: a grouped import inside a GATED fn must still be skipped,
    // or the hardening would turn every legitimate gated use into a failure.
    let src = "#[cfg(unix)]\nfn gated() {\n    use cerulion_cli_engine::resim_cmd::{run_resim, PlayFlags};\n}\n";
    assert!(
        unconditional_resim_refs(src).is_empty(),
        "a gated fn's grouped import is not an unconditional ref: {:?}",
        unconditional_resim_refs(src)
    );
}

/// A grouped import that wraps across source lines
/// is still seen.
///
/// rustfmt line-wraps a `use` group once the list outgrows the width, so this
/// is the shape a long import ENDS UP in — not an exotic one. A
/// `{`-branch that reads only as far as a closing brace ON THE SAME LINE
/// records NOTHING for a wrapped group, and the symmetry assertion goes silently
/// inert for it. (A fixture of two single-line statements does not cover this
/// shape, which is why this test exists.)
#[test]
fn a_wrapped_grouped_import_is_carried_across_lines() {
    // Exactly what rustfmt emits: brace alone on the opening line, one item per
    // line, trailing comma, closing brace on its own line.
    let wrapped = "use cerulion_cli_engine::resim_cmd::{\n\
                   \x20   run_resim,\n\
                   \x20   run_play_resim,\n\
                   \x20   PlayFlags,\n\
                   };\n";
    let refs = unconditional_resim_refs(wrapped);
    for want in ["PlayFlags", "run_play_resim", "run_resim"] {
        assert!(
            refs.contains(&want.to_string()),
            "a wrapped group must record {want}: {refs:?}"
        );
    }

    // No trailing comma on the last item, and the close sharing that item's
    // line — both legal, and each is where a naive carry drops a name.
    let tight = "use cerulion_cli_engine::resim_cmd::{\n\
                 \x20   EXIT_USAGE,\n\
                 \x20   EXIT_PASS};\n";
    let refs = unconditional_resim_refs(tight);
    assert!(refs.contains(&"EXIT_USAGE".to_string()), "{refs:?}");
    assert!(
        refs.contains(&"EXIT_PASS".to_string()),
        "the last item without a trailing comma must survive: {refs:?}"
    );

    // Several items sharing one wrapped line, plus `self` and an `as` rename.
    let mixed = "use cerulion_cli_engine::resim_cmd::{\n\
                 \x20   self, run_resim, EXIT_USAGE as USAGE,\n\
                 };\n";
    let refs = unconditional_resim_refs(mixed);
    assert_eq!(
        refs,
        vec!["EXIT_USAGE".to_string(), "run_resim".to_string()]
    );

    // ANTI-TAUTOLOGY: the carry must STOP at the closing brace. A reference
    // after the group is an ordinary one, and a name that merely follows the
    // group must not be invented as a member of it.
    let after = "use cerulion_cli_engine::resim_cmd::{\n\
                 \x20   run_resim,\n\
                 };\n\
                 let x = resim_cmd::EXIT_PASS;\n\
                 let y = something_else::NotAResimItem;\n";
    let refs = unconditional_resim_refs(after);
    assert_eq!(
        refs,
        vec!["EXIT_PASS".to_string(), "run_resim".to_string()],
        "the carry must close at `}}` and invent nothing: {refs:?}"
    );
}
