// SPDX-License-Identifier: AGPL-3.0-only
//! P12, mechanized: library code never prints.
//!
//! The logging convention in AGENTS.md says library code never prints, and
//! had ZERO mechanical enforcement. This file is the structural half of the fix.
//!
//! # Why the ban is a crate-root attribute and not a `[workspace.lints]` row
//!
//! A manifest lint table applies to EVERY target of a package — lib, bins,
//! examples, and every integration-test binary. P12's rule is about library
//! code only: a bin's job IS to print, and a test binary printing evidence
//! under `--nocapture` is the house pattern for box harnesses. Measured on this
//! tree, a workspace-wide `print_stdout = "deny"` would have fired on **522**
//! print sites, over 200 of them in test binaries — i.e. it would have been
//! reverted or blanket-allowed within a day.
//!
//! The one granularity in Rust that says "library code" is a crate-root
//! attribute on the lib target:
//!
//! ```ignore
//! #![cfg_attr(not(test), deny(clippy::print_stdout, clippy::print_stderr))]
//! ```
//!
//! `not(test)` exempts the crate's own unit tests (compiled into the lib target
//! with `cfg(test)`); integration tests are separate crates and never see it.
//! A legitimate print site takes a targeted `#[allow]` with a written reason.
//!
//! # Which crates this covers
//!
//! Every library crate cargo RESOLVES into the root workspace, plus the library
//! members of each MIRRORED workspace (see `workspace_lints_manifest_test.rs`
//! for what a mirrored workspace is). Both universes matter and neither is the
//! declared `[workspace] members` list:
//!
//!  * cargo folds a foreign path dependency into the workspace whether or not
//!    the list names it, so `examples/go2/nodes/go2_tf_source` — a robot-loaded
//!    node cdylib, exactly the class this ban exists for — was BUILT by
//!    `cargo check --workspace` while being invisible to a declared-list walk;
//!  * its sibling `examples/go2` node crates are the same class and are not
//!    resolved into the root workspace at all, so only the mirrored arm reaches
//!    them.
//!
//! # Why a walk and not a list
//!
//! Same reason as `workspace_lints_manifest_test.rs`: a hand-maintained list of
//! crates reproduces the failure mode it is meant to catch (its sweep
//! missed `rerun_sink`; three earlier fixes each un-ran a package by not naming
//! it). A library crate added tomorrow is covered here with no edit to this
//! file — it just fails until it carries the line.
//!
//! # What the needle match does and does not prove
//!
//! The needle is matched against a COMMENT-STRIPPED view, so prose quoting the
//! attribute (this file's own module docs, `clippy.toml`, AGENTS.md) cannot
//! satisfy it. `code_only` is oracle-tested below rather than trusted. The match
//! additionally requires the attribute to be its own line at COLUMN 0, which is
//! where a crate-root inner attribute sits: an occurrence indented inside a
//! nested `mod { … }` block would apply only to that module, and is rejected.
//!
//! Residual, stated rather than papered over: the match is textual, so it does
//! not prove the attribute is positioned before every item (rustc enforces that
//! — a misplaced `#![…]` is a compile error, so the compiler is the backstop),
//! and it does not model string literals, so a crate-root file that held the
//! needle inside a string would pass. No crate does, and it would be a bizarre
//! thing to write.

mod common;

use common::{declared_members_of, lib_target_path, repo_root, resolved_members};
use std::collections::BTreeSet;

/// The attribute every library crate root must carry, matched verbatim.
const PRINT_BAN: &str = "#![cfg_attr(not(test), deny(clippy::print_stdout, clippy::print_stderr))]";

/// Workspaces whose library members must carry the ban even though the root
/// workspace does not resolve them.
///
/// Kept in step with `workspace_lints_manifest_test.rs`'s
/// `MIRRORED_LINT_WORKSPACES` — these are robot-shipped node crates, the class
/// P12 is written for.
const MIRRORED_WORKSPACES: &[&str] = &["examples/go2"];

/// Library crates that deliberately do NOT carry the ban.
///
/// `(member path relative to the repo root, reason)`. EMPTY is the intended
/// steady state: a legitimate print site takes a targeted `#[allow]` naming
/// itself, which keeps the ban armed for the rest of the crate. Exempting a
/// whole crate disarms it for every future line in that crate too.
const CRATES_WITHOUT_THE_PRINT_BAN: &[(&str, &str)] = &[];

/// `source` with `//` line comments and `/* … */` block comments removed.
///
/// Block comments NEST in Rust, so the depth is tracked rather than matched to
/// the first `*/`. String literals are deliberately not modelled — see the
/// residual note in the module docs. An UNTERMINATED block comment fails CLOSED
/// (the rest of the file is treated as comment), so a broken parse cannot
/// silently satisfy the needle.
fn code_only(source: &str) -> String {
    let bytes = source.as_bytes();
    let mut out = String::with_capacity(source.len());
    let mut i = 0usize;
    let mut depth = 0usize;
    while i < bytes.len() {
        if depth > 0 {
            if bytes[i..].starts_with(b"/*") {
                depth += 1;
                i += 2;
            } else if bytes[i..].starts_with(b"*/") {
                depth -= 1;
                i += 2;
            } else {
                if bytes[i] == b'\n' {
                    out.push('\n');
                }
                i += 1;
            }
            continue;
        }
        if bytes[i..].starts_with(b"/*") {
            depth = 1;
            i += 2;
            continue;
        }
        if bytes[i..].starts_with(b"//") {
            while i < bytes.len() && bytes[i] != b'\n' {
                i += 1;
            }
            continue;
        }
        // Not a comment start: copy this byte (UTF-8 safe — multi-byte
        // sequences never contain an ASCII byte).
        let ch_len = utf8_len(bytes[i]);
        out.push_str(&source[i..i + ch_len]);
        i += ch_len;
    }
    out
}

fn utf8_len(b: u8) -> usize {
    match b {
        0x00..=0x7f => 1,
        0xc0..=0xdf => 2,
        0xe0..=0xef => 3,
        _ => 4,
    }
}

/// True iff `source` carries the ban as a CRATE-ROOT inner attribute.
///
/// Column 0 and its own line: an indented occurrence is inside a nested module,
/// where it would govern that module only.
fn carries_crate_root_ban(source: &str) -> bool {
    code_only(source)
        .lines()
        .any(|line| line.trim_end() == PRINT_BAN)
}

/// Every library crate the ban must cover: root-workspace RESOLVED members plus
/// the library members of each mirrored workspace.
fn library_crates(root: &std::path::Path) -> Vec<String> {
    let mut all = resolved_members(root);
    for ws in MIRRORED_WORKSPACES {
        all.extend(declared_members_of(root, ws));
    }
    all.sort();
    all.dedup();
    all.into_iter()
        .filter(|m| lib_target_path(root, m).is_some())
        .collect()
}

#[test]
fn every_library_crate_root_denies_printing_in_library_code() {
    let root = repo_root();
    let libs = library_crates(&root);

    // Anti-tautology: a broken walk would make every assertion below vacuous.
    assert!(
        libs.len() >= 40,
        "the walk found only {} library members — it is not reading the \
         workspace correctly, so this gate proves nothing",
        libs.len()
    );
    // The mirrored arm must actually contribute, and this has to DISCRIMINATE:
    // cargo folds `examples/go2/nodes/go2_tf_source` into the ROOT workspace on its
    // own (it is `cerulion_viz`'s path dev-dependency), so "some examples/go2
    // member is present" is satisfied with the mirrored walk deleted entirely —
    // that variant passed the first version of this assertion while the
    // other seven robot crates went silently uncovered. Those seven are reachable
    // ONLY through the mirrored walk, so require MORE than the one cargo folds in.
    let go2_libs = libs
        .iter()
        .filter(|m| m.starts_with("examples/go2/"))
        .count();
    assert!(
        go2_libs >= 2,
        "only {go2_libs} `examples/go2` library member(s) reached the walk. Cargo \
         resolves exactly one of them into the root workspace by itself, so a \
         count of 1 means the mirrored-workspace half of the universe is not \
         being read and the robot's other node crates are silently uncovered."
    );

    let declared: BTreeSet<&str> = CRATES_WITHOUT_THE_PRINT_BAN
        .iter()
        .map(|(p, _)| *p)
        .collect();
    assert_eq!(
        declared.len(),
        CRATES_WITHOUT_THE_PRINT_BAN.len(),
        "CRATES_WITHOUT_THE_PRINT_BAN contains a duplicate path"
    );

    let mut without = BTreeSet::new();
    for member in &libs {
        let lib_rs = lib_target_path(&root, member).expect("filtered to library members above");
        let source = std::fs::read_to_string(&lib_rs)
            .unwrap_or_else(|e| panic!("cannot read {}: {e}", lib_rs.display()));
        if !carries_crate_root_ban(&source) {
            without.insert(member.clone());
        }
    }

    let missing: Vec<&String> = without
        .iter()
        .filter(|m| !declared.contains(m.as_str()))
        .collect();
    assert!(
        missing.is_empty(),
        "these library crate roots do NOT ban printing in library code, so P12 \
         (\"never `println!` in library code\") is unenforced there:\n  {}\n\n\
         FIX: add this line to each crate's library root, above the first item —\n\n    \
         {PRINT_BAN}\n\n\
         A legitimate print site then takes a targeted `#[allow(clippy::print_stdout)]` \
         (or `print_stderr`) WITH A REASON, which keeps the ban armed for the rest of \
         the crate. Exempting a whole crate (CRATES_WITHOUT_THE_PRINT_BAN in {}) \
         disarms it for every future line too.",
        missing
            .iter()
            .map(|m| format!("{m} (crate root)"))
            .collect::<Vec<_>>()
            .join("\n  "),
        file!()
    );

    let stale: Vec<&str> = declared
        .iter()
        .filter(|d| !without.contains(**d))
        .copied()
        .collect();
    assert!(
        stale.is_empty(),
        "CRATES_WITHOUT_THE_PRINT_BAN names crates that DO carry the ban: \
         {stale:?}. An exemption that describes nothing is a pre-authorised \
         future opt-out — delete the entries from {}.",
        file!()
    );
}

#[test]
fn the_scaffolding_templates_emit_the_print_ban() {
    // The walk above covers crates that EXIST. A node scaffolded tomorrow by
    // `cerulion node create` inherits the ban only if the generator emits it,
    // and there are TWO generators. The raw-FFI one is pinned by
    // `templates::tests::test_generate_lib_rs_matches_oracle_fixture` (its emit
    // is frozen byte-for-byte as a workspace fixture); the MACRO one was pinned
    // by nothing — measured, by deleting its `push_str` and watching all 94
    // templates unit tests and both policy gates stay green.
    use cerulion_cli_engine::node_metadata::NodeMetadata;
    use cerulion_cli_engine::templates::{generate_lib_rs, generate_macro_lib_rs};

    // The macro generator `debug_assert!`s against `policy=None` with no inputs
    // (the shape `node_create_with_options` rejects at the API boundary), so the
    // probe carries a real policy — the scaffold `cerulion node create --policy
    // period_ms=100` produces.
    let macro_metadata = NodeMetadata {
        node_type: "print_ban_probe".to_string(),
        policy: Some(cerulion_core::MacroPolicy::Period { period_ms: 100 }),
        inputs: vec![],
        outputs: vec![],
    };
    // The raw-FFI generator has no such gate; keep its input the same shape the
    // frozen oracle fixture uses.
    let raw_metadata = NodeMetadata {
        node_type: "print_ban_probe".to_string(),
        policy: None,
        inputs: vec![],
        outputs: vec![],
    };

    for (label, emitted) in [
        (
            "generate_macro_lib_rs",
            generate_macro_lib_rs(&macro_metadata, None),
        ),
        ("generate_lib_rs (raw FFI)", generate_lib_rs(&raw_metadata)),
    ] {
        assert!(
            carries_crate_root_ban(&emitted),
            "`{label}` does not emit the P12 print ban at the crate root, so \
             every node scaffolded through it ships with printing UNBANNED — \
             the walk in this file cannot catch that, because it only sees \
             crates that already exist.\n\nFIX: emit\n\n    {PRINT_BAN}\n\n\
             from that generator.\n\n--- emitted ---\n{emitted}"
        );
    }
}

#[test]
fn the_repo_clippy_config_declares_the_banned_paths_with_reasons() {
    let root = repo_root();
    let path = root.join("clippy.toml");
    let text = std::fs::read_to_string(&path).unwrap_or_else(|e| {
        panic!(
            "cannot read {} — the repo-wide clippy config is what gives \
             `disallowed_macros` / `disallowed_methods` anything to enforce; \
             without it both lints are inert: {e}",
            path.display()
        )
    });
    let config: toml::Value =
        toml::from_str(&text).unwrap_or_else(|e| panic!("clippy.toml does not parse: {e}"));

    for (key, wanted) in [
        ("disallowed-macros", "std::dbg"),
        ("disallowed-methods", "std::process::exit"),
    ] {
        let entries = config
            .get(key)
            .and_then(|v| v.as_array())
            .unwrap_or_else(|| panic!("clippy.toml declares `{key}`"));
        let hit = entries
            .iter()
            .find(|e| e.get("path").and_then(|p| p.as_str()) == Some(wanted));
        let hit = hit.unwrap_or_else(|| {
            panic!(
                "clippy.toml `{key}` no longer lists `{wanted}`. If that ban was \
                 dropped on purpose, drop this assertion with it; otherwise the \
                 lint is now silently allowing it."
            )
        });
        let reason = hit
            .get("reason")
            .and_then(|r| r.as_str())
            .unwrap_or_else(|| panic!("`{wanted}` in clippy.toml `{key}` carries no `reason`"));
        assert!(
            reason.len() > 20,
            "`{wanted}`'s reason is `{reason}` — the reason IS the error message \
             an offender reads; it has to say what to do instead"
        );
    }
}

#[test]
fn the_workspace_clippy_table_denies_the_policy_lints() {
    let root = repo_root();
    let manifest = common::read_manifest(&root.join("Cargo.toml"));
    let clippy = manifest
        .get("workspace")
        .and_then(|w| w.get("lints"))
        .and_then(|l| l.get("clippy"))
        .and_then(|c| c.as_table())
        .expect("root Cargo.toml has [workspace.lints.clippy]");

    // Two classes, MEASURED against clippy 1.96 rather than read off a group
    // name — the consequence of a missing line differs, so the message must too.
    for (lint, missing_consequence) in [
        // Restriction group, ALLOW by default: the line IS the enforcement.
        ("dbg_macro", "It is an allow-by-default restriction lint, so leaving it out makes it INERT."),
        ("todo", "It is an allow-by-default restriction lint, so leaving it out makes it INERT."),
        ("unimplemented", "It is an allow-by-default restriction lint, so leaving it out makes it INERT."),
        // WARN by default: clippy.toml still fires without the line, and CI's
        // `-D warnings` still fails — the line buys local/CI agreement.
        ("disallowed_macros", "It is WARN by default, so clippy.toml's entries still fire without this line and CI's `-D warnings` still fails; the line is what makes a plain local `cargo clippy` agree with CI."),
        ("disallowed_methods", "It is WARN by default, so clippy.toml's entries still fire without this line and CI's `-D warnings` still fails; the line is what makes a plain local `cargo clippy` agree with CI."),
    ] {
        let got = clippy
            .get(lint)
            .and_then(|v| {
                v.as_str()
                    .or_else(|| v.get("level").and_then(|l| l.as_str()))
            })
            .unwrap_or_else(|| {
                panic!("[workspace.lints.clippy] does not set `{lint}`. {missing_consequence}")
            });
        assert_eq!(
            got, "deny",
            "[workspace.lints.clippy].{lint} is `{got}`, expected `deny`"
        );
    }
}

#[test]
fn code_only_strips_both_comment_syntaxes_and_nothing_else() {
    // Hand-written oracle vectors: without these, every "the needle is absent"
    // verdict above could be a stripper bug rather than a missing attribute.
    assert_eq!(
        code_only("let a = 1; // trailing\nlet b = 2;\n"),
        "let a = 1; \nlet b = 2;\n"
    );
    assert_eq!(code_only("a /* b */ c"), "a  c");
    assert_eq!(code_only("a /* b\nc */ d"), "a \n d");
    // NESTED block comments: a first-`*/` match would leave ` x */ d` behind.
    assert_eq!(code_only("a /* b /* inner */ x */ d"), "a  d");
    // A block-comment opener INSIDE a line comment opens nothing.
    assert_eq!(code_only("// /* not a block\nkept\n"), "\nkept\n");
    // Unterminated block comment fails CLOSED — the rest of the file is gone.
    assert_eq!(
        code_only("kept /* swallowed\nalso swallowed\n"),
        "kept \n\n"
    );
    // Multi-byte characters survive intact.
    assert_eq!(code_only("é — ok // dropped\n"), "é — ok \n");
}

#[test]
fn the_needle_match_is_comment_blind_and_position_aware() {
    // The needle written as prose is NOT code.
    assert!(!carries_crate_root_ban(&format!("// {PRINT_BAN}\n")));
    assert!(!carries_crate_root_ban(&format!("//! {PRINT_BAN}\n")));
    assert!(!carries_crate_root_ban(&format!("/* {PRINT_BAN} */\n")));
    // At column 0 on its own line, it counts.
    assert!(carries_crate_root_ban(&format!("{PRINT_BAN}\n")));
    assert!(carries_crate_root_ban(&format!(
        "//! docs\n{PRINT_BAN}\n\nfn f() {{}}\n"
    )));
    // Trailing whitespace is not meaningful.
    assert!(carries_crate_root_ban(&format!("{PRINT_BAN}   \n")));
    // INDENTED, i.e. inside a nested `mod { … }`, governs that module only and
    // must NOT satisfy the crate-root requirement.
    assert!(!carries_crate_root_ban(&format!(
        "mod inner {{\n    {PRINT_BAN}\n}}\n"
    )));
}
