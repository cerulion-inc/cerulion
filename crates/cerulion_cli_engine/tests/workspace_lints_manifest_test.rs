// SPDX-License-Identifier: AGPL-3.0-only
//! Every workspace member inherits the ONE strict lint table.
//!
//! The repo's "Linting (Strict)" policy — `dead_code`, `unused_imports` and
//! `unused_variables` at `deny`, `clippy::all` at `warn` — used to live as six
//! hand-copied lines in each crate manifest. That is a policy a new crate opts
//! OUT of by simply not copying it, and 35 of the workspace's members had done
//! exactly that: the 33 `test_fixtures/*` cdylibs, `cerulion_viz` and `go2_tf`
//! carried no `[lints]` table at all, so `dead_code = "deny"` did not apply to
//! a third of the tree. Nothing failed; the policy was just absent there.
//!
//! The table now lives once, in the root `[workspace.lints]`, and each member
//! carries
//!
//! ```toml
//! [lints]
//! workspace = true
//! ```
//!
//! This file is the STRUCTURAL half that keeps it that way. It is a WALK, not a
//! hand-maintained list of crates, for the reason the repo keeps re-learning
//! (an earlier sweep missed `rerun_sink`; earlier hand lists each
//! un-ran a package by not naming it): a list reproduces the failure mode it is
//! supposed to catch. A crate added tomorrow is covered here by construction.
//!
//! The inheritance arms are set-equality against a DECLARED inventory rather
//! than a one-directional "these must inherit" scan:
//!
//!  * a member that does not inherit and is not declared exempt FAILS, naming
//!    the manifest and the two lines that fix it;
//!  * a member declared exempt that DOES inherit also FAILS, so the inventory
//!    cannot rot into a list of pre-authorised future opt-outs (the "orphaned
//!    waiver" class from `upstream_drift_test.rs`).
//!
//! Cargo itself supplies the third guarantee this rests on: `[lints] workspace
//! = true` may not be mixed with crate-local lint keys ("cannot override
//! `workspace.lints` in `lints`"), so an inheriting member cannot quietly
//! weaken one lint — it must opt out visibly, which this walk then catches.
//!
//! # Three universes, because one walk cannot see them all
//!
//! 1. **Declared** — the root `[workspace] members` list.
//! 2. **Resolved** — what cargo actually builds under `--workspace`. Cargo folds
//!    a path dependency into the workspace whether or not the list names it, so
//!    `examples/go2/nodes/go2_tf_source` (reached through `cerulion_viz`'s path
//!    dev-dep) is BUILT by `cargo check --workspace` while being invisible to
//!    the declared walk. It sat outside this gate entirely until
//!    `every_member_cargo_resolves_inherits_the_lint_table` started asking cargo.
//! 3. **Mirrored workspaces** — the OTHER workspace that also declares a
//!    cross-workspace member. `[lints] workspace = true` resolves against
//!    whichever workspace is LOADING, so that side needs its own
//!    `[workspace.lints]` or the manifest hard-fails from there. Those
//!    workspaces have members of their own, which neither (1) nor (2) reaches —
//!    `every_mirrored_workspace_member_inherits_its_own_lint_table` covers them,
//!    without which the mirrored table is inert for all but one crate.

mod common;

use common::{
    declared_members, declared_members_of, inherits_workspace_lints, owning_workspace,
    read_manifest, repo_root, resolved_members, workspace_lints_table,
};
use std::collections::BTreeSet;

/// Workspaces that MIRROR the root `[workspace.lints]` table, and why.
///
/// A crate loaded by TWO workspaces resolves `[lints] workspace = true` against
/// whichever one is doing the LOADING, so a member shared across a workspace
/// boundary needs the table on BOTH sides or the manifest hard-fails from
/// whichever side lacks it. `examples/go2/nodes/go2_tf_source` is exactly that: its
/// declaring workspace is `examples/go2`, and the ROOT workspace reaches it through
/// `cerulion_viz/lib/cerulion_viz`'s path dev-dependency.
///
/// This list is not trusted to stay complete on its own —
/// `every_cross_workspace_member_has_its_workspace_declared` DERIVES the set
/// that belongs here from the resolved-vs-declared difference and fails if this
/// inventory misses one.
const MIRRORED_LINT_WORKSPACES: &[(&str, &str)] = &[(
    "examples/go2",
    "go2_tf_source is loaded by this workspace AND by the root (via \
     cerulion_viz's path dev-dep), and `lints.workspace = true` resolves \
     per-loader",
)];

/// Members cargo RESOLVES that deliberately do not inherit the shared table.
///
/// `(member path relative to the repo root, reason)`. EMPTY is the intended
/// steady state.
const RESOLVED_ONLY_NON_INHERITING: &[(&str, &str)] = &[];

/// Members of a MIRRORED workspace that deliberately do not inherit its table.
const MIRRORED_MEMBERS_NON_INHERITING: &[(&str, &str)] = &[];

/// Workspace members that deliberately do NOT inherit `[workspace.lints]`.
///
/// `(member path relative to the repo root, reason)`. EMPTY is the intended
/// steady state: a crate that needs a lint the shared table does not carry
/// should say so in its own source (`#![deny(...)]` at the crate root, as
/// `cerulion_macros` does for `unsafe_code`) rather than fork the manifest
/// table, because a forked table silently drops every OTHER lint in it.
const NON_INHERITING_MEMBERS: &[(&str, &str)] = &[];

/// Lints the shared table must carry, with the level each must be at.
///
/// Pins the table's STRENGTH, not merely its existence: without this a
/// `dead_code = "allow"` would leave every `workspace = true` line in place and
/// every other assertion here green while the policy evaporated.
const REQUIRED_RUST_LINTS: &[(&str, &str)] = &[
    ("dead_code", "deny"),
    ("unused_imports", "deny"),
    ("unused_variables", "deny"),
];

#[test]
fn every_workspace_member_inherits_the_workspace_lint_table() {
    let root = repo_root();
    let members = declared_members(&root);

    // Anti-tautology: a parse that yielded a handful of members would make
    // every assertion below vacuous.
    assert!(
        members.len() >= 40,
        "the walk found only {} workspace members — it is not reading the root \
         manifest correctly, so this gate proves nothing",
        members.len()
    );

    let declared: BTreeSet<&str> = NON_INHERITING_MEMBERS.iter().map(|(p, _)| *p).collect();
    assert_eq!(
        declared.len(),
        NON_INHERITING_MEMBERS.len(),
        "NON_INHERITING_MEMBERS contains a duplicate path"
    );

    let actual: BTreeSet<String> = members
        .iter()
        .filter(|m| !inherits_workspace_lints(&root, m))
        .cloned()
        .collect();

    let missing: Vec<&String> = actual
        .iter()
        .filter(|m| !declared.contains(m.as_str()))
        .collect();
    assert!(
        missing.is_empty(),
        "these workspace members do NOT inherit the shared lint table, so \
         CLAUDE.md's strict policy (dead_code / unused_imports / \
         unused_variables = deny) does not apply to them:\n  {}\n\nFIX: add \
         these two lines to each manifest —\n\n    [lints]\n    workspace = \
         true\n\nIf a crate genuinely must opt out, add it to \
         NON_INHERITING_MEMBERS in {} WITH A REASON. Prefer a crate-root \
         `#![deny(...)]` over forking the manifest table: a forked table drops \
         every other lint in the shared one.",
        missing
            .iter()
            .map(|m| format!("{m}/Cargo.toml"))
            .collect::<Vec<_>>()
            .join("\n  "),
        file!()
    );

    let stale: Vec<&str> = declared
        .iter()
        .filter(|d| !actual.contains(**d))
        .copied()
        .collect();
    assert!(
        stale.is_empty(),
        "NON_INHERITING_MEMBERS names members that DO inherit the shared lint \
         table: {stale:?}. An exemption that describes nothing is a \
         pre-authorised future opt-out — delete the entries from {}.",
        file!()
    );
}

#[test]
fn the_workspace_lint_table_is_the_strict_one() {
    let root = repo_root();
    let manifest = read_manifest(&root.join("Cargo.toml"));

    let rust = manifest
        .get("workspace")
        .and_then(|w| w.get("lints"))
        .and_then(|l| l.get("rust"))
        .and_then(|r| r.as_table())
        .expect(
            "root Cargo.toml has [workspace.lints.rust] — every member inherits \
             it, so deleting it disarms the whole workspace",
        );

    for (lint, level) in REQUIRED_RUST_LINTS {
        let got = rust
            .get(*lint)
            .unwrap_or_else(|| panic!("[workspace.lints.rust] is missing `{lint}`"));
        // A lint is either a bare level string or a `{ level = "..." }` table.
        let got_level = got
            .as_str()
            .or_else(|| got.get("level").and_then(|l| l.as_str()))
            .unwrap_or_else(|| panic!("[workspace.lints.rust].{lint} has no level"));
        assert_eq!(
            got_level, *level,
            "[workspace.lints.rust].{lint} is `{got_level}`, but CLAUDE.md's \
             linting policy requires `{level}`. Every workspace member \
             inherits this table, so weakening one line here weakens all of \
             them at once."
        );
    }

    let clippy = manifest
        .get("workspace")
        .and_then(|w| w.get("lints"))
        .and_then(|l| l.get("clippy"))
        .and_then(|c| c.as_table())
        .expect("root Cargo.toml has [workspace.lints.clippy]");
    let all_entry = clippy
        .get("all")
        .expect("[workspace.lints.clippy] declares `all`");
    // Either a bare level string or a `{ level = "...", priority = N }` table —
    // the group MUST take the table form the moment any individual clippy lint
    // sits beside it (cargo ignores table order, so equal priorities error).
    let all = all_entry
        .as_str()
        .or_else(|| all_entry.get("level").and_then(|l| l.as_str()))
        .expect("[workspace.lints.clippy].all has no level");
    assert!(
        all == "warn" || all == "deny",
        "[workspace.lints.clippy].all is `{all}`; CI runs clippy with \
         `-D warnings`, so anything below `warn` silently disarms it"
    );
}

#[test]
fn every_mirrored_workspace_carries_the_same_lint_table_as_the_root() {
    let root = repo_root();
    let root_table = workspace_lints_table(&root, "")
        .expect("root Cargo.toml has [workspace.lints] — the other assertions here rest on it");

    for (workspace, reason) in MIRRORED_LINT_WORKSPACES {
        let mirrored = workspace_lints_table(&root, workspace).unwrap_or_else(|| {
            panic!(
                "{workspace}/Cargo.toml has no [workspace.lints] table.\n\nIt \
                 MIRRORS the root's by design: {reason}. Without it, that shared \
                 member's `[lints] workspace = true` HARD-FAILS the manifest load \
                 from this side (`error: failed to load manifest for workspace \
                 member`), which takes the whole workspace down rather than \
                 merely weakening a lint.\n\nFIX: copy the [workspace.lints.rust] \
                 and [workspace.lints.clippy] tables from Cargo.toml."
            )
        });

        // Semantic equality, NOT bytes: comments, key order and line wrapping may
        // differ between the two files; the lints they declare may not. Equality
        // also means the root table's STRENGTH pin above transfers here, so the
        // mirror cannot be strong-looking and weak.
        assert_eq!(
            mirrored, root_table,
            "the [workspace.lints] tables in Cargo.toml and \
             {workspace}/Cargo.toml have DRIFTED.\n\n{workspace}'s table MIRRORS \
             the root's by design: {reason}. A member loaded from both sides must \
             see the SAME lints whichever workspace loads it, or the strictness of \
             its build depends on who built it.\n\n  root:      {root_table}\n  \
             {workspace}: {mirrored}\n\nFIX: make them match. Formatting may \
             differ; content may not."
        );
    }
}

#[test]
fn every_mirrored_workspace_member_inherits_its_own_lint_table() {
    let root = repo_root();
    let allowed: BTreeSet<&str> = MIRRORED_MEMBERS_NON_INHERITING
        .iter()
        .map(|(p, _)| *p)
        .collect();

    for (workspace, _) in MIRRORED_LINT_WORKSPACES {
        let members = declared_members_of(&root, workspace);
        // Anti-tautology: an empty expansion would make the loop below vacuous.
        assert!(
            members.len() >= 2,
            "{workspace} expanded to {} member(s) — the walk is not reading its \
             [workspace] members, so this arm proves nothing",
            members.len()
        );

        let missing: Vec<&String> = members
            .iter()
            .filter(|m| !inherits_workspace_lints(&root, m))
            .filter(|m| !allowed.contains(m.as_str()))
            .collect();
        assert!(
            missing.is_empty(),
            "these members of the MIRRORED workspace `{workspace}` do not \
             inherit its [workspace.lints] table:\n  {}\n\nThe table was added \
             so a cross-workspace member could inherit it, but a table only ONE \
             member opts into is inert for every other crate in that \
             workspace — they sit under no lint table at all, which is the \
             condition the hoist was done to end.\n\nFIX: add\n\n    [lints]\n    \
             workspace = true\n\nto each manifest.",
            missing
                .iter()
                .map(|m| format!("{m}/Cargo.toml"))
                .collect::<Vec<_>>()
                .join("\n  "),
        );
    }

    let stale: Vec<&str> = allowed
        .iter()
        .filter(|d| inherits_workspace_lints(&root, d))
        .copied()
        .collect();
    assert!(
        stale.is_empty(),
        "MIRRORED_MEMBERS_NON_INHERITING names members that DO inherit: \
         {stale:?} — delete the entries from {}.",
        file!()
    );
}

#[test]
fn every_cross_workspace_member_has_its_workspace_declared() {
    let root = repo_root();
    let declared: BTreeSet<String> = declared_members(&root).into_iter().collect();
    let known: BTreeSet<&str> = MIRRORED_LINT_WORKSPACES.iter().map(|(w, _)| *w).collect();

    // DERIVE which workspaces belong in MIRRORED_LINT_WORKSPACES rather than
    // trusting the hand list: any member cargo resolves that the root does not
    // declare is owned by some OTHER workspace, and that workspace needs the
    // mirrored table (and its own members covered).
    let cross: Vec<String> = resolved_members(&root)
        .into_iter()
        .filter(|m| !declared.contains(m))
        .collect();

    // LOUD if the shape cargo produces ever changes: today exactly one member is
    // folded in this way, and the arms above are written around that. A cargo
    // that stopped folding foreign path-deps would empty this set and make the
    // resolved arm pass while covering nothing — fail instead of quietly
    // shrinking.
    assert!(
        !cross.is_empty(),
        "cargo now resolves NO members outside the root's declared list.\n\n\
         That used to include `examples/go2/nodes/go2_tf_source`, folded in through \
         `cerulion_viz`'s path dev-dependency. If cargo changed how it folds \
         foreign-workspace path dependencies, then \
         `every_member_cargo_resolves_inherits_the_lint_table` now covers \
         strictly less than it did while still passing, and that crate is \
         covered ONLY by the mirrored-workspace arm. Re-check both arms \
         deliberately rather than deleting this assertion."
    );

    for member in &cross {
        let owner = owning_workspace(&root, member).unwrap_or_else(|| {
            panic!(
                "cargo resolves `{member}`, which the root does not declare, and \
                 no ancestor of it declares a [workspace] either — so no \
                 workspace's lint table governs it."
            )
        });
        assert!(
            known.contains(owner.as_str()),
            "`{member}` is resolved into the ROOT workspace but declared by \
             `{owner}`, which is NOT in MIRRORED_LINT_WORKSPACES.\n\nA crate \
             loaded by two workspaces resolves `[lints] workspace = true` against \
             whichever is LOADING, so `{owner}` needs a [workspace.lints] table \
             mirroring the root's — and its OTHER members need to inherit it, or \
             that table is inert for them.\n\nFIX: add `{owner}` to \
             MIRRORED_LINT_WORKSPACES in {} with a reason.",
            file!()
        );
    }
}

#[test]
fn every_member_cargo_resolves_inherits_the_lint_table() {
    let root = repo_root();
    let declared: BTreeSet<String> = declared_members(&root).into_iter().collect();
    let resolved = resolved_members(&root);

    // Anti-tautology that DISCRIMINATES: `resolved >= declared` is satisfied by a
    // parse that returned the declared list verbatim, which is exactly the
    // failure this arm exists to catch. Cargo folds at least one foreign
    // path-dependency in, so the resolved set must be strictly LARGER.
    assert!(
        resolved.len() > declared.len(),
        "cargo resolves {} members and the root declares {} — this arm only \
         means something if the resolved set is strictly larger (it is what \
         catches path-dependency-only members). An equal count means the \
         metadata parse is returning the declared list, so the gate proves \
         nothing.",
        resolved.len(),
        declared.len()
    );

    let allowed: BTreeSet<&str> = RESOLVED_ONLY_NON_INHERITING
        .iter()
        .map(|(p, _)| *p)
        .collect();
    assert_eq!(
        allowed.len(),
        RESOLVED_ONLY_NON_INHERITING.len(),
        "RESOLVED_ONLY_NON_INHERITING contains a duplicate path"
    );

    let undeclared: Vec<&String> = resolved.iter().filter(|m| !declared.contains(*m)).collect();

    let missing: Vec<&String> = resolved
        .iter()
        .filter(|m| !inherits_workspace_lints(&root, m))
        .filter(|m| !allowed.contains(m.as_str()))
        .collect();
    assert!(
        missing.is_empty(),
        "these members CARGO RESOLVES do not inherit the shared lint table, so \
         CLAUDE.md's strict policy does not apply to them:\n  {}\n\nOf the {} \
         members cargo resolves, {} are NOT named in the root `[workspace] \
         members` list ({:?}) — cargo adds a path dependency to the workspace \
         whether or not the list mentions it, so the declared-members walk in \
         this same file cannot see them.\n\nFIX: add\n\n    [lints]\n    \
         workspace = true\n\nto each manifest. If the crate is ALSO a member of \
         another workspace, that workspace needs its own mirrored \
         [workspace.lints] table (see MIRRORED_LINT_WORKSPACES) or the line \
         fails the load from that side.",
        missing
            .iter()
            .map(|m| format!("{m}/Cargo.toml"))
            .collect::<Vec<_>>()
            .join("\n  "),
        resolved.len(),
        undeclared.len(),
        undeclared,
    );

    let stale: Vec<&str> = allowed
        .iter()
        .filter(|d| !resolved.iter().any(|m| m == *d) || inherits_workspace_lints(&root, d))
        .copied()
        .collect();
    assert!(
        stale.is_empty(),
        "RESOLVED_ONLY_NON_INHERITING names members that DO inherit the shared \
         table (or that cargo no longer resolves): {stale:?}. An exemption that \
         describes nothing is a pre-authorised future opt-out — delete the \
         entries from {}.",
        file!()
    );
}
