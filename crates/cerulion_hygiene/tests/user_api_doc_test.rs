// SPDX-License-Identifier: AGPL-3.0-only
//! The socket-directory rule is a USER-facing contract — it can chmod a
//! directory of the user's, and it can refuse to start a daemon — and
//! `USER_API.md` is where a user reads it. The rule itself is pinned by the
//! oracle vectors in `socket_dir_verdict`'s own tests; what those cannot see
//! is the DOCUMENTATION drifting away from them, which is how a user learns a
//! behaviour the code no longer has.
//!
//! So this file pins the four arms of the rule as PHRASES in the environment
//! table's socket-path cell: the sticky-shared accept (scoped to a share owned
//! by root or by the user), the accept-as-is of a directory of the user's that
//! nobody else can write into (the commonest outcome of all, and the one the
//! default ladder produces), the automatic tightening, the refusal of a
//! directory of ours whose bits cannot be stripped, and the refusal of
//! everything else — including another user's `1777` share, whose owner the
//! sticky bit does not bind. Each is a behaviour a user can be surprised by,
//! and each is asserted with the reason it matters, so a failure here reads as
//! "the docs no longer describe the code" rather than as a spelling complaint.
//! Rewording is fine — updating this list is the cost of rewording a security
//! contract.
//!
//! The rule has exactly TWO statements: `cerulion_hygiene`'s module docs
//! (code-side, where it is decided) and this cell (user-facing, where a user
//! reads it). This file is what keeps them in agreement — every other file
//! POINTS at one of the two rather than carrying a third copy, which
//! `cerulion_wsd`'s `no_wsd_doc_restates_the_socket_directory_rule` enforces
//! for the daemon that had grown one.

use std::path::PathBuf;

/// `docs/user-api.md` sits under the workspace root, two levels up from this crate.
fn user_api() -> String {
    let path = PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .parent()
        .expect("the crate directory has a parent")
        .parent()
        .expect("crates/ has a parent (the repo root)")
        .join("docs/user-api.md");
    std::fs::read_to_string(&path).unwrap_or_else(|e| {
        panic!(
            "could not read {} ({e}) — this test pins the socket-directory rule \
             against the document users read; it must fail rather than skip",
            path.display()
        )
    })
}

/// The ONE line of `doc` that starts with `needle` — the environment table's
/// row for that variable, found by its variable rather than by line number.
///
/// EXACTLY one, not the first: a `find` takes the earliest match and lets a
/// stale duplicate row sit below it, which is precisely the shape a table edit
/// leaves behind. Every phrase below would then be checked against a row that
/// is still correct while a user reading the second one is told something the
/// code no longer does — the gate green, the document wrong, which is the whole
/// failure mode this file exists to catch.
fn select_unique_row<'a>(doc: &'a str, needle: &str) -> &'a str {
    let rows: Vec<&'a str> = doc
        .lines()
        .filter(|line| line.starts_with(needle))
        .collect();
    assert_eq!(
        rows.len(),
        1,
        "USER_API.md must carry EXACTLY ONE row starting {needle:?} — its row in the \
         environment table, where the socket-directory rule is documented (grep for it). \
         Found {} of them; a second copy drifts from the first, and this gate would only \
         ever read the first.\nrows: {rows:#?}",
        rows.len()
    );
    rows[0]
}

/// The row of the environment table that documents the socket path and the
/// directory rule.
fn socket_dir_cell(doc: &str) -> &str {
    select_unique_row(doc, "| `CERULION_NETD_SOCK`")
}

#[test]
fn the_user_api_socket_directory_cell_states_every_arm_of_the_rule() {
    let doc = user_api();
    let cell = socket_dir_cell(&doc);

    for (phrase, why) in [
        (
            "sticky, world-writable directory (mode `1777`) owned by root or by you is \
             accepted as is",
            "the sticky-shared accept, stated PRECISELY: sticky AND world-writable (a \
             group-shared `1770` sticky directory is not this arm), and scoped to the two \
             owners the sticky bit leaves us able to trust. A root-run daemon must still \
             not decide on ownership first, or it would re-mode the machine's /tmp",
        ),
        (
            "directory of yours that no other user can write into is used as is",
            "the accept-as-is arm — the outcome the default ladder's own `0700` rungs \
             produce, and the one a cell that listed only accept-sticky / tighten / \
             refuse left a `0700` directory of the user's covered by no sentence at all",
        ),
        (
            "is `0775` and is tightened once, with the warn",
            "the default ladder is NOT always silent: a `~/.cerulion` another Cerulion \
             command made under Ubuntu's default umask 0002 is 0775 and gets tightened, so \
             the cell must not promise \"nothing is logged\" for every rung",
        ),
        (
            "a search-only `0300` directory of yours is refused",
            "the read-bit requirement the fd-based check introduced: the directory is opened \
             O_RDONLY|O_NOFOLLOW|O_DIRECTORY and judged through that descriptor, so a \
             search-only directory that the old path-based stat accepted is now refused — a \
             behaviour change to a documented env var, which must be stated where the env var \
             is documented",
        ),
        (
            "tightened automatically",
            "the tighten arm: the daemon changes a directory of the user's",
        ),
        (
            "the write bits are stripped, sticky and setgid bits kept",
            "exactly WHICH bits change, so the user can predict the result",
        ),
        (
            "one `warn!` line says so",
            "the tightening is announced — the only signal the user gets",
        ),
        (
            "could **not** be stripped",
            "the arm where a daemon that used to start now REFUSES (a fixed-mode \
             filesystem such as a CIFS home)",
        ),
        (
            "neither yours nor a root-owned `1777` share",
            "the foreign-directory refusal",
        ),
        (
            "its owner can unlink your socket and bind an impostor",
            "the residual the sticky bit does NOT close — why another user's `1777` \
             share is refused rather than accepted like /tmp. Documenting the accept \
             without this taught the user that any sticky share was safe",
        ),
        (
            "created `0700`",
            "what happens when the directory does not exist yet",
        ),
    ] {
        assert!(
            cell.contains(phrase),
            "USER_API.md's socket-directory cell no longer states {why}.\n\
             missing phrase: {phrase:?}\n\
             cell: {cell}"
        );
    }
}

/// The `cerulion-wsd` row must keep pointing at the same rule rather than
/// growing a second, drifting copy of it: three daemons with three different
/// documented rules is the state the shared crate exists to end.
#[test]
fn the_wsd_row_defers_to_the_one_rule_instead_of_restating_it() {
    let doc = user_api();
    let row = select_unique_row(&doc, "| `CERULION_WSD_SOCKET`");
    assert!(
        row.contains("the same ladder and directory rule as the row above"),
        "the wsd row must defer to the shared rule, not restate it: {row}"
    );
    assert!(
        !row.contains("tightened"),
        "a second copy of the rule in the wsd row is exactly the drift \
         `cerulion_hygiene` was extracted to prevent: {row}"
    );
}

/// `select_unique_row`'s own oracle, on a HAND-BUILT document rather than
/// `USER_API.md`: the real file has one row of each today, so nothing here
/// could otherwise tell "took the first" from "required the only one", and a
/// duplicate cannot be demonstrated by editing the document the other pins
/// read.
///
/// The two rows differ in their CELL, not their key — a stale copy left behind
/// by a table edit says something different, which is exactly why reading only
/// the first is unsafe.
#[test]
fn a_duplicate_environment_row_fails_the_gate_instead_of_being_shadowed() {
    let fresh = "| `CERULION_NETD_SOCK` | tightened automatically |";
    let stale = "| `CERULION_NETD_SOCK` | refused outright |";
    let key = "| `CERULION_NETD_SOCK`";

    // One row: returned, and it is the row itself (not a prefix or a trim).
    let single = format!("| var | meaning |\n{fresh}\n| other | x |\n");
    assert_eq!(select_unique_row(&single, key), fresh);

    // Two rows: a FAILURE. Reading the first would find every phrase it
    // expects in `fresh` and never look at `stale`.
    let doubled = format!("| var | meaning |\n{fresh}\n| other | x |\n{stale}\n");
    let message = panic_message(|| {
        let _ = select_unique_row(&doubled, key);
    });
    assert!(
        message.contains("EXACTLY ONE"),
        "the refusal must say what it wanted: {message}"
    );
    for row in [fresh, stale] {
        assert!(
            message.contains(row),
            "and print BOTH rows, so a maintainer sees which copy drifted — missing \
             {row:?}: {message}"
        );
    }

    // Zero rows: also a failure, and it keeps the original guidance.
    let missing = panic_message(|| {
        let _ = select_unique_row("| var | meaning |\n", key);
    });
    assert!(missing.contains("EXACTLY ONE"), "{missing}");
    assert!(
        missing.contains("grep for it"),
        "the missing-row refusal must still tell a maintainer how to find it: {missing}"
    );
}

/// The panic payload of `f`, which MUST panic. The process-global panic hook is
/// left alone: replacing it — even briefly — races every other test in this
/// binary (a concurrent failure lands in the muted window and reports a bare
/// FAILED with no message), and muting buys nothing anyway, because libtest
/// captures a passing test's stderr, so the expected panic's report is never
/// seen unless this test fails — exactly when it is wanted.
fn panic_message(f: impl FnOnce() + std::panic::UnwindSafe) -> String {
    let payload = std::panic::catch_unwind(f).expect_err(
        "`select_unique_row` must PANIC on this document — it returned a row instead, which \
         is the `find`-takes-the-first behaviour that lets a stale duplicate row keep this \
         gate green",
    );
    payload
        .downcast_ref::<String>()
        .cloned()
        .or_else(|| payload.downcast_ref::<&str>().map(|s| s.to_string()))
        .expect("a string panic payload")
}
