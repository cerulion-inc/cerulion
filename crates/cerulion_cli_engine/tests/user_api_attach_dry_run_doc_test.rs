// SPDX-License-Identifier: AGPL-3.0-only
//! The `ros2 attach` row in `docs/user-api.md` is where a user reads what a
//! `--dry-run` outside a workspace needs. The verb creates one empty temporary
//! directory for that run and refuses, naming two remedies, when it cannot; a
//! row that said only "it does not require a workspace" left the refusal
//! undocumented, so a host with an unwritable temporary directory reads as a
//! bug in the verb.
//!
//! The remedy sentence is DERIVED from the constant both refusal arms
//! interpolate (`ros_attach_root::DRY_RUN_ROOT_REMEDIES`), never hand-copied,
//! so a reworded refusal fails this arm instead of leaving the row quietly
//! wrong. A CONTENT pin, not a formatting one: the page is hard-wrapped prose,
//! so the row is whitespace-normalized before it is read.
//!
//! Parallel-safe: one file read, no transport, no process spawn.

use cerulion_cli_engine::ros_attach_root::DRY_RUN_ROOT_REMEDIES;

/// The `ros2 attach` row, whitespace-normalized, REQUIRED to be the only one.
///
/// ANTI-TAUTOLOGY: with no reachable row every assertion below is vacuous
/// against a renamed or deleted row, and with two rows the one this arm does
/// not read is checked by nothing.
fn attach_row() -> String {
    let raw = std::fs::read_to_string(
        std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("../../docs/user-api.md"),
    )
    .expect("docs/user-api.md must be readable from the crate root");
    let rows: Vec<&str> = raw
        .lines()
        .filter(|line| line.starts_with("| `ros2 attach "))
        .collect();
    let [row] = rows.as_slice() else {
        panic!(
            "docs/user-api.md must carry EXACTLY ONE `ros2 attach` row; found {}",
            rows.len()
        );
    };
    row.split_whitespace().collect::<Vec<_>>().join(" ")
}

#[test]
fn the_attach_row_states_what_a_workspace_less_dry_run_needs() {
    let row = attach_row();
    assert!(
        row.contains("it does not require a workspace"),
        "the row must keep saying a dry-run needs no workspace; row:\n{row}"
    );
    assert!(
        row.contains("creates an empty one under `TMPDIR` and removes it again at exit"),
        "the row must state the temporary directory a workspace-less dry-run \
         creates and removes; row:\n{row}"
    );
    assert!(
        row.contains(DRY_RUN_ROOT_REMEDIES),
        "the row must carry the refusal's remedies verbatim \
         ({DRY_RUN_ROOT_REMEDIES:?}); row:\n{row}"
    );
}
