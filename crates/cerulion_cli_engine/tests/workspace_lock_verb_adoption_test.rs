// SPDX-License-Identifier: AGPL-3.0-only
//! The verbs that OWN `<root>/.cerulion` must take the constructor that says so.
//!
//! The `.gitignore` top-up moved out of the default-named
//! `WorkspaceLock::acquire` and into `acquire_and_track_gitignore`, and
//! every workspace-owning verb moved onto the latter. The MOVE was pinned by nothing:
//! the wrong choice is the SHORTER name, it is what every one of those lines
//! used to say, and reverting any of them compiles and passes the rest of the
//! suite. `ros2 migrate`'s choice has a source walk guarding it
//! (`migrate_takes_the_interruptible_constructor_and_neither_blocking_one`);
//! this is the same guard for the other direction.
//!
//! TWO layers, because neither alone covers the set:
//!
//! * BEHAVIOURAL, over the verbs that are cheap to drive end to end. The
//!   observable is the `.gitignore` top-up itself, through the REAL verbs — not
//!   a lock call in a fixture — because that is the user-visible consequence: a
//!   workspace scaffolded before the lock existed must gain `.cerulion/` in its
//!   ignore file the first time a Cerulion verb mutates it, or the lock
//!   directory shows up in `git status` forever.
//! * STRUCTURAL, over ALL of them. `graph partition`'s write, `ros attach`'s
//!   consent batch and `cerulion-wsd`'s mutation dispatch need scaffolding out
//!   of proportion to what they would prove here, and a behavioural arm that
//!   covers three of eight sites while being named for all of them is worse
//!   than no arm — a later reader trusts it. The walk is total and cannot go
//!   stale: `WorkspaceLock::acquire` has NO shipped caller, so "no verb file
//!   names the default constructor" is exactly the invariant.

#![cfg(unix)]

use cerulion_cli_engine::workspace_lock::LOCK_DIR;
use cerulion_cli_engine::{graph_cmd, node_cmd, schema_cmd, workspace};
use cerulion_core::graph::config::NodeDef;
use std::path::Path;

/// A workspace in the PRE-LOCK shape: scaffolded, then rewound so `.gitignore`
/// does not mention `.cerulion/` and the lock directory does not exist.
///
/// `workspace create` writes the entry itself, so without the rewind every
/// assertion below would pass on a build where no verb tops up anything.
fn pre_lock_workspace(parent: &Path, name: &str) -> std::path::PathBuf {
    let ws = workspace::workspace_create(parent, name)
        .expect("scaffold a workspace")
        .root;
    std::fs::write(ws.join(".gitignore"), "target/\n").expect("rewind .gitignore");
    let lock_dir = ws.join(LOCK_DIR);
    if lock_dir.exists() {
        std::fs::remove_dir_all(&lock_dir).expect("rewind the lock directory");
    }
    ws
}

fn gitignore(ws: &Path) -> String {
    std::fs::read_to_string(ws.join(".gitignore")).expect("read .gitignore")
}

#[test]
fn the_cheaply_drivable_verbs_top_up_a_pre_lock_gitignore() {
    let temp = tempfile::tempdir().unwrap();

    // Each verb gets its OWN pre-lock workspace: the top-up is tied to CREATING
    // `.cerulion/`, so a shared fixture would let the first verb satisfy the
    // assertion for all three and hide two reverts.
    let ws = pre_lock_workspace(temp.path(), "by_node_create");
    node_cmd::node_create(&ws.join("nodes"), &ws.join("Cargo.toml"), "camera", None)
        .expect("node create");
    assert_eq!(
        gitignore(&ws),
        "target/\n.cerulion/\n",
        "`node create` must take the constructor that tracks `.gitignore` — reverting it to \
         the default-named `WorkspaceLock::acquire` leaves `{LOCK_DIR}/` untracked forever"
    );

    let ws = pre_lock_workspace(temp.path(), "by_schema_create");
    schema_cmd::schema_create(&ws.join("schemas"), "widget").expect("schema create");
    assert_eq!(
        gitignore(&ws),
        "target/\n.cerulion/\n",
        "`schema create` must take the tracking constructor"
    );

    let ws = pre_lock_workspace(temp.path(), "by_graph_create");
    graph_cmd::graph_create(&ws.join("graphs"), "perception", None).expect("graph create");
    assert_eq!(
        gitignore(&ws),
        "target/\n.cerulion/\n",
        "`graph create` must take the tracking constructor"
    );

    let ws = pre_lock_workspace(temp.path(), "by_schema_delete");
    schema_cmd::schema_create(&ws.join("schemas"), "widget").expect("schema create");
    std::fs::write(ws.join(".gitignore"), "target/\n").expect("rewind .gitignore");
    std::fs::remove_dir_all(ws.join(LOCK_DIR)).expect("rewind the lock directory");
    schema_cmd::schema_delete(&ws.join("schemas"), "widget").expect("schema delete");
    assert_eq!(
        gitignore(&ws),
        "target/\n.cerulion/\n",
        "`schema delete` is a workspace WRITER too — the second lock site in that file"
    );

    let ws = pre_lock_workspace(temp.path(), "by_node_stage");
    node_cmd::node_create(&ws.join("nodes"), &ws.join("Cargo.toml"), "camera", None)
        .expect("node create");
    graph_cmd::graph_create(&ws.join("graphs"), "perception", None).expect("graph create");
    std::fs::write(ws.join(".gitignore"), "target/\n").expect("rewind .gitignore");
    std::fs::remove_dir_all(ws.join(LOCK_DIR)).expect("rewind the lock directory");
    graph_cmd::node_stage(
        &ws.join("graphs"),
        "perception",
        NodeDef {
            fuse: None,
            ros2: None,
            id: "camera".to_string(),
            node_type: "camera".to_string(),
            inputs: vec![],
            outputs: vec![],
        },
    )
    .expect("node stage");
    assert_eq!(
        gitignore(&ws),
        "target/\n.cerulion/\n",
        "`node stage` is the OTHER lock site in graph_cmd.rs"
    );
}

/// The TOTAL half. Five of the eight tracking call sites are impractical to
/// drive end to end here; this covers all eight at once, by the only invariant
/// that is exactly equivalent — `WorkspaceLock::acquire` has no shipped caller,
/// so any verb file naming it has taken the wrong constructor.
///
/// Comment-stripped, because these files discuss the constructors in prose (the
/// walk would otherwise fail on an explanation rather than on code), and with
/// an ANTI-TAUTOLOGY assertion per file: a stripper that ate the input would
/// make every "does not contain" check vacuous.
#[test]
fn no_workspace_owning_verb_names_the_default_constructor() {
    let engine = Path::new(env!("CARGO_MANIFEST_DIR"));
    let files = [
        engine.join("src/node_cmd.rs"),
        engine.join("src/graph_cmd.rs"),
        engine.join("src/schema_cmd.rs"),
        engine.join("src/ros_cmd.rs"),
        engine.join("src/partition_emit.rs"),
        engine.join("../cerulion_wsd/src/protocol.rs"),
    ];
    for path in files {
        let source = std::fs::read_to_string(&path)
            .unwrap_or_else(|error| panic!("{} is readable: {error}", path.display()));
        let code = code_only(&source);
        assert!(
            code.contains("acquire_and_track_gitignore("),
            "{}: the comment-stripped view lost the call site — the walk proves nothing",
            path.display()
        );
        assert!(
            !code.contains("WorkspaceLock::acquire("),
            "{}: calls `WorkspaceLock::acquire(`, the DEFAULT-named constructor, which no \
             longer tops up `<root>/.gitignore`. A verb that owns `<root>/.cerulion` must \
             call `acquire_and_track_gitignore`, or a workspace scaffolded before the lock \
             existed keeps an untracked `.cerulion/` forever.",
            path.display()
        );
    }
}

/// Strip `//` and nested `/* */` comments AND string literals, keeping newlines
/// so a failure still points at a line.
///
/// String literals are stripped for a reason this walk found the hard way: a
/// FIRST draft stripped only comments, and `schema_cmd.rs` contains the string
/// `"… looked up by schemas/*.yaml file stem …"`. That `/*` opened a block
/// comment that never closed, so the stripper swallowed the rest of the file
/// and every "does not contain" assertion below went vacuous — caught only by
/// the anti-tautology assertion, which is exactly what it is there for.
///
/// Raw strings are handled too (`r"…"`, `r#"…"#`), since a hash-delimited raw
/// string can contain an unescaped quote.
fn code_only(src: &str) -> String {
    let chars: Vec<char> = src.chars().collect();
    let mut out = String::with_capacity(src.len());
    let mut i = 0;
    let keep_newlines = |out: &mut String, from: usize, to: usize, chars: &[char]| {
        for c in &chars[from..to] {
            if *c == '\n' {
                out.push('\n');
            }
        }
    };
    while i < chars.len() {
        // Block comment, depth-tracked: Rust's NEST.
        if chars[i] == '/' && chars.get(i + 1) == Some(&'*') {
            let start = i;
            let mut depth = 1usize;
            i += 2;
            while i < chars.len() && depth > 0 {
                if chars[i] == '/' && chars.get(i + 1) == Some(&'*') {
                    depth += 1;
                    i += 2;
                } else if chars[i] == '*' && chars.get(i + 1) == Some(&'/') {
                    depth -= 1;
                    i += 2;
                } else {
                    i += 1;
                }
            }
            keep_newlines(&mut out, start, i, &chars);
            continue;
        }
        if chars[i] == '/' && chars.get(i + 1) == Some(&'/') {
            while i < chars.len() && chars[i] != '\n' {
                i += 1;
            }
            continue;
        }
        // Raw string: `r`, some `#`s, then the quote.
        if chars[i] == 'r' {
            let mut j = i + 1;
            let mut hashes = 0usize;
            while chars.get(j) == Some(&'#') {
                hashes += 1;
                j += 1;
            }
            if chars.get(j) == Some(&'"') {
                let start = i;
                i = j + 1;
                loop {
                    if i >= chars.len() {
                        break;
                    }
                    if chars[i] == '"' {
                        let closes = (1..=hashes).all(|k| chars.get(i + k) == Some(&'#'));
                        if closes {
                            i += hashes + 1;
                            break;
                        }
                    }
                    i += 1;
                }
                keep_newlines(&mut out, start, i, &chars);
                continue;
            }
        }
        if chars[i] == '"' {
            let start = i;
            i += 1;
            while i < chars.len() {
                if chars[i] == '\\' {
                    i += 2;
                    continue;
                }
                if chars[i] == '"' {
                    i += 1;
                    break;
                }
                i += 1;
            }
            keep_newlines(&mut out, start, i, &chars);
            continue;
        }
        out.push(chars[i]);
        i += 1;
    }
    out
}

/// The stripper, on inputs of its own — against the real files an IDENTITY
/// stripper yields a passing walk, so the walk cannot test this.
#[test]
fn the_stripper_removes_comments_and_string_literals_and_nothing_else() {
    assert_eq!(
        code_only("a // b\nc"),
        "a \nc",
        "line comment to end of line"
    );
    assert_eq!(code_only("a /* b */ c"), "a  c", "block comment");
    assert_eq!(
        code_only("a /* b /* c */ d */ e"),
        "a  e",
        "Rust block comments NEST — a depth-blind stripper leaves ` d */ e`"
    );
    assert_eq!(
        code_only("// /* a\nb"),
        "\nb",
        "a block opener inside a line comment opens nothing"
    );
    // THE ONE THAT BIT. `schema_cmd.rs` really contains this string, and a
    // comments-only stripper read its `/*` as a block opener, swallowed the rest
    // of the file, and made every assertion in the walk vacuous — caught by the
    // walk's own anti-tautology assertion, which is what it is there for.
    assert_eq!(
        code_only("let s = \"schemas/*.yaml file\";\nWorkspaceLock::acquire_and_track_gitignore("),
        "let s = ;\nWorkspaceLock::acquire_and_track_gitignore(",
        "a `/*` inside a STRING opens no comment"
    );
    assert_eq!(
        code_only("let s = \"a \\\" /* b\";\nc"),
        "let s = ;\nc",
        "an escaped quote does not end the string, so the `/*` after it stays inert"
    );
    assert_eq!(
        code_only("let s = r#\"a \" /* b\"#;\nc"),
        "let s = ;\nc",
        "a raw string ends only at its own hash count"
    );
    assert_eq!(
        code_only("let c = '/';\nWorkspaceLock::acquire("),
        "let c = '/';\nWorkspaceLock::acquire(",
        "a char literal is ordinary code — stripping it would risk eating a lifetime, and \
         `'/'` cannot open a comment because the next char is a quote"
    );
    assert_eq!(
        code_only(
            "/// names WorkspaceLock::acquire(\nWorkspaceLock::acquire_and_track_gitignore(&ws"
        ),
        "\nWorkspaceLock::acquire_and_track_gitignore(&ws",
        "the exact shape the walk depends on: prose naming the forbidden form is stripped \
         while the real call site survives"
    );
}

/// ANTI-TAUTOLOGY, and it is the half that makes the arm above mean something.
///
/// Without it, "the entry is present afterwards" would pass on a build that
/// wrote `.cerulion/` into every `.gitignore` unconditionally, or one that
/// appended a second copy. (What it does NOT cover is a build where `workspace
/// create`'s own entry did the work — this test plants the entry by hand. That
/// variant is excluded by the shared fixture's rewind, in the test above.)
#[test]
fn a_workspace_that_already_ignores_the_lock_directory_is_left_byte_identical() {
    let temp = tempfile::tempdir().unwrap();
    let ws = pre_lock_workspace(temp.path(), "already_ignoring");
    std::fs::write(ws.join(".gitignore"), "target/\n.cerulion/\n").unwrap();

    node_cmd::node_create(&ws.join("nodes"), &ws.join("Cargo.toml"), "camera", None)
        .expect("node create");
    assert_eq!(
        gitignore(&ws),
        "target/\n.cerulion/\n",
        "the top-up is idempotent: a workspace that already ignores the lock directory must \
         come back byte-identical, not with a second entry"
    );

    // And a workspace with NO `.gitignore` must not grow one: the engine never
    // invents a file the user did not ask for.
    let bare = pre_lock_workspace(temp.path(), "no_gitignore");
    std::fs::remove_file(bare.join(".gitignore")).unwrap();
    node_cmd::node_create(
        &bare.join("nodes"),
        &bare.join("Cargo.toml"),
        "camera",
        None,
    )
    .expect("node create");
    assert!(
        !bare.join(".gitignore").exists(),
        "a workspace without a `.gitignore` must not be given one"
    );
    assert!(
        bare.join(LOCK_DIR).exists(),
        "…but it must still have taken the lock"
    );
}
