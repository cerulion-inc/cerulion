// SPDX-License-Identifier: AGPL-3.0-only
//! Four properties of the shipped tree that a vocabulary regex cannot see,
//! because none of them is a word: what a published crate PACKAGES, what a
//! file is NAMED, what environment a harness RUNS the binary in, and whether a
//! documented list still matches the code that decides it.
//!
//! They live together because they share one need, the repository root, and
//! one failure mode: each was enforced by a sentence in a document, and a
//! sentence does not fail a build. Each test names the document it is the
//! machine half of.
//!
//! No transport, no cargo, no network: every arm is a walk over the tracked
//! tree and a comparison against a list written out here by hand.

use std::collections::BTreeSet;
use std::path::{Path, PathBuf};

/// The workspace root, two levels up from this crate.
fn repo_root() -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .parent()
        .expect("the crate directory has a parent")
        .parent()
        .expect("crates/ has a parent (the repo root)")
        .to_path_buf()
}

fn read(path: &Path) -> String {
    std::fs::read_to_string(path).unwrap_or_else(|e| {
        panic!(
            "could not read {} ({e}): these tests pin the shipped tree and must \
             fail rather than skip",
            path.display()
        )
    })
}

/// A view of `text` with every `#` comment removed, so a rule can never be
/// satisfied (or broken) by prose ABOUT it. Quoting is honoured, because a `#`
/// inside a string is data, not a comment.
fn strip_hash_comments(text: &str) -> String {
    let mut out = String::with_capacity(text.len());
    for line in text.lines() {
        let mut quote: Option<char> = None;
        for ch in line.chars() {
            match quote {
                Some(q) => {
                    out.push(ch);
                    if ch == q {
                        quote = None;
                    }
                }
                None => {
                    if ch == '#' {
                        break;
                    }
                    if ch == '\'' || ch == '"' {
                        quote = Some(ch);
                    }
                    out.push(ch);
                }
            }
        }
        out.push('\n');
    }
    out
}

/// Every `"…"` in `text`, in order.
fn quoted(text: &str) -> Vec<String> {
    let mut out = Vec::new();
    let mut rest = text;
    while let Some(open) = rest.find('"') {
        let after = &rest[open + 1..];
        match after.find('"') {
            Some(close) => {
                out.push(after[..close].to_string());
                rest = &after[close + 1..];
            }
            None => break,
        }
    }
    out
}

// ---------------------------------------------------------------------------
// (a) No agent-instruction file can be packaged into a published crate.
// ---------------------------------------------------------------------------

/// The two file suffixes below are ASSEMBLED rather than written out, the way
/// `leak_scan.py` assembles every literal its own classes would match: this
/// file names the wording in order to refuse it, and the public-surface gate
/// scans test files too. Spelling them whole would cost an allow entry per
/// line and would make this file the only place they ship.
const DESIGN_SUFFIX: &str = concat!("-DESIGN", ".md");
const LOG_SUFFIX: &str = concat!("-RUNNING", "-LOG.md");

/// The names that serve work inside this repository and must never reach a
/// crates.io download. `.claude` and `notes` are directories; the rest are
/// files at a package root. The last two are stand-ins for a real name of that
/// shape, which is what the suffix check below covers.
fn never_packaged() -> Vec<String> {
    vec![
        "AGENTS.md".to_string(),
        "CLAUDE.md".to_string(),
        ".claude".to_string(),
        "notes".to_string(),
        format!("note{DESIGN_SUFFIX}"),
        format!("note{LOG_SUFFIX}"),
    ]
}

/// True when a cargo `include` entry would package `path` (a package-relative
/// path). Cargo matches gitignore-style: an entry naming a directory carries
/// everything under it, and `*` / `**` are globs.
fn include_entry_packages(entry: &str, path: &str) -> bool {
    let entry = entry.trim_end_matches('/');
    if entry.is_empty() {
        // `""`, `"/"`: cargo treats the package root as included, which is
        // exactly the shape this test exists to refuse.
        return true;
    }
    if entry.contains('*') || entry.contains('?') {
        return glob_matches(entry, path);
    }
    path == entry || path.starts_with(&format!("{entry}/"))
}

/// `**` matches any characters including `/`; `*` matches any run without a
/// `/`; `?` matches one character that is not a `/`.
fn glob_matches(pattern: &str, path: &str) -> bool {
    fn walk(p: &[char], s: &[char]) -> bool {
        if p.is_empty() {
            return s.is_empty();
        }
        if p[0] == '*' {
            if p.len() > 1 && p[1] == '*' {
                let rest = &p[2..];
                for i in 0..=s.len() {
                    if walk(rest, &s[i..]) {
                        return true;
                    }
                }
                return false;
            }
            let rest = &p[1..];
            for i in 0..=s.len() {
                if walk(rest, &s[i..]) {
                    return true;
                }
                if s[i..].first() == Some(&'/') {
                    break;
                }
            }
            return false;
        }
        if s.is_empty() {
            return false;
        }
        if p[0] == '?' && s[0] != '/' {
            return walk(&p[1..], &s[1..]);
        }
        if p[0] == s[0] {
            return walk(&p[1..], &s[1..]);
        }
        false
    }
    let p: Vec<char> = pattern.chars().collect();
    let s: Vec<char> = path.chars().collect();
    walk(&p, &s)
}

/// The `include = [...]` array of a manifest, or `None` when it declares none.
fn include_list(manifest: &str) -> Option<Vec<String>> {
    let src = strip_hash_comments(manifest);
    let at = if src.starts_with("include = ") {
        0
    } else {
        src.find("\ninclude = ")? + 1
    };
    let from = &src[at..];
    let end = from.find(']')?;
    Some(quoted(&from[..end]))
}

/// The workspace members, read from the root manifest the way `cargo
/// --workspace` reads them.
fn workspace_members(root: &Path) -> Vec<String> {
    let src = strip_hash_comments(&read(&root.join("Cargo.toml")));
    let at = src
        .find("members = [")
        .expect("the root manifest declares workspace members");
    let end = src[at..].find(']').expect("the members array is closed") + at;
    quoted(&src[at..end])
}

#[test]
fn no_published_crate_can_package_an_agent_instruction_file() {
    let root = repo_root();
    let mut checked = 0usize;
    for member in workspace_members(&root) {
        let dir = root.join(&member);
        let manifest_path = dir.join("Cargo.toml");
        if !manifest_path.is_file() {
            continue;
        }
        let manifest = read(&manifest_path);
        if strip_hash_comments(&manifest)
            .lines()
            .any(|l| l.trim() == "publish = false")
        {
            continue;
        }
        checked += 1;
        let include = include_list(&manifest).unwrap_or_else(|| {
            panic!(
                "{member}/Cargo.toml declares no `include`, so a `cargo publish` packages \
                 whatever the directory holds, AGENTS.md and CLAUDE.md included. Declare \
                 the list (see any sibling crate)."
            )
        });
        // The refusal: nothing in the list may reach one of these names.
        for forbidden in never_packaged() {
            for entry in &include {
                assert!(
                    !include_entry_packages(entry, &forbidden),
                    "{member}/Cargo.toml include entry {entry:?} packages {forbidden:?}. \
                     These files serve work inside this repository; a dependent building from \
                     a crates.io download never reads them."
                );
            }
        }
        // …and nothing under an included DIRECTORY may be one either, which a
        // list of root names cannot see.
        for entry in &include {
            let sub = dir.join(entry.trim_end_matches('/'));
            if sub.is_dir() {
                assert_no_forbidden_name_under(&sub, &member, entry);
            }
        }
        // The positive control. Without it a crate could pass by including
        // nothing at all, which packages nothing and proves nothing.
        for wanted in ["README.md", "src/lib.rs"] {
            if wanted == "src/lib.rs" && !dir.join(wanted).is_file() {
                continue; // a binary-only crate
            }
            assert!(
                include.iter().any(|e| include_entry_packages(e, wanted)),
                "{member}/Cargo.toml packages no {wanted}: the include list is {include:?}"
            );
        }
    }
    assert!(
        checked >= 15,
        "only {checked} publishable members were checked; the members list or the \
         `publish = false` spelling moved and this test is scanning almost nothing"
    );
}

fn assert_no_forbidden_name_under(dir: &Path, member: &str, entry: &str) {
    let Ok(read_dir) = std::fs::read_dir(dir) else {
        return;
    };
    for item in read_dir.flatten() {
        let name = item.file_name().to_string_lossy().to_string();
        assert!(
            !never_packaged().contains(&name)
                && !name.ends_with(DESIGN_SUFFIX)
                && !name.ends_with(LOG_SUFFIX),
            "{member} packages {entry:?}, which carries {}: an agent instruction file \
             inside a packaged directory ships with the crate",
            item.path().display()
        );
        if item.path().is_dir() {
            assert_no_forbidden_name_under(&item.path(), member, entry);
        }
    }
}

// ---------------------------------------------------------------------------
// (b) No source or test file is named after a plan step.
// ---------------------------------------------------------------------------

/// The file names the audit found, each carrying the step of the work that
/// produced it rather than the behaviour it covers. They are renamed after the
/// release, when the references (test maps, explicit cargo test targets, CI
/// selection) can move with them in one change.
///
/// THIS LIST MAY ONLY SHRINK. A new name that needs an entry here is a new
/// name that should have been written functionally in the first place.
const PLAN_STEP_NAMES_TO_RENAME: &[&str] = &[
    "chunk25b_live_default_iox2_test.rs",
    "chunk_a_bounds_test.rs",
    "chunk_c_ffi_codes_3_4_test.rs",
    "chunk_c_ffi_error_test.rs",
    "chunk_j_pass2_test.rs",
    "d2_sim_ns_warn_test.rs",
    "macro_chunks_ef_adversarial_test.rs",
];

/// Why `stem` reads as a plan step, or `None`.
///
/// The index is what separates a step from a word: `node_stage` is the `node
/// stage` VERB and `policy_round_trip` is a round trip, so `stage` and `round`
/// are tells only when a number follows them. `chunk` is a tell on its own
/// (`chunk_a`, `chunks_ef`, `chunk25b`), and a leading `<letter><digit>`
/// segment is a lane label (`d2_`), which `e2e_` is not.
fn plan_step_token(stem: &str) -> Option<String> {
    for (i, seg) in stem.split('_').enumerate() {
        let (word, index) =
            seg.split_at(seg.find(|c: char| c.is_ascii_digit()).unwrap_or(seg.len()));
        let indexed = !index.is_empty()
            && index
                .chars()
                .all(|c| c.is_ascii_digit() || c.is_ascii_lowercase());
        if word == "chunk" || word == "chunks" {
            return Some(seg.to_string());
        }
        if indexed && matches!(word, "pass" | "stage" | "phase" | "wave" | "round") {
            return Some(seg.to_string());
        }
        if i == 0
            && seg.len() <= 3
            && seg.starts_with(|c: char| c.is_ascii_lowercase())
            && seg[1..].chars().all(|c| c.is_ascii_digit())
            && seg.len() >= 2
        {
            return Some(seg.to_string());
        }
    }
    None
}

#[test]
fn no_source_or_test_file_is_named_after_a_plan_step() {
    let root = repo_root();
    let mut found: BTreeSet<String> = BTreeSet::new();
    let mut scanned = 0usize;
    collect_rs_files(&root.join("crates"), &mut |path| {
        let in_src_or_tests = path
            .components()
            .any(|c| c.as_os_str() == "src" || c.as_os_str() == "tests");
        if !in_src_or_tests {
            return;
        }
        scanned += 1;
        let name = path.file_name().unwrap().to_string_lossy().to_string();
        let stem = name.trim_end_matches(".rs");
        if let Some(token) = plan_step_token(stem) {
            found.insert(format!("{name} ({token})"));
        }
    });
    assert!(
        scanned > 500,
        "only {scanned} files were scanned; the walk found almost nothing and would pass \
         whatever the tree is named"
    );
    let declared: BTreeSet<String> = PLAN_STEP_NAMES_TO_RENAME
        .iter()
        .map(|n| {
            let token = plan_step_token(n.trim_end_matches(".rs"))
                .expect("every declared name really carries a plan-step token");
            format!("{n} ({token})")
        })
        .collect();
    let extra: Vec<&String> = found.difference(&declared).collect();
    assert!(
        extra.is_empty(),
        "these files are named after the step of the work that produced them, not the \
         behaviour they cover: {extra:?}. Name the file for what it exercises. The list in \
         this test may only shrink."
    );
    let gone: Vec<&String> = declared.difference(&found).collect();
    assert!(
        gone.is_empty(),
        "these names are in this test's list but not in the tree: {gone:?}. A renamed file \
         is the point: delete its line here."
    );
}

fn collect_rs_files(dir: &Path, f: &mut impl FnMut(&Path)) {
    let Ok(read_dir) = std::fs::read_dir(dir) else {
        return;
    };
    for item in read_dir.flatten() {
        let path = item.path();
        if path.is_dir() {
            if path.file_name().is_some_and(|n| n == "target") {
                continue;
            }
            collect_rs_files(&path, f);
        } else if path.extension().is_some_and(|e| e == "rs") {
            f(&path);
        }
    }
}

// ---------------------------------------------------------------------------
// (c) Everything that runs the binary outside cargo turns the login gate off.
// ---------------------------------------------------------------------------

/// The files that run the `cerulion` binary without going through cargo, so
/// the workspace `.cargo/config.toml` cannot reach them. Each must set the
/// gate itself. The rule is in `docs/internals/ci-and-gates.md`, "The login
/// gate in CI".
const GATE_CARRIERS: &[&str] = &[
    ".cargo/config.toml",
    ".github/workflows/examples-replay.yml",
    ".github/workflows/release-artifacts.yml",
    "benches/latency/check_percentile_parity.py",
    "benches/latency/workspace/run_workspace.sh",
    "tools/ros2_migrate/run_matrix.sh",
    "tools/scripts/verify_rmw_deb_container.sh",
];

/// Files where the detector below sees an invocation that is not one: the
/// token sits inside a string that opened on an earlier line (prose in a
/// here-doc or a `printf`), or inside a fixture that exists to be scanned.
/// Each is a MENTION, so none of them needs the gate.
const NOT_AN_INVOCATION: &[(&str, &str)] = &[
    (
        "benches/latency/bench.py",
        "a sentence inside a multi-line message string",
    ),
    (
        "tools/scripts/build_deb.sh",
        "a sentence inside the package description text",
    ),
    (
        "tools/scripts/check_public_surface.py",
        "the public-surface gate's own fixture page, which exists to be scanned",
    ),
    (
        "tools/scripts/install.sh",
        "a sentence inside the installer's closing message",
    ),
];

/// True when a token is a wrapper rather than the command itself: `sudo`,
/// `timeout 60s`, `"$TIMEOUT_COMMAND"`, `VAR=value`, a flag.
fn is_wrapper_token(tok: &str) -> bool {
    let t = tok.trim_matches('"').trim_matches('\'');
    if t.is_empty() {
        return true;
    }
    if matches!(
        t,
        "sudo" | "env" | "timeout" | "exec" | "command" | "nohup" | "stdbuf" | "time" | "-"
    ) {
        return true;
    }
    if t.starts_with('-') || t.starts_with('$') {
        return true;
    }
    // A YAML key: `run: cerulion graph run demo` puts the command one token in.
    if t.ends_with(':') && !t.contains('/') {
        return true;
    }
    // A duration argument to a timeout wrapper, or a `VAR=value` prefix.
    let bare = t.trim_end_matches(['s', 'm', 'h']);
    if !bare.is_empty() && bare.chars().all(|c| c.is_ascii_digit()) {
        return true;
    }
    t.split_once('=').is_some_and(|(name, _)| {
        !name.is_empty() && name.chars().all(|c| c.is_ascii_alphanumeric() || c == '_')
    })
}

/// True when `text` runs the `cerulion` binary at a command position: the head
/// of a command is the binary, and the next word is a verb rather than a flag.
/// Comments are stripped first, so a sentence about the binary is not a run of
/// it, and `cerulion --version` is not one either (clap answers it above the
/// gate).
fn runs_the_binary(text: &str) -> bool {
    for line in strip_hash_comments(text).lines() {
        for seg in line.split(['|', ';', '&']) {
            let toks: Vec<&str> = seg.split_whitespace().collect();
            let Some(head_at) = toks.iter().position(|t| !is_wrapper_token(t)) else {
                continue;
            };
            let head = toks[head_at].trim_matches('"').trim_matches('\'');
            let base = head.rsplit('/').next().unwrap_or(head);
            if base != "cerulion" {
                continue;
            }
            let Some(next) = toks.get(head_at + 1) else {
                continue;
            };
            let verbish = next.starts_with(|c: char| c.is_ascii_lowercase())
                && next
                    .chars()
                    .all(|c| c.is_ascii_lowercase() || c.is_ascii_digit() || c == '-');
            if verbish {
                return true;
            }
        }
    }
    false
}

#[test]
fn everything_that_runs_the_binary_outside_cargo_turns_the_login_gate_off() {
    let root = repo_root();
    // The positive control FIRST: each declared carrier really carries it,
    // spelled the way the gate reads it.
    for carrier in GATE_CARRIERS {
        let text = read(&root.join(carrier));
        assert!(
            text.contains("CERULION_LOGIN_GATE"),
            "{carrier} runs the cerulion binary outside cargo but does not set \
             CERULION_LOGIN_GATE (docs/internals/ci-and-gates.md, \"The login gate in CI\")"
        );
        assert!(
            text.contains("\"off\"") || text.contains("=off") || text.contains("] = \"off\""),
            "{carrier} names CERULION_LOGIN_GATE but not the value `off`, which the gate \
             matches byte for byte"
        );
    }
    // Then the walk: anything else that invokes the binary is undeclared.
    let mut undeclared: Vec<String> = Vec::new();
    let mut scanned = 0usize;
    for dir in [".github/workflows", "tools", "benches"] {
        collect_harness_files(&root.join(dir), &mut |path| {
            scanned += 1;
            let rel = path
                .strip_prefix(&root)
                .unwrap_or(path)
                .to_string_lossy()
                .replace('\\', "/");
            if GATE_CARRIERS.contains(&rel.as_str())
                || NOT_AN_INVOCATION.iter().any(|(p, _)| *p == rel)
            {
                return;
            }
            if runs_the_binary(&read(path)) {
                undeclared.push(rel);
            }
        });
    }
    assert!(
        scanned > 40,
        "only {scanned} harness files were scanned; the walk found almost nothing"
    );
    assert!(
        undeclared.is_empty(),
        "these run the cerulion binary outside cargo and are in neither list: {undeclared:?}. \
         Set CERULION_LOGIN_GATE=off and add the file to GATE_CARRIERS \
         (docs/internals/ci-and-gates.md, \"The login gate in CI\"), or, if the match is a \
         sentence rather than a command, record it in NOT_AN_INVOCATION with the reason."
    );
    // A stale entry in either list is a waiver that excuses nothing.
    for (path, reason) in NOT_AN_INVOCATION {
        let full = root.join(path);
        assert!(
            full.is_file() && runs_the_binary(&read(&full)),
            "NOT_AN_INVOCATION names {path} ({reason}) but the detector no longer matches it: \
             delete the line"
        );
    }
}

fn collect_harness_files(dir: &Path, f: &mut impl FnMut(&Path)) {
    let Ok(read_dir) = std::fs::read_dir(dir) else {
        return;
    };
    for item in read_dir.flatten() {
        let path = item.path();
        if path.is_dir() {
            if path
                .file_name()
                .is_some_and(|n| n == "target" || n == "node_modules")
            {
                continue;
            }
            collect_harness_files(&path, f);
        } else if path
            .extension()
            .is_some_and(|e| e == "yml" || e == "yaml" || e == "sh" || e == "py")
        {
            f(&path);
        }
    }
}

// ---------------------------------------------------------------------------
// (d) The documented login exemptions are the ones the code exempts.
// ---------------------------------------------------------------------------

/// The two exemptions in the sentence that are NOT `Commands` variants: clap
/// answers them while parsing, above the gate. They are asserted present, so
/// they cannot quietly vanish from the sentence either.
const CLAP_ANSWERED: &[&str] = &["--help", "--version"];

/// `RunWorker` -> `run-worker`.
fn kebab(camel: &str) -> String {
    let mut out = String::new();
    for (i, ch) in camel.chars().enumerate() {
        if ch.is_ascii_uppercase() {
            if i > 0 {
                out.push('-');
            }
            out.push(ch.to_ascii_lowercase());
        } else {
            out.push(ch);
        }
    }
    out
}

/// A view of Rust source with `//` line comments and `/* */` blocks removed,
/// so a variant NAMED in a comment beside the list is not read as a member of
/// it. Block comments nest in Rust, so the depth is tracked.
fn rust_code_only(src: &str) -> String {
    let chars: Vec<char> = src.chars().collect();
    let mut out = String::with_capacity(src.len());
    let mut i = 0usize;
    let mut depth = 0usize;
    while i < chars.len() {
        if depth > 0 {
            if chars[i] == '/' && chars.get(i + 1) == Some(&'*') {
                depth += 1;
                i += 2;
                continue;
            }
            if chars[i] == '*' && chars.get(i + 1) == Some(&'/') {
                depth -= 1;
                i += 2;
                continue;
            }
            if chars[i] == '\n' {
                out.push('\n');
            }
            i += 1;
            continue;
        }
        if chars[i] == '/' && chars.get(i + 1) == Some(&'*') {
            depth = 1;
            i += 2;
            continue;
        }
        if chars[i] == '/' && chars.get(i + 1) == Some(&'/') {
            while i < chars.len() && chars[i] != '\n' {
                i += 1;
            }
            continue;
        }
        out.push(chars[i]);
        i += 1;
    }
    out
}

#[test]
fn the_documented_login_exemptions_are_the_ones_the_code_exempts() {
    let root = repo_root();

    // The code half: the `!matches!` body of `command_needs_identity`.
    let src = rust_code_only(&read(&root.join("crates/cerulion_cli/src/main.rs")));
    let at = src
        .find("fn command_needs_identity")
        .expect("crates/cerulion_cli/src/main.rs defines command_needs_identity");
    let body = &src[at..];
    let end = body
        .find("\n}")
        .expect("the function body is closed at column 0");
    let body = &body[..end];
    let mut from_code: BTreeSet<String> = BTreeSet::new();
    for marker in ["Commands::", "GraphAction::"] {
        let mut rest = body;
        while let Some(at) = rest.find(marker) {
            let tail = &rest[at + marker.len()..];
            let name: String = tail
                .chars()
                .take_while(|c| c.is_ascii_alphanumeric() || *c == '_')
                .collect();
            if !name.is_empty() {
                from_code.insert(kebab(&name));
            }
            rest = &tail[name.len()..];
        }
    }
    assert!(
        from_code.len() >= 3,
        "only {from_code:?} was read out of command_needs_identity; the function's shape \
         moved and this test is comparing almost nothing"
    );

    // The document half: the backticked spans of the exemption sentence.
    let doc = read(&root.join("docs/user-api.md"));
    let sentence = doc
        .lines()
        .find(|l| l.contains("Exempt: `login`"))
        .unwrap_or_else(|| {
            panic!(
                "docs/user-api.md no longer carries the login exemption sentence \
                 (\"Exempt: `login` …\"), which is where a user reads which commands run \
                 without an account"
            )
        });
    let sentence = &sentence[sentence
        .find("Exempt:")
        .expect("the sentence starts at its own word")..];
    let mut from_doc: BTreeSet<String> = BTreeSet::new();
    let mut rest = sentence;
    while let Some(open) = rest.find('`') {
        let after = &rest[open + 1..];
        let Some(close) = after.find('`') else { break };
        let span = &after[..close];
        rest = &after[close + 1..];
        if CLAP_ANSWERED.contains(&span) {
            continue;
        }
        for word in span.split_whitespace() {
            from_doc.insert(word.to_string());
        }
    }
    for flag in CLAP_ANSWERED {
        assert!(
            sentence.contains(&format!("`{flag}`")),
            "the exemption sentence no longer names `{flag}`, which clap answers above the \
             gate: a user reading it would expect `cerulion {flag}` to need an account"
        );
    }

    assert_eq!(
        from_doc, from_code,
        "docs/user-api.md's login exemption sentence and \
         crates/cerulion_cli/src/main.rs's `command_needs_identity` disagree. The sentence \
         says {from_doc:?}; the code exempts {from_code:?}. Whichever moved, the other has to \
         move with it: this sentence is where a user learns which commands run on a machine \
         that has never signed in."
    );
}
