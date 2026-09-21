// SPDX-License-Identifier: AGPL-3.0-only
//! The SOURCE-WALK GATE over the fork-child module — the established
//! pattern for "nothing in this module may call X", applied to the fork child's ban list.
//!
//! # Why a walk, and not a runtime check
//!
//! A fork child has exactly ONE thread, so whatever any other parent thread held at the
//! fork instant is held forever in the child's image by nobody. Every forbidden call
//! below is one that takes such a lock, or allocates, or leaves via a path that runs
//! `Drop` for the parent's live data plane. None of them FAILS when it is wrong
//! — it WEDGES, or corrupts, in a process nobody is watching, and the parent reports it
//! five seconds later as `ChildStalled` pointing at an innocent node.
//!
//! Two of the rules are moreover the ABSENCE of code, which no runtime check can see:
//!
//! - the child module's **step 1** — the child calls `panic::set_hook` NEVER (the hook is
//!   installed by the parent, branching on `getpid()`), because a child that touched
//!   std's hook lock can wedge on a reader the fork inherited;
//! - its **step 7**, `_exit`, never `exit` — the second one runs destructors and `atexit` in a
//!   process that owns none of what it would be tearing down.
//!
//! # The walk reads the FILE; it does not consult a list of what the file may do
//!
//! A hand-maintained list of what the file may do goes stale the day someone adds a
//! call the list does not name. The gate here reads the file instead:
//! a comment-stripped view of the module's real source, searched for
//! tokens, with an ANTI-TAUTOLOGY arm requiring the stripped view to still contain the
//! code the module DOES have — without which every "absent" assertion is vacuous the
//! moment the stripper breaks.
//!
//! `code_only` strips line comments AND depth-tracked (Rust block comments NEST) block
//! comments, and blanks string literals. Both are load-bearing here: the module's docs
//! NAME every banned token in order to explain its absence, and the `oom_score_adj`
//! path is a byte-string literal containing `/proc`.
//!
//! Parallel-safe — pure file parse, no transport.

use std::path::PathBuf;

/// The module under the gate.
fn child_module_path() -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("src/state_carrier/child.rs")
}

/// A view of Rust source with comments and every literal BODY removed.
///
/// # It FAILS CLOSED, and the reason is the failure mode this file exists to prevent
///
/// Every assertion below is NEGATIVE — "this token does not appear". A stripper that
/// silently returns a PREFIX therefore enforces them over the part it managed to read
/// and reports success for the rest, which is a gate that looks green while policing
/// nothing. So an unterminated block comment, string, raw string or char literal is a
/// PANIC, never a truncated view.
///
/// # What it models, and why each one is load-bearing HERE
///
/// - **Comments** (line + depth-tracked nesting block): the child module NAMES every
///   banned token in its own docs to explain the absence.
/// - **Strings and byte strings**: the `oom_score_adj` path is `b"/proc/self/..."`.
/// - **RAW strings** (`r"…"`, `r#"…"#`, `br#"…"#`): a raw string does NOT honour `\`,
///   so a regular-string scanner reading `r"a\"` treats the closing quote as escaped,
///   runs to the next quote ANYWHERE in the file, and deletes everything between —
///   with `depth` still 0, so the loud guard cannot see it. Fail-OPEN.
/// - **CHAR literals**: `'"'` is legal Rust and puts a lone `"` in the token stream.
///   A scanner without a char arm enters string mode there and swallows the rest of the
///   file to the next quote. Also fail-OPEN, and the shortest of the two to write by
///   accident.
///
/// Char literals are distinguished from LIFETIMES (`'a`, `'_`, `'static`) by the only
/// rule that needs no type information: a char literal is `'` followed by an escape, or
/// by exactly one character and then a closing `'`. Anything else is a lifetime and is
/// emitted verbatim.
fn code_only(src: &str) -> String {
    let b = src.as_bytes();
    let mut out = String::with_capacity(src.len());
    let mut i = 0usize;

    // `true` when the previous emitted byte could end an identifier, which is what
    // separates the raw-string prefix in `r"x"` from the trailing `r` of `for"`.
    let ident_byte = |c: u8| c.is_ascii_alphanumeric() || c == b'_';

    while i < b.len() {
        // ---- comments -------------------------------------------------------------
        if b[i] == b'/' && i + 1 < b.len() && b[i + 1] == b'/' {
            while i < b.len() && b[i] != b'\n' {
                i += 1;
            }
            continue;
        }
        if b[i] == b'/' && i + 1 < b.len() && b[i + 1] == b'*' {
            let mut depth = 1usize;
            i += 2;
            while i < b.len() && depth > 0 {
                if b[i] == b'/' && i + 1 < b.len() && b[i + 1] == b'*' {
                    depth += 1;
                    i += 2;
                } else if b[i] == b'*' && i + 1 < b.len() && b[i + 1] == b'/' {
                    depth -= 1;
                    i += 2;
                } else {
                    if b[i] == b'\n' {
                        out.push('\n');
                    }
                    i += 1;
                }
            }
            assert_eq!(
                depth, 0,
                "unterminated block comment: the stripped view would be a PREFIX, and \
                 every absence assertion would hold only over the part it read"
            );
            continue;
        }

        // ---- raw strings: r"…", r#"…"#, br##"…"## ---------------------------------
        // Checked BEFORE the plain-string arm, because a raw string honours no escapes.
        {
            let prev_is_ident = i > 0 && ident_byte(b[i - 1]);
            let mut j = i;
            if !prev_is_ident && b[j] == b'b' {
                j += 1;
            }
            if !prev_is_ident && j < b.len() && b[j] == b'r' {
                let hash_start = j + 1;
                let mut hashes = hash_start;
                while hashes < b.len() && b[hashes] == b'#' {
                    hashes += 1;
                }
                if hashes < b.len() && b[hashes] == b'"' {
                    let n = hashes - hash_start;
                    out.push('"');
                    let mut k = hashes + 1;
                    let mut closed = false;
                    while k < b.len() {
                        if b[k] == b'"' {
                            let mut m = 0usize;
                            while m < n && k + 1 + m < b.len() && b[k + 1 + m] == b'#' {
                                m += 1;
                            }
                            if m == n {
                                k += 1 + n;
                                closed = true;
                                break;
                            }
                        }
                        if b[k] == b'\n' {
                            out.push('\n');
                        }
                        k += 1;
                    }
                    assert!(
                        closed,
                        "unterminated raw string: the stripped view would be a PREFIX"
                    );
                    out.push('"');
                    i = k;
                    continue;
                }
            }
        }

        // ---- char literals (NOT lifetimes) ----------------------------------------
        if b[i] == b'\'' {
            let escaped = i + 1 < b.len() && b[i + 1] == b'\\';
            // `'x'`: exactly one character between the quotes. Multi-byte safe — the
            // char's own length is taken from the UTF-8 lead byte.
            let one_char_len = if i + 1 < b.len() {
                let lead = b[i + 1];
                if lead < 0x80 {
                    1
                } else if lead >> 5 == 0b110 {
                    2
                } else if lead >> 4 == 0b1110 {
                    3
                } else {
                    4
                }
            } else {
                0
            };
            let simple = one_char_len > 0
                && b[i + 1] != b'\''
                && i + 1 + one_char_len < b.len()
                && b[i + 1 + one_char_len] == b'\'';
            if escaped || simple {
                out.push('\'');
                let mut k = i + 1;
                let mut closed = false;
                while k < b.len() {
                    if b[k] == b'\\' {
                        k += 2;
                        continue;
                    }
                    if b[k] == b'\'' {
                        closed = true;
                        k += 1;
                        break;
                    }
                    k += 1;
                }
                assert!(
                    closed,
                    "unterminated char literal: the stripped view would be a PREFIX"
                );
                out.push('\'');
                i = k;
                continue;
            }
            // A LIFETIME. Emit it and move on; nothing inside it can open a literal.
            out.push('\'');
            i += 1;
            continue;
        }

        // ---- plain and byte strings -----------------------------------------------
        if b[i] == b'"' {
            // Blank the body, keep the quotes, so `open(PATH)` still reads as a call
            // while `/proc/self/...` stops looking like a forbidden token.
            out.push('"');
            let mut k = i + 1;
            let mut closed = false;
            while k < b.len() {
                if b[k] == b'\\' {
                    k += 2;
                    continue;
                }
                if b[k] == b'"' {
                    closed = true;
                    k += 1;
                    break;
                }
                if b[k] == b'\n' {
                    out.push('\n');
                }
                k += 1;
            }
            assert!(
                closed,
                "unterminated string literal: the stripped view would be a PREFIX"
            );
            out.push('"');
            i = k;
            continue;
        }

        out.push(b[i] as char);
        i += 1;
    }
    out
}

/// `(token, why it is banned)`.
///
/// Each reason is the failure the token would cause in a fork child — not a style note.
const BANNED: &[(&str, &str)] = &[
    (
        "set_hook",
        "the child must acquire std's hook lock NEVER. \
         rust_panic_with_hook holds the READ guard across the whole user hook and \
         set_hook takes WRITE, so a fork landing mid-panic wedges the child before \
         its unwind protection exists. The hook is installed by the PARENT.",
    ),
    ("take_hook", "same lock as set_hook, same wedge."),
    (
        "tracing::",
        "a global dispatcher behind a lock this process does not control; the child \
         may only write(2).",
    ),
    ("info!", "a tracing macro — see tracing:: above."),
    ("warn!", "a tracing macro."),
    ("error!", "a tracing macro."),
    ("debug!", "a tracing macro."),
    (
        "println!",
        "std's stdout lock, plus formatting machinery that allocates.",
    ),
    (
        "eprintln!",
        "std's stderr lock, plus formatting machinery that allocates.",
    ),
    (
        "format!",
        "allocates on a path that must pre-size allocation away.",
    ),
    ("to_string(", "allocates."),
    ("String::", "allocates."),
    ("Vec::", "allocates."),
    ("Box::", "allocates."),
    (
        "iceoryx2",
        "any iceoryx2 call in the child can corrupt the parent's LIVE \
         data plane.",
    ),
    (
        "TransportManager",
        "the iceoryx2 entry point; see iceoryx2 above.",
    ),
    (
        "std::process",
        "process::exit runs destructors and atexit in a process that owns none of what \
         it would tear down (_exit, never exit).",
    ),
    (
        "Mutex",
        "a lock another parent thread could have held at the fork instant is held \
         forever in the child's image, by nobody.",
    ),
    ("RwLock", "same as Mutex."),
    (".lock()", "same as Mutex."),
    (
        "Instant::now",
        "a clock read: banned on this path, and a wall-derived decision here \
         would put wall time back into the anchor (Principle #7).",
    ),
    ("SystemTime", "a clock read; see Instant::now."),
];

/// Tokens the module DOES contain. The anti-tautology arm.
///
/// Without these, a stripper that returned an empty string would satisfy every absence
/// assertion above and the gate would be permanently green while policing nothing.
const REQUIRED: &[&str] = &[
    "libc::_exit",
    "libc::getppid",
    "fn apply_child_discipline",
    "fn child_main",
    "libc::sigprocmask",
    "libc::close",
    "catch_unwind",
];

#[test]
fn nothing_in_the_fork_child_module_may_take_a_lock_allocate_or_leave_through_exit() {
    let path = child_module_path();
    let src =
        std::fs::read_to_string(&path).unwrap_or_else(|e| panic!("read {}: {e}", path.display()));
    let code = code_only(&src);

    // ANTI-TAUTOLOGY FIRST: if the stripped view lost the code, every assertion after
    // this is vacuous, so the gate must fail HERE rather than pass silently.
    for needle in REQUIRED {
        assert!(
            code.contains(needle),
            "the stripped view of {} lost `{needle}` — the walk is reading nothing, so \
             its absence assertions would be vacuous",
            path.display()
        );
    }

    for (token, why) in BANNED {
        assert!(
            !code.contains(token),
            "the fork-child module names `{token}` in CODE.\n\nWhy it is banned: {why}\n\n\
             If this is genuinely safe in a fork child, the ban list in this test is \
             where that argument belongs — not a silent deletion.",
        );
    }
}

#[test]
fn the_module_leaves_only_through_underscore_exit() {
    // `_exit`, never `exit`. `std::process` is on the ban list, but the bare
    // `exit(` spelling would slip past it, so the shape is checked directly: every
    // `exit(` in the module must be `_exit(`.
    let src = std::fs::read_to_string(child_module_path()).expect("read child module");
    let code = code_only(&src);
    let mut checked = 0usize;
    let bytes = code.as_bytes();
    for (i, _) in code.match_indices("exit(") {
        checked += 1;
        assert!(
            i > 0 && bytes[i - 1] == b'_',
            "the child module calls a bare `exit(` at byte {i}: that runs destructors \
             and atexit in a process that owns none of what it would tear down"
        );
    }
    assert!(
        checked > 0,
        "no `exit(` at all in the child module — the child has to leave somehow, so \
         this assertion is measuring the wrong file"
    );
}

/// The `{ … }` block that follows `needle`, brace-matched, WITHOUT its outer braces.
///
/// Run on a `code_only` view, where literal bodies are already blanked — so a `{` can
/// only ever be a real block delimiter and the match cannot be thrown by a brace inside
/// a string or a char literal.
///
/// Panics if `needle` is absent or the block never closes: this feeds a NEGATIVE
/// assertion, so a silent `None` would make it vacuous.
fn braced_block_after(code: &str, needle: &str) -> String {
    let at = code
        .find(needle)
        .unwrap_or_else(|| panic!("`{needle}` not found — the walk is reading the wrong thing"));
    let rest = &code[at..];
    let open = rest
        .find('{')
        .unwrap_or_else(|| panic!("no block opens after `{needle}`"));
    let bytes = rest.as_bytes();
    let mut depth = 0usize;
    let mut i = open;
    while i < bytes.len() {
        match bytes[i] {
            b'{' => depth += 1,
            b'}' => {
                depth -= 1;
                if depth == 0 {
                    return rest[open + 1..i].to_string();
                }
            }
            _ => {}
        }
        i += 1;
    }
    panic!("the block after `{needle}` never closes");
}

/// The carrier modules the hook-lock invariant covers, and the ONE that may install.
const CARRIER_MODULES: &[&str] = &[
    "anchor.rs",
    "breadcrumb.rs",
    "child.rs",
    "dontfork.rs",
    "fork.rs",
    "mod.rs",
    "reaper.rs",
    "watchdog.rs",
];

/// The installer's file, excluded from the ban and checked separately.
const INSTALLER_MODULE: &str = "hook.rs";

fn carrier_dir() -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("src/state_carrier")
}

#[test]
fn nothing_in_the_carrier_takes_stds_hook_lock_for_write_except_the_one_installer() {
    // `rust_panic_with_hook` takes a READ lock before invoking any
    // hook, so a fork taken while a WRITE lock is held deadlocks the child's own panic
    // at `HOOK.read()` — and NO std API can prevent that (there is no way to hold the
    // read lock across `fork`, and `update_hook` also takes WRITE).
    //
    // The residual is therefore bounded by an INVARIANT, not a mechanism: nothing may
    // take the hook lock for WRITE once a capture fork is reachable. The carrier's
    // install is a `Once` at ARM time, strictly before any fork exists. That is the
    // whole argument, so it is ENFORCED here rather than asserted in a comment.
    const WRITE_TAKERS: &[&str] = &["set_hook", "take_hook", "update_hook"];

    let dir = carrier_dir();
    let mut seen = Vec::new();
    for entry in std::fs::read_dir(&dir).expect("read state_carrier") {
        let path = entry.expect("dir entry").path();
        let Some(name) = path.file_name().and_then(|n| n.to_str()) else {
            continue;
        };
        if !name.ends_with(".rs") {
            continue;
        }
        seen.push(name.to_string());
        let code = code_only(&std::fs::read_to_string(&path).expect("read module"));
        if name == INSTALLER_MODULE {
            // The installer may take it EXACTLY once each ...
            for token in WRITE_TAKERS {
                let count = code.matches(token).count();
                let expect = usize::from(*token != "update_hook");
                assert_eq!(
                    count, expect,
                    "{INSTALLER_MODULE} must name `{token}` exactly {expect} time(s) — \
                     the install is a Once at arm time, and a second write-taker \
                     anywhere is the interleave this invariant exists to exclude"
                );
            }

            // ... and ONLY from INSIDE the `Once`, which is the half a count cannot
            // see. `install_fork_panic_hook` is called on every ARM, so a write-taker
            // sitting outside the `call_once` runs on every re-arm — i.e. after forks
            // are already reachable — which is exactly the interleave the invariant
            // exists to exclude. A counting-only check passes an EMPTY `call_once` with
            // both calls hoisted above it.
            let installer = braced_block_after(&code, "pub fn install_fork_panic_hook");
            let once_body = braced_block_after(&installer, "INSTALL.call_once");
            let outside = installer.replacen(&once_body, "", 1);
            for token in ["set_hook", "take_hook"] {
                assert_eq!(
                    once_body.matches(token).count(),
                    1,
                    "`{token}` must be taken INSIDE `INSTALL.call_once` — that is what \
                     makes it once-per-process rather than once-per-arm"
                );
                assert_eq!(
                    outside.matches(token).count(),
                    0,
                    "`{token}` appears in `install_fork_panic_hook` OUTSIDE the \
                     `call_once`, so it runs on every re-arm — after forks are \
                     reachable, which can deadlock a capture child's own panic at \
                     std's `HOOK.read()`"
                );
            }
            continue;
        }
        for token in WRITE_TAKERS {
            assert!(
                !code.contains(token),
                "{name} names `{token}` in CODE.\n\nTaking std's hook lock for WRITE \
                 once a capture fork is reachable can deadlock a child's own panic at \
                 std's `HOOK.read()`, which no API of ours can prevent. The carrier \
                 installs ONE hook, in {INSTALLER_MODULE}, at arm time."
            );
        }
    }

    // ANTI-TAUTOLOGY: the walk must really have visited the modules it claims to.
    // A directory walk that found nothing satisfies every assertion above.
    for module in CARRIER_MODULES {
        assert!(
            seen.iter().any(|s| s == module),
            "the walk never reached {module}; its absence assertions are vacuous"
        );
    }
    assert!(
        seen.iter().any(|s| s == INSTALLER_MODULE),
        "the walk never reached the installer, so its exactly-once pin never ran"
    );
}

#[test]
fn nothing_runs_between_fork_returning_zero_and_the_childs_own_discipline() {
    // The child's step ordering, which is STRUCTURAL rather than a rule the caller follows: the
    // child branch must go straight into `child_main`, which applies steps 2-5 itself.
    // Anything inserted here runs in a process that has not yet reset its inherited
    // signal dispositions, closed its inherited descriptors, or biased the OOM killer
    // onto itself — and it runs there with one thread and every lock the parent's other
    // threads held frozen shut.
    //
    // Only a source walk can see this. A behavioural test cannot: work inserted in that
    // window would usually SUCCEED (a fork child can allocate, and can log, right up
    // until the once it deadlocks), so the failure it introduces is a rare wedge in a
    // process nobody is watching — reported five seconds later as a stalled encoder.
    let path = PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("src/state_carrier/fork.rs");
    let src =
        std::fs::read_to_string(&path).unwrap_or_else(|e| panic!("read {}: {e}", path.display()));
    let code = code_only(&src);

    // ANTI-TAUTOLOGY: the walk must be reading the real call site.
    assert!(
        code.contains("libc::fork()"),
        "the stripped view of {} lost the fork call itself, so everything below is \
         vacuous",
        path.display()
    );

    let branch = braced_block_after(&code, "if pid == 0");
    let body: String = branch.split_whitespace().collect::<Vec<_>>().join(" ");
    assert_eq!(
        body, "child_main(crumb, setup, encoder);",
        "the child branch must contain NOTHING but the call into `child_main`, which \
         applies the child discipline's steps 2-5 itself. Found:\n{branch}"
    );
}

#[test]
fn code_only_handles_char_literals_without_entering_string_mode() {
    // `'"'` is legal Rust and puts a LONE quote in the token stream. A scanner with no
    // char arm enters string mode there and swallows everything to the next quote —
    // with no unterminated-anything to trip the loud guard, so it fails OPEN and every
    // absence assertion after that point becomes vacuous.
    assert_eq!(
        code_only("let q = '\"'; let m = Mutex::new(1);"),
        "let q = ''; let m = Mutex::new(1);",
        "a char literal holding a quote must not open a string"
    );
    // An ESCAPED quote inside a char literal, same hazard one level down.
    assert_eq!(
        code_only("let q = '\\''; let m = Mutex::new(1);"),
        "let q = ''; let m = Mutex::new(1);"
    );
    // Ordinary escapes.
    assert_eq!(
        code_only("let n = '\\n'; let x = 1;"),
        "let n = ''; let x = 1;"
    );
    // LIFETIMES must survive verbatim — they are not literals, and eating them would
    // corrupt every signature the anti-tautology arm looks for.
    assert_eq!(
        code_only("impl<'a> Foo<'a> { fn f(&'static self) {} }"),
        "impl<'a> Foo<'a> { fn f(&'static self) {} }"
    );
    assert_eq!(code_only("fn f(x: &'_ str) {}"), "fn f(x: &'_ str) {}");
    // A multi-byte char literal must not be split mid-character.
    assert_eq!(
        code_only("let c = 'é'; let y = 2;"),
        "let c = ''; let y = 2;"
    );
}

#[test]
fn code_only_handles_raw_strings_which_honour_no_escapes() {
    // THE fail-open shape: a raw string ending in a backslash. A regular-string scanner
    // reads `\"` as an escaped quote, misses the real terminator, and runs to the next
    // quote anywhere in the file — deleting the code between while leaving the block
    // depth at 0, so nothing fails.
    assert_eq!(
        code_only(r####"let p = r"a\"; let m = Mutex::new(1); let q = "x";"####),
        r####"let p = ""; let m = Mutex::new(1); let q = "";"####,
        "a raw string ending in a backslash must terminate at its own quote"
    );
    // Hashed forms, including a quote INSIDE the body.
    assert_eq!(
        code_only(r####"let p = r#"has " inside"#; let x = 1;"####),
        r####"let p = ""; let x = 1;"####
    );
    assert_eq!(
        code_only(r####"let p = r##"ends with "# here"##; let x = 1;"####),
        r####"let p = ""; let x = 1;"####
    );
    // Byte raw strings.
    assert_eq!(
        code_only(r####"let p = br#"Mutex"#;"####),
        r####"let p = "";"####
    );
    // A trailing `r` that is part of an IDENTIFIER is not a raw-string prefix.
    assert_eq!(
        code_only(r#"let sender = "x"; let y = 1;"#),
        r#"let sender = ""; let y = 1;"#
    );
    assert_eq!(
        code_only(r#"for x in "abc".chars() {}"#),
        r#"for x in "".chars() {}"#
    );
}

#[test]
fn code_only_fails_closed_on_every_unterminated_literal() {
    // Each of these leaves the scanner mid-literal at EOF. Returning a prefix would
    // make every negative assertion in this file hold over the part that was read.
    for (name, src) in [
        ("string", "let s = \"never closed; let m = Mutex::new(1);"),
        ("raw string", "let s = r#\"never closed"),
        // An ESCAPE with no closing quote is the only unterminated CHAR shape, because
        // it is the only one that ENTERS char mode. A bare `'a` at EOF is a LIFETIME —
        // syntactically indistinguishable and correctly emitted verbatim, swallowing
        // nothing — so it is not a fail-closed case and is asserted as such below.
        ("char", "let c = '\\n"),
    ] {
        let panicked = std::panic::catch_unwind(|| code_only(src));
        assert!(
            panicked.is_err(),
            "an unterminated {name} must fail LOUDLY, not return a prefix"
        );
    }

    // The complement: a trailing lifetime is not an error and must pass through.
    assert_eq!(code_only("fn f<'a"), "fn f<'a");
}

#[test]
fn code_only_strips_what_it_must_and_keeps_what_it_must() {
    // The stripper is the thing every other assertion in this file rests on, so it is
    // oracle-tested directly rather than trusted for self-agreement.
    assert_eq!(code_only("let a = 1; // Mutex\n"), "let a = 1; \n");
    assert_eq!(code_only("/* Mutex */let a = 1;"), "let a = 1;");
    // Rust block comments NEST — a depth-blind stripper stops at the first `*/` and
    // leaves the rest of the outer comment in the "code" view.
    assert_eq!(code_only("/* a /* Mutex */ b */let x = 2;"), "let x = 2;");
    // A `//` INSIDE a block comment must not end it.
    assert_eq!(code_only("/* // Mutex\n */let y = 3;"), "\nlet y = 3;");
    // String bodies are blanked, quotes kept: `open(PATH)` still reads as a call.
    assert_eq!(
        code_only(r#"let p = "/proc/self/Mutex";"#),
        r#"let p = "";"#
    );
    // An escaped quote must not end the literal early.
    assert_eq!(code_only(r#"let p = "a\"Mutex";"#), r#"let p = "";"#);
    // Multi-byte source survives (byte indexing must not split a char).
    assert!(code_only("let s = 1; // ünïcödé\n").starts_with("let s = 1;"));
}

#[test]
fn code_only_fails_closed_on_an_unterminated_block_comment() {
    // The failure mode this repo has actually been bitten by: a stripper that silently
    // returns a PREFIX makes every absence assertion hold over the part it read.
    let panicked = std::panic::catch_unwind(|| code_only("/* never closed\nlet x = 1;"));
    assert!(
        panicked.is_err(),
        "an unterminated block comment must fail LOUDLY, not return a prefix"
    );
}
