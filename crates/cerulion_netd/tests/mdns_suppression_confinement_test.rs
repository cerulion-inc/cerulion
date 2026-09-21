// SPDX-License-Identifier: AGPL-3.0-only
//! The mDNS-SUPPRESSION test constructors have NO caller in
//! `cerulion_netd`'s production code — a STRUCTURAL guard, because a violation is
//! invisible to every behavioural arm in the crate.
//!
//! # The regression this exists to catch
//!
//! `GatewayBeacon::without_mdns_for_test` / `GatewayEgressPlane::new_without_mdns_for_test`
//! build a plane that takes every beacon decision EXACTLY as production does and then
//! reaches no multicast socket. That is precisely what the beacon polish wanted
//! for the netd test suite, which drives robot-shaped fixtures through the REAL egress
//! plane and was publishing a genuine, resolvable `_cerulion._tcp` record on the
//! developer's (or CI runner's) LAN for the length of every run.
//!
//! The same property makes them the most dangerous thing in the crate to call by
//! accident. A production path built with the suppressed constructor would:
//!
//! * take the same decision, so `mdns_beacon_decision()` reports `Advertise { .. }`;
//! * log the same lines, so an operator reading the log sees "netd is advertising";
//! * leave `GatewayEgressPlane::new` untouched, so the desk arm that guards it
//!   (`a_desk_shaped_netd_boots_its_gateway_and_advertises_nothing`) still
//!   passes — a desk declines BEFORE the advertiser is reached, so it cannot see
//!   which advertiser was installed on a ROBOT-shaped path;
//!
//! …and advertise NOTHING. That is the original beacon defect, re-shipped, with every
//! observable in the crate reporting health. `GatewayBeacon::advertiser_is_real()` makes
//! the substitution DETECTABLE, but only somewhere that asks — and the only arm that
//! asks is the desk one, on the constructor that is not the hazard.
//!
//! Both modules therefore state the rule in prose ("Nothing in `src/` may call this").
//! Prose is not a gate. This file is.
//!
//! # Why the walk, and what "production code" means here
//!
//! Cribbed from `cerulion_viz/bin/cerulion_vizd/tests/convergence_adoption_test.rs` and
//! `cerulion_cli_engine/tests/convergence_adoption_test.rs`: a DIRECTORY walk over
//! `src/**/*.rs` rather than a hand-written file list (a list reproduces the log-level sweep's own
//! failure mode, where the sweep missed the then-shipping `rerun_sink` because nothing
//! enumerated the sites), over a view with comments AND string/char literals blanked
//! (both modules NAME these constructors in their docs to explain the rule, so a naive
//! search is satisfied by prose), failing LOUDLY on an unbalanced scan.
//!
//! Two further strips define "production code" precisely, and each is the difference
//! between a gate and a false alarm:
//!
//! * `#[cfg(test)] mod <name> { … }` — a call there ships in no daemon, and
//!   `beacon.rs`'s own suppression oracle lives in exactly such a block.
//! * the SIGNATURE AND BODY of every `fn …_for_test` — which removes each
//!   constructor's own definition, and the ONE sanctioned delegation
//!   (`GatewayEgressPlane::new_without_mdns_for_test` building a suppressed
//!   `GatewayBeacon`). What remains is the shipped daemon, and the needle must not
//!   appear in it at all.
//!
//! So the invariant this file enforces is: **a suppressed-advertiser constructor may be
//! named only by its own definition or by another `_for_test` seam.** A new call in
//! `GatewayEgressPlane::new`, in `main.rs`, in `daemon.rs` — anywhere a shipped daemon
//! reaches — fails it.
//!
//! It lives in `tests/` rather than in the module so its own needle literals are not
//! inside the text it searches.

use std::path::{Path, PathBuf};

/// The names that must not reach production code.
///
/// `new_without_mdns_for_test` CONTAINS `without_mdns_for_test`, so the short needle
/// covers both; the long one is listed anyway, because the failure message should
/// name the constructor a reader actually typed.
const SUPPRESSED_CONSTRUCTORS: &[&str] = &["without_mdns_for_test", "new_without_mdns_for_test"];

/// `cerulion_netd`'s `src/` directory.
fn src_dir() -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("src")
}

/// Every `*.rs` file under `src/`, RECURSIVELY — never a hand-written list.
///
/// Fails LOUDLY on a walk that yields almost nothing: a silently-empty walk would make
/// every negative assertion below vacuous.
fn src_files() -> Vec<PathBuf> {
    let mut out = Vec::new();
    walk(&src_dir(), &mut out);
    out.sort();
    assert!(
        out.len() >= 8,
        "the src walk found only {} file(s) under {} — a walk that yields nothing makes \
         every `!contains` assertion in this file vacuous",
        out.len(),
        src_dir().display()
    );
    out
}

fn walk(dir: &Path, out: &mut Vec<PathBuf>) {
    let entries = std::fs::read_dir(dir)
        .unwrap_or_else(|e| panic!("read_dir {}: {e}", dir.display()))
        .collect::<Result<Vec<_>, _>>()
        .unwrap_or_else(|e| panic!("read_dir {}: {e}", dir.display()));
    for entry in entries {
        let path = entry.path();
        if path.is_dir() {
            walk(&path, out);
        } else if path.extension().is_some_and(|e| e == "rs") {
            out.push(path);
        }
    }
}

fn read(path: &Path) -> String {
    std::fs::read_to_string(path).unwrap_or_else(|e| panic!("read {}: {e}", path.display()))
}

/// The outcome of stripping comments + string literals.
struct Stripped {
    code: String,
    /// Depth of unclosed `/*` at EOF. Anything but 0 means the tail was silently
    /// dropped — see [`code_only`].
    unclosed_depth: usize,
}

/// A view of `src` with comments AND string/char literals blanked.
///
/// Both halves are load-bearing here:
///
/// * COMMENTS, because `beacon.rs` and `egress.rs` both NAME the suppressed
///   constructors in their docs in order to explain that nothing may call them — a
///   naive search would be satisfied by exactly the prose this file exists to replace.
/// * STRING LITERALS, because a lone `"` inside a char literal would otherwise open the
///   string arm and swallow everything to the next quote anywhere in the file, leaving
///   `unclosed_depth == 0` so the loud guard could not see it. Only the SHORT char
///   shapes are consumed, so a lifetime survives.
fn code_only(src: &str) -> Stripped {
    let bytes = src.as_bytes();
    let mut out = String::with_capacity(src.len());
    let mut i = 0usize;
    let mut depth = 0usize;
    while i < bytes.len() {
        if depth == 0 {
            // A CHAR literal — only the short shapes, so a lifetime is never eaten.
            if bytes[i] == b'\'' {
                let close = if bytes.get(i + 1) == Some(&b'\\') {
                    (bytes.get(i + 3) == Some(&b'\'')).then_some(i + 3)
                } else {
                    (bytes.get(i + 2) == Some(&b'\'')).then_some(i + 2)
                };
                if let Some(end) = close {
                    out.push(' ');
                    i = end + 1;
                    continue;
                }
                // No closer in range ⇒ a lifetime; fall through and keep it.
            }
            // A RAW string literal: `r`, zero or more `#`, then `"`.
            if bytes[i] == b'r' {
                let mut j = i + 1;
                while j < bytes.len() && bytes[j] == b'#' {
                    j += 1;
                }
                if j < bytes.len() && bytes[j] == b'"' {
                    let hashes = j - i - 1;
                    let mut k = j + 1;
                    'raw: while k < bytes.len() {
                        if bytes[k] == b'"' {
                            let mut h = 0usize;
                            while h < hashes && k + 1 + h < bytes.len() && bytes[k + 1 + h] == b'#'
                            {
                                h += 1;
                            }
                            if h == hashes {
                                k = k + 1 + hashes;
                                break 'raw;
                            }
                        }
                        k += 1;
                    }
                    out.push(' ');
                    i = k.min(bytes.len());
                    continue;
                }
            }
            // A regular (or byte) string literal.
            if bytes[i] == b'"' {
                let mut k = i + 1;
                while k < bytes.len() {
                    match bytes[k] {
                        b'\\' => k += 2,
                        b'"' => {
                            k += 1;
                            break;
                        }
                        _ => k += 1,
                    }
                }
                out.push(' ');
                i = k.min(bytes.len());
                continue;
            }
            if bytes[i..].starts_with(b"//") {
                while i < bytes.len() && bytes[i] != b'\n' {
                    i += 1;
                }
                continue;
            }
        }
        if bytes[i..].starts_with(b"/*") {
            depth += 1;
            i += 2;
            continue;
        }
        if depth > 0 && bytes[i..].starts_with(b"*/") {
            depth -= 1;
            i += 2;
            continue;
        }
        if depth == 0 {
            let ch = src[i..].chars().next().expect("valid utf-8 boundary");
            out.push(ch);
            i += ch.len_utf8();
        } else {
            i += 1;
        }
    }
    Stripped {
        code: out,
        unclosed_depth: depth,
    }
}

/// The byte offset just past the brace matching the FIRST `{` at or after `from`, or
/// `None` if the source has no balanced closer (which a compiling file cannot).
fn end_of_block(src: &str, from: usize) -> Option<usize> {
    let open = from + src[from..].find('{')?;
    let mut depth = 0usize;
    for (off, ch) in src[open..].char_indices() {
        match ch {
            '{' => depth += 1,
            '}' => {
                depth -= 1;
                if depth == 0 {
                    return Some(open + off + 1);
                }
            }
            _ => {}
        }
    }
    None
}

/// Blank every `[start, end)` span, preserving newlines so a failure diagnostic still
/// reads like the file it came from.
fn blank_spans(src: &str, spans: &mut [(usize, usize)]) -> String {
    spans.sort_unstable();
    let mut out = String::with_capacity(src.len());
    let mut cursor = 0usize;
    for &(start, end) in spans.iter() {
        if start < cursor {
            continue; // Already inside a blanked span.
        }
        out.push_str(&src[cursor..start]);
        for ch in src[start..end].chars() {
            out.push(if ch == '\n' { '\n' } else { ' ' });
        }
        cursor = end;
    }
    out.push_str(&src[cursor..]);
    out
}

/// The SHIPPED view of one file: comments + literals blanked, then every
/// `#[cfg(test)] mod … { }` block and every `fn …_for_test` (signature AND body)
/// blanked too.
///
/// What survives is code a shipped `cerulion-netd` can execute — the only place a call
/// to a suppressed-advertiser constructor is a defect.
fn production_code_of(path: &Path) -> String {
    let stripped = code_only(&read(path));
    assert_eq!(
        stripped.unclosed_depth,
        0,
        "{}: the stripper ended inside {} unclosed block comment(s), so the tail was \
         dropped and every assertion over it would run on a PREFIX",
        path.display(),
        stripped.unclosed_depth
    );
    let code = stripped.code;

    let mut spans: Vec<(usize, usize)> = Vec::new();

    // (1) `#[cfg(test)] mod <name> { … }`. Matched on the `mod` that FOLLOWS the
    //     attribute, so a `#[cfg(test)]` on some other item is left alone.
    let mut from = 0usize;
    while let Some(rel) = code[from..].find("#[cfg(test)]") {
        let at = from + rel;
        let rest = &code[at + "#[cfg(test)]".len()..];
        let lead = rest.len() - rest.trim_start().len();
        if rest.trim_start().starts_with("mod ") {
            let mod_at = at + "#[cfg(test)]".len() + lead;
            if let Some(end) = end_of_block(&code, mod_at) {
                spans.push((at, end));
                from = end;
                continue;
            }
        }
        from = at + "#[cfg(test)]".len();
    }

    // (2) Every `fn <ident>_for_test` — signature and body. This removes each
    //     constructor's own definition AND the one sanctioned delegation, which is
    //     what makes the surviving text a pure "does the daemon call it" question.
    let mut from = 0usize;
    while let Some(rel) = code[from..].find("_for_test") {
        let at = from + rel;
        // Walk back over the identifier to its start, then require `fn ` before it.
        let mut name_start = at;
        while name_start > 0 {
            let prev = code.as_bytes()[name_start - 1];
            if prev == b'_' || prev.is_ascii_alphanumeric() {
                name_start -= 1;
            } else {
                break;
            }
        }
        let is_definition = code[..name_start].trim_end().ends_with("fn");
        if is_definition {
            if let Some(end) = end_of_block(&code, at) {
                let start = code[..name_start].rfind("fn").unwrap_or(name_start);
                spans.push((start, end));
                from = end;
                continue;
            }
        }
        from = at + "_for_test".len();
    }

    blank_spans(&code, &mut spans)
}

/// THE pin: no shipped code path in `cerulion_netd` names a suppressed-advertiser
/// constructor.
///
/// The failure mode it closes is silent by construction — a production plane built with
/// the test constructor takes every decision, logs every line, satisfies
/// `mdns_beacon_decision()`, and advertises nothing.
#[test]
fn no_production_path_builds_a_suppressed_mdns_advertiser() {
    let files = src_files();
    let mut scanned = Vec::new();
    let mut saw_needle_before_stripping = false;

    for f in &files {
        let raw = read(f);
        if SUPPRESSED_CONSTRUCTORS.iter().any(|n| raw.contains(n)) {
            saw_needle_before_stripping = true;
        }
        let production = production_code_of(f);
        for needle in SUPPRESSED_CONSTRUCTORS {
            assert!(
                !production.contains(needle),
                "{} calls `{needle}` from code a SHIPPED cerulion-netd executes.\n\n\
                 That constructor takes every beacon decision exactly as production does \
                 and then reaches no multicast socket — so the daemon would report \
                 `Advertise {{ .. }}` from `mdns_beacon_decision()`, log that it is \
                 advertising, pass the desk-shaped adoption arm (a desk declines before \
                 the advertiser is reached, so it cannot see which one was installed on a \
                 robot), and advertise NOTHING. That is the original beacon defect with \
                 every observable reporting health.\n\n\
                 Use `GatewayBeacon::new()` / `GatewayEgressPlane::new()`; the suppressed \
                 constructors are reachable only from a `#[cfg(test)] mod` or another \
                 `fn …_for_test` seam.",
                f.display()
            );
        }
        scanned.push(f.file_name().unwrap().to_string_lossy().to_string());
    }

    // ANTI-TAUTOLOGY (1): the walk reached the two modules that actually define these
    // constructors — otherwise a broken walk passes in silence.
    for known in ["beacon.rs", "egress.rs", "main.rs", "daemon.rs"] {
        assert!(
            scanned.iter().any(|f| f == known),
            "the src walk did not reach {known}; it saw {scanned:?}"
        );
    }

    // ANTI-TAUTOLOGY (2): the needles really are present in the tree BEFORE stripping.
    // Without this, deleting the constructors (or renaming them) would make the whole
    // test vacuously green while the rule it enforces quietly stopped existing.
    assert!(
        saw_needle_before_stripping,
        "no file under {} mentions any of {SUPPRESSED_CONSTRUCTORS:?} at all — if the \
         suppression seam was renamed, rename it here too; if it was DELETED, delete \
         this guard with it",
        src_dir().display()
    );
}

/// ANTI-TAUTOLOGY (3), and the one the other two cannot make: the strip removes the
/// TEST scaffolding and nothing else.
///
/// `production_code_of` blanks two whole classes of span. If either over-reached — a
/// `#[cfg(test)]` match that swallowed the rest of the file, a `_for_test` walk that
/// ate its enclosing `impl` — the negative assertion above would pass over an empty
/// string. So the shipped view of `beacon.rs` must still carry its production surface,
/// while the test call the suppression oracle makes must be gone.
#[test]
fn the_shipped_view_keeps_production_code_and_drops_only_the_test_scaffolding() {
    let beacon = src_dir().join("beacon.rs");
    let production = production_code_of(&beacon);

    for kept in [
        "pub fn plan_gateway_beacon",
        "pub fn ensure_raised",
        "pub fn advertiser_is_real",
        // The tail: `decision()` is the last item before `mod tests`, so its presence
        // proves the strip did not truncate the file on its way past the tests.
        "pub fn decision",
    ] {
        assert!(
            production.contains(kept),
            "the shipped view of beacon.rs lost `{kept}` — the strip over-reached, and \
             every negative assertion in this file would be running on a hole"
        );
    }

    // …and the scaffolding really did go: the definition, and the oracle's own call.
    assert!(
        !production.contains("without_mdns_for_test"),
        "the shipped view of beacon.rs still names the suppressed constructor"
    );
    assert!(
        !production.contains("a_suppressed_beacon_takes_the_same_decision"),
        "the `#[cfg(test)] mod tests` block was not stripped"
    );

    // The same, for the plane: its production constructor survives, its test twin does
    // not. Stated separately because the two files exercise DIFFERENT strip rules —
    // `beacon.rs`'s call lives in a `mod tests`, `egress.rs`'s inside a `_for_test` fn.
    let egress = production_code_of(&src_dir().join("egress.rs"));
    assert!(
        egress.contains("pub fn new(manager: Arc<TransportManager>)"),
        "the shipped view of egress.rs lost `GatewayEgressPlane::new`"
    );
    assert!(
        !egress.contains("without_mdns_for_test"),
        "the shipped view of egress.rs still names a suppressed constructor"
    );
}

/// The stripper is oracle-tested rather than trusted: every negative assertion above is
/// enforced over ITS output, so a stripper that dropped the tail would make them all
/// vacuous.
#[test]
fn code_only_strips_comments_and_literals_and_reports_an_unbalanced_scan() {
    let s = code_only("keep1 // gone\nkeep2 /* gone */ keep3");
    assert!(s.code.contains("keep1") && s.code.contains("keep2") && s.code.contains("keep3"));
    assert!(!s.code.contains("gone"));
    assert_eq!(s.unclosed_depth, 0);

    // Block comments NEST.
    let s = code_only("a /* one /* two */ still */ b");
    assert!(s.code.contains('a') && s.code.contains('b'));
    assert!(!s.code.contains("still"));

    // A `/*` inside a LINE comment opens nothing.
    let s = code_only("// /* not an opener\nreal_code");
    assert!(s.code.contains("real_code"));
    assert_eq!(s.unclosed_depth, 0);

    // String literals are blanked — including the raw form. This is the arm that
    // matters most here: both modules name the constructors in prose.
    let s = code_only("let x = \"without_mdns_for_test\"; let y = r#\"also /* here\"#; z");
    assert!(!s.code.contains("without_mdns_for_test"), "{}", s.code);
    assert!(!s.code.contains("also"), "{}", s.code);
    assert!(s.code.contains('z'));
    assert_eq!(
        s.unclosed_depth, 0,
        "a `/*` inside a raw literal must not open a comment"
    );

    // A char literal carrying a lone quote must not swallow the rest of the file.
    let s = code_only("let q = '\"'; marker_after");
    assert!(s.code.contains("marker_after"), "{}", s.code);

    // ... while a LIFETIME survives.
    let s = code_only("fn f<'a>(x: &'a str) -> &'a str { x }");
    assert!(s.code.contains("'a"), "{}", s.code);

    // An unterminated block is REPORTED, not silently swallowed.
    assert_eq!(code_only("a /* unterminated").unclosed_depth, 1);
    assert_eq!(code_only("a /* closed */ b").unclosed_depth, 0);
}

/// The two SPAN strips, oracle-tested on hand-written input for the same reason: against
/// the real (passing) tree their over- and under-reach are both invisible.
#[test]
fn the_span_strips_remove_exactly_the_test_scaffolding() {
    let tmp = std::env::temp_dir().join(format!(
        "strip_oracle_{}_{}.rs",
        std::process::id(),
        std::time::SystemTime::now()
            .duration_since(std::time::UNIX_EPOCH)
            .expect("clock")
            .as_nanos()
    ));
    let src = r#"
pub fn shipped() { helper_without_mdns_for_test_name_only(); }
pub fn thing_for_test() { without_mdns_for_test(); }
fn tail_marker() {}
#[cfg(test)]
mod tests {
    fn inner() { without_mdns_for_test(); }
}
pub fn after_tests() {}
"#;
    std::fs::write(&tmp, src).expect("write oracle fixture");
    let out = production_code_of(&tmp);
    std::fs::remove_file(&tmp).ok();

    // A `_for_test` fn and a `#[cfg(test)] mod` are both gone, bodies included.
    assert!(!out.contains("fn thing_for_test"), "{out}");
    assert!(!out.contains("mod tests"), "{out}");
    assert!(!out.contains("fn inner"), "{out}");

    // Everything on either side of them survives — the UNDER-reach half. `after_tests`
    // is the one that catches a `#[cfg(test)]` strip that ran to EOF.
    assert!(out.contains("fn shipped"), "{out}");
    assert!(out.contains("fn tail_marker"), "{out}");
    assert!(out.contains("fn after_tests"), "{out}");

    // And a production call whose identifier merely CONTAINS the needle is still
    // reported: the strip keys on `fn <name>_for_test` DEFINITIONS, never on a
    // substring appearing anywhere.
    assert!(
        out.contains("helper_without_mdns_for_test_name_only"),
        "{out}"
    );
}
