// SPDX-License-Identifier: AGPL-3.0-only
//! Source-walking pin: only a listener SOMETHING drains may be registered as a
//! wake source.
//!
//! Under iceoryx2 0.10 a read no longer drains the event listener that woke it
//! (`CerulionSubscriber::drain_with_accounting_impl` says why), and the argument
//! that makes that safe is a claim about the whole tree rather than about that
//! function: the listener reached from there is a node's BODY subscriber on a
//! unified binding, which is attached to no WaitSet and polled by nothing, so
//! an event left in its queue wakes nobody.
//!
//! That claim is true today and it is written in a comment, which is the wrong
//! place for a premise the live loop depends on. Register a body subscriber's
//! listener in the WaitSet or in the idle poll and the never drained doorbell
//! leaves its fd readable forever: the live loop stops parking and free runs,
//! which is exactly the multi-process failure `unified_stale_wake_park_test`
//! exists for. That test would NOT catch it — it pins the Unified binding's
//! standalone `ListenerOnly`, not the body subscriber — so the premise is
//! walked here instead.
//!
//! The rule: every `WaitSource::Listener(...)` CONSTRUCTED in this crate takes
//! its listener from a trigger subscriber, from an external `Notified` binding's
//! doorbell, or from a `WakeSource` a caller handed to `WakeSet::wait`. Each of
//! those is drained by something. Anything else has to be justified, and adding
//! it here is where that argument gets made.

use std::path::{Path, PathBuf};

/// The argument forms a listener wake source may be built from.
///
/// * `trigger_subscribers[..].listener()` — the live loop's own trigger
///   subscribers, drained inside the step by `drain_level` (Unified) or
///   `try_receive_for_drain` (Separate and Sync).
/// * `&bell.listener` — an `ExternalBindingKind::Notified` doorbell, drained by
///   the external fire path that the wake belongs to.
/// * `&s.listener` — a `WakeSource` the CALLER owns and hands to
///   `WakeSet::wait`, which drains it before it waits again.
fn origin_is_drained(arg: &str) -> bool {
    (arg.contains("trigger_subscribers[") && arg.ends_with(".listener()"))
        || arg == "&bell.listener"
        || arg == "&s.listener"
}

/// Trigger subscriber wraps across the live loop's source builders.
const TRIGGER_SUBSCRIBER_WRAPS: usize = 12;

/// The two remaining origins: the external `Notified` doorbell and the
/// `WakeSource` a caller hands `WakeSet::wait`.
const OTHER_DRAINED_ORIGINS: usize = 2;

/// The construction sites this walk must find, DERIVED from the two counts
/// above rather than written down beside them, so the number and the sentence
/// that explains it cannot drift apart. A walk that stops finding them is
/// inert, so it fails.
const MIN_CONSTRUCTIONS: usize = TRIGGER_SUBSCRIBER_WRAPS + OTHER_DRAINED_ORIGINS;

#[test]
fn every_listener_wake_source_comes_from_a_listener_something_drains() {
    let src = PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("src");
    let mut constructed = 0usize;
    let mut patterns = 0usize;
    let mut offenders: Vec<String> = Vec::new();

    for path in rust_sources(&src) {
        let text = std::fs::read_to_string(&path).expect("readable source file");
        for site in listener_sites(&text) {
            if site.is_pattern {
                patterns += 1;
                continue;
            }
            constructed += 1;
            if !origin_is_drained(&site.arg) {
                offenders.push(format!("{}:{}: {}", path.display(), site.line, site.arg));
            }
        }
    }

    assert!(
        patterns > 0,
        "the walk found no `WaitSource::Listener` binding pattern at all, which means it is \
         not reading the file it thinks it is"
    );
    assert!(
        constructed >= MIN_CONSTRUCTIONS,
        "expected at least {MIN_CONSTRUCTIONS} listener wake-source constructions \
         ({TRIGGER_SUBSCRIBER_WRAPS} trigger subscriber wraps plus \
         {OTHER_DRAINED_ORIGINS} other drained origins), walked {constructed} \
         (a renamed variant would make this guard inert)"
    );
    assert!(
        offenders.is_empty(),
        "a listener was registered as a wake source from an origin nothing drains: \
         {offenders:?}\n\
         Under 0.10 a read does not drain the listener that woke it, so a wake source whose \
         listener no code drains keeps its fd readable forever and the live loop free runs. \
         If the new origin IS drained, say where in `origin_is_drained` and add it there."
    );
}

/// One `WaitSource::Listener(...)` site.
struct ListenerSite {
    /// 1-based line of the opener.
    line: usize,
    /// The argument text, whitespace collapsed, so a multi-line wrap reads the
    /// same as a one-liner.
    arg: String,
    /// A binding PATTERN (a `match` arm, or a `let ... = ... else`) rather than
    /// a construction. A pattern takes a listener the loop already holds out of
    /// a source; it registers nothing.
    is_pattern: bool,
}

/// Every `WaitSource::Listener(` site in `text`, classified.
///
/// The classification is by POSITION, not by the argument's shape. Shape was
/// the first attempt and it had a hole: it called any bare snake_case argument a
/// pattern, so `WaitSource::Listener(body_listener)` — a real construction from
/// a local binding, which is exactly the never-drained body subscriber this file
/// exists to refuse — was silently skipped instead of checked.
///
/// What actually separates the two is what FOLLOWS the balanced close paren.
/// A pattern is always immediately followed by `=>` (a match arm) or `=` (a
/// `let` destructuring). A construction is followed by a comma, a close paren,
/// a brace or end of line. That holds whatever the argument looks like.
fn listener_sites(text: &str) -> Vec<ListenerSite> {
    const OPEN: &str = "WaitSource::Listener(";
    let mut out = Vec::new();
    for (start, _) in text.match_indices(OPEN) {
        let body_at = start + OPEN.len();
        let mut depth = 1usize;
        let mut end = None;
        for (offset, ch) in text[body_at..].char_indices() {
            match ch {
                '(' => depth += 1,
                ')' => {
                    depth -= 1;
                    if depth == 0 {
                        end = Some(body_at + offset);
                        break;
                    }
                }
                _ => {}
            }
        }
        // An unbalanced opener is a truncated read, not a site; skip it rather
        // than guess, and the count floor above catches a walk that skips them all.
        let Some(end) = end else { continue };
        let arg = text[body_at..end]
            .split_whitespace()
            .collect::<Vec<_>>()
            .join(" ")
            .trim_end_matches(',')
            .to_string();
        if arg.is_empty() {
            continue;
        }
        let after = text[end + 1..].trim_start();
        let is_pattern =
            after.starts_with("=>") || (after.starts_with('=') && !after.starts_with("=="));
        out.push(ListenerSite {
            line: text[..start].matches('\n').count() + 1,
            arg,
            is_pattern,
        });
    }
    out
}

/// Every `.rs` file under a directory, recursively.
fn rust_sources(dir: &Path) -> Vec<PathBuf> {
    let mut out = Vec::new();
    let mut stack = vec![dir.to_path_buf()];
    while let Some(d) = stack.pop() {
        for entry in std::fs::read_dir(&d).expect("readable source directory") {
            let path = entry.expect("directory entry").path();
            if path.is_dir() {
                stack.push(path);
            } else if path.extension().and_then(|e| e.to_str()) == Some("rs") {
                out.push(path);
            }
        }
    }
    out
}

// ===========================================================================
// The classifier's own arms, including the counter-example the shape-based
// first attempt got wrong.
// ===========================================================================

/// A construction whose argument is a bare snake_case local is CHECKED, not
/// waved through as a binding pattern.
///
/// This is the hole the shape test had. `WaitSource::Listener(body_listener)`
/// is indistinguishable from a match binding by shape alone, and it is precisely
/// the shape a future change would take if it registered a node body
/// subscriber's listener — the never drained doorbell this whole file exists to
/// refuse. Position tells them apart.
#[test]
fn a_bare_identifier_construction_is_checked_and_refused() {
    let src = "\
        let s = WaitSource::Listener(body_listener);\n\
    ";
    let sites = listener_sites(src);
    assert_eq!(sites.len(), 1, "one site expected, got {}", sites.len());
    assert!(
        !sites[0].is_pattern,
        "a bare identifier in construction position must NOT be classified as a \
         binding pattern; that misclassification is what let an undrained body \
         subscriber's listener through"
    );
    assert!(
        !origin_is_drained(&sites[0].arg),
        "`{}` is not one of the drained origins, so the walk must refuse it",
        sites[0].arg
    );
}

/// The two real pattern positions are still recognised, so the walk does not
/// start reporting every `match` arm as an undrained registration.
#[test]
fn a_match_arm_and_a_let_destructuring_are_still_patterns() {
    for src in [
        "            WaitSource::Listener(listener) => attach(*listener),\n",
        "        let WaitSource::Listener(listener) = source else { return };\n",
        "        let waitset::WaitSource::Listener(listener) = source else { return };\n",
    ] {
        let sites = listener_sites(src);
        assert_eq!(sites.len(), 1, "one site expected in {src:?}");
        assert!(
            sites[0].is_pattern,
            "this is a binding pattern, not a registration: {src:?}"
        );
    }
}

/// Each drained origin is accepted in construction position, so the rule admits
/// what the live loop actually does.
#[test]
fn every_drained_origin_is_accepted_in_construction_position() {
    for src in [
        "                    WaitSource::Listener(\n                        self.trigger_subscribers[binding.subscriber_idx].listener(),\n                    ),\n",
        "                    WaitSource::Listener(&bell.listener)\n",
        "                    WaitSource::Listener(&s.listener),\n",
    ] {
        let sites = listener_sites(src);
        assert_eq!(sites.len(), 1, "one site expected in {src:?}");
        assert!(!sites[0].is_pattern, "construction position expected: {src:?}");
        assert!(
            origin_is_drained(&sites[0].arg),
            "`{}` is a drained origin and must be accepted",
            sites[0].arg
        );
    }
}
