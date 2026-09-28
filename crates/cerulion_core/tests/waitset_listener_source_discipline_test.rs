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

/// The construction sites this walk must find. Twelve trigger subscriber wraps
/// across the live loop's source builders, the external doorbell, and
/// `WakeSet::wait`. A walk that stops finding them is inert, so it fails.
const MIN_CONSTRUCTIONS: usize = 14;

#[test]
fn every_listener_wake_source_comes_from_a_listener_something_drains() {
    let src = PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("src");
    let mut constructed = 0usize;
    let mut patterns = 0usize;
    let mut offenders: Vec<String> = Vec::new();

    for path in rust_sources(&src) {
        let text = std::fs::read_to_string(&path).expect("readable source file");
        for (line_no, arg) in listener_arguments(&text) {
            // `WaitSource::Listener(listener)` in a `match` arm or a `let ... else`
            // BINDS the listener out of a source the loop already holds; it does
            // not register a new one. A binding pattern is a bare identifier,
            // and no legal construction here is.
            if arg
                .chars()
                .all(|c| c.is_ascii_lowercase() || c.is_ascii_digit() || c == '_')
            {
                patterns += 1;
                continue;
            }
            constructed += 1;
            if !origin_is_drained(&arg) {
                offenders.push(format!("{}:{line_no}: {arg}", path.display()));
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
        "expected at least {MIN_CONSTRUCTIONS} listener wake-source constructions, walked \
         {constructed} (a renamed variant would make this guard inert)"
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

/// Every `WaitSource::Listener(` argument in `text`, as `(1-based line, argument
/// text with whitespace collapsed)`. The argument is closed at the paren that
/// BALANCES the opener, so a multi-line wrap reads the same as a one-liner.
fn listener_arguments(text: &str) -> Vec<(usize, String)> {
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
        out.push((text[..start].matches('\n').count() + 1, arg));
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
