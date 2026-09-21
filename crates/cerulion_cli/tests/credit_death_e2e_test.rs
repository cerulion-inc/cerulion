//! The credit-death ACCEPTANCE arms, over REAL
//! processes.
//!
//! The in-process pins cover the reporter itself: hand-built `CreditEdgePlan`s, a
//! hand-called `report_deaths`, and a structural walk asserting the two
//! `PeerLossPolicy::Continue` arms reach the one report site. What none of it
//! can prove is that a REAL worker death on a REAL creditable split edge
//! arrives at that site at all — the supervisor has to mint the word, stamp
//! the plan, spawn the ranks, observe the death, resolve its group, and only
//! then report. This file drives that whole path with `graph run` and reads
//! the supervisor's own log.
//!
//! It is also what catches a report site whose argument has been neutered: the
//! in-process pins stay green on that change, while with the arms below a
//! neutered argument means no line reaches this log.
//!
//! SCOPE — the producer half is NOT driven here.
//! The `parked`-bit sweep clears a bit that only a PARKED producer sets
//! (`park_enter` / `ParkedEdgeGuard` around the producer park), and no arm in
//! this file parks a producer before killing it, so there is NO bit to sweep
//! and an e2e asserting one would be asserting a value nothing writes. The
//! sweep's CORRECTNESS (edge-local slot, right word, idempotence) is pinned
//! over real `MappedCredit` segments in `graph_cmd.rs`'s `c6_*` arms.
#![cfg(unix)]

use std::path::Path;
use std::time::{Duration, Instant};

use serial_test::serial;

mod mp_support;
use mp_support::*;

/// The consumer-death head, as it appears in the supervisor's log.
const HEAD: &str = "lost its CONSUMER";
/// The retraction line.
const RETRACTION: &str = "RETRACTION";

/// `build_mp_workspace`'s split (`p0: [ticker, relay]`, `p1: [sink]`) with the
/// sink's `trigger_in` declaring `backpressure = block`.
///
/// That makes `<prefix>/relay/cmd` a CREDITABLE edge — exactly one in-graph
/// producer (`relay`), every consumer `block` — split across ranks, so the
/// supervisor mints it a cross-process credit word. Both the node's SOURCE and
/// its CDYLIB are replaced, so the plan-time check and the mint agree (a
/// mismatch is its own refusal, pinned in `mp_auto_partition_e2e_test`).
///
/// # Why these arms run under `--record`
///
/// `spawn_mp_record_*` is the shared harness spawner, so `--record` rides along —
/// and it is not incidental: the headline arm needs EVIDENCE that the producer
/// stepped before the kill, and the bag is the second, independent half of that
/// (the first is the credit word's own `outstanding`). The cost is one bagd
/// grandchild per arm, which `BagdGuard` reaps.
fn build_creditable_split_workspace(root: &Path, prefix: &str) {
    build_mp_workspace(root, prefix);
    let fixtures = Path::new(env!("CARGO_MANIFEST_DIR"))
        .parent()
        .unwrap()
        .join("test_fixtures");
    let block = "test_node_macro_trigger_block_cdylib";
    std::fs::copy(
        fixtures.join(block).join("src/lib.rs"),
        root.join("nodes/sink/src/lib.rs"),
    )
    .expect("stage the block sink source");
    std::fs::copy(
        fixture_cdylib(block),
        root.join("target/debug").join(dylib_file("sink")),
    )
    .expect("stage the block sink cdylib");
}

/// Wait until `marker` appears in the merged child log, or the deadline.
fn wait_for_log(stdout: &Path, stderr: &Path, marker: &str, deadline: Duration) -> bool {
    let end = Instant::now() + deadline;
    while Instant::now() < end {
        if merged(stdout, stderr).contains(marker) {
            return true;
        }
        std::thread::sleep(Duration::from_millis(100));
    }
    false
}

/// The merged child log with ANSI escapes stripped.
///
/// `tracing`'s fmt layer wraps a field's NAME and its `=` in escapes (its
/// `ansi` default is a compile-time feature, NOT a tty probe), so a
/// `key=value` assertion against the raw capture is silently unsatisfiable.
/// Fourth copy of this helper in the tree — separate test binaries cannot
/// share one.
fn merged(stdout: &Path, stderr: &Path) -> String {
    // The "\n" is load-bearing: the two captures are concatenated, and without a
    // separator stdout's last line GLUES to stderr's first. That can hide a line
    // from a per-line matcher, merge two heads into one (defeating an
    // `assert_eq!(heads.len(), 1)`), or fabricate a `key=value` whitespace token
    // that neither stream contains.
    let raw = format!("{}\n{}", read_file(stdout), read_file(stderr));
    let bytes = raw.as_bytes();
    let mut out: Vec<u8> = Vec::with_capacity(bytes.len());
    let mut i = 0;
    while i < bytes.len() {
        if bytes[i] == 0x1b && i + 1 < bytes.len() && bytes[i + 1] == b'[' {
            i += 2;
            while i < bytes.len() && !(0x40..=0x7e).contains(&bytes[i]) {
                i += 1;
            }
            i += 1;
            continue;
        }
        out.push(bytes[i]);
        i += 1;
    }
    String::from_utf8_lossy(&out).into_owned()
}

/// `key=value` as a whole WHITESPACE token.
fn has_field(log: &str, key: &str, value: &str) -> bool {
    let want = format!("{key}={value}");
    log.split_whitespace().any(|t| t == want)
}

/// The supervisor's own "deployment live" breadcrumb: every worker spawned
/// AND READY, GO signaled.
const LIVE: &str = "GO signaled; deployment live";

/// Block until the deployment is LIVE — because killing a worker before it
/// signals READY takes an entirely DIFFERENT supervisor path.
///
/// Workers are spawned SEQUENTIALLY, each gated on the previous one's READY
/// sentinel, so a victim's pid exists for a long time before that victim is
/// live (MEASURED: `p0` spawned at `.808`, READY observed at
/// `38.125` — 317 ms, and `p1` is spawned only then). A SIGKILL inside that
/// window makes the supervisor refuse the whole run —
/// `worker process for group 'p1' exited (signal: 9 (SIGKILL)) before
/// signaling READY ... sibling workers were killed`, a nonzero bring-up abort
/// — instead of taking the steady-state departure path these arms exist to
/// drive, so NO death warn is emitted and the arm fails having proved nothing
/// about the reporter.
///
/// The rule to keep in front of the next reader:
/// waiting for a PID is not waiting for a live worker. This is the barrier
/// that is, and it is strictly stronger than the per-victim READY line — GO is
/// written only after EVERY rank is ready, so both halves of the credit word
/// are wired by the time it appears.
fn wait_until_live(stdout: &Path, stderr: &Path) {
    assert!(
        wait_for_log(stdout, stderr, LIVE, Duration::from_secs(90)),
        "the deployment never reported itself live, so a kill here would hit the \
         bring-up refusal path instead of a departure\n{}",
        merged(stdout, stderr)
    );
}

/// The credit-word NAMESPACE this run created its words in, read from the
/// supervisor's OWN log rather than recomputed here.
///
/// The namespace carries the supervisor pid and a per-run nonce, so rebuilding it
/// test-side would be a second copy of a production naming rule — and the failure
/// mode is not a loud mismatch but a SILENT wrong answer: `open_unowned` on a name
/// nothing created returns `NotFound`, which reads as "there is no credit word
/// here" rather than "the test composed the wrong name". Reading the `ns=` field
/// means the arms map the page the run actually minted.
fn credit_ns(stdout: &Path, stderr: &Path) -> String {
    assert!(
        wait_for_log(
            stdout,
            stderr,
            CREDIT_WORDS_CREATED,
            Duration::from_secs(60)
        ),
        "the supervisor must log the credit-word namespace it created\n{}",
        merged(stdout, stderr)
    );
    let log = merged(stdout, stderr);
    log.lines()
        .find(|l| l.contains(CREDIT_WORDS_CREATED))
        .and_then(|l| {
            l.split_whitespace()
                .find_map(|t| t.strip_prefix("ns=").map(str::to_string))
        })
        .unwrap_or_else(|| panic!("the credit-words line must carry `ns=`\n{log}"))
}

/// The supervisor line that names the credit namespace.
const CREDIT_WORDS_CREATED: &str = "cross-process block credit words created";

/// Poll an OPENED credit word until `pred` holds, or fail naming what it saw.
///
/// This is the arm's window onto the CONDITION rather than the report: the word
/// is the same SHM page the producer's pre-fire gate reads, so `outstanding`
/// and `is_full` are the production quantities, not a test-side model of them.
fn wait_for_credit(
    word: &cerulion_core::credit::MappedCredit,
    deadline: Duration,
    what: &str,
    seq_baseline: Option<u32>,
    pred: impl Fn(&cerulion_core::credit::CreditShared) -> bool,
) {
    let end = Instant::now() + deadline;
    while Instant::now() < end {
        if pred(word) {
            return;
        }
        std::thread::sleep(Duration::from_millis(20));
    }
    // THE INTERPRETATION IS PER-WAIT, because the same reading means opposite
    // things at this helper's two call sites and a diagnostic that says
    // otherwise sends the reader at the wrong root cause.
    //
    // Waiting for FIRST TRAFFIC (a baseline was taken): the consumer is alive
    // and draining, so `wake_seq` not advancing is the fault.
    //
    // Waiting for `is_full` AFTER the kill (no baseline): traffic is already
    // established and NOTHING drains any more, so a `wake_seq` sitting still is
    // the EXPECTED state, not the fault — reading it as "no frame ever crossed"
    // would be exactly backwards.
    let reading = match seq_baseline {
        Some(seq0) => {
            let now = word.wake_seq_snapshot();
            format!(
                "wake_seq advanced {} since this wait opened ({seq0} -> {now}). \
                 `outstanding` is published-minus-drained and a consumer that \
                 keeps up holds it at 0, so a 0 there means little on its own; \
                 `wake_seq` counts DRAINS and is monotone, so an advance of 0 is \
                 the reading that says no frame ever crossed this edge.",
                now.wrapping_sub(seq0)
            )
        }
        None => "no wake_seq baseline was taken for this wait, so the absolute \
                 value above is not evidence about traffic. This wait runs AFTER \
                 traffic is established and with the consumer dead NOTHING drains, \
                 so a wake_seq that is not advancing is the EXPECTED state here \
                 rather than the fault — read `outstanding` against `depth` instead."
            .to_string(),
    };
    panic!(
        "the credit word never reached `{what}` within {deadline:?} — \
         outstanding={} wake_seq={} depth={} (is_full={})\n  {reading}",
        word.outstanding(),
        word.wake_seq_snapshot(),
        word.depth(),
        word.is_full()
    );
}

/// The level token in a `tracing` line's HEADER, if it has one.
///
/// Read positionally rather than by scanning the whole line for any level word:
/// a line whose MESSAGE or field value happens to contain "WARN" would satisfy a
/// whole-line scan, which silently widens every "exactly N at this level"
/// oracle. The fmt layer writes `<timestamp> <LEVEL> <target>: <msg>`, so the
/// level is the first token that is a level.
fn level_of(line: &str) -> Option<&str> {
    line.split_whitespace()
        .take(3)
        .find(|t| matches!(*t, "TRACE" | "DEBUG" | "INFO" | "WARN" | "ERROR"))
}

/// Lines carrying `marker` whose HEADER level is exactly `level`.
fn lines_at<'a>(log: &'a str, level: &str, marker: &str) -> Vec<&'a str> {
    log.lines()
        .filter(|l| l.contains(marker) && level_of(l) == Some(level))
        .collect()
}

/// Source with comments AND string literals removed.
///
/// Load-bearing for the walk below, whose own prose names the pattern it
/// forbids: without stripping, the guard fails on its own explanation.
///
/// STRING LITERALS ARE STRIPPED FIRST, and that is not a refinement — it is what
/// makes the walk run at all. A `/*` inside a literal with no closer (three
/// scanned files have one: two spell a ``msgs/*`` topic pattern, one a
/// `"nodes/*"` workspace member) opens a block comment that never closes, so a
/// version that only counted depth returned `Err` for those files and the walk
/// panicked before it classified anything. Skipping such a file instead would
/// re-open the very hole the walk exists to close, so the literals are modelled:
/// normal and byte strings with their escapes, raw strings with hash-count
/// matching, and char literals — the last distinguished from LIFETIMES (`'a`,
/// `'static`), which are not literals and must pass through as code.
///
/// Still FAILS CLOSED on a genuinely unterminated block comment: at that point
/// code and comment are no longer separable, and the correct answer is to refuse
/// rather than return a truncated view in which every forbidden pattern has
/// silently vanished.
///
/// Newlines inside stripped regions are PRESERVED so the line numbers this walk
/// reports stay the file's own.
fn code_only(src: &str) -> Result<String, String> {
    let c: Vec<char> = src.chars().collect();
    let mut out = String::with_capacity(src.len());
    let (mut i, mut depth) = (0usize, 0usize);
    let at = |i: usize, s: &str| -> bool {
        c[i..].iter().copied().take(s.chars().count()).eq(s.chars())
    };

    while i < c.len() {
        // Inside a block comment nothing else applies — not a string, not a
        // line comment. Only nesting and the closer.
        if depth > 0 {
            if i + 1 < c.len() && c[i] == '/' && c[i + 1] == '*' {
                depth += 1;
                i += 2;
            } else if i + 1 < c.len() && c[i] == '*' && c[i + 1] == '/' {
                depth -= 1;
                i += 2;
            } else {
                if c[i] == '\n' {
                    out.push('\n');
                }
                i += 1;
            }
            continue;
        }

        // Comments.
        if i + 1 < c.len() && c[i] == '/' && c[i + 1] == '/' {
            while i < c.len() && c[i] != '\n' {
                i += 1;
            }
            continue;
        }
        if i + 1 < c.len() && c[i] == '/' && c[i + 1] == '*' {
            depth += 1;
            i += 2;
            continue;
        }

        // RAW strings: r"…", r#"…"#, br##"…"##. The closer must carry the SAME
        // number of hashes, which is the whole point of the form.
        let raw_start = if at(i, "br") {
            Some(i + 2)
        } else if at(i, "r") {
            Some(i + 1)
        } else {
            None
        };
        if let Some(mut h) = raw_start {
            let hash_start = h;
            while h < c.len() && c[h] == '#' {
                h += 1;
            }
            if h < c.len() && c[h] == '"' {
                let hashes = h - hash_start;
                let mut j = h + 1;
                loop {
                    if j >= c.len() {
                        // An unterminated raw string is a broken file, not a
                        // comment-vs-code ambiguity: stop stripping here rather
                        // than swallowing the rest.
                        i = c.len();
                        break;
                    }
                    if c[j] == '"' && c[j + 1..].iter().take(hashes).all(|&x| x == '#') {
                        i = j + 1 + hashes;
                        break;
                    }
                    if c[j] == '\n' {
                        out.push('\n');
                    }
                    j += 1;
                }
                continue;
            }
        }

        // Normal and byte strings, with escapes.
        let str_start = if at(i, "b\"") {
            Some(i + 1)
        } else if c[i] == '"' {
            Some(i)
        } else {
            None
        };
        if let Some(q) = str_start {
            let mut j = q + 1;
            while j < c.len() && c[j] != '"' {
                if c[j] == '\\' {
                    j += 1; // skip the escaped char, `\"` and `\\` included
                }
                if j < c.len() && c[j] == '\n' {
                    out.push('\n');
                }
                j += 1;
            }
            i = if j < c.len() { j + 1 } else { c.len() };
            continue;
        }

        // Char literals — but NOT lifetimes. `'x'` and `'\n'` are literals;
        // `'a` / `'static` are lifetimes and stay as code.
        if c[i] == '\'' {
            let is_char = if i + 1 < c.len() && c[i + 1] == '\\' {
                // `'\''`, `'\\'`, `'\n'`, `'\u{7f}'`. Step PAST the escaped
                // char first — otherwise `'\''` stops on the quote it escapes
                // and leaves the real closer to be read as a lifetime.
                let mut j = i + 3;
                while j < c.len() && c[j] != '\'' {
                    j += 1;
                }
                if j < c.len() {
                    i = j + 1;
                    true
                } else {
                    false
                }
            } else if i + 2 < c.len() && c[i + 2] == '\'' {
                i += 3;
                true
            } else {
                false
            };
            if is_char {
                continue;
            }
        }

        out.push(c[i]);
        i += 1;
    }

    if depth > 0 {
        return Err(format!(
            "unterminated `/*` (depth {depth}) — the stripped view cannot be trusted, so \
             this walk refuses rather than scanning a file it may have truncated"
        ));
    }
    Ok(out)
}

/// The receiver a reap marker at `lines[n]` applies to, if any.
///
/// A separate function so it can be PINNED. Inline in the repo walk nothing
/// can reach it: `guard_receivers` has no reap-side logic and the tree
/// contains ZERO chained reaps, so deleting the logic would leave the binary
/// green. [`reap_receiver_resolves_same_line_and_chained_receivers`] drives it
/// directly.
///
/// A same-line reap (`guard.try_wait()`) yields its receiver directly. A CHAINED
/// reap puts the marker on its own continuation line —
/// `guard_a\n    .try_wait()`, which is how rustfmt writes it — so the receiver
/// is the last identifier on the previous non-blank, non-comment line.
fn reap_receiver(lines: &[&str], n: usize, marker: &str) -> Option<String> {
    let line = lines.get(n)?;
    // The marker check comes FIRST: `split` always yields at least one piece, so
    // computing `head` before it would read as a guard running after the work it
    // guards. Deliberate asymmetry below, pinned both ways: the SAME-LINE arm is
    // fail-CLOSED (a non-empty head with no trailing identifier — `Vec::new()
    // .kill()` on one line — is `None` rather than a guess), while the CHAINED
    // arm is fail-OPEN (it takes the last identifier on the previous non-blank
    // line, so `Vec::new()\n    .kill()` resolves to the method name `new`).
    // Fail-open is the safe direction here: a spurious receiver can only add an
    // escape to report, never hide one.
    if !line.contains(marker) {
        return None;
    }
    let head = line.split(marker).next()?;
    let same: String = head
        .chars()
        .rev()
        .take_while(|c| c.is_alphanumeric() || *c == '_')
        .collect::<Vec<_>>()
        .into_iter()
        .rev()
        .collect();
    if !same.is_empty() {
        return Some(same);
    }
    if !head.trim().is_empty() {
        return None;
    }
    for back in (0..n).rev() {
        let t = lines[back].trim();
        if t.is_empty() || t.starts_with("//") {
            continue;
        }
        let prev: String = t
            .chars()
            .rev()
            .skip_while(|c| !(c.is_alphanumeric() || *c == '_'))
            .take_while(|c| c.is_alphanumeric() || *c == '_')
            .collect::<Vec<_>>()
            .into_iter()
            .rev()
            .collect();
        return if prev.is_empty() { None } else { Some(prev) };
    }
    None
}

/// Every name in `code` that holds — or is fed — an `mp_support::ChildGuard`.
///
/// The walk is only as good as this function: a receiver it misses is a reap it
/// cannot see, and a same-line constructor match misses five of thirteen binaries.
/// The cause is NOT one thing: rustfmt
/// wrapping a long constructor onto the next line accounts for one of the five,
/// while four are guards whose BIND SITE carries no constructor — handed back by
/// a TUPLE-returning local helper, or by a helper in `mp_support`. Two of those
/// four files DO construct a guard somewhere (`mp_support_verdict_test.rs:72`,
/// `signal_matrix_e2e_test.rs:113`, both inside a local helper); what is missing
/// is a `ChildGuard::` on the line the name is bound. Pinned directly
/// by [`guard_receivers_sees_every_shape_the_tree_actually_uses`], and the reap
/// side by [`reap_receiver_resolves_same_line_and_chained_receivers`], rather
/// than only through the repo walk, which cannot exhibit every shape.
fn guard_receivers(code: &str, helpers: &str) -> Vec<String> {
    let lines: Vec<&str> = code.lines().collect();
    let mut out: Vec<String> = Vec::new();

    // (0) FUNCTIONS THAT HAND BACK A GUARD, recognised by a return type that
    // carries `ChildGuard`. (A `&mut Child)` needle is no substitute: it matches
    // nothing in the tree.) Without this source the receiver set is EMPTY in every file that never
    // writes `ChildGuard::` itself: `signal_matrix` and `mp_support_verdict`
    // return one from a helper, and `mp_record`/`mp_record_replay` get theirs
    // from `spawn_mp_record`, which lives in `mp_support` — so the shared module
    // is scanned too, or those two can never be covered from their own text.
    let mut guard_fns: Vec<String> = Vec::new();
    for src in [code, helpers] {
        let ls: Vec<&str> = src.lines().collect();
        for (i, l) in ls.iter().enumerate() {
            let Some(rest) = l.split("fn ").nth(1) else {
                continue;
            };
            let name: String = rest
                .chars()
                .take_while(|c| c.is_alphanumeric() || *c == '_')
                .collect();
            if name.is_empty() {
                continue;
            }
            // A signature can wrap; join until the body opens.
            let mut sig = String::new();
            for l2 in ls.iter().skip(i).take(10) {
                sig.push_str(l2);
                if l2.contains('{') {
                    break;
                }
            }
            if let Some(ret) = sig.split("->").nth(1) {
                if ret.contains("ChildGuard") {
                    guard_fns.push(name);
                }
            }
        }
    }

    let hands_back_a_guard = |line: &str| -> bool {
        line.contains("ChildGuard::")
            || guard_fns
                .iter()
                .any(|f| line.contains(&format!("{f}(")) && !line.contains(&format!("fn {f}(")))
    };

    // Push every name a `let` pattern binds: `x`, `mut x`, and each element of a
    // tuple destructure `(a, mut b, _c)` — `mp_default_ns`'s reaped `guard_a` is
    // tuple-bound, so a single-name parse would miss it.
    fn push_binds(after_let: &str, out: &mut Vec<String>) {
        let head = after_let.split('=').next().unwrap_or("");
        for tok in head.split(|c: char| !(c.is_alphanumeric() || c == '_')) {
            if tok.is_empty() || tok == "mut" {
                continue;
            }
            out.push(tok.to_string());
        }
    }

    for (i, line) in lines.iter().enumerate() {
        if hands_back_a_guard(line) {
            for back in (0..=i).rev() {
                let t = lines[back].trim();
                // SKIP a comment line rather than stopping on it. A blank line
                // needs no skip of its own: it
                // matches neither the `let ` probe nor the `;` break below,
                // so the walk-back continues over it — an explicit
                // `t.is_empty()` test here is output-equivalent (every arm and
                // all 25 receiver sets stay identical without it).
                //
                // This skip is PRODUCTION-INERT: a literal-`//` line drives a
                // branch no production input reaches, because
                // the walk is fed `code_only`
                // output, where a comment has already become a blank. It
                // is kept for one reason — the synthetic `// set up;`
                // arm in `guard_receivers_sees_every_shape_the_tree_actually_uses`
                // goes RED when it is deleted, because a comment ending in `;`
                // would otherwise satisfy the `;` break below and stop the
                // walk-back. A branch reachable only from a test is worth
                // keeping only while a test kills its deletion; this one does.
                if back < i && t.starts_with("//") {
                    continue;
                }
                if let Some(after) = lines[back].split("let ").nth(1) {
                    push_binds(after, &mut out);
                    break;
                }
                // Never walk past a PREVIOUS statement (the guard does not apply
                // to the constructor line itself, which ends in `;` when it fits
                // on one line).
                if back < i && lines[back].trim_end().ends_with(';') {
                    break;
                }
            }
        }
        // PARAMETERS typed `ChildGuard` / `&mut ChildGuard`, and — in a file that
        // OWNS guards — a parameter typed `&mut Child` / `&mut std::process::Child`.
        // The second is the DEREF FEED: `Deref` lets every caller pass a guard
        // straight into such a sink, so a helper shaped like
        // `wait_for_log_line(child: &mut Child, ..)` can reap one. Without this
        // source, a file that declares such a sink and feeds it a guard (signature
        // AND call) leaves the walk GREEN.
        //
        // `ChildGuard` ENDS with `Child`, so the type is taken up to whatever
        // ENDS it and compared for EQUALITY (never by substring), or every guard
        // parameter would also read as a raw-`Child` sink. The discriminating
        // negative is the `&mut ChildProcess` arm: it passes this gate and must
        // still not be a receiver.
        //
        // The gate is a PARAMETER-SHAPED line, not `owns_guards` alone. Opening
        // it on every line of a guard-owning file mints 9 spurious receivers
        // across 8 files (`extra`, `trace_dir`, `label`, `dir`, `established`,
        // `root`, …) and ZERO true ones — any `name: Type` fragment in any
        // expression qualifies.
        //
        // `owns_guards` is read on `code_only`-stripped text, so it is FALSE for
        // `mp_record_e2e_test.rs` and `mp_record_replay_e2e_test.rs`, whose only
        // mention of `ChildGuard` sits in a whole-line comment: the deref-feed
        // source is off for exactly the two binaries the return-type source
        // exists to cover. Neither declares a raw-`Child` parameter today, so no
        // escape is missed — but it is a hole, not a design.
        let owns_guards = code.contains("ChildGuard");
        let parameter_shaped = line.contains(": ChildGuard")
            // also covers `: &mut ChildGuard`, which the equality check then
            // classifies as a guard parameter rather than a deref feed
            || line.contains(": &mut Child")
            || line.contains(": &mut std::process::Child");
        if parameter_shaped {
            for part in line.split(',') {
                let Some((nm, ty)) = part.split_once(':') else {
                    continue;
                };
                // Take the TYPE up to whatever ends it — `)`, `,` or the body's
                // `{` — rather than trimming a fixed set, or `&mut Child) {}`
                // (a one-parameter fn on one line) never reduces to `Child`.
                let bare = ty
                    .trim()
                    .trim_start_matches("&mut ")
                    .split([')', ',', '{'])
                    .next()
                    .unwrap_or("")
                    .trim();
                let is_guard_param = ty.contains("ChildGuard");
                let is_deref_feed =
                    owns_guards && (bare == "Child" || bare == "std::process::Child");
                if !is_guard_param && !is_deref_feed {
                    continue;
                }
                let nm = nm
                    .rsplit(|c: char| !(c.is_alphanumeric() || c == '_'))
                    .find(|t| !t.is_empty())
                    .unwrap_or("");
                if !nm.is_empty() {
                    out.push(nm.to_string());
                }
            }
        }
    }
    out.sort();
    out.dedup();
    out
}

/// `reap_receiver` resolves the receiver a reap marker applies to.
///
/// Pinned DIRECTLY, because nothing else can reach it: the tree contains no
/// chained reaps, so the repo walk never exercises the continuation branch, and
/// `guard_receivers` (the other directly pinned function) has no
/// reap-side logic at all. Without this test, deleting the branch leaves the binary green.
#[test]
fn reap_receiver_resolves_same_line_and_chained_receivers() {
    // SAME LINE.
    let same = ["    if guard.try_wait().is_some() {"];
    assert_eq!(
        reap_receiver(&same, 0, ".try_wait()").as_deref(),
        Some("guard")
    );
    // CHAINED — the marker on its own continuation line, rustfmt's shape and the
    // one a same-line-only resolver cannot see.
    let chained = [
        "    assert!(",
        "        guard_a",
        "            .try_wait()",
        "            .is_none(),",
    ];
    assert_eq!(
        reap_receiver(&chained, 2, ".try_wait()").as_deref(),
        Some("guard_a"),
        "a chained reap resolves to the identifier that owns the chain"
    );
    // Chained across a BLANK line (what a comment looks like post-`code_only`).
    let over_blank = ["        guard_b", "", "            .kill()"];
    assert_eq!(
        reap_receiver(&over_blank, 2, ".kill()").as_deref(),
        Some("guard_b")
    );
    // A line without the marker resolves to nothing.
    assert_eq!(reap_receiver(&same, 0, ".kill()"), None);
    // A chain headed by a LITERAL still resolves — to the method name, and
    // NOT to `None`:
    // the walk-back takes the last identifier on the
    // previous non-blank line, and `Vec::new()` ends in `new`. Non-vacuous
    // (deleting the walk-back makes it `None`), and its consequence is worth
    // naming: `something(root)\n    .kill()` resolves to `root`. Under a
    // parameter gate wide enough to mint `root` as a receiver (two files), that
    // pair reports an escape; the narrow gate does not mint it,
    // and the resolver's fail-OPEN direction remains the safe one — it can
    // only ever over-report an escape, never hide one.
    let literal_head = ["    Vec::new()", "        .kill()"];
    assert_eq!(
        reap_receiver(&literal_head, 1, ".kill()").as_deref(),
        Some("new")
    );
    // The two genuine `None` exits.
    //
    // (i) CHAINED with nothing to take: the previous non-blank line carries no
    // identifier character at all.
    assert_eq!(
        reap_receiver(&["    );", "        .kill()"], 1, ".kill()"),
        None
    );
    // (ii) SAME LINE, fail-CLOSED: the head is non-empty but ends in no
    // identifier, so there is no receiver to name and none is guessed.
    //
    // The fixture carries a PREVIOUS line on purpose. A
    // one-element slice would make the arm unable to fail for the branch it
    // names: with the fail-closed `return None` deleted, the walk-back has
    // nowhere to walk and lands on the SAME trailing `None`, so that deletion
    // passes. With `guard_b` above it the two paths give different
    // answers — `None` fail-closed, `Some("guard_b")` without it — and only
    // then does the assertion mean what `reap_receiver`'s doc claims.
    assert_eq!(
        reap_receiver(&["        guard_b", "    Vec::new().kill()"], 1, ".kill()"),
        None
    );
}

/// The detector sees every shape a guard receiver actually takes.
///
/// Synthetic snippets, because the tree cannot exhibit them all at once — and
/// because the tree passing proves little: a same-line-only detector also
/// passes it, while being blind in five of the thirteen scanned binaries.
#[test]
fn guard_receivers_sees_every_shape_the_tree_actually_uses() {
    let no_helpers = "";
    // WRAPPED bind.
    let wrapped = "    let mut guard =\n        ChildGuard::spawn_group_leader(&mut cmd).unwrap();";
    assert!(guard_receivers(wrapped, no_helpers).contains(&"guard".to_string()));
    // Same-line bind.
    let same = "    let g = ChildGuard::single_process(child);";
    assert!(guard_receivers(same, no_helpers).contains(&"g".to_string()));
    // PARAMETERS, both spellings.
    assert!(
        guard_receivers("fn poll(child: &mut ChildGuard, p: &Path) {}", no_helpers)
            .contains(&"child".to_string())
    );
    assert!(guard_receivers("fn take(g: ChildGuard) {}", no_helpers).contains(&"g".to_string()));

    // --- THE SHAPES A SAME-LINE DETECTOR MISSES. They do not all fail the
    // same way, so what each arm catches is stated per arm, as MEASURED against
    // a detector with that one piece removed:
    //
    //   (a) tuple return, (b) shared-module helper — EMPTY without the
    //       return-type source.
    //   (c) blank between bind and constructor — RED on a walk-back with an
    //       explicit break-on-blank. No single clause of the current code kills
    //       it: a blank matches neither probe, so the walk-back continues on
    //       its own.
    //   (d) wrapped deref feed — RED without the deref-feed source AND with a
    //       `: &mut Child)` needle in its place; bare `&mut Child` —
    //       RED only without the source (the needle matches that spelling).
    //   (e) the `// set up;` comment arm, parked in the (d) block because it is a
    //       walk-back shape rather than a parameter one — `[]` without the
    //       comment skip, `['guard']` with it.
    //   The two type-comparison arms are NOT equivalent controls:
    //   `&mut ChildGuard` cannot fail a substring-based type check
    //   (`is_guard_param` fires first), so only `&mut ChildProcess`
    //   discriminates the equality check.

    // (a) TUPLE RETURN from a local helper, bound as a tuple — signal_matrix's
    // and mp_support_verdict's shape.
    let tuple_ret = "fn spawn_run(root: &Path) -> (ChildGuard, PathBuf) {\n            (ChildGuard::single_process(child), p)\n}\n\
        fn t() {\n    let (mut guard, stderr_path) = spawn_run(root);\n}";
    let got = guard_receivers(tuple_ret, no_helpers);
    assert!(
        got.contains(&"guard".to_string()) && got.contains(&"stderr_path".to_string()),
        "a tuple-returning helper binds every name in the destructure: {got:?}"
    );
    // ...and the NEGATIVE half, which is what the narrow parameter gate
    // buys: `root` is a `&Path` parameter of the helper, and a gate
    // opened by `owns_guards` alone mints it as a receiver in two real files.
    // Without this line the nine spurious receivers are pinned by nothing and a
    // re-widened gate passes the whole binary.
    assert!(
        !got.contains(&"root".to_string()),
        "a `&Path` parameter is not a guard receiver: {got:?}"
    );

    // (b) The helper lives in the SHARED module — mp_record's shape, where the
    // file itself never writes `ChildGuard::` at all.
    let helpers =
        "pub fn spawn_mp_record(root: &Path, extra: &[&str]) -> (ChildGuard, PathBuf, PathBuf) {}";
    let caller =
        "    let (mut guard, stdout_path, stderr_path) = spawn_mp_record(tmp.path(), &[]);";
    let with_helpers = guard_receivers(caller, helpers);
    assert!(
        with_helpers.contains(&"guard".to_string()),
        "a guard handed back by the shared harness is still a guard"
    );
    // The same negative on the helper's own signature — and note WHICH call it
    // is made against, because the obvious one cannot
    // fail. `guard_receivers(code, helpers)` scans `code.lines()` for
    // parameters; `helpers` is read only by the guard-returning-FN-NAME
    // collection. So asserting `!with_helpers.contains("extra")` would assert on a
    // set built from `caller` alone — a line with no `:` and no `ChildGuard` —
    // where the substring `extra` could never appear under any widening of the gate.
    // Asserted here on a call whose SIGNATURE is in `code`: it passes on this gate
    // (`parameter_shaped` wants `: ChildGuard`, `: &mut Child` or
    // `: &mut std::process::Child` — all THREE clauses, and the third is not a
    // superstring of the second, which is why the (d) fixture below matches only
    // it — while this line offers `: &Path` and `: &[&str]`) and goes RED under
    // the re-widening, because `owns_guards` is true for text containing the
    // return type and `extra`'s `ty` then reads ` &[&str]) -> (ChildGuard`,
    // which `is_guard_param` matches.
    //
    // WHAT THIS ARM ADDS, stated so it is not later counted as kill power it
    // does not have: against the `|| owns_guards` re-widening it is SHADOWED by
    // the `root` arm above — `assert!` aborts there first, so this one fires only
    // with that arm neutralised. It is not redundant, though: against a PARAMETER-SHAPE
    // widening (say `|| line.contains(": &[")`) the `root` fixture's line does
    // not match and this is the only arm that fires.
    let helper_sig_receivers = guard_receivers(helpers, no_helpers);
    assert!(
        !helper_sig_receivers.contains(&"extra".to_string()),
        "a `&[&str]` parameter is not a guard receiver: {helper_sig_receivers:?}"
    );
    assert!(
        !guard_receivers(caller, no_helpers).contains(&"guard".to_string()),
        "…and without the helper text there is nothing to know it from — which is \
         exactly why the walk reads mp_support"
    );

    // (c) A BLANK line between the `let` and the constructor — what a comment
    // there LOOKS LIKE by the time the walk sees it, since it is fed `code_only`
    // output. A walk-back that stops on it misses signal_matrix's guard.
    // BLANKED, not a literal `//`: the walk is fed `code_only(&src)`, which has
    // already replaced whole-line comments with empty lines, so a literal-`//`
    // snippet drives a branch no production input reaches — and it passes on a
    // detector that breaks on blanks, so it cannot pin this. Note what this arm
    // does NOT say: a blank reaches NO skip
    // branch. There is no `t.is_empty()` test; a blank
    // falls through both probes and continues on its own. What this arm pins is
    // that the walk-back does not STOP on one.
    let commented = "    let guard =\n\n        ChildGuard::single_process(child);";
    assert!(
        guard_receivers(commented, no_helpers).contains(&"guard".to_string()),
        "a BLANK line between the bind and the constructor must not stop the \
         walk-back — production input is already `code_only`-stripped, so that is \
         what a comment there looks like by the time the walk sees it"
    );

    // (d) The DEREF FEED: a raw-`Child` sink in a guard-owning file,
    // in the REAL spellings the tree uses — including the wrapped form on its own
    // line with a trailing comma.
    let feed = "fn wait_for_log_line(\n    child: &mut std::process::Child,\n    path: &Path,\n) {}\nlet g = ChildGuard::single_process(c);";
    assert!(
        guard_receivers(feed, no_helpers).contains(&"child".to_string()),
        "a `&mut std::process::Child` sink in a guard-owning file is reachable by Deref"
    );
    let feed2 = "fn poll(child: &mut Child) {}\nlet g = ChildGuard::single_process(c);";
    assert!(guard_receivers(feed2, no_helpers).contains(&"child".to_string()));
    // ...and a `&mut ChildGuard` parameter is still a receiver — but note what
    // this arm does NOT prove. It cannot fail on a substring-matching type
    // check, because `is_guard_param` fires first; relaxing the raw-
    // `Child` comparison to `bare.contains("Child")` fails nothing here.
    assert!(guard_receivers(
        "fn p(g: &mut ChildGuard) {}\nlet x = ChildGuard::single_process(c);",
        no_helpers
    )
    .contains(&"g".to_string()));
    // THE discriminating negative: a parameter that passes the parameter gate,
    // carries `Child` as a SUBSTRING, and is not a guard by any route. Equality
    // on the stripped type rejects it; `bare.contains("Child")` would not, which
    // is exactly the case the arm above cannot catch.
    assert!(
        !guard_receivers(
            "fn poll(c: &mut ChildProcess) {}\nlet g = ChildGuard::single_process(x);",
            no_helpers
        )
        .contains(&"c".to_string()),
        "`&mut ChildProcess` merely CONTAINS `Child`; the type is compared for \
         equality after stripping, so it is not a deref feed"
    );
    // A comment ending in `;` between the bind and the constructor must not stop
    // the walk-back — the job of the comment skip at the top of the loop.
    assert!(
        guard_receivers(
            "    let guard =\n        // set up;\n        ChildGuard::single_process(c);",
            no_helpers
        )
        .contains(&"guard".to_string()),
        "a comment ending in `;` satisfies the statement break; the walk-back \
         must skip it rather than give up"
    );

    // NEGATIVE: a raw-`Child` helper in a guard-free file is not a receiver.
    assert!(
        !guard_receivers(
            "fn wait_bounded(child: &mut Child, d: Duration) {}",
            no_helpers
        )
        .contains(&"child".to_string()),
        "a raw-Child helper in a guard-free file is not a guard receiver"
    );
}

/// No test may build an `mp_support::ChildGuard` by TUPLE construction.
///
/// The group kill in `ChildGuard::drop` only reaches a group when the child is
/// its own group LEADER, and that is guaranteed exactly by going through
/// [`ChildGuard::spawn_group_leader`]. A tuple-built guard wraps a hand-rolled
/// `cmd.spawn()` that leads no group, so teardown silently
/// degrades to killing the supervisor alone and orphans its workers. The
/// private field makes that a compile error; this walk is what keeps it
/// one, and says so at the failure site.
///
/// Files that define their OWN `struct ChildGuard` are a DIFFERENT type with
/// its own pid-only `Drop` — their contract is their own, so they are
/// skipped rather than quietly rewritten.
#[test]
fn no_test_builds_a_child_guard_by_tuple_construction() {
    // THIS FILE is excluded explicitly. It necessarily quotes the forbidden
    // spelling (in the failure message below, and in the own-type needle), and
    // if those literals classified it as own-type the walk would
    // skip itself AND that false entry would satisfy its own "the skip is
    // exercised" assertion. `code_only` DOES strip string and char literals
    // (`code_only_strips_comments_and_literals_and_fails_closed_on_an_unterminated_block`
    // pins ten shapes), so that particular route is closed — but the exclusion
    // stays by NAME, because this file also names the spelling in comments and
    // in a `const`, and a walk that decides whether to skip itself by reading
    // its own text is one edit away from skipping itself for a new reason.
    const SELF: &str = "credit_death_e2e_test.rs";
    // Files defining their OWN `struct ChildGuard` — a DIFFERENT type whose
    // `Drop` is `self.0.kill()` (pid-only, never a group kill). They keep that
    // contract and are not migrated to the shared guard.
    // Pinned as an exact set so adding a 12th is a deliberate act.
    //
    // `ros2_migrate_cli_test` is the one entry with a structural reason:
    // the shared guard cannot express its shape, on two counts.
    //   * Its `spawn_write` takes the child's process group as a PER-ARM
    //     parameter (`own_group`) — three arms spawn an ordinary child, one
    //     spawns a group leader so a group-wide SIGINT also reaches the engine
    //     grandchild. Both shared constructors hardcode that decision:
    //     `spawn_group_leader` would make every arm a leader and erase the
    //     discriminator those arms exist to draw, and `single_process` on the
    //     `own_group: true` child would set `group_leader: false` on a child
    //     that really does lead a group — the exact disagreement the private
    //     field was made private to prevent.
    //   * Its `Drop` SIGKILLs, uncatchably, and the shared teardown opens with
    //     a CATCHABLE SIGTERM plus a 5s grace. `cerulion_cli` builds `ctrlc`
    //     with `termination`, so SIGTERM routes to the same handler as SIGINT —
    //     teardown on a panicking arm would run the pre-consent interrupt path
    //     those very arms take their verdict on.
    const OWN_TYPE: &[&str] = &[
        "bagd_subprocess_test.rs",
        "flashback_resim_e2e_test.rs",
        "graph_record_e2e_test.rs",
        "graph_start_order_e2e_test.rs",
        "mp_auto_partition_e2e_test.rs",
        "mp_supervisor_box_test.rs",
        "node_run_e2e_test.rs",
        "replay_cli_test.rs",
        "ros2_migrate_cli_test.rs",
        "topic_introspect_cli_e2e_test.rs",
        "wedge_alarm_e2e_test.rs",
    ];

    let dir = Path::new(env!("CARGO_MANIFEST_DIR")).join("tests");
    // The shared harness, read ONCE: the guard-returning helpers
    // (`spawn_mp_record*`) live here, so a binary that never writes
    // `ChildGuard::` itself is only coverable from this text.
    let helpers = std::fs::read_to_string(dir.join("mp_support/mod.rs"))
        .expect("read mp_support/mod.rs — the walk cannot judge without it");
    let mut offenders = Vec::new();
    let mut escapes = Vec::new();
    let mut scanned = Vec::new();
    let mut found_own = Vec::new();
    for entry in std::fs::read_dir(&dir).expect("tests dir").flatten() {
        let path = entry.path();
        if path.extension().and_then(|e| e.to_str()) != Some("rs") {
            continue;
        }
        let name = path.file_name().unwrap().to_string_lossy().into_owned();
        if name == SELF {
            continue;
        }
        let src = std::fs::read_to_string(&path).expect("read a test file");
        let code = code_only(&src).unwrap_or_else(|e| panic!("{name}: {e}"));
        // Classify by a DEFINITION at line start, not by the needle appearing
        // anywhere: `"struct ChildGuard"` inside a string is not a definition.
        if code
            .lines()
            .any(|l| l.starts_with("struct ChildGuard") || l.starts_with("pub struct ChildGuard"))
        {
            found_own.push(name);
            continue;
        }
        scanned.push(name.clone());
        let uses_shared_wait_bounded =
            code.contains("mod mp_support") && !code.contains("fn wait_bounded(");
        // Names this file treats as a GUARD. FOUR sources, because a receiver
        // that reaps is an escape however it got its name:
        //
        //   * a BINDING whose initializer mentions `ChildGuard::` — walking back
        //     to the nearest `let` over blank lines, because rustfmt wraps a long
        //     constructor and a same-line match is INERT in five of the thirteen
        //     scanned binaries;
        //   * a call to any fn whose RETURN TYPE carries a `ChildGuard`, read from
        //     this file AND from `mp_support` — two binaries never write
        //     `ChildGuard::` at all;
        //   * a PARAMETER typed `ChildGuard` / `&mut ChildGuard`;
        //   * a parameter typed `&mut Child` / `&mut std::process::Child` in a
        //     file that owns guards — the DEREF FEED: a guard passed by `Deref`
        //     into a sink shaped like `wait_for_log_line(child: &mut Child, ..)`
        //     is reaped there, and without this source such a sink and its call
        //     leave the walk green.
        //
        // What it does NOT see, stated so the next reader does not over-trust it:
        // a guard reached through a struct field, a `Vec` element, or a closure
        // capture. It is a net, not a type system.
        let guards = guard_receivers(&code, &helpers);
        let lines_v: Vec<&str> = code.lines().collect();
        for (n, line) in code.lines().enumerate() {
            // THE REAP-ESCAPE CLASS, not one spelling of it. Anything that can
            // REAP a guarded child outside `ChildGuard` skips the point where the
            // live worker set is noted, so the verdict is then taken over an empty
            // set and passes having checked nothing. `try_wait` reaps too, which
            // is why a mere liveness poll belongs on the guard as well.
            //
            // The free `wait_bounded` is additionally PRIVATE, so this rule is
            // belt-and-braces over a compile error; the receiver-scoped checks
            // below are the part a compiler cannot check, since `Deref`
            // offers `Child`'s whole surface on any guard.
            // Only where it could be MP_SUPPORT's: a file that includes the
            // module and does not define its own. `viz_interrupt_e2e_test` has a
            // local `fn wait_bounded` over a raw `Child` and no guard at all —
            // flagging it would be a false positive, and a walk that cries wolf
            // gets weakened rather than obeyed.
            if uses_shared_wait_bounded
                && line.contains("wait_bounded(")
                && !line.contains(".wait_bounded(")
            {
                escapes.push(format!(
                    "{name}:{}: free wait_bounded — {}",
                    n + 1,
                    line.trim()
                ));
            }
            // NOTE the asymmetry:
            // the free-`wait_bounded` rule ABOVE never consults
            // `guards` — it is file-scoped (any `wait_bounded(` that is not
            // `.wait_bounded(` in a file that uses the shared harness). Only the
            // three markers BELOW are receiver-scoped.
            for m in [".wait()", ".try_wait()", ".kill()"] {
                // RECEIVER-SCOPED, via the extracted (and pinned) resolver: only a
                // name this file treats as a guard. A raw `Child` in a file that
                // owns none is not an escape, and banning the markers outright
                // would be a false-positive storm across unrelated arms.
                let Some(recv) = reap_receiver(&lines_v, n, m) else {
                    continue;
                };
                if guards.contains(&recv) {
                    escapes.push(format!(
                        "{name}:{}: `{m}` on the guard `{recv}` — {}",
                        n + 1,
                        line.trim()
                    ));
                }
            }
            // A TYPE position (`RunGuard(ChildGuard)`) ends in `)`, so only a
            // `ChildGuard(<something>` is a construction.
            if let Some(rest) = line.split("ChildGuard(").nth(1) {
                if !rest.starts_with(')') {
                    offenders.push(format!("{name}:{}: {}", n + 1, line.trim()));
                }
            }
        }
    }
    found_own.sort();
    assert_eq!(
        found_own, OWN_TYPE,
        "the set of files defining their own `ChildGuard` changed — if that is \
         deliberate, update OWN_TYPE and say why in the commit body"
    );
    assert!(
        scanned.len() >= 20,
        "the walk must actually reach the harness consumers; it scanned {} files: {scanned:?}",
        scanned.len()
    );
    assert!(
        escapes.is_empty(),
        "reap a guarded child through the guard — `ChildGuard::wait_bounded`, \
         `try_wait_noting()` or `finish()` — never through `Deref` into \
         `Child`'s own `wait`/`try_wait`/`kill`, and never through the free \
         `wait_bounded`. The guard notes its live worker set before it reaps, \
         and a verdict taken after a bare reap has nothing left to check:\n{}",
        escapes.join("\n")
    );
    assert!(
        offenders.is_empty(),
        "build an `mp_support::ChildGuard` through `spawn_group_leader` (a `graph run` \
         with a worker subtree) or `single_process` (no subtree) — a tuple construction \
         cannot establish the group-leader invariant its `Drop` depends on:\n{}",
        offenders.join("\n")
    );
}

/// `code_only` strips what it claims to, and refuses only what it must.
#[test]
fn code_only_strips_comments_and_literals_and_fails_closed_on_an_unterminated_block() {
    assert_eq!(code_only("a // c\nb").unwrap(), "a \nb");
    assert_eq!(code_only("a /* c */ b").unwrap(), "a  b");
    // NESTED, which is why the depth counter exists.
    assert_eq!(code_only("a /* x /* y */ z */ b").unwrap(), "a  b");
    // A `//` inside a block comment does not end the block.
    assert_eq!(code_only("a /* // */ b").unwrap(), "a  b");
    // A block opener inside a LINE comment is not an opener.
    assert_eq!(code_only("a // /*\nb").unwrap(), "a \nb");
    // A genuinely unterminated block is still fatal.
    assert!(code_only("a /* unterminated").is_err());

    // THE CASE THE WALK DEPENDS ON: an unbalanced `/*` inside a literal
    // is not a comment, in any of the forms the scanned tree actually uses.
    // Each must strip to `Ok` AND lose the token, so a `ChildGuard(` hidden in
    // a literal cannot be reported as an offender either.
    for (label, src) in [
        ("normal string", r#"let s = "/* msgs/*"; x"#),
        ("escaped quote", r#"let s = "a \" /* b"; x"#),
        ("escaped backslash", r#"let s = "a \\"; let t = "/*"; x"#),
        (
            "workspace members",
            r#"let s = "[workspace]\nmembers = [\"nodes/*\"]"; x"#,
        ),
        ("byte string", r#"let s = b"/*"; x"#),
        ("raw string", r##"let s = r"/*"; x"##),
        ("raw string, hashes", r###"let s = r#"/* "# ; x"###),
        ("raw byte string", r##"let s = br"/*"; x"##),
        ("char literal", "let c = '/'; let d = '\"'; x"),
        ("escaped char literal", "let c = '\\''; x"),
    ] {
        let got = code_only(src)
            .unwrap_or_else(|e| panic!("{label}: a `/*` inside a literal must not refuse: {e}"));
        assert!(
            got.contains('x'),
            "{label}: the code after the literal must survive, got {got:?}"
        );
    }

    // LIFETIMES are not char literals and must pass through as code — the one
    // way a literal-aware stripper can start eating real source.
    let lifetimes = code_only("fn f<'a>(s: &'a str) -> &'static str { s }").unwrap();
    for needle in ["'a", "'static", "fn f", "str"] {
        assert!(
            lifetimes.contains(needle),
            "a lifetime must survive stripping: {needle} missing from {lifetimes:?}"
        );
    }

    // Newlines inside stripped regions are preserved, so reported line numbers
    // stay the file's own.
    assert_eq!(code_only("a\n/* x\ny */\nb").unwrap().lines().count(), 4);
    // A multi-line raw string keeps its newlines too — the stripped view has the
    // same line COUNT as the source, which is what makes reported line numbers
    // the file's own.
    assert_eq!(
        code_only("a\nlet s = r\"x\ny\";\nb")
            .unwrap()
            .lines()
            .count(),
        4
    );
}

/// No SHM left behind: the trace rings AND the credit segment are unlinked.
///
/// A credited run owns a credit segment per edge, created by the supervisor and
/// mapped by a worker that these arms then SIGKILL — the path most likely to
/// leave one behind. The sibling departure arm
/// asserts the rings; both belong on every arm that kills a worker, not
/// just the headline one.
fn assert_no_shm_left(sup_pid: u32, ns: &str, edge_id: &str) {
    assert_rings_swept(sup_pid);
    assert!(
        cerulion_core::credit::MappedCredit::open_unowned(ns, edge_id).is_err(),
        "the credit segment `{ns}/{edge_id}` must be unlinked once the supervisor that \
         owns it exits — a SIGKILLed consumer must not leak it"
    );
}

/// THE headline: a real CONSUMER death on a real creditable split edge
/// produces exactly one loud head, naming the producer it stranded.
///
/// This is the arm the whole file exists for. The in-process pins prove the reporter
/// is correct and that both `Continue` arms call it; only this proves a genuine
/// worker death reaches it — through the mint, the plan stamp, the spawn, the
/// join pass and the group resolution.
#[test]
#[serial]
fn c7_a_real_consumer_death_strands_its_producer_loudly() {
    let tmp = tempfile::tempdir().unwrap();
    build_creditable_split_workspace(tmp.path(), "cdcons");
    // DEBUG on the reporter's own target, so the once-per-regime contract is
    // OBSERVABLE. A repeat is `debug!`, and the spawn pins
    // `cerulion_cli_engine=info` — under which `assert_eq!(heads.len(), 1)` can
    // only ever see the loud head, making it a pin on the LEVEL FILTER rather
    // than on the latch. With debug on, a second report would appear below and
    // the absence assertion can fail.
    let (mut guard, stdout_path, stderr_path) = spawn_mp_record_with_env(
        tmp.path(),
        &["--peer-loss", "continue"],
        &[(
            "RUST_LOG",
            "cerulion=info,cerulion_cli_engine=info,cerulion_cli_engine::graph_cmd=debug,\
             cerulion_bagd=info",
        )],
    );
    let _bagd_guard = BagdGuard::arm();
    let sup_pid = guard.id();

    // PRECONDITION: the deployment really minted a word. Without this the arm
    // would pass vacuously against a graph whose edge was never creditable.
    assert!(
        wait_for_log(
            &stdout_path,
            &stderr_path,
            "cross-process block credit edges minted",
            Duration::from_secs(60)
        ),
        "the split `block` edge must be credited, or there is nothing to strand\n{}",
        merged(&stdout_path, &stderr_path)
    );
    // ...and the deployment is LIVE, so the kill below is a DEPARTURE and not a
    // bring-up refusal. See `wait_until_live`.
    wait_until_live(&stdout_path, &stderr_path);

    // THE CONDITION, not just the report. Open the edge's own credit word — the
    // same SHM page the producer's pre-fire gate reads — so this arm observes the
    // DEFERRAL and not merely a line of text. A reporter that fired on a healthy
    // edge would satisfy every log assertion below.
    let ns = credit_ns(&stdout_path, &stderr_path);
    let edge_id = cerulion_core::credit::credit_edge_id("/cdcons/relay/cmd", "sink", "trigger_in");
    let word = cerulion_core::credit::MappedCredit::open_unowned(&ns, &edge_id)
        .unwrap_or_else(|e| panic!("open the edge's credit word ({ns}/{edge_id}): {e}"));

    // (a) the producer has actually STEPPED on this edge. `wait_until_live` only
    // proves the publishers attached; a run killed before the first publish would
    // make "the producer was stranded" a claim about a producer that never
    // produced. The bag is required too, so the recorder is genuinely running.
    //
    // WAIT ON A MONOTONE WITNESS, NOT ON `outstanding`.
    //
    // `outstanding` is published-minus-drained: a TRANSIENT that a consumer
    // keeping up drives straight back to 0. Polling it for a positive reading is
    // a coin flip on whether a poll lands inside one of its microscopic windows,
    // and that is MEASURED, not reasoned — the same 20 ms/30 s loop instrumented
    // to sample both quantities, three runs per platform, on this very arm:
    //
    //   Linux        (passes): outstanding seen > 0 in 5/1496, 8/1496, 6/1496 polls
    //                    (0.3-0.5%), max_outstanding = 1 — never above one frame
    //   macOS  (fails):  outstanding seen > 0 in 0/381, 0/375, 0/362 polls
    //   BOTH platforms:  wake_seq advanced ~600 times in the same 30 s, and
    //                    FIRST advanced by poll 2-6 (40-120 ms)
    //
    // So the edge is not short of traffic on either platform — ~600 frames are
    // published AND drained while an `outstanding > 0` wait sees nothing. Such a
    // wait does not pass on Linux because the producer "runs ahead" and holds the
    // mirror up; it passes by catching a sub-1% transient over 1496 samples, and macOS
    // takes ~380 samples in the same window and catches none. On one Linux run the first
    // `outstanding` hit came at poll 329 (~6.6 s) where `wake_seq` had moved by
    // poll 2 (~40 ms) — the luck such a wait depends on, quantified.
    //
    // `wake_seq` is the monotone one. It has exactly ONE bumper in the whole
    // crate — `note_credit_freed`, reached only from `CreditShared::record_drained`
    // — so an advance PROVES a frame was published and then drained on THIS
    // edge's word. It is a count of drains, so it cannot be missed by a late
    // poll the way a transient can. (`wake_seq.store(0)` happens only at
    // create/reinit, before this arm opens the word.)
    //
    // The predicate is the DISJUNCTION so that both shapes count: a poll that
    // lands inside a window still passes immediately, and a consumer that keeps
    // up is witnessed by the drains it performed. Either way the thing this
    // precondition exists to establish — the producer committed a frame on this
    // edge before its consumer is killed — is established. The CONDITION the arm
    // pins is the same either way; only the way it is observed differs.
    let seq_at_open = word.wake_seq_snapshot();
    wait_for_credit(
        &word,
        Duration::from_secs(30),
        "a frame committed on this edge (outstanding > 0, or wake_seq advanced)",
        Some(seq_at_open),
        |c| c.outstanding() > 0 || c.wake_seq_snapshot() != seq_at_open,
    );
    let recordings = tmp.path().join("recordings");
    wait_for_bag(&recordings, Duration::from_secs(60))
        .unwrap_or_else(|| panic!("the --record run must produce a bag"));

    // Kill the CONSUMER rank (p1 = sink = rank 1).
    let victim =
        worker_pid_for_group(sup_pid, "p1", Duration::from_secs(20)).unwrap_or_else(|| {
            panic!(
                "no p1 (sink) worker pid: it never spawned, it spawned and already \
                 EXITED, or the pgrep lookup failed\n{}",
                merged(&stdout_path, &stderr_path)
            )
        });
    // Captured THROUGH THE GUARD, while the supervisor is alive, so the guard's
    // own orphan verdict covers both ranks rather than whatever survived to Drop
    // time. PREMISE, not decoration: `worker_pids_of` yields an EMPTY vec on a
    // pgrep miss, and a verdict over an empty set proves nothing.
    let workers_before = guard.note_workers().to_vec();
    assert_eq!(
        workers_before.len(),
        2,
        "expected both ranks' workers to be alive before the kill, saw {workers_before:?}"
    );
    signal_live_process(victim, libc::SIGKILL);

    assert!(
        wait_for_log(&stdout_path, &stderr_path, HEAD, Duration::from_secs(30)),
        "a consumer death on a credited edge must be reported LOUDLY\n{}",
        merged(&stdout_path, &stderr_path)
    );

    // (b) AND the producer is now genuinely HELD. With its consumer dead nothing
    // drains the word, so the producer fills the window and stops at the bar —
    // and, nothing being able to drain it, `is_full` is STABLE once reached,
    // which is why it is safe to wait for. The complementary "not full before the
    // kill" is deliberately NOT asserted: a healthy edge transiently reaches the
    // bar between drains, so that negative would flake.
    wait_for_credit(
        &word,
        Duration::from_secs(30),
        "is_full (producer held)",
        // NO baseline: nothing drains once the consumer is dead, so a wake_seq
        // delta would be zero by design here and must not be read as a fault.
        None,
        |c| c.is_full(),
    );

    send_signal(sup_pid, libc::SIGINT);
    let status = guard
        .wait_bounded(Duration::from_secs(90))
        .expect("supervisor did not exit after SIGINT");
    assert!(
        status.success(),
        "degraded-continue exits 0, got {status:?}"
    );

    let log = merged(&stdout_path, &stderr_path);
    let heads = lines_at(&log, "WARN", HEAD);
    assert_eq!(
        heads.len(),
        1,
        "exactly ONE loud head — the flood latch opens its regime once\n{log}"
    );
    // The re-observation half, now that DEBUG is on: a repeat pass reporting the
    // same death would land here as a suppressed repeat.
    //
    // PAIRED with a positive DEBUG line first, because the absence assertion rests
    // on three premises this file introduced — the `RUST_LOG` override above
    // reaching the supervisor, the target spelling matching, and `level_of`
    // recognising DEBUG. If any is wrong the capture holds no DEBUG lines at all,
    // `repeats` is empty, and the "no repeat" claim passes having tested nothing.
    // `stamping harvested per-topic provisioning requirements` is unconditional on
    // every multi-process plan path (`graph_cmd.rs:7672`), so its presence is proof
    // DEBUG really is being captured.
    assert!(
        !lines_at(&log, "DEBUG", "stamping harvested per-topic provisioning").is_empty(),
        "DEBUG capture is not working, so the absence check below would be vacuous — \
         expected the unconditional plan-stamping debug line\n{log}"
    );
    let repeats = lines_at(
        &log,
        "DEBUG",
        "credit-edge consumer death (repeat suppressed)",
    );
    assert!(
        repeats.is_empty(),
        "the worker dies ONCE, so there must be no suppressed repeat either — a \
         repeat means the join pass re-observed the same death\n{}",
        repeats.join("\n")
    );
    let head = heads[0];
    // The whole operator field set, as whole tokens.
    for (k, v) in [
        ("group", "p1"),
        ("node_id", "sink"),
        ("consumer_input", "trigger_in"),
        ("dead_rank", "1"),
        ("deferred_producer_ranks", "0"),
        // A real group name, not the `<live>` constant a
        // batch-keyed lookup would produce.
        ("deferred_producer_groups", "p0"),
        ("total_failures", "1"),
    ] {
        assert!(
            has_field(head, k, v),
            "the head must carry `{k}={v}` as a structured field\nhead: {head}"
        );
    }
    assert!(
        has_field(head, "topic", "/cdcons/relay/cmd"),
        "and it must name the topic as a structured field\nhead: {head}"
    );

    // NO ORPHANS, as an ASSERTION. `Drop` only reports (a destructor must never
    // panic); `finish()` is the `#[must_use]` verdict an arm asserts, over the
    // set noted while the supervisor was alive.
    guard.finish().assert_clean();

    // NO SHM LEAK on the SIGKILL path. The sibling departure arm asserts the
    // trace rings are unlinked; a credited run also owns a credit SEGMENT per
    // edge, created by the supervisor and mapped by a worker that was then
    // SIGKILLed — the path most likely to leave one behind.
    assert_no_shm_left(sup_pid, &ns, &edge_id);
}

/// The FREE-RUN path names its dead groups.
///
/// The reporter's suppression of a fully-dead edge is justified by "the ordinary
/// departure line already names both groups" — so that line must name them on
/// BOTH paths, not in the `Some(barrier)` arm only. On the free-run path
/// (`barrier_owner == None`) the per-death warn carries `deaths`,
/// `survivors` and `groups=`; this drives it.
#[test]
#[serial]
fn c7_the_free_run_death_line_names_its_dead_groups() {
    let tmp = tempfile::tempdir().unwrap();
    build_creditable_split_workspace(tmp.path(), "cdfree");
    let (mut guard, stdout_path, stderr_path) = spawn_mp_record_with_env(
        tmp.path(),
        &["--peer-loss", "continue"],
        &[("CERULION_EXECUTION_MODE", "free_run")],
    );
    let _bagd_guard = BagdGuard::arm();
    let sup_pid = guard.id();

    assert!(
        wait_for_log(
            &stdout_path,
            &stderr_path,
            "cross-process block credit edges minted",
            Duration::from_secs(60)
        ),
        "the free-run deployment must still credit the split edge\n{}",
        merged(&stdout_path, &stderr_path)
    );
    wait_until_live(&stdout_path, &stderr_path);
    let ns = credit_ns(&stdout_path, &stderr_path);
    let edge_id = cerulion_core::credit::credit_edge_id("/cdfree/relay/cmd", "sink", "trigger_in");

    let victim =
        worker_pid_for_group(sup_pid, "p1", Duration::from_secs(20)).unwrap_or_else(|| {
            panic!(
                "no p1 worker pid: it never spawned, it spawned and already EXITED, or \
                 the pgrep lookup failed\n{}",
                merged(&stdout_path, &stderr_path)
            )
        });
    let workers_before = guard.note_workers().to_vec();
    assert_eq!(
        workers_before.len(),
        2,
        "expected both ranks' workers to be alive before the kill, saw {workers_before:?}"
    );
    signal_live_process(victim, libc::SIGKILL);
    assert!(
        wait_for_log(
            &stdout_path,
            &stderr_path,
            "continuing degraded",
            Duration::from_secs(30)
        ),
        "the free-run path must report the death\n{}",
        merged(&stdout_path, &stderr_path)
    );

    send_signal(sup_pid, libc::SIGINT);
    let status = guard
        .wait_bounded(Duration::from_secs(90))
        .expect("supervisor did not exit after SIGINT");
    assert!(
        status.success(),
        "degraded-continue exits 0, got {status:?}"
    );

    let log = merged(&stdout_path, &stderr_path);
    let deaths = lines_at(&log, "WARN", "continuing degraded");
    // COUNT, not merely presence: the worker dies once, so a second departure
    // line is a re-observation — the same contract the headline arm pins for the
    // credit head, and the reason a bare `!is_empty()` is too weak.
    assert_eq!(
        deaths.len(),
        1,
        "exactly ONE free-run departure line — the worker dies once\n{log}"
    );
    assert!(
        has_field(deaths[0], "groups", "p1"),
        "and it must NAME the dead group — without it, the dead-edge suppression cites a \
         line that identifies nobody\nline: {}",
        deaths[0]
    );
    // The free-run arm must really have run free: if the env knob were ignored
    // the run would be Lockstep and this line would come from the barrier arm,
    // which names `group=` and not `groups=`.
    assert!(
        has_field(deaths[0], "deaths", "1"),
        "the free-run arm reports a BATCH, so its line carries `deaths=`\nline: {}",
        deaths[0]
    );

    guard.finish().assert_clean();
    assert_no_shm_left(sup_pid, &ns, &edge_id);
}

/// The RETRACTION, driven in the order that produces it.
///
/// Consumer dies first, so the head correctly names producer rank 0 as not
/// known dead. Rank 0 then dies too. The first line is already in the log and
/// cannot be unsaid — so the watch says the correcting thing once, rather than
/// leaving an operator chasing a "permanently deferred" survivor that is a
/// corpse.
#[test]
#[serial]
fn c7_a_producer_that_dies_after_being_named_gets_retracted() {
    let tmp = tempfile::tempdir().unwrap();
    build_creditable_split_workspace(tmp.path(), "cdretr");
    let (mut guard, stdout_path, stderr_path) =
        spawn_mp_record_with_env(tmp.path(), &["--peer-loss", "continue"], &[]);
    let _bagd_guard = BagdGuard::arm();
    let sup_pid = guard.id();

    assert!(
        wait_for_log(
            &stdout_path,
            &stderr_path,
            "cross-process block credit edges minted",
            Duration::from_secs(60)
        ),
        "the split `block` edge must be credited\n{}",
        merged(&stdout_path, &stderr_path)
    );
    wait_until_live(&stdout_path, &stderr_path);
    let ns = credit_ns(&stdout_path, &stderr_path);
    let edge_id = cerulion_core::credit::credit_edge_id("/cdretr/relay/cmd", "sink", "trigger_in");

    // 1) the CONSUMER dies — the head names rank 0 as still standing.
    let consumer =
        worker_pid_for_group(sup_pid, "p1", Duration::from_secs(20)).unwrap_or_else(|| {
            panic!(
                "no p1 worker pid: it never spawned, it spawned and already EXITED, or \
                 the pgrep lookup failed\n{}",
                merged(&stdout_path, &stderr_path)
            )
        });
    let workers_before = guard.note_workers().to_vec();
    assert_eq!(
        workers_before.len(),
        2,
        "expected both ranks' workers to be alive before the kill, saw {workers_before:?}"
    );
    signal_live_process(consumer, libc::SIGKILL);
    assert!(
        wait_for_log(&stdout_path, &stderr_path, HEAD, Duration::from_secs(30)),
        "the consumer death must be reported first\n{}",
        merged(&stdout_path, &stderr_path)
    );

    // 2) the named PRODUCER dies in a LATER pass.
    if let Some(producer) = worker_pid_for_group(sup_pid, "p0", Duration::from_secs(10)) {
        signal_live_process(producer, libc::SIGKILL);
        let retracted = wait_for_log(
            &stdout_path,
            &stderr_path,
            RETRACTION,
            Duration::from_secs(30),
        );
        send_signal(sup_pid, libc::SIGINT);
        // The exit STATUS is part of the contract, not noise — and the contract is
        // NOT the exit 0 that "peer-loss=continue exits 0" suggests.
        // Killing BOTH ranks leaves no survivors, and
        // `graph_cmd.rs` refuses that deliberately —
        //
        //   "every worker process crashed (peer-loss=continue) — no survivors;
        //    deployment failed"
        //
        // The headline arm (one rank dies, one survives) is the exit-0 case and
        // asserts it. This arm pins the other side. A
        // timeout stays a distinct failure — a WEDGED supervisor, not "no
        // retraction was logged".
        let status = guard
            .wait_bounded(Duration::from_secs(90))
            .unwrap_or_else(|| {
                panic!(
                    "the supervisor never exited after SIGINT — wedged with both ranks \
                 dead\n{}",
                    merged(&stdout_path, &stderr_path)
                )
            });
        let log = merged(&stdout_path, &stderr_path);
        assert!(
            !status.success(),
            "a run with NO survivors is a FAILURE even under peer-loss=continue, so this \
             must exit nonzero, got {status:?}\n{log}"
        );
        assert!(
            log.contains("no survivors"),
            "and it must SAY why it failed — an operator reading a bare nonzero exit \
             cannot tell a crashed deployment from a bad flag\n{log}"
        );
        assert!(
            retracted,
            "a producer named as not-known-dead, then killed, must be RETRACTED — the \
             first line is already in the log and cannot be unsaid\n{log}"
        );
        let rets = lines_at(&log, "WARN", RETRACTION);
        assert_eq!(rets.len(), 1, "exactly one retraction per producer\n{log}");
        // `dead_producer_group`, not `group`: verified at the emit site, the head
        // line's `group` is the dead CONSUMER's group while this line's is the
        // dead PRODUCER's, so the keys are deliberately different. `node_id` is
        // the edge's CONSUMER on both lines — it identifies the EDGE, not the
        // corpse.
        for (k, v) in [
            ("dead_rank", "0"),
            ("dead_producer_group", "p0"),
            ("node_id", "sink"),
            ("topic", "/cdretr/relay/cmd"),
        ] {
            assert!(
                has_field(rets[0], k, v),
                "the retraction must carry `{k}={v}` as a structured field — an \
                 operator chasing a 'permanently deferred' survivor greps these\n{}",
                rets[0]
            );
        }
    } else {
        // Both ranks gone is a legitimate outcome of killing the consumer on a
        // 2-rank deployment; say so rather than passing silently.
        send_signal(sup_pid, libc::SIGINT);
        let _ = guard.wait_bounded(Duration::from_secs(90));
        panic!(
            "the p0 worker was already gone, so the retraction order could not be \
             driven — this arm needs the producer rank alive after the consumer's \
             death\n{}",
            merged(&stdout_path, &stderr_path)
        );
    }

    guard.finish().assert_clean();
    assert_no_shm_left(sup_pid, &ns, &edge_id);
}
