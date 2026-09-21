// SPDX-License-Identifier: AGPL-3.0-only
//! The `test-seams` C++ bypass is LOUD — the
//! per-resolve warning's LEVEL and message are pinned here, in an OWN
//! binary because `#[traced_test]` takes the process-global subscriber
//! slot (the `rmw_schema_mismatch_test` precedent). The mode selection
//! itself (`classify_cpp_bypass`) is oracle-pinned in the lib tests;
//! this file pins the emission seam both resolvers call on the
//! `SeamsLoud` arm — dropping the warn fails here.
#![cfg(unix)]

use rmw_cerulion::era::{built_for, CPP_BYPASS_SEAMS_WARN};

const LEVELS: [&str; 5] = ["ERROR", "WARN", "INFO", "DEBUG", "TRACE"];

/// The level token out of a captured line's HEADER (the text before the
/// first `": "`), as a WHOLE whitespace token — never a substring, so a
/// DEBUG/TRACE event whose text says "WARN" cannot pass.
fn line_level(line: &str) -> Option<&'static str> {
    let header = line.split(": ").next().unwrap_or(line);
    header
        .split_whitespace()
        .find_map(|token| LEVELS.into_iter().find(|level| *level == token))
}
use tracing_test::traced_test;

/// A structured field as a WHOLE whitespace token, key AND value — so
/// `total_failures=100` cannot satisfy an expectation of `=10`.
fn has_field(line: &str, key: &str, value: &str) -> bool {
    let needle = format!("{key}={value}");
    line.split_whitespace().any(|token| token == needle)
}

/// Every LOUD line (ERROR / WARN / INFO) that is not one of `expected` —
/// must be empty, so an entry point emitting its expected line PLUS an
/// unrelated loud diagnostic fails. DEBUG and
/// TRACE are deliberately NOT counted: their presence depends on the
/// build profile (tracing's static max level compiles `debug!` out under
/// `release_max_level_*`), so any count of them is the release-only
/// false-pass class — assert only what every profile renders.
fn unexpected_loud_lines<'a>(lines: &[&'a str], expected: &[&str]) -> Vec<&'a str> {
    lines
        .iter()
        .copied()
        .filter(|l| matches!(line_level(l), Some("ERROR" | "WARN" | "INFO")))
        .filter(|l| !expected.contains(l))
        .collect()
}

/// Is `total` a DECADE boundary — a power of ten at or above 10? A hand
/// oracle over the latch's documented rule, never a call back into it.
fn is_decade(total: u64) -> bool {
    let mut decade = 10u64;
    while decade <= total {
        if decade == total {
            return true;
        }
        match decade.checked_mul(10) {
            Some(next) => decade = next,
            None => break,
        }
    }
    false
}

/// The smallest decade STRICTLY above `total`.
fn decade_above(total: u64) -> u64 {
    let mut decade = 10u64;
    while decade <= total {
        decade = decade.checked_mul(10).expect("a decade above a u64 total");
    }
    decade
}

#[traced_test]
#[test]
fn the_seams_bypass_warn_is_loud_first_then_latched_and_re_announces_at_a_decade() {
    // This seam must be latched: "registration cadence, not per-frame"
    // does not describe it. `rmw_serialize` / `rmw_deserialize` resolve the
    // typesupport per MESSAGE and both route through the gate, so a
    // seams build republishing serialized messages would emit this
    // ~450-byte paragraph at frame rate (the
    // disk-fill class) unlatched. It rides the repo's ONE shared
    // `FailureRegimeLatch`, and the design intent survives intact: the
    // FIRST use is loud, every DECADE of the running total re-announces
    // loudly, and the unconditional counter never stops — so a shipped
    // `--features test-seams` cdylib can still never go silent.
    // The latch is PROCESS-global and its decade ladder deliberately
    // survives a re-arm, so the boundary does NOT sit at a fixed index.
    // Read the running total and drive PAST the next decade, so the arm
    // is probative whatever else in this binary called the seam.
    // Re-arm for symmetry with the sibling binary: this file has one
    // test today, and the oracle hard-codes index 0 as the loud head —
    // which stops being true the moment a second test drives the seam.
    rmw_cerulion::era::rearm_cpp_gate_latches_for_test();
    let before = rmw_cerulion::era::cpp_bypass_warns_fired();
    let drive = decade_above(before + 1) - before + 2;
    for _ in 0..drive {
        rmw_cerulion::era::emit_cpp_bypass_warn();
    }
    // The counter is UNCONDITIONAL and log-level independent — the
    // Principle #3 half that makes suppressing repeats safe.
    assert_eq!(
        rmw_cerulion::era::cpp_bypass_warns_fired() - before,
        drive,
        "every call must count, whatever it logged"
    );
    // Hand oracle: the FIRST line is the loud head, and a line is loud
    // again exactly where the running total lands on a decade.
    let loud_at: Vec<usize> = (0..drive as usize)
        .filter(|i| *i == 0 || is_decade(before + *i as u64 + 1))
        .collect();
    assert!(
        loud_at.len() >= 2,
        "the drive must cross a decade to be probative: before={before} drive={drive}"
    );
    logs_assert(|lines: &[&str]| {
        let hits: Vec<&str> = lines
            .iter()
            .copied()
            .filter(|l| l.contains("test-seams C++ bypass active"))
            .collect();
        // LEVEL-token pins: the head and every decade re-announcement
        // must be WARN (a demotion to DEBUG/TRACE is invisible at the
        // default rmw_cerulion=warn filter); the repeats must be DEBUG
        // (an un-suppressed repeat is the flood this fix exists to kill).
        // Whole header tokens, so a lower-level event whose TEXT says
        // WARN cannot pass.
        // PROFILE-INDEPENDENT half: the LOUD lines are exactly the head
        // plus one per decade. `debug!` is compiled OUT under
        // `release_max_level_*` (cerulion_core enables
        // `tracing/release_max_level_info`, and features unify), so the
        // count of lines and their INDICES are only assertable where
        // DEBUG renders — the files' own helper states that rule and
        // these arms honour it.
        let loud_lines: Vec<&str> = hits
            .iter()
            .copied()
            .filter(|l| line_level(l) == Some("WARN"))
            .collect();
        if loud_lines.len() != loud_at.len() {
            return Err(format!(
                "loud bypass lines: {} loud, expected {} (head + each decade):\n{}",
                loud_lines.len(),
                loud_at.len(),
                hits.join("\n")
            ));
        }
        if tracing::level_enabled!(tracing::Level::DEBUG) {
            if hits.len() as u64 != drive {
                return Err(format!("expected {drive} lines, got {}", hits.len()));
            }
            let loud: Vec<usize> = (0..hits.len())
                .filter(|i| line_level(hits[*i]) == Some("WARN"))
                .collect();
            if loud != loud_at {
                return Err(format!(
                    "loud bypass lines at {loud:?}, expected {loud_at:?}:\n{}",
                    hits.join("\n")
                ));
            }
            let suppressed = hits
                .iter()
                .filter(|l| line_level(l) == Some("DEBUG"))
                .count();
            // Release-safe (the tree-wide DEBUG-count discipline): where
            // `debug!` is compiled out `hits` holds only the loud lines, so the
            // expectation is `debug_lines_expected` of the remainder — the
            // remainder where the level compiles in, 0 where it does not; the
            // loud sweep below is the level-free twin.
            let want = cerulion_core::testing::debug_lines_expected(hits.len() - loud_at.len());
            if suppressed != want {
                return Err(format!("expected {want} DEBUG repeats, got {suppressed}"));
            }
            // Level-free TWIN of the gated count above: a suppressed repeat — a
            // line carrying `suppressed_count=` and no `total_failures=` — must
            // never surface LOUDLY at any profile; this is what fails a
            // `debug!`→`warn!` promotion in release, where the count reads 0.
            for level in ["WARN", "INFO", "ERROR"] {
                if hits.iter().any(|l| {
                    line_level(l) == Some(level)
                        && l.contains("suppressed_count=")
                        && !l.contains("total_failures=")
                }) {
                    return Err(format!("a suppressed repeat surfaced at {level}"));
                }
            }
        }
        // A re-announcement carries the running total: it exists for the
        // operator who missed the head, and the rmw counters are
        // unreachable through the standardized C ABI.
        for (n, i) in loud_at.iter().enumerate().skip(1) {
            let total = before + *i as u64 + 1;
            if !has_field(loud_lines[n], "total_failures", &total.to_string()) {
                return Err(format!(
                    "the decade line omits total_failures={total}: {}",
                    loud_lines[n]
                ));
            }
        }
        let extra = unexpected_loud_lines(lines, &loud_lines);
        if !extra.is_empty() {
            return Err(format!(
                "unexpected loud line(s) beside the bypass warns:\n{}",
                extra.join("\n")
            ));
        }
        // Every line, at every level, still carries the whole remedy and
        // the build identity — a suppressed repeat is quieter, never
        // thinner. The MESSAGE is compared to the
        // constant EXACTLY (the body is `<message> <fields>`, and the
        // message renders first), and every line after the head — the
        // suppressed repeats and the decade re-announcements — carries
        // `suppressed_count=`.
        // The body is EXACTLY `<message> <fields>`: the
        // constant, then ONLY the arm's permitted structured fields in the
        // emission's declaration order — `built_for` on every line, then
        // `total_failures` + `suppressed_count` on a re-announcement,
        // `suppressed_count` alone on a suppressed repeat — and nothing after.
        // A prefix match plus independent field probes let text slip in
        // between the message and `built_for`.
        fn strip_numeric_field<'a>(
            rest: &'a str,
            key: &str,
            line: &str,
        ) -> Result<&'a str, String> {
            let rest = rest.strip_prefix(&format!(" {key}=")).ok_or_else(|| {
                format!("bypass line lacks ` {key}=` where it must follow: {line}")
            })?;
            let digits = rest.len() - rest.trim_start_matches(|c: char| c.is_ascii_digit()).len();
            if digits == 0 {
                return Err(format!("`{key}=` carries no number: {line}"));
            }
            Ok(&rest[digits..])
        }
        for (i, line) in hits.iter().enumerate() {
            let body = line.splitn(3, ": ").nth(2).unwrap_or("");
            let Some(rest) = body.strip_prefix(CPP_BYPASS_SEAMS_WARN) else {
                return Err(format!(
                    "bypass line does not open with the constant: {line}"
                ));
            };
            // INDEPENDENT of the constant: the exact
            // comparison above proves the line renders whatever the constant
            // says — it cannot notice the constant itself losing the
            // deployment ban. This literal pins the ban on the shipped text.
            if !line.contains("must never deploy") {
                return Err(format!("bypass line missing the deploy ban: {line}"));
            }
            let Some(mut rest) = rest.strip_prefix(&format!(" built_for={}", built_for())) else {
                return Err(format!(
                    "bypass line does not continue with ` built_for={}` right after the \
                     message: {line}",
                    built_for()
                ));
            };
            // Classified by the LINE'S OWN level, not by its index among the
            // drives: in release the DEBUG repeats are
            // compiled out, so `hits` holds only the WARN lines and a hit's
            // index is not its call index — the second WARN sits at `i == 1`,
            // which `loud_at` would call a suppressed repeat.
            let re_announcement = i > 0 && line_level(line) == Some("WARN");
            if re_announcement {
                rest = strip_numeric_field(rest, "total_failures", line)?;
            }
            if i > 0 {
                rest = strip_numeric_field(rest, "suppressed_count", line)?;
            }
            if !rest.is_empty() {
                return Err(format!(
                    "bypass line carries text beyond its permitted fields ({rest:?}): {line}"
                ));
            }
        }
        Ok(())
    });
}
