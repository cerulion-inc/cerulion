// SPDX-License-Identifier: AGPL-3.0-only
//! The C++ bridge REFUSAL is loud, constant and
//! per-verdict — its LEVEL, its paragraph and its `verdict=` field are
//! pinned here, in an OWN binary because `#[traced_test]` takes the
//! process-global subscriber slot (the `rmw_cpp_bypass_warn_test`
//! precedent). The verdict itself (`classify_cpp_bridge`) and the TEXT
//! per verdict (`refusal_message`) are oracle-pinned in the lib tests;
//! this file pins the EMISSION seam both resolvers call —
//! `CppBridgeGate::emit_refusal` — which exists because the
//! repo's tracing-discipline walk (`cerulion_core`'s
//! `tracing_field_discipline_test`) refuses a bare
//! `error!("{}", refusal)` pass-through from the resolvers.
//!
//! Each refusing verdict emits ONE `error!` whose message is the
//! `{SCREAMING_CONST}` capture of its OWN paragraph. That makes a drift
//! a `"{}", msg` forwarding structurally could not have — the
//! seam rendering the OTHER verdict's text — a real defect, so every
//! emitted line is cross-checked against `refusal_message()`.
#![cfg(unix)]

use rmw_cerulion::era::{built_for, CppBridgeGate};
use serial_test::serial;
use tracing_test::traced_test;

const LEVELS: [&str; 5] = ["ERROR", "WARN", "INFO", "DEBUG", "TRACE"];

/// The phrase every refusal paragraph carries and nothing else logged by
/// this seam does — the marker the counting predicates key on.
const REFUSAL_MARKER: &str = "refusing the rclcpp (C++ typesupport) path";

/// The level token out of a captured line's HEADER (the text before the
/// first `": "`), matched as a WHOLE whitespace token — never a
/// substring, so a paragraph mentioning "error" or a span name carrying
/// a level word cannot satisfy it (the `rmw_publish_reject_test` R1
/// lesson).
fn line_level(line: &str) -> Option<&'static str> {
    let header = line.split(": ").next().unwrap_or(line);
    header
        .split_whitespace()
        .find_map(|token| LEVELS.into_iter().find(|level| *level == token))
}

/// `key=value` as a whole whitespace token — a prefixed or prose
/// occurrence is not the field.
fn has_field(line: &str, key: &str, value: &str) -> bool {
    let needle = format!("{key}={value}");
    line.split_whitespace().any(|token| token == needle)
}

/// A structured field whose KEY starts a whole whitespace token — a
/// prefixed rendering such as `notbuilt_for=` can never satisfy it (a
/// bare `contains("built_for=")` would) — and whose
/// rendered value, which may itself carry spaces as `built_for`'s
/// `distro=… bindings=… caps=…` does, is EXACTLY `value` — bounded on
/// both sides, not merely prefixed by it.
fn has_field_starting_a_token(line: &str, key: &str, value: &str, next_keys: &[&str]) -> bool {
    let needle = format!("{key}={value}");
    line.match_indices(&needle).any(|(at, _)| {
        let key_starts_a_token = at == 0 || line[..at].ends_with(char::is_whitespace);
        // …and the VALUE ends at a boundary too: a prefix match
        // would accept
        // `built_for=<expected>corrupted`. The value may contain spaces,
        // so the boundary is end-of-line or whitespace — not "the next
        // token", which `built_for`'s own `bindings=…`/`caps=…` halves
        // would satisfy from inside a truncated expectation.
        // One step further: "ends at whitespace"
        // would still accept `built_for=<expected> corrupted` — the value
        // contains spaces, so whitespace is not a boundary of it. The
        // boundary is what can legitimately FOLLOW the value: end of
        // line, or one of the site's known NEXT field keys (`next_keys`,
        // declared by the caller from the emission's field order). A
        // trailing word, or an unknown `key=`, is extra text.
        let value_ends_at_a_boundary = {
            let rest = &line[at + needle.len()..];
            rest.is_empty()
                || (rest.starts_with(char::is_whitespace)
                    && next_keys
                        .iter()
                        .any(|k| rest.trim_start().starts_with(&format!("{k}="))))
        };
        key_starts_a_token && value_ends_at_a_boundary
    })
}

/// Is `rest` EXACTLY the decade re-announcement's two extra fields —
/// ` total_failures=<digits> suppressed_count=<digits>` and nothing more?
///
/// A looser "starts with
/// `total_failures=` and contains `suppressed_count=` anywhere" accepts
/// `… total_failures=10 suppressed_count=1 extra=bad`, which is precisely
/// the extra text the oracle claims to reject.
///
/// The same class one step further: tokenising with
/// `split_whitespace` accepts `… suppressed_count=1 ` — a TRAILING space (or
/// a doubled separator) still yields exactly two numeric tokens, so a
/// token count never checks the "NOTHING after them" contract. The suffix
/// is parsed as its exact byte shape: one leading space, the first
/// field, ONE space, the second field, end of string. A digit check on the
/// whole tail is what rejects trailing whitespace — nothing is tokenised.
fn is_decade_suffix(rest: &str) -> bool {
    fn numeric_field(field: &str, key: &str) -> bool {
        field
            .strip_prefix(key)
            .is_some_and(|v| !v.is_empty() && v.chars().all(|c| c.is_ascii_digit()))
    }
    let Some(fields) = rest.strip_prefix(' ') else {
        return false;
    };
    let Some((total, suppressed)) = fields.split_once(' ') else {
        return false;
    };
    numeric_field(total, "total_failures=") && numeric_field(suppressed, "suppressed_count=")
}

/// The rendered BODY of a captured line — everything after the
/// `<timestamp> LEVEL <span>: <target>: ` header: the message first, then
/// the structured fields in declaration order, single-space separated.
fn body_of(line: &str) -> &str {
    line.splitn(3, ": ").nth(2).unwrap_or("")
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

/// The contract ONE refusal line must meet for `gate`: at ERROR, its BODY
/// exactly `gate`'s own paragraph followed by the two structured fields
/// (so a seam emitting the right paragraph plus extra text, or an extra
/// field, fails), NOT `other`'s paragraph anywhere, and no
/// other loud line in the capture.
fn check_refusal_line(
    lines: &[&str],
    gate: CppBridgeGate,
    verdict_token: &str,
    foreign: &str,
) -> Result<(), String> {
    let own = gate
        .refusal_message()
        .expect("a refusing verdict carries text");
    // A paragraph that must never appear on this verdict's line: the
    // post-Jazzy refusal text that the Lyrical mirror retired, kept as the
    // regression guard against it returning under any verdict.
    let others = foreign;
    let hits: Vec<&str> = lines
        .iter()
        .copied()
        .filter(|l| l.contains(REFUSAL_MARKER))
        .collect();
    if hits.len() != 1 {
        return Err(format!(
            "expected exactly 1 refusal line, got {}",
            hits.len()
        ));
    }
    let line = hits[0];
    if line_level(line) != Some("ERROR") {
        // LEVEL-token pin: a demotion (invisible at the default
        // rmw_cerulion=warn filter, or merely less alarming) fails loudly.
        return Err(format!("refusal not at ERROR level: {line}"));
    }
    // EXACT body: the paragraph verbatim, then `built_for` and `verdict`
    // in declaration order — nothing before, between or after. This is
    // the drift guard (a const swap in the seam dies here) AND the
    // no-extra-text guard.
    // `rcl_error_channel=unavailable`: a cargo test process maps no
    // librcutils, so the channel is provably absent here — "set" is what
    // a real ROS host shows (a refusal that reaches
    // `tracing` only leaves an rclcpp user with RMW_RET_INVALID_ARGUMENT
    // plus "error not set").
    let expected = format!(
        "{own} built_for={} verdict={verdict_token} rcl_error_channel=unavailable",
        built_for()
    );
    let body = body_of(line);
    // A single refusal whose running total happens to land ON a decade
    // is a re-announcement, which appends `total_failures=` and
    // `suppressed_count=` (the latch is process-global and its ladder
    // survives the `rearm()` above, so which arm hits a decade depends on
    // execution order). The paragraph and the three fields before them
    // must still match EXACTLY, and NOTHING but those two known fields
    // may follow — so this stays the drift guard and the no-extra-text
    // guard both.
    // EXACTLY the two decade fields, in order, with numeric values and
    // NOTHING after them: a looser
    // "starts with total_failures= and contains suppressed_count="
    // accepts `… total_failures=10 suppressed_count=1 extra=bad`, which
    // is precisely the extra text this oracle claims to reject.
    let decade_suffix = body
        .strip_prefix(&expected)
        .filter(|rest| is_decade_suffix(rest));
    if body != expected && decade_suffix.is_none() {
        return Err(format!(
            "refusal body is not exactly the paragraph + fields:\n  got:  {body}\n  want: {expected}"
        ));
    }
    if line.contains(others) {
        return Err(format!(
            "refusal carries the OTHER verdict's paragraph: {line}"
        ));
    }
    if !has_field(line, "verdict", verdict_token) {
        return Err(format!("refusal missing verdict={verdict_token}: {line}"));
    }
    if !has_field_starting_a_token(line, "built_for", built_for(), &["verdict"]) {
        return Err(format!("refusal missing built_for={}: {line}", built_for()));
    }
    let extra = unexpected_loud_lines(lines, &[line]);
    if !extra.is_empty() {
        return Err(format!(
            "unexpected loud line(s) beside the refusal:\n{}",
            extra.join("\n")
        ));
    }
    Ok(())
}

/// The C++-gate refusal latch is PROCESS-global (the verdict is a
/// build-time constant, so production never recovers), and every arm
/// below wants the LOUD head — so each re-arms first. A re-arm restores
/// the REGIME only: `total_failures` and the decade ladder are untouched
/// by design, so an arm that cares WHERE a decade falls must derive that
/// from the running counter (the cadence arm does) rather than assume it
/// runs first. `#[serial]` on every arm is the other half: with one
/// shared latch the arms are no longer independent, and under default
/// threads three of four fail.
fn rearm() {
    rmw_cerulion::era::rearm_cpp_gate_latches_for_test();
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

#[test]
fn the_tightened_field_oracles_reject_what_they_claim_to() {
    // Hand vectors — the two helpers are what
    // every log assertion in this binary rests on, and a looser
    // version of each accepts the malformed shape they advertise rejecting.
    //
    // The decade suffix: EXACTLY the two numeric fields, in order.
    assert!(is_decade_suffix(" total_failures=10 suppressed_count=8"));
    assert!(!is_decade_suffix(
        " total_failures=10 suppressed_count=8 extra=bad"
    ));
    assert!(!is_decade_suffix(" suppressed_count=8 total_failures=10"));
    assert!(!is_decade_suffix(" total_failures=ten suppressed_count=8"));
    assert!(!is_decade_suffix(" total_failures= suppressed_count=8"));
    assert!(!is_decade_suffix(" total_failures=10"));
    assert!(!is_decade_suffix("total_failures=10 suppressed_count=8"));
    assert!(!is_decade_suffix(""));
    // Whitespace is not "nothing" — a trailing space, a doubled
    // separator, a tab separator and a trailing newline are all extra text.
    assert!(!is_decade_suffix(" total_failures=10 suppressed_count=8 "));
    assert!(!is_decade_suffix(" total_failures=10  suppressed_count=8"));
    assert!(!is_decade_suffix("  total_failures=10 suppressed_count=8"));
    assert!(!is_decade_suffix(" total_failures=10\tsuppressed_count=8"));
    assert!(!is_decade_suffix(" total_failures=10 suppressed_count=8\n"));
    // A field value is bounded on BOTH sides: a trailing corruption is
    // no longer a match, and a prefixed KEY still is not.
    let line = "msg built_for=distro=x caps=a,b verdict=RefusePreGalactic";
    let next = &["verdict"];
    assert!(has_field_starting_a_token(
        line,
        "built_for",
        "distro=x caps=a,b",
        next
    ));
    // A WHITESPACE-TERMINATED
    // PREFIX of a value that legitimately contains spaces is not a
    // match either, because what follows the value must be end-of-line or one
    // of the site's declared NEXT field keys — `caps=` is not one.
    assert!(!has_field_starting_a_token(
        line,
        "built_for",
        "distro=x",
        next
    ));
    // A trailing word and an UNKNOWN trailing field are extra text; the
    // declared next field is not.
    assert!(!has_field_starting_a_token(
        "msg built_for=distro=x caps=a,b corrupted",
        "built_for",
        "distro=x caps=a,b",
        next
    ));
    assert!(!has_field_starting_a_token(
        "msg built_for=distro=x caps=a,b extra=1",
        "built_for",
        "distro=x caps=a,b",
        next
    ));
    assert!(!has_field_starting_a_token(
        "msg built_for=distro=x caps=a,bcorrupted",
        "built_for",
        "distro=x caps=a,b",
        next
    ));
    assert!(!has_field_starting_a_token(
        "msg notbuilt_for=distro=x caps=a,b",
        "built_for",
        "distro=x caps=a,b",
        next
    ));
    // End of line is a boundary too — and with NO declared next key it is
    // the ONLY one.
    assert!(has_field_starting_a_token(
        "msg built_for=distro=x caps=a,b",
        "built_for",
        "distro=x caps=a,b",
        &[]
    ));
    assert!(!has_field_starting_a_token(
        "msg built_for=distro=x caps=a,b verdict=RefusePreGalactic",
        "built_for",
        "distro=x caps=a,b",
        &[]
    ));
}

#[traced_test]
#[test]
#[serial]
fn a_refusing_verdict_is_loud_first_then_latched_and_re_announces_at_a_decade() {
    // This seam fires per RESOLVE, so it must be latched.
    // `rmw_serialize` / `rmw_deserialize` resolve the typesupport per
    // MESSAGE and both route through the gate, so a refusing build
    // republishing serialized messages would emit this paragraph at frame
    // rate (the disk-fill class) without one. It
    // rides the repo's ONE shared `FailureRegimeLatch`: loud head,
    // `debug!` repeats, a loud re-announcement at each decade, and an
    // unconditional counter — so a refusing build can never go silent,
    // and never floods.
    rearm();
    // The latch is PROCESS-global and its decade ladder deliberately
    // survives a re-arm (a recovery must not buy a still-refusing build a
    // fresh quota of silence), so the boundary does NOT sit at a fixed
    // index — it depends on how many refusals this binary's other arms
    // already drove. Read the running total and drive PAST the next
    // decade, so the arm is probative whatever the execution order.
    let before = rmw_cerulion::era::cpp_bridge_refusals_fired();
    let asks_before = rmw_cerulion::ffi::rcl_error_asks();
    let drive = decade_above(before + 1) - before + 2;
    for _ in 0..drive {
        assert!(CppBridgeGate::RefusePreGalactic.emit_refusal());
    }
    // EVERY refusal asks rcl's error channel, suppressed repeats included
    // (the last text alone cannot tell "set on
    // every refusal" from "set once, never refreshed" — the ask count can).
    assert_eq!(
        rmw_cerulion::ffi::rcl_error_asks() - asks_before,
        drive,
        "every refusal must ask rcl's error channel, not only the loud ones"
    );
    assert_eq!(
        rmw_cerulion::era::cpp_bridge_refusals_fired() - before,
        drive,
        "every refusal must count, whatever it logged"
    );
    // The rcl channel was really ASKED: every cargo
    // test process maps no librcutils, so the OUTCOME is always
    // `unavailable` and asserting that value pins nothing — replacing the
    // call with a hardcoded `Unavailable` passed the whole suite. The
    // seam records the ask, so the message rcl would have received is
    // checkable: the verdict's own paragraph plus the two fields.
    let asked = rmw_cerulion::ffi::last_rcl_error_text()
        .expect("the refusal must have asked rcl's error channel");
    let paragraph = CppBridgeGate::RefusePreGalactic
        .refusal_message()
        .expect("a refusing verdict carries a paragraph");
    // EXACT, not a prefix + two `contains` (the same oracle
    // class as the decade suffix): the whole text rcl would report, the
    // verdict's own paragraph and both fields, with nothing else.
    assert_eq!(
        asked,
        format!(
            "{paragraph} built_for={} verdict=RefusePreGalactic",
            built_for()
        ),
        "the rcl channel was asked with the wrong text"
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
            .filter(|l| l.contains(REFUSAL_MARKER))
            .collect();
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
            .filter(|l| line_level(l) == Some("ERROR"))
            .collect();
        if loud_lines.len() != loud_at.len() {
            return Err(format!(
                "loud refusals: {} loud, expected {} (head + each decade):\n{}",
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
                .filter(|i| line_level(hits[*i]) == Some("ERROR"))
                .collect();
            if loud != loud_at {
                return Err(format!(
                    "loud refusals at {loud:?}, expected {loud_at:?}:\n{}",
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
        // Quieter, never thinner: every line keeps the remedy paragraph,
        // the build identity, the verdict and the rcl outcome.
        for line in &hits {
            if !has_field(line, "verdict", "RefusePreGalactic") {
                return Err(format!("refusal line missing verdict: {line}"));
            }
            if !has_field(line, "rcl_error_channel", "unavailable") {
                return Err(format!("refusal line missing rcl_error_channel: {line}"));
            }
            if !has_field_starting_a_token(line, "built_for", built_for(), &["verdict"]) {
                return Err(format!("refusal line missing built_for: {line}"));
            }
        }
        let extra = unexpected_loud_lines(lines, &loud_lines);
        if !extra.is_empty() {
            return Err(format!(
                "unexpected loud line(s) beside the refusals:\n{}",
                extra.join("\n")
            ));
        }
        Ok(())
    });
    // Leave the latch re-armed for whatever runs next.
    rearm();
}

#[traced_test]
#[test]
#[serial]
fn a_pre_galactic_verdict_refuses_at_error_with_its_own_constant_paragraph() {
    rearm();
    assert!(
        CppBridgeGate::RefusePreGalactic.emit_refusal(),
        "a pre-Galactic verdict must tell the caller to refuse"
    );
    logs_assert(|lines: &[&str]| {
        check_refusal_line(
            lines,
            CppBridgeGate::RefusePreGalactic,
            "RefusePreGalactic",
            "rosidl-Buffer struct growth",
        )
    });
}

#[traced_test]
#[test]
#[serial]
fn a_supported_verdict_emits_nothing_and_does_not_refuse() {
    rearm();
    assert!(
        !CppBridgeGate::Supported.emit_refusal(),
        "a Supported verdict must NOT tell the caller to refuse"
    );
    logs_assert(|lines: &[&str]| {
        // The contract is "logs NOTHING", so the oracle is the WHOLE
        // capture, not the refusal-marked subset: a Supported arm that
        // logs an UNRELATED error line passes a marker-only count
        // and must fail the whole-capture check.
        if !lines.is_empty() {
            return Err(format!(
                "Supported emitted {} log line(s):\n{}",
                lines.len(),
                lines.join("\n")
            ));
        }
        Ok(())
    });
    // Anti-tautology, same capture: a zero above proves nothing if the
    // capture is blind to the seam, so a refusing verdict must now show —
    // and as the ONLY line, so the whole-capture oracle stays meaningful.
    assert!(CppBridgeGate::RefusePreGalactic.emit_refusal());
    logs_assert(|lines: &[&str]| {
        if lines.len() != 1 || !lines[0].contains(REFUSAL_MARKER) {
            return Err(format!(
                "expected exactly 1 line after re-arm, the refusal; got {}:\n{}",
                lines.len(),
                lines.join("\n")
            ));
        }
        Ok(())
    });
}
