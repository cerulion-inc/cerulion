// SPDX-License-Identifier: AGPL-3.0-only
//! Oracle-vector tests for the repo's shared flood-suppression
//! machine (`FailureRegimeLatch`) and for the frame-drop reporting layer built
//! on it (`frame_drop_latch`).
//!
//! Same pattern as `drain_latch_test.rs` / `output_discard_latch_test.rs`:
//! every assertion is a HAND-WRITTEN expectation, never a self-compare of two
//! drives of the same code. Pure state machine + `tracing` mapping — no
//! transport, no SHM, no mock — so the whole file is parallel-safe.
//!
//! Every log assertion matches the LEVEL TOKEN as well as the message text
//! (`WARN` / `DEBUG` / `INFO`). Matching text alone would let a variant that
//! logs the suppressed arm at `warn!` — suppression ineffective, message and
//! counter intact, i.e. the exact defect this change exists to prevent —
//! survive the entire suite.
//!
//! Division of labour: this file pins the POLICY and the level MAPPING. That
//! the production call sites actually route through it (and that the counter
//! they expose is the latch's) is pinned at the sites themselves —
//! `service_test.rs` for the four `cerulion_core` service arms, and
//! `rmw_cerulion/tests/rmw_schema_mismatch_test.rs` for the `rmw_take` arm.

use cerulion_core::testing::{count_at, count_at_exclusively, debug_lines_expected, never_loud};
use cerulion_core::transport::failure_regime_latch::{
    lock_regime_latch, FailureRegimeLatch, RegimeDecision,
};
use cerulion_core::transport::frame_drop_latch::{
    report_envelope_intact, report_schema_hash_match, report_schema_hash_mismatch,
    report_short_envelope, FrameDropSite,
};
use tracing_test::traced_test;

/// Substring unique to the LOUD (`warn!`) head of the mismatch reporter.
const LOUD_MARKER: &str = "dropping a frame whose wire schema hash does not match";
/// Substring unique to the DECADE RE-ANNOUNCEMENT (`warn!`) arm.
const STILL_FAILING_MARKER: &str = "schema-hash mismatch STILL dropping every frame";
/// Substring unique to the SUPPRESSED (`debug!`) arm.
const SUPPRESSED_MARKER: &str = "schema-hash mismatch suppressed";
/// Substring unique to the RECOVERY (`info!`) arm.
const RECOVERY_MARKER: &str = "schema hashes match again";

/// Substring unique to the LOUD head of the SHORT-ENVELOPE (framing) reporter.
const SHORT_LOUD_MARKER: &str = "dropping a frame too short to hold the service envelope";
/// Substring unique to the SHORT-ENVELOPE DECADE RE-ANNOUNCEMENT (`warn!`) arm.
const SHORT_STILL_FAILING_MARKER: &str = "short-envelope drops STILL happening";
/// Substring unique to the SHORT-ENVELOPE suppressed (`debug!`) arm.
const SHORT_SUPPRESSED_MARKER: &str = "short-envelope drop suppressed";
/// Substring unique to the SHORT-ENVELOPE recovery (`info!`) arm.
const SHORT_RECOVERY_MARKER: &str = "service envelopes parse again";

// =====================================================================
// The pure policy
// =====================================================================

/// THE canonical cycle, against a hand-written oracle:
/// loud → suppressed(1) → suppressed(2) → recovery(2) → loud.
#[test]
fn canonical_loud_then_suppressed_then_recovery_then_rearm() {
    let mut latch = FailureRegimeLatch::new();
    assert!(!latch.is_failing());
    assert_eq!(latch.total_failures(), 0);

    assert_eq!(latch.on_failure(), RegimeDecision::Loud);
    assert!(latch.is_failing());
    assert_eq!(latch.total_failures(), 1);

    assert_eq!(
        latch.on_failure(),
        RegimeDecision::Suppressed { suppressed: 1 }
    );
    assert_eq!(
        latch.on_failure(),
        RegimeDecision::Suppressed { suppressed: 2 }
    );
    assert_eq!(latch.total_failures(), 3);

    // Recovery reports the two SUPPRESSED failures — NOT all three. The
    // loud head was never suppressed, so counting it would overstate what
    // the operator missed.
    assert_eq!(latch.on_success(), Some(2));
    assert!(!latch.is_failing());
    // The running total is NEVER reset by recovery (Principle #3).
    assert_eq!(latch.total_failures(), 3);

    // Re-armed: a fresh breakage is loud again.
    assert_eq!(latch.on_failure(), RegimeDecision::Loud);
    assert_eq!(latch.total_failures(), 4);
}

/// A lone failure that heals on the very next observation re-arms SILENTLY —
/// the anti-flood rule for an every-other-frame flapper, which would otherwise
/// DOUBLE its log volume by announcing a recovery per blip.
#[test]
fn lone_failure_rearms_silently() {
    let mut latch = FailureRegimeLatch::new();
    assert_eq!(latch.on_failure(), RegimeDecision::Loud);
    assert_eq!(latch.on_success(), None, "no flood ⇒ no recovery line");
    assert!(!latch.is_failing());
    // Silent recovery still RE-ARMS.
    assert_eq!(latch.on_failure(), RegimeDecision::Loud);
    assert_eq!(latch.total_failures(), 2);
}

/// Steady state: success on a healthy latch is a branch returning `None`,
/// forever, and never fabricates a recovery line or a failure count.
#[test]
fn healthy_success_is_silent_and_idempotent() {
    let mut latch = FailureRegimeLatch::new();
    for _ in 0..1000 {
        assert_eq!(latch.on_success(), None);
    }
    assert!(!latch.is_failing());
    assert_eq!(latch.total_failures(), 0);
}

/// INVERSION 1 — "always loud" (the disk-fill regression this whole change
/// exists to prevent). A sustained regime must NOT emit a loud decision per
/// failure. Since the decade rule it emits a BOUNDED ladder: the head,
/// then a re-announcement each time the running total crosses a power of ten.
///
/// Hand oracle over 1000 consecutive failures: `Loud` at #1,
/// `StillFailing` at #10 / #100 / #1000, `Suppressed` for the other 996 —
/// and a re-announcement never bumps the suppressed count, because the
/// operator was not denied that line.
#[test]
fn sustained_regime_is_a_bounded_decade_ladder_not_a_flood() {
    let mut latch = FailureRegimeLatch::new();
    let mut louds = 0u32;
    let mut suppressed_lines = 0u32;
    let mut announced_totals = Vec::new();
    let mut expected_suppressed = 0u64;

    for i in 1..=1000u64 {
        match latch.on_failure() {
            RegimeDecision::Loud => {
                louds += 1;
                assert_eq!(i, 1, "only the head may take the plain loud arm");
            }
            RegimeDecision::Suppressed { suppressed } => {
                suppressed_lines += 1;
                expected_suppressed += 1;
                assert_eq!(suppressed, expected_suppressed);
            }
            RegimeDecision::StillFailing { total, suppressed } => {
                announced_totals.push(total);
                assert_eq!(total, i, "the re-announcement carries the running total");
                assert_eq!(
                    suppressed, expected_suppressed,
                    "a re-announced failure was NOT downgraded — it must not bump the \
                     suppressed count, or recovery would understate what was missed"
                );
            }
        }
    }

    assert_eq!(louds, 1, "a flood must not re-open the plain loud arm");
    assert_eq!(
        announced_totals,
        vec![10, 100, 1000],
        "an open regime re-announces at each DECADE of the running total"
    );
    assert_eq!(suppressed_lines, 996);
    assert_eq!(latch.total_failures(), 1000);
    // Recovery reports every downgraded failure and none of the loud ones.
    assert_eq!(latch.on_success(), Some(996));
}

/// The decade ladder is MONOTONE across regimes — a recovery must not hand a
/// still-broken peer a fresh quota of nine silent failures per regime.
///
/// Hand oracle: nine failures (head + 8 suppressed), heal, then the TENTH
/// failure overall opens a new regime. It is `Loud` because it is a head, and
/// it CONSUMES the decade (no double announcement), so the next
/// re-announcement is at #100 and carries 89 suppressed.
#[test]
fn the_decade_ladder_is_monotone_across_regimes_and_a_head_consumes_its_decade() {
    let mut latch = FailureRegimeLatch::new();

    assert_eq!(latch.on_failure(), RegimeDecision::Loud); // #1
    for i in 1..=8u64 {
        assert_eq!(
            latch.on_failure(), // #2..#9
            RegimeDecision::Suppressed { suppressed: i }
        );
    }
    assert_eq!(latch.on_success(), Some(8));
    assert_eq!(latch.total_failures(), 9);

    // #10 — a decade boundary AND a regime head. One loud line, not two.
    assert_eq!(
        latch.on_failure(),
        RegimeDecision::Loud,
        "a head that lands on a decade is loud ONCE"
    );
    assert_eq!(latch.total_failures(), 10);

    // #11..#99 are suppressed; the ladder has already advanced past 10, so
    // nothing re-announces until #100.
    for i in 1..=89u64 {
        assert_eq!(
            latch.on_failure(),
            RegimeDecision::Suppressed { suppressed: i }
        );
    }
    assert_eq!(latch.total_failures(), 99);

    assert_eq!(
        latch.on_failure(), // #100
        RegimeDecision::StillFailing {
            total: 100,
            suppressed: 89
        },
        "the ladder must not have been reset by the recovery at #9"
    );
}

/// INVERSION 2 — "never loud again" (the condition becomes invisible after the
/// first regime). Every regime CLOSED by a success must re-open loud.
#[test]
fn each_closed_regime_reopens_loud() {
    let mut latch = FailureRegimeLatch::new();
    let mut louds = 0u32;
    for _ in 0..3 {
        if latch.on_failure() == RegimeDecision::Loud {
            louds += 1;
        }
        latch.on_failure();
        latch.on_failure();
        assert_eq!(latch.on_success(), Some(2));
    }
    assert_eq!(louds, 3);
    // Nine failures total — deliberately under the first decade, so this
    // oracle stays about re-arming and nothing else.
    assert_eq!(latch.total_failures(), 9);
}

/// A second consecutive success is a no-op: one regime can never report two
/// recoveries.
#[test]
fn recovery_is_reported_at_most_once_per_regime() {
    let mut latch = FailureRegimeLatch::new();
    latch.on_failure();
    latch.on_failure();
    assert_eq!(latch.on_success(), Some(1));
    assert_eq!(latch.on_success(), None);
    assert_eq!(latch.on_success(), None);
    assert_eq!(latch.total_failures(), 2);
}

/// The UNCONDITIONAL counter — the release-safe complement to the log lines
/// (Principle #3). It bumps on EVERY failure regardless of the log-level
/// regime, is never bumped by a success, and is never reset by recovery.
#[test]
fn total_is_unconditional_never_reset_and_never_bumped_by_success() {
    let mut latch = FailureRegimeLatch::new();
    latch.on_failure(); // 1, loud
    latch.on_failure(); // 2, suppressed
    latch.on_failure(); // 3, suppressed
    assert_eq!(latch.total_failures(), 3);
    assert_eq!(latch.on_success(), Some(2));
    assert_eq!(latch.total_failures(), 3, "recovery must not reset it");
    latch.on_failure(); // 4, loud again
    assert_eq!(latch.total_failures(), 4);
    latch.on_success();
    latch.on_success();
    assert_eq!(latch.total_failures(), 4, "successes must not bump it");
}

/// `Default` must behave EXACTLY like `new()` over a run long enough to reach
/// the first decade.
///
/// The short version of this test (construct both, check one decision) is not
/// enough since the decade ladder landed: a DERIVED `Default` would zero `next_decade`, and
/// `total >= 0` holds for every failure, so every repeat would take the loud
/// re-announce arm — a full flood produced by one missing `impl`. Driving 12
/// failures through both is what catches it.
#[test]
fn default_matches_new_over_a_full_decade() {
    let mut d = FailureRegimeLatch::default();
    let mut n = FailureRegimeLatch::new();
    assert_eq!(d.is_failing(), n.is_failing());
    assert_eq!(d.total_failures(), n.total_failures());

    let drive =
        |latch: &mut FailureRegimeLatch| (0..12).map(|_| latch.on_failure()).collect::<Vec<_>>();
    let from_default = drive(&mut d);
    let from_new = drive(&mut n);

    // Hand oracle, not a self-compare: head, 8 suppressed, the decade
    // re-announcement at failure 10, then two more suppressed.
    let mut oracle = vec![RegimeDecision::Loud];
    for suppressed in 1..=8u64 {
        oracle.push(RegimeDecision::Suppressed { suppressed });
    }
    oracle.push(RegimeDecision::StillFailing {
        total: 10,
        suppressed: 8,
    });
    oracle.push(RegimeDecision::Suppressed { suppressed: 9 });
    oracle.push(RegimeDecision::Suppressed { suppressed: 10 });

    assert_eq!(from_new, oracle, "new() must follow the decade ladder");
    assert_eq!(
        from_default, oracle,
        "Default must be identical to new() — a zeroed next_decade would make \
         every repeat loud"
    );
}

/// A DIAGNOSTIC latch must never wedge the path it observes: a mutex poisoned
/// by an unrelated panic must still hand out its (structurally intact) state.
#[test]
fn poisoned_latch_still_locks_and_keeps_its_count() {
    let latch = std::sync::Mutex::new(FailureRegimeLatch::new());
    lock_regime_latch(&latch).on_failure();

    // Poison it from a panicking thread, exactly as a panic inside a take
    // callback would.
    let poisoner = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
        let _g = latch.lock().expect("first lock");
        panic!("poison the latch");
    }));
    assert!(poisoner.is_err(), "the harness must actually poison it");
    assert!(latch.is_poisoned());

    // Still usable, and the pre-poison count survived.
    let mut guard = lock_regime_latch(&latch);
    assert_eq!(guard.total_failures(), 1);
    assert_eq!(
        guard.on_failure(),
        RegimeDecision::Suppressed { suppressed: 1 },
        "a poisoned latch must keep its regime state, not silently reset"
    );
}

// =====================================================================
// The frame-drop reporting layer (decision → tracing level + field name)
// =====================================================================

/// The site noun is what tells an operator WHICH consumer dropped the frame.
/// Hand oracle — a swap here would misattribute every line at a site.
#[test]
fn site_nouns_are_distinct_and_stable() {
    assert_eq!(FrameDropSite::Message.noun(), "message");
    assert_eq!(FrameDropSite::ServiceRequest.noun(), "service request");
    assert_eq!(FrameDropSite::ServiceResponse.noun(), "service response");
}

/// Operators grep their logs by FIELD NAME. A message drop must be logged
/// under `topic=` and a service drop under `service=` — collapsing both onto a
/// generic `name=` silently
/// breaks every existing query. The reporters are pinned against these values
/// by the `#[traced_test]` arms below.
#[test]
fn site_field_names_are_topic_for_messages_and_service_for_services() {
    assert_eq!(FrameDropSite::Message.field_name(), "topic");
    assert_eq!(FrameDropSite::ServiceRequest.field_name(), "service");
    assert_eq!(FrameDropSite::ServiceResponse.field_name(), "service");
}

/// THE level-mapping pin: 9 mismatches through the reporter ⇒ exactly 1
/// `WARN` head + 8 `DEBUG` repeats, and the unconditional counter reads 9
/// regardless. Nine keeps this arm strictly under the first decade, so it is
/// about the head/repeat mapping and nothing else.
///
/// Mutation sensitivity: an always-loud reporter fails the WARN count; a
/// never-loud one fails it at 0; a reporter that logs the SUPPRESSED arm at
/// `warn!` fails BOTH counts (9 warns, 0 debugs) — such a defect survives a
/// text-only filter, which is why the level token is in the predicate; a
/// counter gated on the log level fails the `total_failures()` arm while the
/// line counts still pass.
#[traced_test]
#[test]
fn reporter_emits_one_warn_head_and_n_minus_one_debug_and_counts_all() {
    const N: usize = 9;
    let mut latch = FailureRegimeLatch::new();
    for _ in 0..N {
        report_schema_hash_mismatch(
            &mut latch,
            FrameDropSite::Message,
            "/pure/topic",
            0x1111_1111_1111_1111,
            0x2222_2222_2222_2222,
        );
    }
    assert_eq!(
        latch.total_failures(),
        N as u64,
        "the counter is UNCONDITIONAL — it must not depend on the log level"
    );
    logs_assert(|lines: &[&str]| {
        let warns = count_at_exclusively(lines, "WARN", &[LOUD_MARKER])?;
        never_loud(lines, SUPPRESSED_MARKER)?;
        let debugs = count_at_exclusively(lines, "DEBUG", &[SUPPRESSED_MARKER])?;
        if warns != 1 {
            return Err(format!("expected exactly 1 WARN loud head, got {warns}"));
        }
        let want_debugs = debug_lines_expected(N - 1);
        if debugs != want_debugs {
            return Err(format!(
                "expected {want_debugs} DEBUG suppressed lines, got {debugs}"
            ));
        }
        // No arm may leak upward: the suppressed marker must NEVER appear at
        // WARN, and the loud marker must never appear at DEBUG.
        let leaked_up = count_at(lines, "WARN", SUPPRESSED_MARKER);
        let leaked_down = count_at(lines, "DEBUG", LOUD_MARKER);
        if leaked_up != 0 || leaked_down != 0 {
            return Err(format!(
                "level inversion: {leaked_up} suppressed lines at WARN, \
                 {leaked_down} loud lines at DEBUG"
            ));
        }
        // The loud head must carry the diagnosis: both hashes, the site, and
        // the TOPIC field name (not a generic `name=`).
        let head = lines
            .iter()
            .find(|l| l.contains(LOUD_MARKER))
            .ok_or("no loud head")?;
        // `kind=` is spelled out on purpose: a bare `message` needle is
        // satisfied by the message TEXT itself ("… the message definition"),
        // so it pins nothing about the site noun.
        for needle in [
            "0x1111111111111111",
            "0x2222222222222222",
            "topic=/pure/topic",
            "kind=\"message\"",
        ] {
            if !head.contains(needle) {
                return Err(format!("loud head is missing {needle}: {head}"));
            }
        }
        Ok(())
    });
}

/// The DECADE re-announcement, at the reporting layer: 10 mismatches in one
/// open regime ⇒ 1 `WARN` head + 8 `DEBUG` repeats + 1 `WARN` re-announcement
/// carrying the running total.
///
/// This is the operator-surface half of the rmw counter problem: the rmw C ABI
/// is standardized, so `hash_mismatches` cannot be exposed to rclcpp/rclpy —
/// the log is the whole window, and it must not go silent forever after one
/// line.
#[traced_test]
#[test]
fn reporter_re_announces_an_open_regime_at_each_decade_at_warn() {
    let mut latch = FailureRegimeLatch::new();
    for _ in 0..10 {
        report_schema_hash_mismatch(
            &mut latch,
            FrameDropSite::ServiceResponse,
            "/pure/decade",
            1,
            2,
        );
    }
    assert_eq!(latch.total_failures(), 10);
    logs_assert(|lines: &[&str]| {
        let heads = count_at_exclusively(lines, "WARN", &[LOUD_MARKER])?;
        let still = count_at_exclusively(lines, "WARN", &[STILL_FAILING_MARKER])?;
        never_loud(lines, SUPPRESSED_MARKER)?;
        let debugs = count_at_exclusively(lines, "DEBUG", &[SUPPRESSED_MARKER])?;
        if heads != 1 {
            return Err(format!("expected 1 WARN head, got {heads}"));
        }
        if still != 1 {
            return Err(format!(
                "expected exactly 1 WARN decade re-announcement at the 10th \
                 failure, got {still}"
            ));
        }
        let want_debugs = debug_lines_expected(8);
        if debugs != want_debugs {
            return Err(format!(
                "expected {want_debugs} DEBUG repeats, got {debugs}"
            ));
        }
        if count_at(lines, "DEBUG", STILL_FAILING_MARKER) != 0 {
            return Err(
                "the re-announcement must be LOUD — it exists precisely to survive \
                 a debug filter"
                    .to_string(),
            );
        }
        let line = lines
            .iter()
            .find(|l| l.contains(STILL_FAILING_MARKER))
            .ok_or("no re-announcement line")?;
        for needle in [
            "total_failures=10",
            "suppressed=8",
            "service=/pure/decade",
            "kind=\"service response\"",
        ] {
            if !line.contains(needle) {
                return Err(format!("re-announcement is missing {needle}: {line}"));
            }
        }
        Ok(())
    });
}

/// Recovery reports ONCE, at `INFO`, carrying the SUPPRESSED count (not the
/// total), and then re-arms so the next breakage is loud again.
#[traced_test]
#[test]
fn reporter_recovery_is_one_info_carrying_the_suppressed_count_then_rearms() {
    let mut latch = FailureRegimeLatch::new();
    let name = "/pure/recover";
    for _ in 0..4 {
        report_schema_hash_mismatch(&mut latch, FrameDropSite::ServiceRequest, name, 7, 9);
    }
    // Two matching frames: the FIRST closes the regime, the second must be
    // silent (one regime can only recover once).
    report_schema_hash_match(&mut latch, FrameDropSite::ServiceRequest, name);
    report_schema_hash_match(&mut latch, FrameDropSite::ServiceRequest, name);
    // Re-armed ⇒ loud again (2 loud heads total).
    report_schema_hash_mismatch(&mut latch, FrameDropSite::ServiceRequest, name, 7, 9);

    assert_eq!(latch.total_failures(), 5);
    logs_assert(|lines: &[&str]| {
        let warns = count_at_exclusively(lines, "WARN", &[LOUD_MARKER])?;
        let recoveries = count_at_exclusively(lines, "INFO", &[RECOVERY_MARKER])?;
        if recoveries != 1 {
            return Err(format!(
                "expected exactly 1 INFO recovery line, got {recoveries}"
            ));
        }
        if warns != 2 {
            return Err(format!(
                "recovery must RE-ARM the loud arm: expected 2 WARN heads, got {warns}"
            ));
        }
        let rec = lines
            .iter()
            .find(|l| l.contains(RECOVERY_MARKER))
            .ok_or("no recovery line")?;
        // 3 SUPPRESSED (the head of the 4 was loud), 5 total failures.
        if !rec.contains("suppressed_count=3") {
            return Err(format!("recovery must report 3 suppressed: {rec}"));
        }
        if !rec.contains("kind=\"service request\"") {
            return Err(format!("recovery must carry the site noun: {rec}"));
        }
        if !rec.contains("service=/pure/recover") {
            return Err(format!("recovery must log under `service=`: {rec}"));
        }
        Ok(())
    });
}

/// A lone mismatch that heals immediately emits its loud head and then NOTHING
/// — no recovery line. The anti-flood rule, at the reporting layer.
#[traced_test]
#[test]
fn reporter_lone_mismatch_recovers_silently() {
    let mut latch = FailureRegimeLatch::new();
    let name = "/pure/lone";
    report_schema_hash_mismatch(&mut latch, FrameDropSite::ServiceResponse, name, 1, 2);
    report_schema_hash_match(&mut latch, FrameDropSite::ServiceResponse, name);
    logs_assert(|lines: &[&str]| {
        let warns = count_at_exclusively(lines, "WARN", &[LOUD_MARKER])?;
        // An ABSENCE guard, so the NON-exclusive count (see the module note on
        // `count_at_exclusively`): an absence holds at every level.
        let recoveries = count_at(lines, "INFO", RECOVERY_MARKER);
        if warns != 1 {
            return Err(format!("expected 1 WARN head, got {warns}"));
        }
        if recoveries != 0 {
            return Err(format!(
                "a lone mismatch must re-arm SILENTLY, got {recoveries} recovery lines"
            ));
        }
        Ok(())
    });
}

/// A healthy consumer's matching frames are completely silent — the reporter
/// must not narrate the happy path (this is a per-frame call site).
#[traced_test]
#[test]
fn reporter_is_silent_on_a_healthy_consumer() {
    let mut latch = FailureRegimeLatch::new();
    for _ in 0..500 {
        report_schema_hash_match(&mut latch, FrameDropSite::Message, "/pure/quiet");
    }
    assert_eq!(latch.total_failures(), 0);
    logs_assert(|lines: &[&str]| {
        let any = lines
            .iter()
            .filter(|l| {
                l.contains(LOUD_MARKER)
                    || l.contains(STILL_FAILING_MARKER)
                    || l.contains(SUPPRESSED_MARKER)
                    || l.contains(RECOVERY_MARKER)
            })
            .count();
        if any == 0 {
            Ok(())
        } else {
            Err(format!(
                "a healthy consumer must log nothing, got {any} lines"
            ))
        }
    });
}

// ---------------------------------------------------------------------
// The SHORT-ENVELOPE (framing-skew) twin
// ---------------------------------------------------------------------

/// The framing twin has the same level mapping as the hash arm: 1 `WARN` head,
/// `DEBUG` repeats, one `INFO` recovery carrying the suppressed count, and an
/// unconditional counter. It logs the size that was too small, so the operator
/// can tell a truncated frame from a foreign framing.
#[traced_test]
#[test]
fn short_envelope_reporter_maps_warn_then_debug_then_info() {
    let mut latch = FailureRegimeLatch::new();
    let name = "/pure/short";
    for _ in 0..4 {
        report_short_envelope(&mut latch, FrameDropSite::ServiceRequest, name, 10);
    }
    report_envelope_intact(&mut latch, FrameDropSite::ServiceRequest, name);
    assert_eq!(latch.total_failures(), 4);

    logs_assert(|lines: &[&str]| {
        let heads = count_at_exclusively(lines, "WARN", &[SHORT_LOUD_MARKER])?;
        never_loud(lines, SHORT_SUPPRESSED_MARKER)?;
        let debugs = count_at_exclusively(lines, "DEBUG", &[SHORT_SUPPRESSED_MARKER])?;
        let recoveries = count_at_exclusively(lines, "INFO", &[SHORT_RECOVERY_MARKER])?;
        let want_debugs = debug_lines_expected(3);
        if (heads, debugs, recoveries) != (1, want_debugs, 1) {
            return Err(format!(
                "expected (1 WARN, {want_debugs} DEBUG, 1 INFO), got ({heads}, {debugs}, {recoveries})"
            ));
        }
        let head = lines
            .iter()
            .find(|l| l.contains(SHORT_LOUD_MARKER))
            .ok_or("no loud head")?;
        for needle in ["size_bytes=10", "service=/pure/short"] {
            if !head.contains(needle) {
                return Err(format!("loud head is missing {needle}: {head}"));
            }
        }
        let rec = lines
            .iter()
            .find(|l| l.contains(SHORT_RECOVERY_MARKER))
            .ok_or("no recovery line")?;
        if !rec.contains("suppressed_count=3") {
            return Err(format!("recovery must report 3 suppressed: {rec}"));
        }
        // The RECOVERY line is grepped by the same field key as the head — an
        // operator watching `service=<name>` must see the regime close, so the
        // per-site key is pinned on both ends, not just the loud one.
        for needle in ["service=/pure/short", "kind=\"service request\""] {
            if !rec.contains(needle) {
                return Err(format!("recovery is missing {needle}: {rec}"));
            }
        }
        Ok(())
    });
}

/// The DECADE re-announcement of the SHORT-ENVELOPE twin — the framing
/// condition's own `warn!` ladder.
///
/// The latch emits `StillFailing` at THREE sites, and only the hash
/// one was level-pinned. This arm and rmw's decode `error!` were emitted by code no assertion
/// could reach, because every other drive in the suite stops at 4 failures and
/// the first decade boundary is 10 — so both were free to be `debug!`.
///
/// Hand oracle over 10 framing drops in ONE open regime: 1 `WARN` head + 8
/// `DEBUG` repeats + 1 `WARN` re-announcement carrying `total_failures=10` and the
/// UNCHANGED `suppressed=8` (a re-announced failure was not downgraded).
/// Changing `warn!` to `debug!` on that arm fails both the WARN triple and the
/// DEBUG-negative guard — which is the whole point of the arm, since it exists
/// precisely to survive a filter that hides `debug!`.
#[traced_test]
#[test]
fn short_envelope_reporter_re_announces_an_open_regime_at_each_decade_at_warn() {
    let mut latch = FailureRegimeLatch::new();
    let name = "/pure/short_decade";
    for _ in 0..10 {
        report_short_envelope(&mut latch, FrameDropSite::ServiceResponse, name, 12);
    }
    assert_eq!(latch.total_failures(), 10);

    logs_assert(|lines: &[&str]| {
        let heads = count_at_exclusively(lines, "WARN", &[SHORT_LOUD_MARKER])?;
        let still = count_at_exclusively(lines, "WARN", &[SHORT_STILL_FAILING_MARKER])?;
        never_loud(lines, SHORT_SUPPRESSED_MARKER)?;
        let debugs = count_at_exclusively(lines, "DEBUG", &[SHORT_SUPPRESSED_MARKER])?;
        let want_debugs = debug_lines_expected(8);
        if (heads, still, debugs) != (1, 1, want_debugs) {
            return Err(format!(
                "expected (1 WARN head, 1 WARN decade re-announcement, {want_debugs} DEBUG \
                 repeats), got ({heads}, {still}, {debugs})"
            ));
        }
        if count_at(lines, "DEBUG", SHORT_STILL_FAILING_MARKER) != 0 {
            return Err(
                "the framing re-announcement must be LOUD — it exists precisely to \
                 survive a debug filter"
                    .to_string(),
            );
        }
        let line = lines
            .iter()
            .find(|l| l.contains(SHORT_STILL_FAILING_MARKER))
            .ok_or("no re-announcement line")?;
        for needle in [
            "total_failures=10",
            "suppressed=8",
            "size_bytes=12",
            "service=/pure/short_decade",
            "kind=\"service response\"",
        ] {
            if !line.contains(needle) {
                return Err(format!("re-announcement is missing {needle}: {line}"));
            }
        }
        Ok(())
    });
}

/// SEPARATENESS, at the reporting layer: a consumer holds one latch per
/// CONDITION, and an open regime on one must never swallow the other's loud
/// head.
///
/// Drive a hash regime to 3 (head + 2 suppressed), then hit the framing
/// condition twice: the framing head must still be `WARN`, with its own
/// independent counter. Then heal the framing condition only — the hash latch
/// must be untouched, so a further hash mismatch is still suppressed, not a
/// fresh head.
///
/// (The production twin of this test, over the real rmw take path, is
/// `rmw_schema_mismatch_test::an_open_decode_regime_does_not_swallow…`.)
#[traced_test]
#[test]
fn two_conditions_on_one_consumer_keep_independent_regimes_and_counters() {
    let mut hash = FailureRegimeLatch::new();
    let mut framing = FailureRegimeLatch::new();
    let name = "/pure/two_conditions";

    for _ in 0..3 {
        report_schema_hash_mismatch(&mut hash, FrameDropSite::ServiceRequest, name, 7, 9);
    }
    for _ in 0..2 {
        report_short_envelope(&mut framing, FrameDropSite::ServiceRequest, name, 4);
    }
    assert_eq!(hash.total_failures(), 3);
    assert_eq!(framing.total_failures(), 2, "counters are independent");

    // Heal the framing condition ONLY.
    report_envelope_intact(&mut framing, FrameDropSite::ServiceRequest, name);
    assert!(!framing.is_failing());
    assert!(hash.is_failing(), "the hash regime must be untouched");
    report_schema_hash_mismatch(&mut hash, FrameDropSite::ServiceRequest, name, 7, 9);

    logs_assert(|lines: &[&str]| {
        let hash_heads = count_at_exclusively(lines, "WARN", &[LOUD_MARKER])?;
        let framing_heads = count_at_exclusively(lines, "WARN", &[SHORT_LOUD_MARKER])?;
        if hash_heads != 1 {
            return Err(format!(
                "the hash regime must stay open across the framing regime: expected 1 \
                 WARN head, got {hash_heads}"
            ));
        }
        if framing_heads != 1 {
            return Err(format!(
                "the framing head must be LOUD even with a hash regime open: expected 1 \
                 WARN head, got {framing_heads}"
            ));
        }
        // 4 hash failures ⇒ 3 suppressed; 2 framing failures ⇒ 1 suppressed.
        never_loud(lines, SUPPRESSED_MARKER)?;
        let hash_debugs = count_at_exclusively(lines, "DEBUG", &[SUPPRESSED_MARKER])?;
        never_loud(lines, SHORT_SUPPRESSED_MARKER)?;
        let framing_debugs = count_at_exclusively(lines, "DEBUG", &[SHORT_SUPPRESSED_MARKER])?;
        let (want_hash, want_framing) = (debug_lines_expected(3), debug_lines_expected(1));
        if (hash_debugs, framing_debugs) != (want_hash, want_framing) {
            return Err(format!(
                "expected ({want_hash} hash DEBUG, {want_framing} framing DEBUG), got \
                 ({hash_debugs}, {framing_debugs})"
            ));
        }
        // Healing the framing condition must not announce a hash recovery.
        // An ABSENCE guard, so the NON-exclusive count.
        let hash_recoveries = count_at(lines, "INFO", RECOVERY_MARKER);
        if hash_recoveries != 0 {
            return Err(format!(
                "healing ONE condition must not report the other's recovery, got \
                 {hash_recoveries}"
            ));
        }
        Ok(())
    });
}
