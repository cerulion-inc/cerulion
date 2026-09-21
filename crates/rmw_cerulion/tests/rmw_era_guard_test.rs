// SPDX-License-Identifier: AGPL-3.0-only
//! The ABI guard: the load-time era guard is
//! HOISTED to the first rmw entry points that touch the caller's
//! `rmw_init_options_t`, and refuses BEFORE any byte of it is read or
//! written.
//!
//! rmw.h's order is `rmw_init_options_init` → (`rmw_init_options_copy`) →
//! `rmw_init` → … → `rmw_init_options_fini`. With the guard only in
//! `rmw_init`, a Jazzy-built `.so` under a Kilted runtime would already have
//! stamped `allocator`/`impl_` at ITS offsets (120/160 in a 168-byte
//! struct) into the caller's 160-byte object — the `impl_` write landing
//! PAST it — before it refused. The guard's verdict comes from the baked
//! claim vs `ROS_DISTRO` only (the `test-seams` override supplies the
//! baked side so a SPECIFIC pair — jazzy vs kilted — is under test; this
//! dev build bakes "vendored-dev", which admits only its own rolling
//! era), so it needs no struct read at all. Two fixtures pin that: a
//! POISONED caller buffer byte-compared before and after each entry
//! point, and — the stronger one — an `mmap`'d PROT_NONE page handed to
//! each export in a re-exec'd child process, where any touch faults.
//!
//! Own binary: `#[traced_test]` takes the process-global subscriber slot
//! (the `rmw_transient_local_ceiling_test` precedent), and every arm is
//! `#[serial]` because the override and `ROS_DISTRO` are process-global.
//! A refused entry point returns before `runtime()`; only the
//! unclaimed-banner arm brings the transport up (a real, admitted
//! `rmw_init`).
#![cfg(unix)]

use std::mem::size_of;
use std::os::raw::c_char;

use rmw_cerulion::era::{
    baked_distro, built_for, classify_distro_pair, DISTRO_MISMATCH_REFUSAL, VENDORED_DEV_DISTRO,
};
use rmw_cerulion::era_check::{
    era_claim_admits, era_claim_members, ERA_CLAIM_PREFIX, VENDORED_SNAPSHOT_ERA_TOKEN,
};
use rmw_cerulion::ffi::{
    self, rcutils_allocator_t, rmw_context_t, rmw_init_options_t, RMW_RET_ERROR,
    RMW_RET_INCORRECT_RMW_IMPLEMENTATION, RMW_RET_OK,
};
use rmw_cerulion::test_seams::{
    era_guard_panics_fired, BakedDistroOverrideGuard, EnvVarGuard, EraGuardPanicGuard,
    ERA_GUARD_PANIC_MSG,
};
use rmw_cerulion::{
    rmw_context_fini, rmw_init, rmw_init_options_copy, rmw_init_options_fini,
    rmw_init_options_init, rmw_shutdown,
};
use serial_test::serial;
use tracing_test::traced_test;

const LEVELS: [&str; 5] = ["ERROR", "WARN", "INFO", "DEBUG", "TRACE"];

/// The phrase every refusal paragraph carries.
const REFUSAL_MARKER: &str = "built for a DIFFERENT ROS distro";

/// The poison word: no field of a zeroed-then-initialized options struct
/// ever holds it, so a single stamped field shows up in the byte compare.
const POISON: u64 = 0xA5A5_A5A5_A5A5_A5A5;

/// Word index of `implementation_identifier` — offset 8 under EVERY
/// candidate layout (the common prefix `instance_id`@0 / identifier@8 /
/// `domain_id`@16 / `security_options`@24..40 is layout-invariant; the
/// fields `rmw_init_options_init` stamps beyond it are not).
const IDENTIFIER_WORD: usize = 1;

/// The runtime distro to name so the guard genuinely compares the
/// build's OWN claim and must ADMIT it: a concrete claim names itself;
/// the unclaimed dev build names a member of the vendored snapshot's
/// era (the only names it admits); an `era:<token>` claim (a no-env
/// generated build) is a
/// CLAIM, not a runtime name — era.rs admits only that era's concrete
/// members — so the first member is taken from
/// the SAME table the guard consults, never from a literal.
fn agreeing_runtime_for(claim: &'static str) -> &'static str {
    // The unclaimed marker is the vendored ROLLING-era snapshot
    // and admits only that era's members — derived from the same row the
    // guard consults, never a literal (`kilted`, the earlier choice, is
    // now refused: 160-byte init options but a 112-byte MessageMember).
    let token = if claim == VENDORED_DEV_DISTRO {
        VENDORED_SNAPSHOT_ERA_TOKEN
    } else if let Some(token) = claim.strip_prefix(ERA_CLAIM_PREFIX) {
        token
    } else {
        return claim;
    };
    era_claim_members(token)
        .and_then(|members| members.first().copied())
        .unwrap_or_else(|| panic!("the build bakes `{claim}` but era.rs admits no member for it"))
}

/// An identifier that is NOT ours — a foreign rmw's, NUL-terminated.
const FOREIGN_IDENTIFIER: &[u8] = b"rmw_fastrtps_cpp\0";

/// The mismatch's exact shape: a Jazzy-built library under a Kilted
/// runtime (168-byte vs 160-byte `rmw_init_options_t`).
fn arm_mismatch() -> (BakedDistroOverrideGuard, EnvVarGuard) {
    (
        BakedDistroOverrideGuard::set("jazzy"),
        EnvVarGuard::set("ROS_DISTRO", "kilted"),
    )
}

/// A poisoned, 8-aligned buffer the size of `T` (this build's layout —
/// the only size the test can name; the point is that NOTHING in it moves).
fn poisoned_words<T>() -> Vec<u64> {
    assert_eq!(
        size_of::<T>() % 8,
        0,
        "the struct is a whole number of words"
    );
    vec![POISON; size_of::<T>() / 8]
}

/// Poisoned options whose identifier field is NULL — so that, absent the
/// hoisted guard, `rmw_init_options_init` would proceed to WRITE
/// instead of bouncing on "already initialized". That is
/// what makes the untouched-bytes arm bite on a guard moved after the
/// writes rather than pass on an earlier argument check.
fn poisoned_uninitialized_options() -> Vec<u64> {
    let mut words = poisoned_words::<rmw_init_options_t>();
    words[IDENTIFIER_WORD] = 0;
    words
}

/// A validly initialized options struct, built under a runtime this
/// build's OWN claim admits — set for the duration of the call, whatever
/// the process inherited (the guard is
/// in `rmw_init_options_init` too, so a fixture built under an inherited
/// contradicting `ROS_DISTRO` — the documented headerless Foxy run — would be
/// refused before the arm's real assertion runs).
fn initialized_options() -> rmw_init_options_t {
    let _admitted = EnvVarGuard::set("ROS_DISTRO", agreeing_runtime_for(baked_distro()));
    let mut options: rmw_init_options_t = unsafe { std::mem::zeroed() };
    let allocator: rcutils_allocator_t = unsafe { std::mem::zeroed() };
    assert_eq!(
        unsafe { rmw_init_options_init(&mut options, allocator) },
        RMW_RET_OK
    );
    options
}

/// The struct's words, read in place (the byte-chunking form
/// tripped rustc 1.98's `clippy::chunks_exact_to_as_chunks` on CI; a
/// word-aligned struct is simply a `[u64]` view).
fn words_of<T>(value: &T) -> Vec<u64> {
    assert_eq!(
        size_of::<T>() % 8,
        0,
        "the struct is a whole number of words"
    );
    assert!(std::mem::align_of::<T>() >= 8, "the struct is word-aligned");
    // SAFETY: `value` is a live `T`; the size and alignment asserts above
    // make the pointer a valid `[u64; size_of::<T>() / 8]` view for the
    // duration of the borrow.
    unsafe { std::slice::from_raw_parts(value as *const T as *const u64, size_of::<T>() / 8) }
        .to_vec()
}

fn line_level(line: &str) -> Option<&'static str> {
    let header = line.split(": ").next().unwrap_or(line);
    header
        .split_whitespace()
        .find_map(|token| LEVELS.into_iter().find(|level| *level == token))
}

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
/// The unclaimed-bindings BANNER renders `built_for` first and
/// `runtime_ros_distro` after it (`api/init.rs`) — the only key that may
/// follow `built_for` on that line.
const BANNER_KEYS_AFTER_BUILT_FOR: &[&str] = &["runtime_ros_distro"];

fn has_field_starting_a_token(line: &str, key: &str, value: &str, next_keys: &[&str]) -> bool {
    let needle = format!("{key}={value}");
    line.match_indices(&needle).any(|(at, _)| {
        let key_starts_a_token = at == 0 || line[..at].ends_with(char::is_whitespace);
        // …and the value ends at a boundary too: a prefix match
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

/// Exactly one refusal line, at ERROR, whose BODY is exactly the constant
/// paragraph followed by the four structured fields in declaration order
/// (nothing extra), naming the entry point and both sides of
/// the contradiction as whole `key=value` tokens — and NO other loud line
/// in the capture.
fn check_refusal(lines: &[&str], entry: &str) -> Result<(), String> {
    let hits: Vec<&str> = lines
        .iter()
        .copied()
        .filter(|l| l.contains(REFUSAL_MARKER))
        .collect();
    if hits.len() != 1 {
        return Err(format!(
            "expected exactly 1 refusal line, got {}:\n{}",
            hits.len(),
            lines.join("\n")
        ));
    }
    let line = hits[0];
    if line_level(line) != Some("ERROR") {
        return Err(format!("refusal not at ERROR level: {line}"));
    }
    // `rcl_error_channel=unavailable`: a cargo test process maps no
    // librcutils, so the rcl channel is provably absent here — "set" is
    // what a real host shows.
    let expected = format!(
        "{DISTRO_MISMATCH_REFUSAL} entry={entry} baked_ros_distro=jazzy runtime_ros_distro=kilted rcl_error_channel=unavailable built_for={}",
        built_for()
    );
    let body = body_of(line);
    if body != expected {
        return Err(format!(
            "refusal body is not exactly the paragraph + fields:\n  got:  {body}\n  want: {expected}"
        ));
    }
    for (key, value) in [
        ("entry", entry),
        ("baked_ros_distro", "jazzy"),
        ("runtime_ros_distro", "kilted"),
    ] {
        if !has_field(line, key, value) {
            return Err(format!("refusal missing {key}={value}: {line}"));
        }
    }
    // The refusal line renders `built_for` LAST (era.rs, the mismatch
    // refusal): nothing may follow it.
    if !has_field_starting_a_token(line, "built_for", built_for(), &[]) {
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

#[test]
#[traced_test]
#[serial]
fn options_init_on_a_mismatched_era_refuses_and_leaves_the_callers_bytes_untouched() {
    let _armed = arm_mismatch();
    let mut buffer = poisoned_uninitialized_options();
    let before = buffer.clone();
    let allocator: rcutils_allocator_t = unsafe { std::mem::zeroed() };
    let ret =
        unsafe { rmw_init_options_init(buffer.as_mut_ptr() as *mut rmw_init_options_t, allocator) };
    assert_eq!(
        ret, RMW_RET_ERROR,
        "a jazzy-built .so under ROS_DISTRO=kilted must refuse"
    );
    // THE pin: not one word of the caller's struct moved — a guard placed
    // after the writes still returns the refusal code but fails here.
    assert_eq!(
        buffer, before,
        "rmw_init_options_init wrote into the caller's struct before refusing"
    );
    logs_assert(|lines: &[&str]| check_refusal(lines, "rmw_init_options_init"));
}

#[test]
#[traced_test]
#[serial]
fn options_copy_on_a_mismatched_era_refuses_and_leaves_the_destination_untouched() {
    let src = initialized_options();
    let src_before = words_of(&src);
    let _armed = arm_mismatch();
    let mut dst = poisoned_uninitialized_options();
    let dst_before = dst.clone();
    let ret = unsafe { rmw_init_options_copy(&src, dst.as_mut_ptr() as *mut rmw_init_options_t) };
    assert_eq!(ret, RMW_RET_ERROR);
    assert_eq!(
        dst, dst_before,
        "rmw_init_options_copy memcpy'd into `dst` before refusing"
    );
    assert_eq!(
        words_of(&src),
        src_before,
        "`src` must be read-only either way"
    );
    logs_assert(|lines: &[&str]| check_refusal(lines, "rmw_init_options_copy"));
}

#[test]
#[traced_test]
#[serial]
fn options_fini_on_a_mismatched_era_refuses_and_does_not_zero() {
    let mut options = initialized_options();
    let before = words_of(&options);
    assert_ne!(
        before[IDENTIFIER_WORD], 0,
        "the fixture really is initialized"
    );
    let _armed = arm_mismatch();
    let ret = unsafe { rmw_init_options_fini(&mut options) };
    assert_eq!(ret, RMW_RET_ERROR);
    assert_eq!(
        words_of(&options),
        before,
        "rmw_init_options_fini zeroed the struct before refusing"
    );
    logs_assert(|lines: &[&str]| check_refusal(lines, "rmw_init_options_fini"));
}

#[test]
#[traced_test]
#[serial]
fn rmw_init_stays_a_defense_in_depth_guard_for_a_hand_built_options_struct() {
    // A caller that skipped rmw_init_options_init entirely (a static,
    // hand-stamped struct) never met the hoisted guard — rmw_init must
    // still refuse, before the FIRST context write.
    let mut options: rmw_init_options_t = unsafe { std::mem::zeroed() };
    options.implementation_identifier = ffi::implementation_identifier_ptr();
    let _armed = arm_mismatch();
    let mut context = poisoned_words::<rmw_context_t>();
    let before = context.clone();
    let ret = unsafe { rmw_init(&options, context.as_mut_ptr() as *mut rmw_context_t) };
    assert_eq!(ret, RMW_RET_ERROR);
    assert_eq!(
        context, before,
        "rmw_init wrote into the caller's context before refusing"
    );
    logs_assert(|lines: &[&str]| check_refusal(lines, "rmw_init"));
}

#[test]
#[traced_test]
#[serial]
fn the_happy_path_is_unchanged_and_really_writes() {
    // Anti-tautology for the untouched-bytes arms: with NO contradiction
    // the same entry points DO write — init stamps the identifier, copy
    // reproduces src, fini zeroes — and nothing loud is logged.
    //
    // Pass 1 runs with NO override, so the guard compares the build's OWN
    // baked claim (an override held across both
    // passes would leave the baked-identity path unexercised).
    // The environment must genuinely be consulted for that path to be
    // under test: a claiming build (container lanes) gets its own name,
    // an unclaimed dev build gets a member of its snapshot's era (the
    // only names it admits) — either way `runtime` is `Some`, so a broken baked
    // arm cannot hide behind an absent env. Pass 2 arms an explicitly
    // AGREEING override (jazzy/jazzy) — the literal-equality arm.
    let pass1_runtime = agreeing_runtime_for(baked_distro());
    for pass in ["build's own claim", "agreeing override"] {
        let _pass_env: (Option<BakedDistroOverrideGuard>, EnvVarGuard) =
            if pass == "build's own claim" {
                (None, EnvVarGuard::set("ROS_DISTRO", pass1_runtime))
            } else {
                (
                    Some(BakedDistroOverrideGuard::set("jazzy")),
                    EnvVarGuard::set("ROS_DISTRO", "jazzy"),
                )
            };
        let mut buffer = poisoned_uninitialized_options();
        let before = buffer.clone();
        let allocator: rcutils_allocator_t = unsafe { std::mem::zeroed() };
        let ret = unsafe {
            rmw_init_options_init(buffer.as_mut_ptr() as *mut rmw_init_options_t, allocator)
        };
        assert_eq!(ret, RMW_RET_OK, "{pass}: init must succeed");
        assert_ne!(
            buffer, before,
            "{pass}: a successful init must WRITE (the poison moved)"
        );
        assert_ne!(
            buffer[IDENTIFIER_WORD], 0,
            "{pass}: the identifier was stamped"
        );

        let mut dst = poisoned_uninitialized_options();
        let ret = unsafe {
            rmw_init_options_copy(
                buffer.as_ptr() as *const rmw_init_options_t,
                dst.as_mut_ptr() as *mut rmw_init_options_t,
            )
        };
        assert_eq!(ret, RMW_RET_OK, "{pass}: copy must succeed");
        assert_eq!(dst, buffer, "{pass}: copy reproduces src word for word");

        let ret = unsafe { rmw_init_options_fini(dst.as_mut_ptr() as *mut rmw_init_options_t) };
        assert_eq!(ret, RMW_RET_OK, "{pass}: fini must succeed");
        assert!(
            dst.iter().all(|w| *w == 0),
            "{pass}: fini zeroes the struct"
        );
    }
    // Anti-tautology anchor: everything above is a
    // SILENCE assertion, and "no refusal line, no loud line" is also
    // satisfied by an EMPTY capture — a blind capture (this crate's own
    // `install_tracing` winning the subscriber slot, a `tracing-test`
    // format change, a filter that stops covering this target) would
    // pass it while asserting nothing. So the test ends by deliberately
    // driving ONE refusal and requiring EXACTLY that line: the capture
    // must be able to see the seam it just claimed was silent. (The
    // sibling `rmw_cpp_bridge_refusal_test` already does this; the
    // discipline was not swept here.)
    {
        let _armed = arm_mismatch();
        let mut buffer = poisoned_uninitialized_options();
        let ret = unsafe { rmw_init_options_fini(buffer.as_mut_ptr() as *mut rmw_init_options_t) };
        assert_eq!(ret, RMW_RET_ERROR, "the anchor refusal must refuse");
    }
    logs_assert(|lines: &[&str]| {
        // Exactly one refusal — the ANCHOR — and nothing else loud, so
        // this subsumes the old "a happy path logged nothing" pair while
        // failing outright on an empty capture.
        check_refusal(lines, "rmw_init_options_fini")
    });
}

#[test]
#[traced_test]
#[serial]
fn a_panic_inside_the_era_guard_degrades_to_the_failure_code_not_an_abort() {
    // FFI safety: the era guard allocates and
    // installs tracing, so it must run INSIDE `ffi_guard` — a panic there
    // has to come back as RMW_RET_ERROR, never unwind through the C ABI
    // (which aborts the host; an unguarded variant aborts THIS process).
    // The seam fires at the top of the guard, before any caller-memory
    // access, so the caller's bytes must also be untouched.
    let src = initialized_options();
    let mut fini_target = initialized_options();
    let fini_before = words_of(&fini_target);
    let fired_before = era_guard_panics_fired();
    let _seam = EraGuardPanicGuard::arm();

    let mut buffer = poisoned_uninitialized_options();
    let before = buffer.clone();
    let allocator: rcutils_allocator_t = unsafe { std::mem::zeroed() };
    let ret =
        unsafe { rmw_init_options_init(buffer.as_mut_ptr() as *mut rmw_init_options_t, allocator) };
    assert_eq!(
        ret, RMW_RET_ERROR,
        "options_init: a panic degrades to the failure code"
    );
    assert_eq!(
        buffer, before,
        "options_init: the caller's bytes are untouched"
    );

    let mut dst = poisoned_uninitialized_options();
    let dst_before = dst.clone();
    let ret = unsafe { rmw_init_options_copy(&src, dst.as_mut_ptr() as *mut rmw_init_options_t) };
    assert_eq!(
        ret, RMW_RET_ERROR,
        "options_copy: a panic degrades to the failure code"
    );
    assert_eq!(dst, dst_before, "options_copy: `dst` is untouched");

    let ret = unsafe { rmw_init_options_fini(&mut fini_target) };
    assert_eq!(
        ret, RMW_RET_ERROR,
        "options_fini: a panic degrades to the failure code"
    );
    assert_eq!(
        words_of(&fini_target),
        fini_before,
        "options_fini: not zeroed"
    );

    // rmw_init shares the seam (defense in depth): a hand-stamped options
    // struct and a poisoned context — the refusal code back, the context
    // untouched.
    let mut stamped: rmw_init_options_t = unsafe { std::mem::zeroed() };
    stamped.implementation_identifier = ffi::implementation_identifier_ptr();
    let mut context = poisoned_words::<rmw_context_t>();
    let context_before = context.clone();
    let ret = unsafe { rmw_init(&stamped, context.as_mut_ptr() as *mut rmw_context_t) };
    assert_eq!(
        ret, RMW_RET_ERROR,
        "rmw_init: a panic degrades to the failure code"
    );
    assert_eq!(
        context, context_before,
        "rmw_init: the context is untouched"
    );

    // Attribution: the seam fired exactly once per entry point.
    assert_eq!(era_guard_panics_fired() - fired_before, 4);
    logs_assert(|lines: &[&str]| {
        // ffi_guard's containment line, at ERROR, carrying the injected
        // payload as the `panic=` field — three of them, and nothing else
        // loud (no refusal: the seam fires before the verdict).
        let contained: Vec<&str> = lines
            .iter()
            .copied()
            .filter(|l| l.contains("rmw entry point panicked"))
            .collect();
        if contained.len() != 4 {
            return Err(format!(
                "expected 4 containment lines, got {}:\n{}",
                contained.len(),
                lines.join("\n")
            ));
        }
        for line in &contained {
            if line_level(line) != Some("ERROR") {
                return Err(format!("containment line not at ERROR: {line}"));
            }
            if !line.contains(ERA_GUARD_PANIC_MSG) {
                return Err(format!(
                    "containment line does not carry the seam payload: {line}"
                ));
            }
        }
        if lines.iter().any(|l| l.contains(REFUSAL_MARKER)) {
            return Err("a refusal was logged although the seam fired first".to_string());
        }
        let extra = unexpected_loud_lines(lines, &contained);
        if !extra.is_empty() {
            return Err(format!("unexpected loud line(s):\n{}", extra.join("\n")));
        }
        Ok(())
    });
}

#[test]
#[traced_test]
#[serial]
fn pass_one_names_a_runtime_the_builds_own_claim_admits_for_every_claim_shape() {
    // Every claim shape build.rs can bake, pinned
    // to the guard ITSELF — the selected runtime must be admitted by that
    // claim, and must be a runtime NAME, never an `era:` label.
    for claim in [
        "jazzy",
        "kilted",
        VENDORED_DEV_DISTRO,
        "era:lyrical",
        "era:jazzy",
    ] {
        let runtime = agreeing_runtime_for(claim);
        assert!(
            !runtime.starts_with(ERA_CLAIM_PREFIX),
            "`{claim}` selected `{runtime}`, a claim rather than a runtime name"
        );
        assert_eq!(
            classify_distro_pair(claim, Some(runtime)),
            None,
            "the claim `{claim}` must admit its pass-one runtime `{runtime}`"
        );
    }
    // The era shapes pick a member of the guard's own membership table.
    for (claim, token) in [("era:lyrical", "lyrical"), ("era:jazzy", "jazzy")] {
        assert!(
            era_claim_admits(token, agreeing_runtime_for(claim)),
            "{claim}"
        );
    }
    // And whatever THIS build baked admits its own agreeing runtime (read the
    // process-global claim ONCE — a sibling's override drop between two
    // reads was a local flake).
    let own = baked_distro();
    assert_eq!(
        classify_distro_pair(own, Some(agreeing_runtime_for(own))),
        None
    );
}

#[test]
#[traced_test]
#[serial]
fn rmw_init_refuses_a_mismatched_era_before_reading_the_identifier() {
    // ABI safety: if rmw_init read the
    // options' identifier BEFORE the era guard, a mismatched distro
    // with a foreign identifier would surface as an identifier error instead
    // of the refusal. The guard sits right after the null check.
    let mut options: rmw_init_options_t = unsafe { std::mem::zeroed() };
    options.implementation_identifier = FOREIGN_IDENTIFIER.as_ptr() as *const c_char;

    // Control (no contradiction — the build's own claim, an admitting
    // env): the foreign identifier is rejected as such, nothing loud.
    {
        let _env = EnvVarGuard::set("ROS_DISTRO", agreeing_runtime_for(baked_distro()));
        let mut context = poisoned_words::<rmw_context_t>();
        let before = context.clone();
        let ret = unsafe { rmw_init(&options, context.as_mut_ptr() as *mut rmw_context_t) };
        assert_eq!(
            ret, RMW_RET_INCORRECT_RMW_IMPLEMENTATION,
            "control: identifier error"
        );
        assert_eq!(context, before, "control: context untouched");
    }
    logs_assert(|lines: &[&str]| {
        let loud = unexpected_loud_lines(lines, &[]);
        if !loud.is_empty() {
            return Err(format!(
                "control logged something loud:\n{}",
                loud.join("\n")
            ));
        }
        Ok(())
    });

    // The pin: with the contradiction armed, the era refusal WINS over
    // the identifier error (the reversed order returns 12 here).
    let _armed = arm_mismatch();
    let mut context = poisoned_words::<rmw_context_t>();
    let before = context.clone();
    let ret = unsafe { rmw_init(&options, context.as_mut_ptr() as *mut rmw_context_t) };
    assert_eq!(
        ret, RMW_RET_ERROR,
        "the era refusal must win over the identifier error"
    );
    assert_eq!(context, before, "context untouched");
    logs_assert(|lines: &[&str]| check_refusal(lines, "rmw_init"));
}

// =====================================================================
// A READ-DETECTING caller-memory fixture.
//
// A readable buffer with a null identifier cannot separate "refused
// before touching the struct" from "read the identifier, then refused":
// a variant that hoists the identifier read above the era guard but still
// returns before writing passes every byte-unchanged arm. So the caller
// struct is now an `mmap`'d PROT_NONE page — ANY access faults — and the
// arms run in a CHILD PROCESS (the crate's `rmw_shadow_take_test`
// pattern: the test binary re-execs itself into an `#[ignore]`d
// entrypoint, no `fork`, so no post-fork allocator/env-lock hazards in a
// threaded harness), because a fault must end a process and the parent
// classifies how the child ended:
//
// * MISMATCH mode (override `jazzy`, `ROS_DISTRO=kilted`): the entry must
//   return the refusal WITHOUT touching the page — the child exits
//   `40 + RMW_RET_ERROR`. A pre-guard read faults ⇒ the child dies by
//   signal ⇒ the arm fails (never passes).
// * AGREE mode (no override, the build's own claim admitted): the same
//   page under the same entry must FAULT — the child dies by
//   SIGSEGV/SIGBUS (per-platform via libc). This is the anti-tautology:
//   it proves the fixture really faults on the first touch, so "returned
//   normally" in MISMATCH mode means "never touched", not "page was
//   readable after all".
//
// Both modes require the REACHED marker on stderr before the call, so a
// child that died or exited anywhere else proves nothing. The exit code
// is offset by 40 to keep it apart from libtest's own 0/101.
// =====================================================================

const PROBE_ENTRY_ENV: &str = "CER_ERA_PROBE_ENTRY";
const PROBE_MODE_ENV: &str = "CER_ERA_PROBE_MODE";
const PROBE_EXIT_OFFSET: i32 = 40;
const PROBE_REACHED: &str = "ERA-PROBE-REACHED";

/// One page of address space mapped PROT_NONE: every load or store faults.
struct InaccessiblePage {
    ptr: *mut libc::c_void,
    len: usize,
}

impl InaccessiblePage {
    fn new() -> Self {
        let len = unsafe { libc::sysconf(libc::_SC_PAGESIZE) } as usize;
        assert!(
            len >= size_of::<rmw_init_options_t>() && len >= size_of::<rmw_context_t>(),
            "a page must hold the caller structs"
        );
        let ptr = unsafe {
            libc::mmap(
                std::ptr::null_mut(),
                len,
                libc::PROT_NONE,
                libc::MAP_PRIVATE | libc::MAP_ANON,
                -1,
                0,
            )
        };
        assert_ne!(ptr, libc::MAP_FAILED, "mmap(PROT_NONE) failed");
        Self { ptr, len }
    }

    fn as_options(&self) -> *mut rmw_init_options_t {
        self.ptr as *mut rmw_init_options_t
    }

    fn as_context(&self) -> *mut rmw_context_t {
        self.ptr as *mut rmw_context_t
    }
}

impl Drop for InaccessiblePage {
    fn drop(&mut self) {
        unsafe {
            libc::munmap(self.ptr, self.len);
        }
    }
}

/// The CHILD: hand a PROT_NONE page to the named entry point under the
/// named mode and exit with `40 + ret` — or die by signal on a touch.
/// `#[ignore]`d so it only ever runs when a parent spawns it.
#[test]
#[ignore = "child arm of the PROT_NONE probes; spawned by the parent tests"]
fn era_probe_child() {
    // `var_os` + a loud refusal: the parent sets
    // both to ASCII, so an unreadable value is a broken harness, never
    // "not a child" — `var(..).ok()` would fold it into the silent return.
    let read = |key: &str| {
        std::env::var_os(key).map(|raw| {
            raw.into_string()
                .unwrap_or_else(|raw| panic!("{key} is set but not UTF-8 ({raw:?})"))
        })
    };
    let (Some(entry), Some(mode)) = (read(PROBE_ENTRY_ENV), read(PROBE_MODE_ENV)) else {
        // Both ABSENT is the un-spawned run (`--ignored` over the whole
        // binary), which reports PASS having probed nothing — inherent to
        // the child-entrypoint idiom (`shm_ring_test::cross_process_-
        // child_entrypoint` has the same shape). A PARTIAL configuration
        // is not silent: the parent asserts the child printed
        // `PROBE_REACHED`.
        return;
    };
    // MISMATCH: the override supplies the baked side; the parent supplied
    // ROS_DISTRO=kilted. AGREE: no override — the build's own claim
    // against a ROS_DISTRO it admits (also supplied by the parent).
    let _override = (mode == "mismatch").then(|| BakedDistroOverrideGuard::set("jazzy"));
    let page = InaccessiblePage::new();
    let other = InaccessiblePage::new();
    {
        use std::io::Write as _;
        eprintln!("{PROBE_REACHED} entry={entry} mode={mode}");
        std::io::stderr().flush().expect("flush marker");
    }
    let allocator: rcutils_allocator_t = unsafe { std::mem::zeroed() };
    let ret = unsafe {
        match entry.as_str() {
            "options_init" => rmw_init_options_init(page.as_options(), allocator),
            "options_copy" => rmw_init_options_copy(page.as_options(), other.as_options()),
            "options_fini" => rmw_init_options_fini(page.as_options()),
            "rmw_init" => rmw_init(page.as_options(), other.as_context()),
            unknown => panic!("unknown probe entry `{unknown}`"),
        }
    };
    // `_exit`, not `process::exit`: a probe child must not run destructors
    // or atexit hooks — it reports one number and is gone.
    unsafe { libc::_exit(PROBE_EXIT_OFFSET + ret) }
}

/// Spawn the child for `entry` under `mode` with the given `ROS_DISTRO`.
fn probe(entry: &str, mode: &str, runtime_distro: &str) -> (std::process::ExitStatus, String) {
    let exe = std::env::current_exe().expect("current_exe");
    let out = std::process::Command::new(exe)
        .args([
            "--exact",
            "era_probe_child",
            "--ignored",
            "--nocapture",
            "--test-threads=1",
        ])
        .env(PROBE_ENTRY_ENV, entry)
        .env(PROBE_MODE_ENV, mode)
        .env("ROS_DISTRO", runtime_distro)
        // The child has NO test subscriber: the refusal reaches its stderr
        // only through the PRODUCTION installer and the DEFAULT filter —
        // exactly the channel a real host sees — so a lane's RUST_LOG must
        // not stand in for it.
        .env_remove("RUST_LOG")
        .output()
        .expect("spawn child");
    (
        out.status,
        String::from_utf8_lossy(&out.stderr).into_owned(),
    )
}

/// The two-mode contract for one entry point (see the section comment).
fn assert_refuses_without_touching_caller_memory(entry: &str) {
    use std::os::unix::process::ExitStatusExt;

    // MISMATCH: refusal, and the PROT_NONE page was never touched.
    let (status, stderr) = probe(entry, "mismatch", "kilted");
    assert!(
        stderr.contains(PROBE_REACHED),
        "{entry}: the child must REACH the call before anything else happens:\n{stderr}"
    );
    assert_eq!(
        status.signal(),
        None,
        "{entry}: the child died by signal {:?} under a MISMATCH — the entry point touched \
         the caller's struct BEFORE the era guard:\n{stderr}",
        status.signal()
    );
    assert_eq!(
        status.code(),
        Some(PROBE_EXIT_OFFSET + RMW_RET_ERROR),
        "{entry}: the child must exit with the refusal code, got {status:?}:\n{stderr}"
    );
    // The production channel: every in-process arm runs
    // under `#[traced_test]`, whose subscriber makes `install_tracing`'s
    // `try_init` a no-op — so a deleted installer passed them all while
    // a deployed .so refused in silence. The child has no such
    // subscriber: the paragraph must reach its stderr through the real
    // installer and the default `rmw_cerulion=warn` filter, at ERROR,
    // naming the entry point.
    let field = if entry == "rmw_init" {
        "rmw_init".to_string()
    } else {
        format!("rmw_init_{entry}")
    };
    // Every refusal line, not the first: a
    // duplicated refusal or a second loud diagnostic beside it is a
    // regression this probe exists to see.
    let refusals: Vec<&str> = stderr
        .lines()
        .filter(|l| l.contains(REFUSAL_MARKER))
        .collect();
    let [refusal] = refusals[..] else {
        panic!(
            "{entry}: expected exactly one refusal on the child's stderr, got {}:\n{stderr}",
            refusals.len()
        );
    };
    assert_eq!(line_level(refusal), Some("ERROR"), "{entry}: {refusal}");
    let other_loud: Vec<&str> = stderr
        .lines()
        .filter(|l| *l != refusal && matches!(line_level(l), Some("ERROR") | Some("WARN")))
        .collect();
    assert!(
        other_loud.is_empty(),
        "{entry}: loud line(s) beside the refusal:\n{}",
        other_loud.join("\n")
    );
    assert!(
        has_field(refusal, "entry", &field),
        "{entry}: refusal missing entry={field}: {refusal}"
    );

    // AGREE: the same page under the same entry must FAULT on first touch —
    // proof the fixture detects reads at all.
    let (status, stderr) = probe(entry, "agree", agreeing_runtime_for(baked_distro()));
    assert!(
        stderr.contains(PROBE_REACHED),
        "{entry}: the agreeing child must REACH the call:\n{stderr}"
    );
    let signal = status.signal();
    assert!(
        matches!(signal, Some(s) if s == libc::SIGSEGV || s == libc::SIGBUS),
        "{entry}: under an AGREEING era the PROT_NONE page must fault (SIGSEGV/SIGBUS) — \
         a child that exited ({status:?}) means the page was never touched, which would \
         make the MISMATCH arm vacuous:\n{stderr}"
    );
}

#[test]
#[serial]
fn options_init_on_a_mismatched_era_refuses_without_reading_the_callers_struct() {
    assert_refuses_without_touching_caller_memory("options_init");
}

#[test]
#[serial]
fn options_copy_on_a_mismatched_era_refuses_without_reading_either_struct() {
    assert_refuses_without_touching_caller_memory("options_copy");
}

#[test]
#[serial]
fn options_fini_on_a_mismatched_era_refuses_without_reading_the_callers_struct() {
    assert_refuses_without_touching_caller_memory("options_fini");
}

#[test]
#[serial]
fn rmw_init_on_a_mismatched_era_refuses_without_reading_options_or_context() {
    assert_refuses_without_touching_caller_memory("rmw_init");
}

#[test]
#[serial]
fn a_probe_env_the_child_cannot_read_is_a_loud_harness_failure_not_a_silent_pass() {
    // Found in a class sweep: the child read its two variables with
    // `var(..).ok()`, so an unreadable value folded into the "not a
    // child" early return — the child exited 0 having probed NOTHING,
    // and a parent that trusted the exit code alone would have called
    // that a pass. The read now refuses loudly: the child must end
    // UNSUCCESSFULLY, name the variable, and never reach a probe.
    use std::os::unix::ffi::OsStrExt as _;
    let exe = std::env::current_exe().expect("current_exe");
    let out = std::process::Command::new(exe)
        .args([
            "--exact",
            "era_probe_child",
            "--ignored",
            "--nocapture",
            "--test-threads=1",
        ])
        .env(
            PROBE_ENTRY_ENV,
            std::ffi::OsStr::from_bytes(b"options_init\xff"),
        )
        .env(PROBE_MODE_ENV, "mismatch")
        .env_remove("RUST_LOG")
        .output()
        .expect("spawn child");
    let stderr = String::from_utf8_lossy(&out.stderr);
    assert!(
        !out.status.success(),
        "an unreadable {PROBE_ENTRY_ENV} must fail the child, got {:?}:\n{stderr}",
        out.status
    );
    assert!(
        stderr.contains(&format!("{PROBE_ENTRY_ENV} is set but not UTF-8")),
        "the child must name the unreadable variable:\n{stderr}"
    );
    assert!(
        !stderr.contains(PROBE_REACHED),
        "the child must not reach a probe under an env it could not read:\n{stderr}"
    );
}

#[test]
#[traced_test]
#[serial]
fn an_unclaimed_build_admitted_under_its_own_era_is_announced_at_warn() {
    // The `rmw_init` UNCLAIMED-bindings banner needs
    // executing coverage, not only its pure predicate. Under a runtime the
    // build's own claim ADMITS, rmw_init must succeed (the transport comes
    // up for real), and iff the claim is the unclaimed marker it must say
    // so — one WARN naming the runtime distro — while a claiming lane
    // must stay silent. No ERROR line either way.
    let own = baked_distro();
    let runtime = agreeing_runtime_for(own);
    let _env = EnvVarGuard::set("ROS_DISTRO", runtime);
    let mut options = initialized_options();
    let mut context: rmw_context_t = unsafe { std::mem::zeroed() };
    assert_eq!(unsafe { rmw_init(&options, &mut context) }, RMW_RET_OK);
    assert_eq!(unsafe { rmw_shutdown(&mut context) }, RMW_RET_OK);
    assert_eq!(unsafe { rmw_context_fini(&mut context) }, RMW_RET_OK);
    assert_eq!(unsafe { rmw_init_options_fini(&mut options) }, RMW_RET_OK);
    // Anti-tautology anchor: on a claiming lane
    // (`unclaimed == false`) every assertion below is negative, so an
    // EMPTY capture would satisfy the whole arm. Drive ONE refusal at
    // the end and require exactly it, so the capture must be able to see
    // the seam. (On the unclaimed lane the banner already anchors it;
    // the anchor is harmless there and keeps both lanes identical.)
    {
        let _armed = arm_mismatch();
        let mut buffer = poisoned_uninitialized_options();
        let ret = unsafe { rmw_init_options_fini(buffer.as_mut_ptr() as *mut rmw_init_options_t) };
        assert_eq!(ret, RMW_RET_ERROR, "the anchor refusal must refuse");
    }
    let unclaimed = own == VENDORED_DEV_DISTRO;
    logs_assert(|lines: &[&str]| {
        let banners: Vec<&str> = lines
            .iter()
            .copied()
            .filter(|l| l.contains("UNCLAIMED bindings"))
            .collect();
        if unclaimed {
            if banners.len() != 1 {
                return Err(format!(
                    "expected exactly 1 unclaimed banner, got {}:\n{}",
                    banners.len(),
                    lines.join("\n")
                ));
            }
            let line = banners[0];
            if line_level(line) != Some("WARN") {
                return Err(format!("banner not at WARN: {line}"));
            }
            if !has_field(line, "runtime_ros_distro", runtime) {
                return Err(format!(
                    "banner missing runtime_ros_distro={runtime}: {line}"
                ));
            }
            if !has_field_starting_a_token(
                line,
                "built_for",
                built_for(),
                BANNER_KEYS_AFTER_BUILT_FOR,
            ) {
                return Err(format!("banner missing built_for: {line}"));
            }
        } else if !banners.is_empty() {
            return Err(format!(
                "a claiming build announced itself as unclaimed:\n{}",
                banners.join("\n")
            ));
        }
        // EXACTLY ONE refusal — the anchor above, naming the entry it
        // was driven through — and no OTHER ERROR. An ambient error (a
        // stale iceoryx2 SHM state under the real transport bring-up
        // shows as a cerulion_core error) is a harness problem, not the
        // guard, and must be loud rather than filtered; a SECOND refusal
        // would mean the admitted `rmw_init` refused too.
        let refusals: Vec<&str> = lines
            .iter()
            .copied()
            .filter(|l| l.contains(REFUSAL_MARKER))
            .collect();
        if refusals.len() != 1 {
            return Err(format!(
                "expected exactly the 1 anchor refusal, got {}:\n{}",
                refusals.len(),
                lines.join("\n")
            ));
        }
        if !has_field(refusals[0], "entry", "rmw_init_options_fini") {
            return Err(format!(
                "the only refusal is not the anchor: {}",
                refusals[0]
            ));
        }
        if line_level(refusals[0]) != Some("ERROR") {
            return Err(format!(
                "the anchor refusal is not at ERROR: {}",
                refusals[0]
            ));
        }
        if let Some(err) = lines
            .iter()
            .find(|l| line_level(l) == Some("ERROR") && !l.contains(REFUSAL_MARKER))
        {
            return Err(format!("an admitted rmw_init logged an ERROR: {err}"));
        }
        Ok(())
    });
}
