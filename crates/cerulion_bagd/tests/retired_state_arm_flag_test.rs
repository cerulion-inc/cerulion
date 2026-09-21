// SPDX-License-Identifier: AGPL-3.0-only
//! `--state-arm` is retired, and `--state-tag` is
//! what replaced it.
//!
//! # Why a flag gets a test for NOT existing
//!
//! Killed, not deprecated — the `--network` precedent. No release
//! carries `--state-arm`, so there is no compatibility
//! debt to pay and no reason to leave a no-op behind: a flag that parses and does
//! nothing is worse than one that does not parse, because the operator who typed
//! it goes away believing they armed something.
//!
//! What makes this worth pinning rather than trusting is that its deletion is
//! INVISIBLE to every other test in the repo. No test anywhere passed the literal
//! `--state-arm` string — the whole `#[arg]` was reachable only through argv —
//! so re-introducing it (or any of its two parameter siblings) would sail through
//! the entire suite, and a second party able to CREATE the arm word is exactly the
//! competing-plane hazard that retiring the flag collapsed.
//!
//! The `--state-tag` half is here for the same reason from the other side: it is
//! the flag `graph run --record` now passes on every recording, and a rename or a
//! value-shape change would make every recorded bag silently anchor-free.

use clap::Parser;

/// `cerulion bagd`'s argv, parsed exactly as the binary parses it.
#[derive(Parser, Debug)]
struct Harness {
    #[command(flatten)]
    args: cerulion_bagd::BagdArgs,
}

fn parse(extra: &[&str]) -> Result<Harness, clap::Error> {
    let mut argv = vec!["bagd", "--out", "/tmp/x.mcap", "--topic", "/t"];
    argv.extend_from_slice(extra);
    Harness::try_parse_from(argv)
}

/// ANTI-TAUTOLOGY: the harness really parses this binary's arguments.
///
/// Without it, a harness broken in any way at all would make every refusal below
/// pass for the wrong reason — "unknown argument" is what a broken parser says
/// about everything.
#[test]
fn the_harness_parses_the_recorders_real_arguments() {
    let ok = parse(&[]).expect("the base argv is valid");
    assert_eq!(ok.args.out, std::path::PathBuf::from("/tmp/x.mcap"));
    assert_eq!(ok.args.topics, vec!["/t".to_string()]);
    // A sibling that still exists, so "this flag is gone" is a claim about THAT
    // flag rather than about the whole `--state-*` family.
    let rings = parse(&["--state-ring", "cer_sr_0"]).expect("--state-ring survives");
    assert_eq!(rings.args.state_rings, vec!["cer_sr_0".to_string()]);
}

/// THE PIN: `--state-arm` no longer parses, in either of the two shapes it used
/// to accept.
///
/// It took an OPTIONAL value (`num_args = 0..=1`), so both the bare flag and the
/// valued form were legal argv and both must now be refused. Asserting only one
/// would leave the other free to come back.
#[test]
fn a_retired_state_arm_flag_is_an_unknown_argument() {
    for form in [
        vec!["--state-arm"],
        vec!["--state-arm", "cer_run_0000000000000000000000000000002a"],
    ] {
        let err = parse(&form).expect_err("--state-arm is retired and must not parse");
        assert_eq!(
            err.kind(),
            clap::error::ErrorKind::UnknownArgument,
            "`{form:?}` must be refused as an UNKNOWN argument, not accepted and \
             ignored: {err}"
        );
    }
}

/// …and so are the two flags that existed only to parameterise it.
///
/// They are separate `#[arg]`s, so deleting one does not delete the others: a
/// revert that restored the cadence flag alone would leave an argument that
/// parses, is stored, and can reach nothing.
#[test]
fn the_flags_that_only_parameterised_the_arm_are_gone_too() {
    for form in [
        vec!["--state-cadence-steps", "30000"],
        vec!["--state-first-anchor-step", "7"],
    ] {
        let err = parse(&form).expect_err("retired with the arm they parameterised");
        assert_eq!(
            err.kind(),
            clap::error::ErrorKind::UnknownArgument,
            "`{form:?}`: {err}"
        );
    }
}

/// The REPLACEMENT parses, and carries the tag verbatim into the config the
/// recorder sweeps with.
///
/// The value is a mapped-SHM tag both halves DERIVE from a run id, so a transform
/// applied here — a trim that ate a legitimate character, a case fold, a
/// truncation — would send the sweep to a name no rank publishes under and the
/// bag would come out anchor-free with nothing logged.
#[test]
fn the_state_tag_flag_carries_the_plane_name_verbatim() {
    const TAG: &str = "cer_run_0000000000000000000000000000002a";
    let parsed = parse(&["--state-tag", TAG]).expect("--state-tag parses");
    assert_eq!(parsed.args.state_tag.as_deref(), Some(TAG));

    // It takes a REQUIRED value: a bare `--state-tag` names no plane, and
    // accepting it would arm a sweep against the empty string.
    let err = parse(&["--state-tag"]).expect_err("a bare --state-tag names nothing");
    assert_eq!(err.kind(), clap::error::ErrorKind::InvalidValue);

    // Absent = drain nothing, which is what keeps an ordinary `cerulion bagd`
    // recording byte-identical to one made before node-state anchors existed.
    assert_eq!(parse(&[]).expect("base").args.state_tag, None);
}
