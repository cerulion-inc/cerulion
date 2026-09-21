// SPDX-License-Identifier: AGPL-3.0-only
//! Well-known-port + env-override parsing oracles.

use cerud::constants::{
    parse_ops_port, CERULION_OPS_PORT, LEASE_DEADMAN_WINDOW_MS_ROBOT_CONFIRM,
    SAFE_FRAME_DESCRIPTION_ROBOT_CONFIRM,
};
use cerud::error::CerudError;

#[test]
fn default_port_is_the_well_known_ops_port() {
    // Unset override → the well-known port.
    assert_eq!(parse_ops_port(None).unwrap(), CERULION_OPS_PORT);
    // 7684, and DELIBERATELY not zenoh's 7447 nor the bare gRPC 50051.
    assert_eq!(CERULION_OPS_PORT, 7684);
    assert_ne!(CERULION_OPS_PORT, 7447);
    assert_ne!(CERULION_OPS_PORT, 50051);
}

#[test]
fn valid_override_is_honored() {
    assert_eq!(parse_ops_port(Some("9000")).unwrap(), 9000);
    // Surrounding whitespace is tolerated.
    assert_eq!(parse_ops_port(Some("  9000  ")).unwrap(), 9000);
}

#[test]
fn malformed_override_is_refused_loudly_not_silently_ignored() {
    for bad in ["notaport", "-1", "70000", "0", ""] {
        match parse_ops_port(Some(bad)) {
            Err(CerudError::Config(msg)) => assert!(msg.contains("CERULION_OPS_PORT")),
            other => panic!("expected a Config error for {bad:?}, got {other:?}"),
        }
    }
}

#[test]
fn deadman_window_is_the_500ms_robot_confirm_placeholder_the_docs_cite() {
    // DOC PIN: `docs/remote_plane.md` documents the deadman
    // window as 500 ms (the ROBOT_CONFIRM placeholder) and reasons about it vs
    // relayed-WAN RTT. This oracle hard-codes the value the doc cites, so the
    // constant cannot silently drift out from under the doc: if the window is
    // re-tuned, THIS test fails, forcing the doc + the on-robot-confirm note to be
    // updated in the same change. (Not a self-compare — 500 is the hand-pasted
    // value from the prose.)
    assert_eq!(
        LEASE_DEADMAN_WINDOW_MS_ROBOT_CONFIRM, 500,
        "docs/remote_plane.md cites a 500 ms deadman window; update the doc if you re-tune this"
    );
    // The safe-frame description the doc paraphrases is a non-empty placeholder
    // (its exact CONTENTS are ROBOT_CONFIRM — the doc must not claim otherwise).
    assert!(
        SAFE_FRAME_DESCRIPTION_ROBOT_CONFIRM.contains("confirmed on-robot"),
        "the safe-frame description must flag its contents as ROBOT_CONFIRM: {SAFE_FRAME_DESCRIPTION_ROBOT_CONFIRM:?}"
    );
}
