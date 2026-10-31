// SPDX-License-Identifier: AGPL-3.0-only
//! `docs/networking.md` shows the banner a running `cerulion-netd` writes
//! first on every control connection. A reader compares it with a live
//! daemon's banner, so the example has to be the line this build writes:
//! the same `protocol` ([`PROTOCOL_VERSION`]), the same `wire_header`
//! ([`cerulion_core::wire::WIRE_HEADER_VERSION`]) and the same field order.
//! This test reads every banner line in the page and compares it with
//! `Hello::new().to_json_line()`, so a version bump fails here until the page
//! is corrected.

use std::path::PathBuf;

use cerulion_netd::{Hello, PROTOCOL_VERSION};

/// The marker every netd banner line starts with.
const BANNER_PREFIX: &str = r#"{"hello":"cerulion-netd""#;

/// `docs/networking.md` sits under the workspace root, two levels up from this crate.
fn networking_page() -> String {
    let path = PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .parent()
        .expect("the crate directory has a parent")
        .parent()
        .expect("crates/ has a parent (the repo root)")
        .join("docs/networking.md");
    std::fs::read_to_string(&path).unwrap_or_else(|e| panic!("cannot read {}: {e}", path.display()))
}

#[test]
fn the_networking_page_shows_the_banner_this_daemon_writes() {
    let page = networking_page();
    let banners: Vec<&str> = page
        .lines()
        .map(str::trim)
        .filter(|line| line.starts_with(BANNER_PREFIX))
        .collect();
    assert!(
        !banners.is_empty(),
        "docs/networking.md carries no line starting with {BANNER_PREFIX}; the worked \
         example of the netd banner is gone or was reworded"
    );
    let expected = Hello::new().to_json_line();
    for banner in banners {
        assert_eq!(
            banner, expected,
            "docs/networking.md shows a netd banner this build does not write \
             (PROTOCOL_VERSION is {PROTOCOL_VERSION}); correct the page"
        );
    }
}

#[test]
fn the_expected_banner_states_both_versions() {
    // The comparison above is only as strong as the line it compares with:
    // it must carry both version fields, with this build's values.
    let expected = Hello::new().to_json_line();
    let value: serde_json::Value = serde_json::from_str(&expected).expect("the banner is JSON");
    assert_eq!(value["protocol"], PROTOCOL_VERSION);
    assert_eq!(
        value["wire_header"],
        cerulion_core::wire::WIRE_HEADER_VERSION
    );
}
