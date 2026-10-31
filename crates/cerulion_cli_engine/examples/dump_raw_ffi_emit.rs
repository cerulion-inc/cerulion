// SPDX-License-Identifier: AGPL-3.0-only
//! MAINTAINER TOOL, not an example of using Cerulion: it dumps the raw-FFI
//! generator output for the canonical oracle metadata so the hand-pasted
//! fixture stays in sync. An application never calls the template generator.
//!
//! Regenerate the fixture's lib.rs from the repo root with:
//!   cargo run -p cerulion_cli_engine --example dump_raw_ffi_emit > \
//!       crates/test_fixtures/test_node_raw_ffi_template_cdylib/src/lib.rs

use cerulion_cli_engine::node_metadata::NodeMetadata;
use cerulion_cli_engine::templates::generate_lib_rs;

fn main() {
    let m = NodeMetadata {
        throttle_ms: None,
        node_type: "raw_ffi_template".to_string(),
        policy: None,
        inputs: vec![],
        outputs: vec![],
    };
    print!("{}", generate_lib_rs(&m));
}
