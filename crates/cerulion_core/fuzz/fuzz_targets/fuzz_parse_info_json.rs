// SPDX-License-Identifier: AGPL-3.0-only
#![no_main]

use libfuzzer_sys::fuzz_target;

fuzz_target!(|data: &[u8]| {
    // Fuzz the cdylib JSON info parser with arbitrary bytes.
    // Should never panic — returns TransportError for invalid input.
    if let Ok(json_str) = std::str::from_utf8(data) {
        let _ = cerulion_core::graph::fuzz_parse_info_json(json_str);
    }
});
