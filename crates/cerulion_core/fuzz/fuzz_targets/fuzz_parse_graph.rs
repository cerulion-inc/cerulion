// SPDX-License-Identifier: AGPL-3.0-only
#![no_main]

use libfuzzer_sys::fuzz_target;

fuzz_target!(|data: &[u8]| {
    // Fuzz YAML graph parsing with arbitrary bytes.
    // Should never panic — returns TransportError for invalid input.
    if let Ok(yaml_str) = std::str::from_utf8(data) {
        let _ = cerulion_core::graph::parse_graph(yaml_str);
    }
});
