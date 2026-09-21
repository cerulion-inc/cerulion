// SPDX-License-Identifier: AGPL-3.0-only
#![no_main]

use libfuzzer_sys::fuzz_target;

fuzz_target!(|data: &[u8]| {
    // Fuzz WireHeader::read_from_buf with arbitrary bytes.
    // Should never panic — returns WireError for invalid input.
    let _ = cerulion_core::wire::WireHeader::read_from_buf(data);
});
