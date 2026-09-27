// SPDX-License-Identifier: MIT OR Apache-2.0
//! Pure account-service UUID mapping shared by login and account consumers.

use crate::format::AccountId;
use sha2::{Digest, Sha256};

const UUID_NAMESPACE: &[u8] = b"cerulion:supabase-account:v1\0";

/// Map a canonical hyphenated UUID, accepting either hex case. Authentication
/// retains the original service id; these domain-separated bytes identify pairing.
pub fn account_from_uuid(value: &str) -> Option<AccountId> {
    let bytes = uuid_bytes(value)?;
    let mut hash = Sha256::new();
    hash.update(UUID_NAMESPACE);
    hash.update(bytes);
    Some(AccountId(hash.finalize().into()))
}

/// Parse exactly 36 characters into the UUID's 16 network-order bytes.
/// Whitespace, braces and alternate separator layouts are refused.
pub fn uuid_bytes(value: &str) -> Option<[u8; 16]> {
    let bytes = value.as_bytes();
    if bytes.len() != 36 {
        return None;
    }
    let mut output = [0u8; 16];
    let mut nibble = 0;
    for (index, &byte) in bytes.iter().enumerate() {
        if matches!(index, 8 | 13 | 18 | 23) {
            if byte != b'-' {
                return None;
            }
            continue;
        }
        let digit = match byte {
            b'0'..=b'9' => byte - b'0',
            b'a'..=b'f' => byte - b'a' + 10,
            b'A'..=b'F' => byte - b'A' + 10,
            _ => return None,
        };
        output[nibble / 2] = (output[nibble / 2] << 4) | digit;
        nibble += 1;
    }
    Some(output)
}
