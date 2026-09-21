// SPDX-License-Identifier: AGPL-3.0-only
//! SHA-256 helpers shared by the receipt hash-chain and the deploy bundles,
//! plus a deterministic canonical-JSON encoder.
//!
//! The canonical encoder makes hashing of structured values reproducible and
//! injection-safe: object keys are emitted in sorted order (independent of the
//! `serde_json` map backing), and scalars go through `serde_json` for correct
//! escaping. This is what lets the receipt chain hash a preimage struct
//! (rather than a `|`-delimited string a caller could forge across fields).

use std::io::Read;
use std::path::Path;

use sha2::{Digest, Sha256};

use crate::error::CerudResult;

/// Lowercase-hex SHA-256 of a byte slice.
pub fn sha256_hex(bytes: &[u8]) -> String {
    let mut hasher = Sha256::new();
    hasher.update(bytes);
    hex::encode(hasher.finalize())
}

/// Lowercase-hex SHA-256 of a file, read in bounded chunks (never slurps the
/// whole file into memory — deploy cdylibs can be large).
pub fn sha256_file(path: &Path) -> CerudResult<String> {
    let mut file = std::fs::File::open(path)?;
    let mut hasher = Sha256::new();
    let mut buf = [0u8; 64 * 1024];
    loop {
        let n = file.read(&mut buf)?;
        if n == 0 {
            break;
        }
        hasher.update(&buf[..n]);
    }
    Ok(hex::encode(hasher.finalize()))
}

/// Deterministic canonical JSON bytes for a value: object keys sorted, no
/// insignificant whitespace, scalars escaped by `serde_json`.
pub fn canonical_json_bytes(value: &serde_json::Value) -> Vec<u8> {
    let mut out = String::new();
    write_canonical(value, &mut out);
    out.into_bytes()
}

fn write_canonical(value: &serde_json::Value, out: &mut String) {
    use serde_json::Value;
    match value {
        Value::Object(map) => {
            let mut entries: Vec<(&String, &Value)> = map.iter().collect();
            entries.sort_by(|a, b| a.0.cmp(b.0));
            out.push('{');
            for (i, (k, v)) in entries.iter().enumerate() {
                if i > 0 {
                    out.push(',');
                }
                // Keys are strings; serialize through serde_json for escaping.
                out.push_str(
                    &serde_json::to_string(k).expect("string key serialization is infallible"),
                );
                out.push(':');
                write_canonical(v, out);
            }
            out.push('}');
        }
        Value::Array(items) => {
            out.push('[');
            for (i, item) in items.iter().enumerate() {
                if i > 0 {
                    out.push(',');
                }
                write_canonical(item, out);
            }
            out.push(']');
        }
        // Null / Bool / Number / String: serde_json is deterministic + infallible.
        scalar => out.push_str(
            &serde_json::to_string(scalar).expect("scalar json serialization is infallible"),
        ),
    }
}
